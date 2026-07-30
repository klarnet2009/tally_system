#ifndef TALLY_RADIO_H
#define TALLY_RADIO_H

#include "E28_SX1280.h"
#include "TallyConfig.h"
#include "TallyLink.h"
#include "TallyProtocol.h"

// Apply the shared tally RF profile. begin() resets the radio to its chip
// defaults, so this MUST run after every (re)init — on the hub, the slaves and
// the radio_test bring-up firmwares — so every device agrees on the air
// interface. Single source of truth for "what the physical layer is".
//
// The WHOLE FLEET must match SF/BW/preamble: a mismatched node simply won't
// hear the hub. Reflash together. (CR is carried per-packet by the explicit
// header, so it's the one parameter a receiver adapts to on its own.)
//
// Rationale for each value lives in TallyConfig.h beside the constant.
// txPowerDbm is a parameter, not a constant: the hub's module PA saturates at
// chip 0 dBm while the slaves' module has no PA at all, so there is no single
// correct value for both roles.
static inline void tallyApplyRadioProfile(E28Radio &radio, int8_t txPowerDbm) {
  radio.setFrequency(TALLY_RF_FREQ_HZ);
  radio.setPreambleLength(TALLY_PREAMBLE_SYMBOLS);
  radio.setTxPower(txPowerDbm);
  radio.setSpreadingFactor(LORA_SF9);
  radio.setBandwidth(LORA_BW_0400);
  radio.setCodingRate(LORA_CR_4_6);
}

// ===== Slotted telemetry (slave -> hub) =====
// Collision-free BY CONSTRUCTION instead of thinned by jitter: the heartbeat
// carries a cycle counter, so camera `id` transmits only in the cycle where
// hbCount % TALLY_TLM_CYCLES == (id-1) % TALLY_TLM_CYCLES, at a fixed offset
// after that heartbeat ARRIVED here. Cameras 1-8 and 9-16 can share a cycle but
// sit a full frame time apart (TALLY_TLM_BANK_MS), so at most two transmit per
// cycle and they never overlap.
//
// A slave that cannot hear heartbeats does not transmit at all. That is
// correct: its telemetry would be stale, and the hub marking it unreachable IS
// the signal we want.
//
// `allowNow` lets a firmware defer around UI-critical sections (the locator
// blink, where a blocking ~43ms send would stutter the pattern). battMv=0 +
// noBattery until a VBAT divider is wired.
static inline void tallyTelemetryTick(E28Radio &radio, TallyLink &link,
                                      uint8_t camId, bool allowNow) {
  static uint8_t lastSentHb = 0;
  static bool haveSent = false;

  if (!allowNow || !radio.isConnected() || !link.hbSeen())
    return;
  if (camId < 1 || camId > 16)
    return;

  uint8_t slot = (uint8_t)((camId - 1) % TALLY_TLM_CYCLES);
  uint8_t hb = link.lastHbCount();
  if ((uint8_t)(hb % TALLY_TLM_CYCLES) != slot)
    return; // not our cycle
  if (haveSent && hb == lastSentHb)
    return; // already transmitted in this cycle

  uint32_t bank = (uint32_t)(camId - 1) / TALLY_TLM_CYCLES; // 0: ids 1-8, 1: 9-16
  uint32_t dueAt =
      link.lastHbAtMs() + TALLY_TLM_OFFSET_MS + bank * TALLY_TLM_BANK_MS;
  if ((int32_t)(millis() - dueAt) < 0)
    return; // our slot hasn't opened yet
  // Slot already passed (we were busy in the locator or a long RX drain) —
  // skip this cycle rather than transmit late into the next device's slot.
  if (millis() - dueAt > TALLY_TLM_BANK_MS)
    return;
  // A frame arrived within the last frame-time: the hub is mid change-burst, so
  // transmitting now would collide with a copy we actually care about. Telemetry
  // is observability — yield. (Slots cannot avoid bursts entirely: burst offsets
  // are measured from the CUT, not from a heartbeat. The hub's reachability
  // window is 3 telemetry periods, so single skips are absorbed.)
  if (link.msSinceLastRx() < 50)
    return;

  lastSentHb = hb;
  haveSent = true;

  TallyPacket t = TallyProtocol::createTelemetryPacket(
      camId, radio.getRSSI(), link.missedBeats(), 0, true);
  uint8_t buf[TALLY_PACKET_SIZE];
  TallyProtocol::serialize(t, buf);
  radio.send(buf, TALLY_PACKET_SIZE); // blocking, ~43ms of airtime
  radio.restartReceive();             // back to listening
}

// Slave radio recovery: if a runtime fault (stuck BUSY) latched the radio
// disconnected, re-init at most every 10s so a transient glitch can't leave the
// receiver permanently deaf. afterInit runs on a successful re-init, before
// noteAlive() — that's where a firmware restores its runtime radio state (the
// current channel) and re-arms RX.
static inline void tallyRadioRecover(E28Radio &radio, TallyLink &link,
                                     int8_t sck, int8_t miso, int8_t mosi,
                                     int8_t nss, int8_t busy, int8_t dio1,
                                     int8_t rst, int8_t rxen, int8_t txen,
                                     void (*afterInit)()) {
  static uint32_t lastTry = 0;
  if (radio.isConnected())
    return;
  if (millis() - lastTry < 10000)
    return;
  lastTry = millis();
  Serial.println("[LoRa] Recovering...");
  if (radio.begin(sck, miso, mosi, nss, busy, dio1, rst, rxen, txen)) {
    tallyApplyRadioProfile(radio, TALLY_SLAVE_TX_POWER);
    if (afterInit)
      afterInit();
    link.noteAlive();
    Serial.println("[LoRa] Recovered");
  } else {
    Serial.printf("[LoRa] Recovery failed: %s\n", radio.initErrorStr());
  }
}

#endif // TALLY_RADIO_H
