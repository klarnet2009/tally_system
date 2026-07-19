#ifndef TALLY_RADIO_H
#define TALLY_RADIO_H

#include "E28_SX1280.h"
#include "TallyConfig.h"
#include "TallyLink.h"
#include "TallyProtocol.h"

// Apply the shared tally RF profile. begin() resets the radio to its 2.4 GHz /
// 12-symbol defaults, so this MUST run after every (re)init — on the hub, both
// slaves, and the radio_test bring-up firmwares, so every device agrees on the
// air interface. Single source of truth for "what frequency/preamble we use".
static inline void tallyApplyRadioProfile(E28Radio &radio) {
  radio.setFrequency(TALLY_RF_FREQ_HZ);
  radio.setPreambleLength(TALLY_PREAMBLE_SYMBOLS);
  radio.setTxPower(TALLY_TX_POWER); // was never applied by the hub before
  // Modulation is part of the shared profile too — never the driver default.
  // SF9/BW406: field range test showed ~30-40m through building walls at SF7
  // (RSSI hit the -108dBm SF7 floor at the edge — a genuine sensitivity
  // limit, not an antenna fault). SF9 buys ~+6dB (floor ~-114dBm) ≈ one more
  // wall / ~2x open-air, spent from our huge airtime budget: 9-byte frames at
  // 2/s. Cost: ~27→~96ms/frame airtime, so worst-case tally latency ~30→~100ms
  // (imperceptible for a light). CR 4/8 = the interference lever (double FEC).
  // WHOLE FLEET must match SF/BW — a mismatched node simply won't hear the hub
  // (unlike CR, which the explicit header carries per-packet). Reflash all
  // together. If SF9 isn't enough, SF10/11 needs the preamble trimmed too.
  radio.setSpreadingFactor(LORA_SF9);
  radio.setBandwidth(LORA_BW_0400);
  radio.setCodingRate(LORA_CR_4_8);
}

// Slave -> hub telemetry beat, shared by v1 and v2 so the two firmwares
// can't drift (they already had: v1's hand-copied block was missing v2's
// locator gate). Call every loop pass; sends at most one frame per jittered
// interval (camId*37ms offset breaks lockstep collisions on the shared
// channel), then re-arms RX. `allowNow` lets a firmware defer TX around
// UI-critical sections (both defer during the locator blink so a blocking
// ~15-100ms send can't stutter it). battMv=0 + TALLY_TLM_NO_BATTERY until a
// VBAT divider is wired on either board.
static inline void tallyTelemetryTick(E28Radio &radio, uint8_t camId,
                                      bool allowNow) {
  static uint32_t lastTlm = 0;
  uint32_t interval = TALLY_TELEMETRY_MS + (uint32_t)camId * 37;
  if (!allowNow || !radio.isConnected() || millis() - lastTlm <= interval)
    return;
  lastTlm = millis();
  TallyPacket t = TallyProtocol::createTelemetryPacket(
      camId, 0, radio.getRSSI(), TALLY_TLM_NO_BATTERY);
  uint8_t buf[TALLY_PACKET_SIZE];
  TallyProtocol::serialize(t, buf);
  radio.send(buf, TALLY_PACKET_SIZE); // blocking, ~one packet airtime
  radio.restartReceive();             // back to listening
}

// Slave radio recovery, shared by v1 and v2: if a runtime fault (stuck BUSY)
// latched the radio disconnected, re-init at most every 10s so a transient
// glitch can't leave the receiver permanently deaf. afterInit runs on a
// successful re-init, before noteAlive() — that's where a firmware restores
// its runtime radio state (power/channel overrides) and re-arms RX.
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
    tallyApplyRadioProfile(radio);
    if (afterInit)
      afterInit();
    link.noteAlive();
    Serial.println("[LoRa] Recovered");
  } else {
    Serial.printf("[LoRa] Recovery failed: %s\n", radio.initErrorStr());
  }
}

#endif // TALLY_RADIO_H
