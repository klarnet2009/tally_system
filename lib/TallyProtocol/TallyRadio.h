#ifndef TALLY_RADIO_H
#define TALLY_RADIO_H

#include "E28_SX1280.h"
#include "TallyConfig.h"
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
  // SF7/BW406 keeps airtime short; CR 4/8 is the interference lever: double
  // FEC redundancy lets a frame survive a WiFi/BT burst clipping part of it.
  // Airtime cost ~23→~27ms/frame on a mostly-idle channel. The explicit LoRa
  // header carries CR per-packet, so a mixed-CR fleet still interoperates
  // during a rolling firmware update (SF/BW/preamble stay unchanged).
  radio.setSpreadingFactor(LORA_SF7);
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

#endif // TALLY_RADIO_H
