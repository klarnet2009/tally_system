#ifndef TALLY_RADIO_H
#define TALLY_RADIO_H

#include "E28_SX1280.h"
#include "TallyConfig.h"

// Apply the shared tally RF profile. begin() resets the radio to its 2.4 GHz /
// 12-symbol defaults, so this MUST run after every (re)init — on the hub, both
// slaves, and the radio_test bring-up firmwares, so every device agrees on the
// air interface. Single source of truth for "what frequency/preamble we use".
static inline void tallyApplyRadioProfile(E28Radio &radio) {
  radio.setFrequency(TALLY_RF_FREQ_HZ);
  radio.setPreambleLength(TALLY_PREAMBLE_SYMBOLS);
  radio.setTxPower(TALLY_TX_POWER); // was never applied by the hub before
}

#endif // TALLY_RADIO_H
