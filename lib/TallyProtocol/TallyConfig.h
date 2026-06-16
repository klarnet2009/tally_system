#ifndef TALLY_CONFIG_H
#define TALLY_CONFIG_H

// Shared radio/protocol parameters for the hub and all slaves.
// Override via build_flags where a per-device value is needed.

#ifndef TALLY_NET_ID
#define TALLY_NET_ID 0xA1 // Change to run two tally systems side by side
#endif

// 2480 MHz: above WiFi ch 1-11, inside the 2400-2483.5 MHz ISM band.
// (The old implicit default, 2400.0 MHz, sat half outside the band edge
// and under WiFi channel 1.)
#define TALLY_RF_FREQ_HZ 2480000000UL

#define TALLY_REFRESH_MS 500      // Periodic STATE_ALL re-send = link heartbeat
// Derived from the heartbeat so they track it automatically. Raising
// TALLY_REFRESH_MS must not silently desync the receivers' timers.
#define TALLY_SIGNAL_LOST_MS (6 * TALLY_REFRESH_MS) // Alarm after 6 missed beats
#define TALLY_RX_REARM_MS (3 * TALLY_REFRESH_MS)    // RX safety-net re-arm

// TX preamble 40 symbols (~12.6 ms at SF7/BW406): guarantees a full RX window
// of a duty-cycled receiver (3 ms on / 6 ms off) lands inside the preamble
#define TALLY_PREAMBLE_SYMBOLS 40
#define TALLY_DC_RX_MS 3    // duty-cycle RX window
#define TALLY_DC_SLEEP_MS 6 // duty-cycle sleep window (~33% radio-on)

// SX1280 *chip* TX power in dBm (-18..+12). The E28-2G4M27S adds ~13-14 dB of
// external PA gain on top, so this is NOT the radiated EIRP. Applied on every
// (re)init via tallyApplyRadioProfile() so it can't silently default.
// WARNING (regulatory): EU EN 300 328 caps 2.4 GHz SRD at +20 dBm EIRP. The
// final radiated power must be measured/chosen deliberately — do NOT raise this
// toward +12 (≈ +26 dBm at the antenna with this module) without (a) confirming
// the legal EIRP for the deployment region and (b) the rail decoupling fix, or
// the PA current spike browns out the shared 3V3 rail. Default kept low.
#ifndef TALLY_TX_POWER
#define TALLY_TX_POWER 1
#endif

#endif // TALLY_CONFIG_H
