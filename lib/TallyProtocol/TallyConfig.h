#ifndef TALLY_CONFIG_H
#define TALLY_CONFIG_H

// Shared radio/protocol parameters for the hub and all slaves.

#define TALLY_NET_ID 0xA1 // Change to run two tally systems side by side

// 2480 MHz: above WiFi ch 1-11, inside the 2400-2483.5 MHz ISM band.
// (The old implicit default, 2400.0 MHz, sat half outside the band edge
// and under WiFi channel 1.)
// Deliberately NOT a runtime knob: hub and slaves must change together or
// the link dies — a coordinated channel-switch protocol is the prerequisite
// for that. Keep within 2400.3–2483.3 MHz (BW/2 margin).
#define TALLY_RF_FREQ_HZ 2480000000UL

// AFA channel set (adaptive frequency agility): the escape channels sit in
// the gaps of the standard WiFi 1/6/11 grid; index 0 is the home channel.
// The hub announces a switch on the current channel (CMD_SET_CHANNEL), the
// fleet follows; a slave that misses the announcement finds the hub again by
// scanning this list (~2s per channel) once it declares signal-lost. Keep
// the list SHORT — scan time on a deaf slave grows with every entry.
#define TALLY_CHAN_COUNT 3
#define TALLY_CHAN_LIST                                                       \
  { TALLY_RF_FREQ_HZ, 2449500000UL, 2424500000UL }

#define TALLY_REFRESH_MS 500      // Periodic STATE_ALL re-send = link heartbeat
// Derived from the heartbeat so they track it automatically. Raising
// TALLY_REFRESH_MS must not silently desync the receivers' timers.
#define TALLY_SIGNAL_LOST_MS (6 * TALLY_REFRESH_MS) // Alarm after 6 missed beats
#define TALLY_RX_REARM_MS (3 * TALLY_REFRESH_MS)    // RX safety-net re-arm
// Grace before a slave shows the "source stale" (ATEM frozen) indication, so a
// brief switcher reconnect doesn't flicker the light.
#define TALLY_SOURCE_GRACE_MS 2000
// Slave -> hub telemetry interval (jittered per device to avoid lockstep
// collisions on the shared channel).
#define TALLY_TELEMETRY_MS 2000
#define TALLY_TELEMETRY_JITTER_MS 37 // per-camId offset, worst case 16*37 = 592ms

// TX preamble 40 symbols (~50 ms at SF9/BW406): guarantees a full RX window
// of a duty-cycled receiver lands inside the preamble. Duty-cycle timing must
// track the symbol time (1.26 ms at SF9): the chip's header-wait budget after
// a preamble detect is sleep + 2*rx, so rx >= ~11.3 ms and sleep + 2*rx must
// exceed the preamble — 12/28 gives 52 ms with ~30% radio-on (was 3/6 at SF7,
// which is only 2.4 symbols per window at SF9 and misses most packets).
#define TALLY_PREAMBLE_SYMBOLS 40
#define TALLY_DC_RX_MS 12   // duty-cycle RX window
#define TALLY_DC_SLEEP_MS 28 // duty-cycle sleep window (~30% radio-on)

// SX1280 *chip* TX power in dBm (-18..+12). The E28-2G4M27S adds ~13-14 dB of
// external PA gain on top, so this is NOT the radiated EIRP. Applied on every
// (re)init via tallyApplyRadioProfile() so it can't silently default.
// WARNING (regulatory): EU EN 300 328 caps 2.4 GHz SRD at +20 dBm EIRP. The
// final radiated power must be measured/chosen deliberately — do NOT raise this
// toward +12 (≈ +26 dBm at the antenna with this module) without (a) confirming
// the legal EIRP for the deployment region and (b) the rail decoupling fix, or
// the PA current spike browns out the shared 3V3 rail. Default kept low.
#define TALLY_TX_POWER 1

#endif // TALLY_CONFIG_H
