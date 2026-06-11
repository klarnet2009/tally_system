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
#define TALLY_SIGNAL_LOST_MS 3000 // Slave alarms after this much radio silence

// TX preamble 40 symbols (~12.6 ms at SF7/BW406): guarantees a full RX window
// of a duty-cycled receiver (3 ms on / 6 ms off) lands inside the preamble
#define TALLY_PREAMBLE_SYMBOLS 40
#define TALLY_DC_RX_MS 3    // duty-cycle RX window
#define TALLY_DC_SLEEP_MS 6 // duty-cycle sleep window (~33% radio-on)

#endif // TALLY_CONFIG_H
