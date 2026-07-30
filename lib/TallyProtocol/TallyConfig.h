#ifndef TALLY_CONFIG_H
#define TALLY_CONFIG_H

// Shared radio/protocol parameters for the hub and all slaves.
// Architecture: documentation/ARCHITECTURE_RF_V4.md

// 4-bit network id (packed with the version nibble in byte 0). Change to run
// two tally systems side by side.
#define TALLY_NET_ID 0xA

// ===== Channels =====
// 2480 MHz: clear of the US/common WiFi 1/6/11 grid, inside the 2400-2483.5
// ISM band. NOTE (EU): channels 12/13 are legal here and ch13 (2462-2482)
// covers this frequency — if the venue AP sits on 12/13 this is the WORST of
// the three. Survey with the `noise` command before a show.
#define TALLY_RF_FREQ_HZ 2480000000UL

// AFA channel set. The hub announces a switch inside the heartbeat itself
// (see TallyProtocol chanAnnounce) so ~10 decorrelated frames carry the plan;
// a slave that still misses it is picked up by the old-channel beacon, and
// only then falls back to scanning. Keep the list SHORT — scan time on a deaf
// slave grows with every entry.
#define TALLY_CHAN_COUNT 3
#define TALLY_CHAN_LIST                                                       \
  { TALLY_RF_FREQ_HZ, 2449500000UL, 2424500000UL }

// ===== Timing =====
#define TALLY_REFRESH_MS 500 // Periodic STATE_ALL re-send = link heartbeat
// Derived from the heartbeat so they track it automatically.
#define TALLY_SIGNAL_LOST_MS (6 * TALLY_REFRESH_MS) // Alarm after 6 missed beats
#define TALLY_RX_REARM_MS (3 * TALLY_REFRESH_MS)   // RX safety-net re-arm
// Grace before a slave shows the "source stale" (ATEM frozen) indication, so a
// brief switcher reconnect doesn't flicker the light.
#define TALLY_SOURCE_GRACE_MS 2000

// ===== Change burst (the primary reliability mechanism) =====
// A tally change is sent as N copies at IRREGULAR offsets from the change.
// Irregular on purpose: an even comb can alias with a periodic interferer
// (WiFi beacons are 102.4ms apart) and lose every copy. The >300ms span means
// one long interference burst cannot take all of them. The scheduler always
// serializes the LATEST state at transmit time, so a copy can never carry an
// already-superseded state.
#define TALLY_BURST_COPIES 4
#define TALLY_BURST_OFFSETS_MS                                                \
  { 0, 60, 160, 330 }

// ===== PHY profile =====
// SF9/BW406.25k/CR4/6 with a 10-symbol preamble = 43.2 ms per 8-byte frame
// (8.6% duty at the 500ms heartbeat). SF9 is kept for range: a field test hit
// the SF7 sensitivity floor at 30-40 m through walls, and SF9's ~-114 dBm
// floor buys ~6 dB. Preamble was 40 symbols ONLY to cover a duty-cycled
// receiver's RX window; duty-cycle RX is gone (it has an irreducible deaf
// window and reliability outranks battery here), so 35.8 ms per frame came
// back. CR 4/8 -> 4/6 because FEC fights random bit errors while a WiFi burst
// erases the whole frame — repetition is the stronger lever, and the freed
// airtime pays for it.
// Payload note: 4..8 bytes all cost the same 43.2 ms (the payload symbol count
// quantizes); 9 bytes costs 50.7 ms. The frame is 8 bytes to use that for free.
#define TALLY_PREAMBLE_SYMBOLS 10

// SX1280 *chip* TX power in dBm (-18..+12).
// Slaves (E28-2G4M12SX, no PA): chip power is the output, so max = best uplink
// margin. Telemetry is ~1% duty, so the battery cost is negligible.
#define TALLY_SLAVE_TX_POWER 12
// Hub (E28-2G4M27S): the module's PA SATURATES at chip ~0 dBm (~27 dBm out),
// so anything above 0 adds no range at all — only current draw, and the extra
// draw sags the shared 3V3 rail, which *does* cost output power. 0 dBm is
// strictly better than the old +12.
// WARNING (regulatory): ~27 dBm module output exceeds the EU EN 300 328 limit
// of 20 dBm EIRP at ANY chip setting. Legal operation needs measured
// attenuation — see documentation/ARCHITECTURE_RF_V4.md §12.
#ifndef TALLY_HUB_TX_POWER
#define TALLY_HUB_TX_POWER 0
#endif

// ===== Slotted telemetry (slave -> hub) =====
// The heartbeat carries a cycle counter, giving the fleet a shared time base
// with no extra protocol. Camera `id` transmits only in the cycle where
// hbCount % TALLY_TLM_CYCLES == (id-1) % TALLY_TLM_CYCLES, at a fixed offset
// after that heartbeat's arrival. At most two cameras per cycle (ids 1-8 and
// 9-16), separated by TALLY_TLM_BANK_MS > one frame time, so collisions are
// eliminated by construction rather than thinned by jitter.
#define TALLY_TLM_CYCLES 8
#define TALLY_TLM_OFFSET_MS 60  // after heartbeat arrival (frame is 43ms)
#define TALLY_TLM_BANK_MS 100   // extra offset for cameras 9..16
// Resulting per-camera telemetry period; the hub's reachability window derives
// from it.
#define TALLY_TELEMETRY_MS (TALLY_TLM_CYCLES * TALLY_REFRESH_MS)

// ===== Coordinated channel switch =====
#define TALLY_CHAN_ANNOUNCE_BEATS 10 // heartbeats of countdown before switching
// Slaves retune this much AFTER their computed switch instant, so a small
// clock skew makes them late (still covered by the beacon) rather than early
// (missing the hub's last heartbeats on the old channel).
#define TALLY_CHAN_SWITCH_MARGIN_MS 250
// After switching, the hub beacons the new channel on the OLD one: a stranded
// slave is recovered immediately instead of blind-scanning.
#define TALLY_OLD_BEACON_EVERY_MS 2000
#define TALLY_OLD_BEACON_FOR_MS 30000

// Slave scan dwell: >=2 chances to hear a 500ms heartbeat per channel.
#define TALLY_SCAN_DWELL_MS 1200

#endif // TALLY_CONFIG_H
