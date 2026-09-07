#ifndef TALLY_CONFIG_H
#define TALLY_CONFIG_H

// Shared radio/protocol parameters for the hub and all slaves.
// Architecture: documentation/ARCHITECTURE_RF_V4.md

// Network id, packed into the LOW NIBBLE of byte 0 alongside the version.
// It is 4 bits, so valid values are 0x0..0xF. This used to be a full byte
// (0xA1) and the truncation was silent: two rigs set to 0xA1 and 0xB1 both mask
// to 0x1 and every frame from either hub is accepted by every slave of the
// other, so two "isolated" productions cross-light each other's cameras with no
// error anywhere. The static_assert below makes that impossible to ship.
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
// ch3 is the important one for EU. WiFi channels 1-13 tile
// 2402-2482 contiguously, so ch13 (2462-2482) covers our home frequency and a
// single 40 MHz AP can cover both mid-band escapes. Above 2482 there is no legal
// EU WiFi channel at all, so this is the quietest spot available — but NOT
// "WiFi-immune", which an earlier comment here claimed and which is wrong: at
// ~10 MHz from ch13's centre we sit inside its transmit spectral mask (-20 dBr),
// so a nearby AP on ch13 still lands tens of dB above the SF9 floor here. The
// gain over sitting inside an OCCUPIED 20 MHz channel is roughly 30 dB, which is
// large and worth having; immunity it is not. Confirm with `survey` on site.
// 2482.5 rather than 2483.0: the WiFi difference between them is nil (both are
// in the same mask region) while 2482.5 triples the margin from the 2483.5 band
// edge, which matters because the LoRa skirts extend past the occupied 406 kHz.
#define TALLY_CHAN_COUNT 4
#define TALLY_CHAN_LIST                                                       \
  { TALLY_RF_FREQ_HZ, 2449500000UL, 2424500000UL, 2482500000UL }

// ===== Timing =====
#define TALLY_REFRESH_MS 500 // Periodic STATE_ALL re-send = link heartbeat
// Derived from the heartbeat so they track it automatically.
#define TALLY_SIGNAL_LOST_MS (6 * TALLY_REFRESH_MS) // Alarm after 6 missed beats
// How long a slave may keep painting a SOLID colour without having received a
// STATE frame. Distinct from signal-lost, which any hub frame (a PING included)
// refreshes: under 60-80% loss a whole change burst plus several heartbeats can
// vanish while the odd frame still arrives, and the old single 3 s timer let the
// camera hold confident RED after it had been cut away from. Above this the
// slave shows "colour may be old" (white blink over the held colour).
#define TALLY_STATE_STALE_MS (3 * TALLY_REFRESH_MS)
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
// Copy count NEVER drops below MIN — an adaptive scheme that can reduce margin
// is a scheme where a wrong measurement costs a wrong light. Degradation can
// only ADD copies.
#define TALLY_BURST_COPIES_MIN 4
#define TALLY_BURST_COPIES_MAX 6
#define TALLY_BURST_OFFSETS_MS                                                \
  { 0, 60, 160, 330, 520, 740 }
// Checked, not asserted-by-hand: the previous guard static_asserted a #define
// that a human had to remember to update, so editing the table to start at 20 ms
// left the assert passing while every latency-critical first copy went out late.
// `inline` matters: a plain namespace-scope constexpr array in a header has
// INTERNAL linkage, so the inline function returning a pointer to it would bind
// to a different object per translation unit — an ODR violation, plus a private
// copy in every TU's .rodata.
inline constexpr uint16_t kTallyBurstOffsets[] = TALLY_BURST_OFFSETS_MS;
static_assert(sizeof(kTallyBurstOffsets) / sizeof(kTallyBurstOffsets[0]) ==
                  TALLY_BURST_COPIES_MAX,
              "TALLY_BURST_OFFSETS_MS must have TALLY_BURST_COPIES_MAX entries");
static_assert(kTallyBurstOffsets[0] == 0,
              "the first burst copy is scheduled for 'now', so offset 0 must be 0");

// ===== Listen-before-talk =====
// Transmitting into an already-active interferer is a guaranteed loss; slipping
// the frame a few ms is free. Applied to every frame EXCEPT the first copy of a
// change burst, which must not be delayed for latency reasons. Bounded: after
// the cap we transmit regardless, because a permanently busy channel must not
// silence the link.
#define TALLY_LBT_MARGIN_DB 15    // busy = this many dB above the noise floor
#define TALLY_LBT_MAX_DEFER_MS 12 // then send anyway

// ===== PHY profile =====
// SF9/BW406.25k/CR4/6 with a 10-symbol preamble = 50.7 ms per 9-byte frame
// (10.1% duty at the 500ms heartbeat). SF9 is kept for range: a field test hit
// the SF7 sensitivity floor at 30-40 m through walls, and SF9's ~-114 dBm
// floor buys ~6 dB. Preamble was 40 symbols ONLY to cover a duty-cycled
// receiver's RX window; duty-cycle RX is gone (it has an irreducible deaf
// window and reliability outranks battery here), so 35.8 ms per frame came
// back. CR 4/8 -> 4/6 because FEC fights random bit errors while a WiFi burst
// erases the whole frame — repetition is the stronger lever, and the freed
// airtime pays for it.
// Payload note: the payload symbol count quantizes, so 4..8 bytes all cost
// 43.2 ms and 9..12 bytes all cost 50.7 ms. The frame is 9 bytes because the app
// CRC has to be there (integrity is not authenticity — see TallyProtocol.h), and
// having paid for the step there are 3 spare bytes before the next one.
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
// One 9-byte frame at the production profile (50.7 ms), rounded up. Used by the
// timing invariants below and by the hub to keep its own heartbeat out of the
// fleet's uplink window.
#define TALLY_FRAME_AIRTIME_MS 51
#define TALLY_TLM_OFFSET_MS 60 // after heartbeat arrival (frame is ~51ms)
#define TALLY_TLM_BANK_MS 130  // extra offset for cameras 9..16
// How late a slot may still be used. It must stay BELOW bank separation minus
// one frame time, or a late bank-0 transmission runs straight through bank-1's
// slot — the collisions the slotting exists to eliminate. It was equal to
// TALLY_TLM_BANK_MS, which allowed exactly that.
// 70 ms of slack: a slave loop pass can exceed 40 ms (NeoPixel writes, serial,
// a debounce path), and missing the window costs a WHOLE telemetry period.
#define TALLY_TLM_LATE_MS 70
// Resulting per-camera telemetry period; the hub's reachability window derives
// from it.
#define TALLY_TELEMETRY_MS (TALLY_TLM_CYCLES * TALLY_REFRESH_MS)
// Counted from the START of an anchoring frame's transmission: the interval in
// which some slave may legitimately be on air with telemetry — the frame's own
// airtime (slaves anchor on its end), the slot offset, the bank-1 offset, the
// full late tolerance, the uplink frame itself, and loop-jitter margin. The hub
// must not transmit a lone heartbeat inside it. It used to: the heartbeat timer
// ran independently of the burst, so a heartbeat could land 100 ms after a
// burst's final copy — jamming exactly the camera whose slot that copy had just
// opened, and (half-duplex) costing the hub that camera's report as well.
#define TALLY_TLM_WINDOW_MS                                                    \
  (TALLY_FRAME_AIRTIME_MS + TALLY_TLM_OFFSET_MS + TALLY_TLM_BANK_MS +           \
   TALLY_TLM_LATE_MS + TALLY_FRAME_AIRTIME_MS + 20)

// ===== Coordinated channel switch =====
#define TALLY_CHAN_ANNOUNCE_BEATS 10 // heartbeats of countdown before switching
// The countdown is a 4-bit wire field and the hub requests a switch against the
// NEXT cycle (the current one's frame has already gone out), so the largest
// announceable value is 14, not 15. At 15 the first countdown would be 16, which
// reads as "expired" and retunes the hub with ZERO announcements.
static_assert(TALLY_CHAN_ANNOUNCE_BEATS <= 14,
              "announce beats must leave room for the +1 next-cycle offset");
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

// ---- Compile-time invariants: these constants are coupled, and every one of
// ---- these mistakes would be silent at runtime.
static_assert(TALLY_NET_ID <= 0x0F,
              "TALLY_NET_ID must fit in 4 bits: byte 0 truncates it, so larger "
              "values silently collide between 'different' systems");
static_assert(TALLY_BURST_COPIES_MIN <= TALLY_BURST_COPIES_MAX,
              "burst copy floor above the ceiling");
static_assert(TALLY_TLM_CYCLES >= 2, "telemetry needs at least two cycles");
// A late bank-0 frame must not be able to run through bank-1's slot.
static_assert(TALLY_TLM_LATE_MS + TALLY_FRAME_AIRTIME_MS <= TALLY_TLM_BANK_MS,
              "late-slot tolerance would let bank 0 overlap bank 1");
static_assert(TALLY_TLM_OFFSET_MS + TALLY_TLM_BANK_MS + TALLY_TLM_LATE_MS +
                      TALLY_FRAME_AIRTIME_MS <
                  TALLY_REFRESH_MS,
              "telemetry slots must fit inside one heartbeat cycle even when a "
              "bank-1 camera uses its full late tolerance");
// The hub holds a lone heartbeat back until the uplink window after the previous
// anchoring frame has closed. If that hold could exceed a heartbeat period, the
// held frame would collide with the NEXT timer's heartbeat instead.
static_assert(TALLY_TLM_WINDOW_MS < TALLY_REFRESH_MS,
              "uplink window must close within one heartbeat period");
// The state-stale threshold must tolerate ONE lost heartbeat even when the hub
// held the next one out of the uplink window (a legal ~882 ms gap), and must
// fire before signal-lost does — otherwise it would never be the one to show.
static_assert(TALLY_STATE_STALE_MS > 2 * TALLY_REFRESH_MS + TALLY_TLM_WINDOW_MS,
              "state-stale would trip on a single lost heartbeat");
static_assert(TALLY_STATE_STALE_MS < TALLY_SIGNAL_LOST_MS,
              "state-stale must precede signal-lost or it never shows");
static_assert(TALLY_SOURCE_GRACE_MS < TALLY_SIGNAL_LOST_MS,
              "source-stale must precede signal-lost or it never shows");

#endif // TALLY_CONFIG_H
