#include <ATEMbase.h>
#include <ATEMmin.h>
#include <Arduino.h>
#include <SkaarhojPgmspace.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_system.h> // esp_reset_reason()

#include "E28_SX1280.h"
#include "TallyBurst.h"
#include "TallyEscape.h"
#include "TallyLog.h"
#include "TallyProtocol.h"
#include "TallyRadio.h"
#include "config.h"

static E28Radio radio;
static uint32_t g_loraTxCount = 0;   // frames actually transmitted
static uint32_t g_loraDropCount = 0; // frames lost: radio down / TX failure

// Runtime TX power (serial "power N", chip dBm). radioInit() re-applies this
// after the shared profile so a live override survives radio recovery.
// NOTE: the hub module's PA saturates at chip 0 dBm — raising this adds current
// draw, not range, and the extra draw sags the rail (see TallyConfig.h).
static int8_t g_txPower = TALLY_HUB_TX_POWER;

// ===== Channel state (AFA) =====
static const uint32_t g_chanList[] = TALLY_CHAN_LIST;
static uint8_t g_chanIdx = 0;
static uint32_t g_chanFreq = TALLY_RF_FREQ_HZ;

// Coordinated switch. The plan rides inside the heartbeat (chanIdx + a
// countdown in beats), so ~10 decorrelated frames carry it instead of a burst of
// back-to-back announcements that share one interference burst.
// The countdown is DERIVED from a target beat number (TallyChanSwitch), never
// decremented: the previous counter was decremented in heartbeatTick() while the
// frame carrying it is serialized later in processTx(), so N beats put only N-1
// usable countdowns on air and beats==1 put 0 — which means "no switch pending",
// so the hub retuned with zero announcements and abandoned the fleet.
static TallyChanSwitch g_chanSwitch;

// After switching, beacon the new channel on the OLD one for a while: a slave
// that missed every announcement is recovered immediately instead of scanning.
static uint32_t g_oldChanFreq = 0; // 0 = not beaconing
static uint32_t g_beaconUntilMs = 0;
static uint32_t g_beaconNextMs = 0;

// Heartbeat cycle counter — the fleet's shared time base for telemetry slots.
static uint8_t g_hbCount = 0;
// When the last ANCHORING frame (unflagged STATE_ALL: lone heartbeat or a
// burst's final copy) started transmitting. Slaves anchor their telemetry slots
// on it, so the hub must stay off the air for TALLY_TLM_WINDOW_MS after it.
static uint32_t g_lastAnchorTxMs = 0;

// ===== Debug logging =====
// The S3 has two consoles: Serial = native USB-Serial-JTAG (GPIO19/20),
// Serial0 = UART0 via the devkit's USB bridge (GPIO43/44). Neither touches
// the E28 pins. Log to both so whichever cable is plugged in shows logs.
static void hublogf(const char *fmt, ...) {
  char buf[256]; // worst-case [STATUS] line is ~127 chars; headroom for growth
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  Serial0.print(buf);
  TallyLog.write(buf); // flash sink: field logs survive without a tether
}

// Locator state (file scope: triggered by the BOOT button and the serial
// "ping" command): 3 pings 100ms apart, then a ~2s cooldown before another
// trigger is accepted. locatorPingsLeft == 0 = idle.
static uint8_t locatorPingsLeft = 0;
static uint32_t locatorNextMs = 0; // next ping; after the last, re-arm time

// Current tally masks broadcast to the slaves; the OLED grid is derived from
// these too, so there is one source of truth for "what's on air".
static uint16_t g_progMask = 0;
static uint16_t g_prevMask = 0;

// ATEM connection phase. atemPhase == ATEM_RUNNING is the single "connected"
// signal (no separate shadow flag to keep in sync).
enum AtemPhase : uint8_t { ATEM_IDLE, ATEM_CONNECTING, ATEM_RUNNING };
static AtemPhase atemPhase = ATEM_IDLE;
// When the tally masks were last actually refreshed from the source (ATEM poll
// or the test-mode generator). sourceLiveNow() is keyed off THIS, not off the
// connection phase: a connected ATEM whose masks nobody has polled for half a
// second is a frozen source, whatever the socket says.
static uint32_t g_lastSourcePollMs = 0;
#define SOURCE_FRESH_MS 500UL // >> POLL_MS; tolerates OLED blits and log flushes

// sourceLive: true only while the masks are being refreshed. Any blocking path
// that keeps heartbeats going but stops polling (that was fleetPump() during
// `log` and `survey`) now drops SOURCE_LIVE within SOURCE_FRESH_MS, and the
// fleet shows "don't trust" instead of holding a colour nobody is updating.
static inline bool sourceLiveNow() {
  bool fresh = millis() - g_lastSourcePollMs < SOURCE_FRESH_MS;
#ifdef LORA_TEST_MODE
  return fresh;
#else
  return atemPhase == ATEM_RUNNING && fresh;
#endif
}

// True while the last frame went out on a frequency other than the working one
// (the old-channel beacon). Gates channel measurement, so it must be cleared on
// every path that ends an off-channel transmit — including a radio fault, which
// previously latched it true and killed measurement for the rest of uptime.
static bool g_txOffChannel = false;

// Last frame that actually made it onto the air. The hub can detect a dead radio
// (isConnected false) but NOT a radio that reports fine while every send silently
// fails — heartbeats would simply stop and only the slaves would notice. Cheap
// insurance: no successful TX for this long -> full re-init. Armed by
// radioInit(), the single place that means "the radio is usable from now on".
static uint32_t g_lastTxOkMs = 0;
#define TX_WATCHDOG_MS 5000

// begin() resets the chip to its defaults, so the shared RF profile must be
// reapplied on every (re)init — including in-field recovery.
static bool radioInit() {
  bool ok = radio.begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI, E28_PIN_NSS,
                        E28_PIN_BUSY, E28_PIN_DIO1, E28_PIN_RESET, E28_PIN_RXEN,
                        E28_PIN_TXEN);
  if (ok) {
    tallyApplyRadioProfile(radio, g_txPower);
    // One line of evidence per init that the SF-dependent register the datasheet
    // requires (§14.4.1) holds the profile's value — begin() already fails on a
    // mismatch, but the field log should show the number, not just "OK".
    uint8_t sfReg = 0, fec = 0;
    radio.readRegister(SX1280_REG_LORA_SF_CONFIG, sfReg);
    radio.readRegister(SX1280_REG_FREQ_ERR_CORR, fec);
    hublogf("[E28] SF reg 0x0925=0x%02X FEC bit0=%u (CRC-16 on)\n", sfReg,
            fec & 1);
    radio.setFrequency(g_chanFreq); // keep the current AFA channel
    radio.startReceive(); // listen for slave telemetry between our frames
    g_txOffChannel = false;
    // Arm the TX watchdog HERE, the one place that means "the radio is usable
    // from now on". Arming it in setup() alone made it fire immediately after an
    // in-field repair (the clock had been running while the module was absent),
    // logging a false failure and forcing a redundant re-init; keying it off the
    // first successful transmit instead left it permanently inert when the fault
    // was present from power-on — the very case it exists for.
    g_lastTxOkMs = millis();
  }
  return ok;
}

// ===== Per-camera reachability (from slave CMD_TELEMETRY) =====
// The hub is otherwise blind to whether a slave is actually lit; this closes
// the loop. Indexed by camera ID 1..16.
static uint32_t g_camLastSeen[17] = {0};
static int8_t g_camRssi[17] = {0};   // downlink: how loudly the slave hears us
static int8_t g_camRxRssi[17] = {0}; // uplink: telemetry loudness at the hub
static uint8_t g_camMissed[17] = {0}; // slave's missed-heartbeat gradient (0-15)
static bool g_camReachable[17] = {false};
static uint8_t g_camTag[17] = {0};    // MAC-derived device tag (0 = unknown)
static TallyState g_camShown[17] = {STATE_OFF}; // what the slave says it shows
static uint32_t g_camMismatchSince[17] = {0};   // shown != commanded since when
// Telemetry is slotted, so the period is exact (TALLY_TELEMETRY_MS) rather than
// jittered — but a slave deliberately SKIPS its slot while a change burst is in
// flight, so during fast cutting several periods in a row can be missed. 4 gives
// margin for that; a camera that is merely quiet must never flap offline.
#define CAM_REACHABLE_MS (4 * TALLY_TELEMETRY_MS)
// A camera that is TALKING to us and reporting missed heartbeats is struggling on
// the DOWNLINK — the direction that matters, and unambiguous evidence about the
// channel. "Unreachable" is deliberately NOT folded in here: a silent camera may
// simply be switched off, and treating that as a bad channel migrated the whole
// fleet during a break. The full classification lives in TallyEscape.
static inline bool camComplaining(uint8_t id) {
  return g_camReachable[id] && g_camMissed[id] >= ESC_POOR_MISSED;
}

// ===== Transmit scheduler: ALWAYS the latest state =====
// v3 used a packet FIFO and could transmit an already-superseded state: a
// second cut 50 ms after the first left copies of the OLD masks queued ahead of
// the new ones, so a slave showed the previous colour for up to ~290 ms — a
// visibly wrong light. The scheduler now holds only a repeat COUNT; every copy
// is serialized from the live masks at transmit time, so a stale state can
// never reach the air.
// Scheduling arithmetic lives in TallyBurst (lib/TallyProtocol/TallyBurst.h) so
// it can be exercised on the host with a controllable clock — see tests/.
static TallyBurst g_burst;
// One LBT gate PER TRANSMIT CLASS. A single shared deferral clock let a beacon
// that had been waiting consume the burst's patience, pushing a tally copy
// straight into a busy channel; separate instances make that impossible.
static TallyLbt g_lbtState;
static TallyLbt g_lbtOneShot;

// Channel-escape policy, extracted and unit-tested (lib/TallyProtocol/TallyEscape.h).
static TallyEscape g_escape;

// One-shot frames (locator ping, old-channel beacon) are events rather than
// state, so they keep a tiny FIFO. freqHz != 0 transmits on that frequency
// instead of the current channel — that is the beacon's whole purpose.
struct OneShot {
  TallyPacket pkt;
  uint32_t freqHz;
};
#define ONESHOT_QUEUE_SIZE 8
static OneShot g_oneShot[ONESHOT_QUEUE_SIZE];
static uint8_t g_osHead = 0, g_osTail = 0;

// ===== Channel quality measurement (DIAGNOSTIC + listen-before-talk) =====
// Deliberately NOT in the channel-switch decision. v3 decided on measured noise
// and got it wrong three different ways; the switch decision now uses the direct
// signal (are the slaves missing heartbeats?) and these numbers only inform the
// operator and gate LBT. Nothing here can cause a bad automatic decision.
//
// Sampled passively while the radio sits in RX between frames — one SPI read,
// no blocking, and the radio NEVER leaves the working channel to measure. (v3
// had a scout that retuned away; it produced the worst bug in this subsystem
// and, being off-channel, was a standing risk to the link. Deleted.)
struct ChanStat {
  int8_t floorDbm; // tracked minimum = the real noise floor
  float busyFrac;  // share of samples well above that floor
  bool valid;
};
static ChanStat g_chan[TALLY_CHAN_COUNT];
static int16_t g_accMin = 127;
static uint16_t g_accN = 0;
static uint16_t g_accBusy = 0;
#define CHAN_BUSY_ABOVE_FLOOR 10

static void chanMeasureInit() {
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    g_chan[i].floorDbm = -100;
    g_chan[i].busyFrac = 0.0f;
    g_chan[i].valid = false;
  }
}

static void chanAddSample(ChanStat &cs, int8_t rssi) {
  g_accN++;
  if (rssi < g_accMin)
    g_accMin = rssi;
  if (cs.valid && rssi > cs.floorDbm + CHAN_BUSY_ABOVE_FLOOR)
    g_accBusy++;
}

static void chanCommit(ChanStat &cs) {
  if (g_accN == 0)
    return;
  int8_t f = (int8_t)g_accMin;
  if (!cs.valid || f < cs.floorDbm) {
    cs.floorDbm = f; // a quieter reading is the truth: take it at once
  } else {
    // Creep up by at least 1 dB: a plain (diff >> 2) is 0 below 4 dB, which
    // pinned the floor at any transient dip forever and then inflated busyFrac
    // against a floor that no longer existed.
    int step = (f - cs.floorDbm) >> 2;
    if (step < 1)
      step = 1;
    cs.floorDbm = (int8_t)(cs.floorDbm + step);
  }
  float busy = (float)g_accBusy / (float)g_accN;
  cs.busyFrac = cs.valid ? (0.7f * cs.busyFrac + 0.3f * busy) : busy;
  cs.valid = true;
  g_accMin = 127;
  g_accN = 0;
  g_accBusy = 0;
}

// Free: the hub is in RX between frames anyway.
static void chanMeasureTick() {
  if (!radio.isConnected() || radio.txActive() || g_txOffChannel)
    return;
  if (g_chanIdx >= TALLY_CHAN_COUNT)
    return;
  int8_t r = radio.getRssiInst();
  if (r == 0) // driver's documented "no reading"
    return;
  chanAddSample(g_chan[g_chanIdx], r);
}

// ===== Listen-before-talk =====
// Sending into an already-active interferer is a guaranteed loss; slipping the
// frame a few ms is free. Bounded, and never applied to the first copy of a
// change burst — tally latency outranks collision avoidance.
static bool channelBusyNow() {
  const ChanStat &cs = g_chan[g_chanIdx < TALLY_CHAN_COUNT ? g_chanIdx : 0];
  if (!cs.valid)
    return false; // no floor reference yet — never block on an unknown
  int8_t r = radio.getRssiInst();
  if (r == 0)
    return false;
  return r > cs.floorDbm + TALLY_LBT_MARGIN_DB;
}

// Set by autoRfTick(); heartbeatTick() adds burst copies when the fleet is
// struggling. Copies are only ever ADDED, never removed.
static bool g_linkDegraded = false;


static void enqueueOneShot(const TallyPacket &pkt, uint32_t freqHz = 0) {
  uint8_t next = (uint8_t)((g_osHead + 1) % ONESHOT_QUEUE_SIZE);
  if (next == g_osTail) {
    g_loraDropCount++;
    return;
  }
  g_oneShot[g_osHead].pkt = pkt;
  g_oneShot[g_osHead].freqHz = freqHz;
  g_osHead = next;
}

// Serialized fresh on every copy — this is what makes a stale state impossible.
static TallyPacket buildStateFrame() {
  return TallyProtocol::createStateAllPacket(
      g_progMask, g_prevMask, sourceLiveNow(), g_hbCount,
      g_chanSwitch.pending() ? g_chanSwitch.target() : 0,
      g_chanSwitch.countdown(g_hbCount), g_burst.flagAsBurst());
}

static void applyChannelSwitch() {
  g_oldChanFreq = g_chanFreq; // beacon target
  // Flush the measurement window BEFORE retuning: the accumulators hold samples
  // taken on the OLD channel, and committing them after the retune folded them
  // into the NEW channel's stats — which then became the listen-before-talk
  // reference for a frequency they were never measured on.
  chanCommit(g_chan[g_chanIdx]); // also resets the accumulators
  g_escape.noteSwitching(millis(), g_chanIdx);
  g_chanIdx = g_chanSwitch.target();
  g_chanFreq = g_chanList[g_chanIdx];
  g_chanSwitch.clear();
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  uint32_t now = millis();
  g_beaconUntilMs = now + TALLY_OLD_BEACON_FOR_MS;
  g_beaconNextMs = now + TALLY_OLD_BEACON_EVERY_MS;
  hublogf("[CHAN] hub now on ch%u (%lu.%lu MHz), beaconing old for %lus\n",
          g_chanIdx, (unsigned long)(g_chanFreq / 1000000UL),
          (unsigned long)((g_chanFreq % 1000000UL) / 100000UL),
          (unsigned long)(TALLY_OLD_BEACON_FOR_MS / 1000));
}

// Start announcing a coordinated switch. Idempotent-ish: a new request while
// one is pending simply retargets it.
static void requestChannelSwitch(uint8_t idx, const char *reason,
                                 uint8_t beats = TALLY_CHAN_ANNOUNCE_BEATS) {
  if (idx >= TALLY_CHAN_COUNT || idx == g_chanIdx)
    return;
  // Announcement length is NOT shortened under severe degradation. That was
  // backwards: the severe tier fires exactly when frame loss is high, which is
  // when a marginal slave needs MORE chances to hear the plan, not fewer. At 50%
  // loss, 9 announcements miss with p~0.2% while 3 miss with p~12.5%, and a
  // stranded slave is dark for ~8s. Faster escape comes from the shorter
  // SUSTAIN, not from a shorter announcement.
  // +1 because the frame for the CURRENT cycle has already been transmitted by
  // the time this runs (processTx precedes autoRfTick in loop()). Requesting
  // against g_hbCount produced beats-1 announcements — the same N-vs-N-1 loss the
  // derived countdown was introduced to eliminate, one layer up.
  g_chanSwitch.request(idx, (uint8_t)(g_hbCount + 1), beats);
  hublogf("[CHAN] announcing switch to ch%u over %u beats: %s\n", idx, beats,
          reason);
}

static void processTx() {
  static uint32_t lastTxDoneMs = 0;

  // Finish the in-flight frame first; the loop never blocks on airtime (~43ms).
  if (radio.txActive()) {
    if (radio.checkTxDone()) {
      // A TX that timed out (stuck PA / antenna fault) is a drop, not a send,
      // so the counters surface a failing radio instead of hiding it.
      if (radio.txSucceeded())
        g_loraTxCount++;
      else
        g_loraDropCount++;
      lastTxDoneMs = millis();
      g_lastTxOkMs = radio.txSucceeded() ? lastTxDoneMs : g_lastTxOkMs;
      if (g_txOffChannel) { // beacon finished — back to the working channel
        radio.setFrequency(g_chanFreq);
        g_txOffChannel = false;
      }
      radio.startReceive(); // back to listening for telemetry
      // The last announcing frame is now on air: it is safe to retune. NOT gated
      // on the burst being idle: the slaves retune on their own clocks when the
      // countdown runs out, so under sustained cutting (burst never idle) the
      // hub used to stay behind on the old channel while the whole fleet had
      // moved — and its remaining copies must go where the fleet now is.
      if (g_chanSwitch.expired(g_hbCount))
        applyChannelSwitch();
    }
    return;
  }

  // Radio down: account for what can't go out instead of stalling on SPI.
  if (!radio.isConnected()) {
    if (!g_burst.idle()) {
      g_loraDropCount++;
      g_burst.abandon();
    }
    while (g_osHead != g_osTail) {
      g_loraDropCount++;
      g_osTail = (uint8_t)((g_osTail + 1) % ONESHOT_QUEUE_SIZE);
    }
    // Stale deferral clocks would let the FIRST frame after recovery bypass LBT
    // entirely (its patience "ran out" during the outage), and a latched
    // off-channel flag would disable channel measurement for the rest of uptime.
    g_lbtState.reset();
    g_lbtOneShot.reset();
    g_txOffChannel = false;
    return;
  }

  if (g_chanSwitch.expired(g_hbCount)) { // txActive() is false here
    applyChannelSwitch();
    return;
  }

  if (millis() - lastTxDoneMs < 2) // minimum inter-frame gap
    return;

  // One-shots first: the locator is a human-triggered, time-sensitive event,
  // and a beacon that waits for a burst to finish may miss its window.
  if (g_osHead != g_osTail) {
    OneShot &os = g_oneShot[g_osTail];
    // An off-channel frame (the old-channel beacon) cannot be gated by a busy
    // measurement of the CURRENT channel — that is the wrong spectrum, and we
    // cannot measure the target without retuning. Send it; it is a rescue frame.
    bool offChannel = (os.freqHz != 0 && os.freqHz != g_chanFreq);
    if (!offChannel && !g_lbtOneShot.clear(millis(), false, channelBusyNow()))
      return;
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(os.pkt, buf);
    if (offChannel)
      radio.setFrequency(os.freqHz);
    if (radio.startSend(buf, TALLY_PACKET_SIZE)) {
      g_txOffChannel = offChannel;
      g_osTail = (uint8_t)((g_osTail + 1) % ONESHOT_QUEUE_SIZE);
    } else if (offChannel) {
      radio.setFrequency(g_chanFreq); // never leave the radio parked off-channel
    }
    return;
  }

  // State copies on the burst schedule. The FIRST copy of a change goes out
  // immediately — its latency is the whole point; later copies and heartbeats
  // yield briefly to an active interferer.
  if (g_burst.dueNow(millis())) {
    // One call, one encoding. The && short-circuits the MEASUREMENT (a blocking
    // SPI read the first copy of a cut must never pay for) while still letting
    // clear() see latencyCritical — which is what resets the deferral clock.
    // Hoisting that reset into a separate branch left the parameter dead in
    // production, so the tested path and the live path were different code.
    bool crit = g_burst.latencyCritical();
    if (!g_lbtState.clear(millis(), crit, !crit && channelBusyNow()))
      return;
    // Read BEFORE markSent() advances the index: is this the copy the fleet
    // anchors its uplink slots on (the unflagged one)?
    bool anchor = !g_burst.flagAsBurst();
    TallyPacket pkt = buildStateFrame();
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(pkt, buf);
    if (radio.startSend(buf, TALLY_PACKET_SIZE)) {
      uint32_t now = millis();
      g_burst.markSent(now);
      if (anchor)
        g_lastAnchorTxMs = now; // opens the fleet's uplink window
    }
  }
}

// Receive slave telemetry during idle (non-TX) windows and update the
// reachability table.
static void serviceTelemetry() {
  if (radio.txActive() || !radio.isConnected())
    return;
  if (!radio.available())
    return;
  uint8_t buf[TALLY_PACKET_SIZE];
  uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);
  // Cheap re-arm (IRQ clear + SET_RX): a full startReceive() would bounce
  // through standby and could cut a frame already arriving.
  radio.rearmAfterIrq();
  if (len == 0)
    return;
  TallyPacket pkt;
  if (!TallyProtocol::deserialize(buf, len, pkt))
    return;
  if (TallyProtocol::cmd(pkt) != CMD_TELEMETRY)
    return;
  uint8_t id = TallyProtocol::telemetryCamId(pkt);
  if (id < 1 || id > 16)
    return;
  g_camLastSeen[id] = millis();
  g_camRssi[id] = TallyProtocol::telemetryRssi(pkt); // downlink (slave-heard)
  g_camRxRssi[id] = radio.getRSSI();                 // uplink (hub-heard)
  g_camMissed[id] = TallyProtocol::telemetryMissed(pkt);
  g_camShown[id] = TallyProtocol::telemetryShown(pkt);

  // Two devices with the same camera ID: easy to do by accident now that the ID
  // lives in NVS, and completely invisible before this. Both would light on the
  // same tally and both would transmit in the same slot, so the hub only saw a
  // "flaky" camera. The device tag makes it explicit.
  uint8_t tag = TallyProtocol::telemetryTag(pkt);
  if (tag != 0) {
    if (g_camTag[id] != 0 && g_camTag[id] != tag) {
      static uint32_t lastDupLog = 0;
      if (millis() - lastDupLog > 30000) {
        lastDupLog = millis();
        hublogf("[TLM] *** cam %u: TWO devices with this ID (tags %02X and %02X)"
                " — reassign one\n", id, g_camTag[id], tag);
      }
    }
    g_camTag[id] = tag;
  }

  // Verify what the fleet actually DISPLAYS instead of assuming it obeyed. One
  // mismatch is normal (the slave may be a heartbeat behind); a persistent one
  // means a stuck slave, a wrong camera ID, or a firmware fault.
  uint16_t bit = 1U << (id - 1);
  TallyState want = (g_progMask & bit)
                        ? ((g_prevMask & bit) ? STATE_BOTH : STATE_PROGRAM)
                        : ((g_prevMask & bit) ? STATE_PREVIEW : STATE_OFF);
  if (g_camShown[id] == want) {
    g_camMismatchSince[id] = 0;
  } else if (g_camMismatchSince[id] == 0) {
    g_camMismatchSince[id] = millis();
  } else if (millis() - g_camMismatchSince[id] > 3 * TALLY_TELEMETRY_MS) {
    g_camMismatchSince[id] = millis(); // re-arm so this logs at most per window
    hublogf("[TLM] *** cam %u shows %d but should show %d — stuck slave?\n", id,
            (int)g_camShown[id], (int)want);
  }
}

#include "autorf.h"

// Mark cameras online/offline and log only the transitions (not every beat),
// so a dead/returning slave is visible without log spam.
static void sweepReachability() {
  for (uint8_t id = 1; id <= 16; id++) {
    bool reach = g_camLastSeen[id] != 0 &&
                 (millis() - g_camLastSeen[id] < CAM_REACHABLE_MS);
    if (reach != g_camReachable[id]) {
      g_camReachable[id] = reach;
      hublogf("[TLM] cam %u %s (dl=%d dBm missed=%u)\n", id,
              reach ? "ONLINE" : "OFFLINE", g_camRssi[id], g_camMissed[id]);
    }
  }
}

// === STATE_ALL: change burst + periodic heartbeat.
// Runs even with ATEM down — losing the switcher must not look like a dead
// radio link to the slaves; they keep the last known masks, flagged stale.
static void heartbeatTick() {
  static uint32_t lastHbMs = 0;
  static uint16_t lastProg = 0xFFFF;
  static uint16_t lastPrev = 0xFFFF;
  uint32_t now = millis();

  if (g_progMask != lastProg || g_prevMask != lastPrev) {
    lastProg = g_progMask;
    lastPrev = g_prevMask;
    // A cut supersedes any copies still pending: restart the burst. Because
    // every copy re-reads the live masks at transmit time, the ones already sent
    // were correct for their moment and the ones not yet sent carry the new
    // state. More copies while the fleet struggles, never fewer.
    g_burst.onChange(now, g_linkDegraded);
    return; // the burst covers this interval; heartbeat timer untouched
  }

  if (now - lastHbMs < TALLY_REFRESH_MS)
    return;
  lastHbMs = now;

  // Nothing to decrement: TallyChanSwitch derives the countdown from g_hbCount,
  // so the value a frame carries cannot depend on when it was scheduled.
  // The cycle counter is a STEADY shared clock: it advances on this timer,
  // unconditionally. Gating it on "a heartbeat frame was actually scheduled" was
  // a regression — a tick landing inside a burst consumed the beat without
  // incrementing, so frequent cuts froze the counter, which froze the derived
  // channel countdown, which deadlocked every switch and (via the old
  // frames-per-cycle heuristic) silenced the whole fleet's telemetry.
  // Burst copies are identified by TALLY_FLAG_BURST instead, so nothing depends
  // on this counter standing still.
  g_hbCount++;
  // The FRAME, unlike the counter, waits until the uplink window opened by the
  // last anchoring frame has closed: a burst's final copy tells one camera to
  // transmit ~60 ms later, and a heartbeat from this timer used to land right on
  // top of it. No-op while a burst is in progress (its copies are the heartbeat).
  g_burst.scheduleHeartbeat(now, g_lastAnchorTxMs + TALLY_TLM_WINDOW_MS);
}

// Beacon the current channel on the OLD one after a switch, so a slave that
// missed every in-heartbeat announcement is recovered at once. It is a normal
// STATE_ALL (correct tally!) carrying countdown=1, so the stranded slave gets
// both the right colour and the retune instruction.
static void beaconTick() {
  if (!g_oldChanFreq)
    return;
  uint32_t now = millis();
  if ((int32_t)(now - g_beaconUntilMs) >= 0) {
    g_oldChanFreq = 0;
    return;
  }
  if ((int32_t)(now - g_beaconNextMs) < 0)
    return;
  if (radio.txActive())
    return;
  // One-shots outrank burst copies in processTx(), so take the air only when no
  // copy is due within one beacon's airtime — and never ahead of a cut's first
  // copy. The old gate (txIdle(): burst idle AND queue empty) was never true
  // under sustained cutting, so the beacon — the stranded slaves' only rescue —
  // starved in exactly the situation that strands them.
  if (!g_burst.idle() &&
      (g_burst.latencyCritical() ||
       g_burst.untilNext(now) < TALLY_FRAME_AIRTIME_MS + 15))
    return;
  g_beaconNextMs = now + TALLY_OLD_BEACON_EVERY_MS;
  TallyPacket pkt = TallyProtocol::createStateAllPacket(
      g_progMask, g_prevMask, sourceLiveNow(), g_hbCount, g_chanIdx, 1);
  enqueueOneShot(pkt, g_oldChanFreq);
}

// ===== OLED / ATEM =====
TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(128, 64, &I2Cbus, -1);

#ifndef LORA_TEST_MODE
static ATEMmin atem;
static uint32_t lastPoll = 0;
static uint32_t lastAtemAttempt = 0;
#endif

// ===== Improved OLED UI =====
// Dual-color display: Yellow top 16px, Blue bottom 48px
#define HDR_H 16 // Yellow zone height

// Status bar icons (drawn in yellow zone)
static void drawWifiIcon(int x, int y, bool connected) {
  if (connected) {
    // WiFi arc
    display.drawPixel(x + 2, y, WHITE);
    display.drawLine(x + 1, y + 1, x + 3, y + 1, WHITE);
    display.drawLine(x, y + 2, x + 4, y + 2, WHITE);
    display.drawPixel(x + 2, y + 4, WHITE);
    display.drawPixel(x + 2, y + 5, WHITE);
  } else {
    display.drawLine(x, y, x + 4, y + 5, WHITE);
    display.drawLine(x + 4, y, x, y + 5, WHITE);
  }
}

static void drawLoRaIcon(int x, int y, bool connected) {
  if (connected) {
    // Antenna icon
    display.drawLine(x + 2, y, x + 2, y + 5, WHITE);
    display.drawPixel(x, y + 1, WHITE);
    display.drawPixel(x + 4, y + 1, WHITE);
    display.drawPixel(x + 1, y, WHITE);
    display.drawPixel(x + 3, y, WHITE);
  } else {
    // X mark
    display.drawLine(x, y, x + 4, y + 5, WHITE);
    display.drawLine(x + 4, y, x, y + 5, WHITE);
  }
}

// TX power as % of the chip range (-18..+12 dBm): 100% = full power.
static uint8_t pwrPercent(int8_t p) { return (uint8_t)(((int)p + 18) * 100 / 30); }

void drawStatusBar(const String &ip, bool wifiOk, bool loraOk, bool atemOk) {
  display.fillRect(0, 0, 128, HDR_H, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);

  // Row 1 (y=0): Icons + label
  drawWifiIcon(1, 1, wifiOk);
  drawLoRaIcon(9, 1, loraOk);

  // ATEM dot
  if (atemOk)
    display.fillCircle(19, 3, 2, WHITE);
  else
    display.drawCircle(19, 3, 2, WHITE);

  display.setCursor(25, 0);
  display.print(atemOk ? "ATEM OK" : "NO ATEM");

  // TX power % (right side of row 1)
  display.setCursor(104, 0);
  display.print(pwrPercent(g_txPower));
  display.print("%");

  // Row 2 (y=8): IP + LoRa status
  display.setCursor(1, 8);
  display.print(ip);
  // LoRa debug status (right side)
  display.setCursor(86, 8);
  display.print(loraOk ? "LoRa OK" : "LoRa X");

  // Separator line at zone boundary
  display.drawFastHLine(0, 15, 128, WHITE);
}

void drawCenteredMsg(const char *l1, const char *l2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  int y1 = l2 ? 22 : 28;
  int x1 = (128 - strlen(l1) * 6) / 2;
  if (x1 < 0)
    x1 = 0;
  display.setCursor(x1, y1);
  display.print(l1);

  if (l2) {
    int x2 = (128 - strlen(l2) * 6) / 2;
    if (x2 < 0)
      x2 = 0;
    display.setCursor(x2, y1 + 12);
    display.print(l2);
  }
  display.display();
}

static void drawCell(int x, int y, int w, int h, uint8_t camNum,
                     uint8_t status) {
  // Calculate centered position for camera number
  char numStr[4];
  snprintf(numStr, sizeof(numStr), "%d", camNum);

  if (status == 2) {
    // ====== PROGRAM (ON AIR) ======
    // Fully filled white cell — maximum visibility
    display.fillRoundRect(x + 1, y + 1, w - 2, h - 2, 3, WHITE);

    // Large black number centered
    display.setTextSize(2);
    display.setTextColor(BLACK, WHITE);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);

  } else if (status == 1) {
    // ====== PREVIEW ======
    // Thick border (double outline), white number
    display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, WHITE);
    display.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 2, WHITE);

    // Large white number centered
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);

  } else {
    // ====== OFF ======
    // Thin outline only
    display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 2, WHITE);

    // Small dim number centered
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 6) / 2;
    int ty = y + (h - 7) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);
  }
}

// Derived from the same masks we broadcast, so the OLED can never disagree
// with the slaves. The mono display can't show a distinct BOTH colour, so a
// camera that is both on-air and in preview renders as PROGRAM (filled) —
// on-air is the safety-critical state to surface.
void drawTallyGrid(uint16_t progMask, uint16_t prevMask) {
  const int cols = 4, rows = 2;
  const int cellW = 128 / cols;          // 32px
  const int cellH = (64 - HDR_H) / rows; // 27px
  const int yOff = HDR_H;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      int idx = r * cols + c;
      uint8_t human = TALLY_INPUTS[idx];
      uint8_t status = 0;
      if (human >= 1 && human <= 16) { // guard the bit shift against bad config
        uint16_t bit = 1U << (human - 1);
        status = (progMask & bit) ? 2 : ((prevMask & bit) ? 1 : 0);
      }
      int x = c * cellW;
      int y = yOff + r * cellH;
      drawCell(x, y, cellW, cellH, human, status);
    }
  }
}

// ===== LoRa Debug Screen =====
void drawLoRaDebug() {
  uint8_t st = radio.getChipStatus();
  // ⚡ Bolt: Eliminate redundant SPI transaction by deriving connection state directly from the status byte
  bool connected = (st != 0xFF && st != 0x00);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Yellow zone (top 16px)
  display.setCursor(1, 0);
  display.print(connected ? "LoRa: OK  " : "LoRa: ERR ");

  // Show pins
  display.print("B:");
  display.print(digitalRead(E28_PIN_BUSY) ? "1 " : "0 ");
  display.print("D:");
  display.print(digitalRead(E28_PIN_DIO1) ? "1" : "0");

  display.setCursor(1, 8);
  display.print("SPI:0x");
  if (st < 0x10)
    display.print("0");
  display.print(st, HEX);

  // Decode chip mode — SX1280 status byte: mode = bits [7:5]
  // (0x2=STDBY_RC 0x3=STDBY_XOSC 0x4=FS 0x5=RX 0x6=TX; 0/1/7 invalid)
  uint8_t mode = (st >> 5) & 0x07;
  const char *m = "??";
  if (mode == 2)
    m = "STB";
  else if (mode == 3)
    m = "XOS";
  else if (mode == 4)
    m = "FS ";
  else if (mode == 5)
    m = "RX ";
  else if (mode == 6)
    m = "TX ";
  display.print(" ");
  display.print(m);

  // Blinker to show it's alive
  static bool blinker = false;
  blinker = !blinker;
  display.setCursor(110, 8);
  display.print(blinker ? "*" : "o");

  display.drawFastHLine(0, 15, 128, WHITE);

  // Blue zone (bottom 48px) - 4 rows
  // Row 1: RSSI + SNR
  display.setCursor(1, 18);
  display.print("RSSI: ");
  display.print(radio.getRSSI());
  display.print("dBm");
  display.setCursor(75, 18);
  display.print("SNR:");
  display.print(radio.getSNR());

  // Row 2: TX / drop counters + TX power %
  display.setCursor(1, 28);
  display.print("TX:");
  display.print(g_loraTxCount);
  display.print(" Drop:");
  display.print(g_loraDropCount);
  display.setCursor(100, 28);
  display.print(pwrPercent(g_txPower));
  display.print("%");

  // Row 3: SPI Diagnosis
  display.setCursor(1, 38);
  if (st == 0xFF)
    display.print("ERR: MISO HIGH");
  else if (st == 0x00)
    display.print("ERR: MISO LOW");
  else
    display.print("SPI looks OK");

  // Row 4: Uptime & ATEM
  display.setCursor(1, 48);
  display.print("Up:");
  uint32_t sec = millis() / 1000;
  display.print(sec);
  display.print("s ");
  display.print(atemPhase == ATEM_RUNNING ? "[ATEM OK]" : "[NO ATEM]");

  display.display();
}

#ifndef LORA_TEST_MODE
// ===== ATEM connection: non-blocking state machine =====
// Never blocks the loop — the radio queue, locator and OLED keep running
// while the connection is (re)established in the background.
static uint32_t atemAttemptStart = 0;

static void atemTick() {
  // Resolve the configured IP once
  static bool ipParsed = false;
  static bool ipValid = false;
  static IPAddress target;
  if (!ipParsed) {
    ipParsed = true;
    String cfgIP = String(ATEM_IP_STR);
    if (cfgIP.length())
      ipValid = target.fromString(cfgIP);
  }
  if (!ipValid)
    return; // no ATEM configured — hub keeps broadcasting last-known masks

  switch (atemPhase) {
  case ATEM_IDLE:
    if (WiFi.status() == WL_CONNECTED &&
        millis() - lastAtemAttempt > ATEM_RETRY_MS) {
      lastAtemAttempt = millis();
      atem.begin(target);
      atem.connect();
      atemAttemptStart = millis();
      atemPhase = ATEM_CONNECTING;
      hublogf("[ATEM] connecting to %s...\n", target.toString().c_str());
    }
    break;

  case ATEM_CONNECTING:
    atem.runLoop();
    if (atem.isConnected()) {
      atemPhase = ATEM_RUNNING;
      hublogf("[ATEM] connected\n");
    } else if (millis() - atemAttemptStart > ATEM_CONNECT_TIMEOUT_MS) {
      atemPhase = ATEM_IDLE;
      lastAtemAttempt = millis(); // the logged retry delay must actually happen
      hublogf("[ATEM] connect timeout, retry in %ds\n", ATEM_RETRY_MS / 1000);
    }
    break;

  case ATEM_RUNNING:
    atem.runLoop();
    if (!atem.isConnected()) {
      atemPhase = ATEM_IDLE;
      lastAtemAttempt = millis();
      hublogf("[ATEM] connection lost\n");
    }
    break;
  }
}
#endif // LORA_TEST_MODE

void setup() {
  // Native USB CDC; no host attached must never block the loop
  Serial.begin();
  // 1ms, NOT 0: HWCDC::write()'s retry loop does `tries--` on a uint32_t with
  // no zero-guard, so 0 underflows to 0xFFFFFFFF on the first no-progress pass
  // (host attached but nobody reading, TX ring full) and loop() freezes for
  // ~49 days — the exact hang this timeout is meant to prevent. 1 = one 1ms
  // retry, then CDC is marked disconnected and logs drop silently as intended.
  Serial.setTxTimeoutMs(1);
  Serial0.begin(115200); // UART bridge port (GPIO43/44)

  // Flash log first, so even the boot banner lands in it (2x256KB ring)
  TallyLog.begin(262144);

  I2Cbus.begin(OLED_I2C_SDA, OLED_I2C_SCL, 400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setTextWrap(false);
  display.clearDisplay();
  display.display();

  pinMode(0, INPUT_PULLUP); // Boot button for locator

  hublogf("\n=== Tally HUB (ESP32-S3 + E28-2G4M27S) ===\n");
  hublogf("[LOG] flash log %s, %u KB used — dump: 'log', wipe: 'logclear'\n",
          TallyLog.ok() ? "OK" : "UNAVAILABLE",
          (unsigned)(TallyLog.size() / 1024));
  // Reset reason turns "module not connected after a warm reboot" from a
  // guess into a fact: ESP_RST_BROWNOUT after a TX storm = the PA/WiFi rail
  // is sagging (decouple it), vs ESP_RST_SW/POWERON = ordinary boot.
  hublogf("[BOOT] reset_reason=%d\n", (int)esp_reset_reason());
  hublogf("[CFG] netId=0x%02X freq=%lu preamble=%d power=%d refresh=%dms\n",
          TALLY_NET_ID, (unsigned long)TALLY_RF_FREQ_HZ,
          TALLY_PREAMBLE_SYMBOLS, TALLY_HUB_TX_POWER, TALLY_REFRESH_MS);
#ifdef LORA_TEST_MODE
  hublogf("[CFG] LORA_TEST_MODE active — ATEM disabled, cam1 toggle stream\n");
#endif

  // ==== E28 LoRa FIRST (with retry) ====
  // Before WiFi: the radio is the hub's core function, and WiFi's TX bursts
  // on the shared 3V3 rail are the last thing a 27dBm module needs while
  // it powers up (historically a source of flaky inits).
  bool radioOk = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    drawCenteredMsg("LoRa init...",
                    (String("Attempt ") + String(attempt) + "/5").c_str());
    radioOk = radioInit();
    if (radioOk)
      break;
    // Show WHY on both the OLED and serial, not just "FAILED"
    hublogf("[E28] attempt %d/5 failed: %s (status=0x%02X BUSY=%d DIO1=%d)\n",
            attempt, radio.initErrorStr(), radio.getChipStatus(),
            digitalRead(E28_PIN_BUSY), digitalRead(E28_PIN_DIO1));
    drawCenteredMsg("LoRa FAILED", radio.initErrorStr());
    delay(500);
  }
  if (radioOk)
    hublogf("[E28] init OK (status=0x%02X)\n", radio.getChipStatus());
  else
    hublogf("[E28] init FAILED after 5 attempts — recovery retries every 10s\n");

  // ==== Wi-Fi ====
#ifndef NO_WIFI
  drawCenteredMsg("Wi-Fi: connecting...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // ждём Wi‑Fi (статичный экран уже нарисован выше)
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_RETRY_MS)
    delay(150);
  if (WiFi.status() != WL_CONNECTED) {
    hublogf("[WiFi] FAILED to join '%s' — retrying in background\n", WIFI_SSID);
    drawCenteredMsg("Wi-Fi: FAILED", "Retrying forever...");
  } else {
    hublogf("[WiFi] connected, IP %s\n", WiFi.localIP().toString().c_str());
    drawCenteredMsg("Wi-Fi: connected", WiFi.localIP().toString().c_str());
  }
#else
  // Radio bring-up build: keep the WiFi radio fully off so it can't contend
  // for the 3.3V rail with the E28 PA (the brownout path noted above).
  WiFi.mode(WIFI_OFF);
  hublogf("[WiFi] disabled (NO_WIFI build) — radio debug mode\n");
  drawCenteredMsg("Wi-Fi: OFF", "radio debug build");
#endif

  // NOTE: no boot channel survey. The hub always starts on the HOME channel so
  // every slave finds it immediately at power-up; if the channel turns out bad,
  // the ordinary announced migration moves the fleet. v3 surveyed at boot and
  // could start on an escape channel, which cost every slave ~5s of scanning
  // (a dark camera) at power-on for a guess made in an empty venue.

  // Show LoRa debug screen for 3 seconds
  drawLoRaDebug();
  delay(3000);
}

// Keep the fleet alive across a long blocking operation (the log dump streams
// up to ~512KB at 115200 ≈ 45s; the band survey sweeps for ~9s). Without it the
// loop freezes for longer than TALLY_SIGNAL_LOST_MS and every camera declares
// signal-lost and starts channel-scanning, the TX watchdog fires, and the ATEM
// session goes unserviced.
//
// It calls the REAL heartbeatTick(), deliberately: the cycle counter is the
// fleet's shared clock and the channel-switch countdown is derived from it, so a
// pump that invents its own beats runs the countdown off wall time.
// Refresh the tally masks from the source. ONE function, called from loop() and
// from fleetPump(): the pump used to run heartbeats WITHOUT this, so while `log`
// streamed (~45 s) or `survey` swept (~10 s) every heartbeat carried frozen
// masks flagged SOURCE_LIVE — the fleet trusted a colour nobody was updating,
// and every cut in that window was invisible: the camera cut away from held
// RED, the camera cut to stayed dark. Worse than having no pump at all (the
// fleet would have gone to safe orange in 3 s). sourceLiveNow() additionally
// keys off g_lastSourcePollMs so any FUTURE blocking path fails safe.
static void tallySourcePoll() {
#ifdef LORA_TEST_MODE
  // === TEST STREAM: cam 1 RED <-> GREEN every 1s (radio bring-up without ATEM)
  static uint32_t lastToggle = 0;
  static bool testRed = true;
  if (millis() - lastToggle > 1000) {
    lastToggle = millis();
    testRed = !testRed;
    g_progMask = testRed ? 0x0001 : 0x0000;
    g_prevMask = testRed ? 0x0000 : 0x0001;
  }
  g_lastSourcePollMs = millis();
#else
  atemTick();
  if (atemPhase == ATEM_RUNNING && millis() - lastPoll > POLL_MS) {
    lastPoll = millis();
    g_lastSourcePollMs = lastPoll;
    g_progMask = 0;
    g_prevMask = 0;
    for (int i = 0; i < 8; i++) {
      uint8_t human = TALLY_INPUTS[i];
      if (human < 1 || human > 16)
        continue; // out-of-range entry would make the bit shift below UB
      uint8_t idx0 = human - 1;
      uint8_t tflags = atem.getTallyByIndexTallyFlags(idx0); // bit0 pgm, bit1 pvw
      if (tflags & 0x01)
        g_progMask |= (1U << idx0);
      if (tflags & 0x02)
        g_prevMask |= (1U << idx0);
    }
  }
#endif
}

// Everything the fleet needs while some command blocks loop(): finish the frame
// in flight, take telemetry, refresh the SOURCE, run the heartbeat scheduler,
// and transmit — the trailing processTx() is what lets a cut noticed by this
// call go out now rather than a whole pump interval later.
static void fleetPump() {
  processTx();
  serviceTelemetry();
  tallySourcePoll();
  heartbeatTick();
  processTx();
}

// ===== Full-band survey =====
// Sweep the entire 2.4 GHz ISM band and print where the energy actually is, so
// the channel set can be chosen for THIS venue instead of assumed. It exists
// because the compiled channel plan bakes in the US 1/6/11 grid, and in EU
// (channels 1-13, contiguous 2402-2482) that assumption can be simply wrong —
// the home channel may sit inside an occupied AP channel with no way to know.
//
// Deliberately OPERATOR-TRIGGERED and blocking. This is the opposite of the
// automatic scout that was deleted: that one retuned the radio away from the
// working channel every few seconds DURING a show, went off-channel while the
// fleet was live, and fed a cost function that drove automatic switching. This
// runs before doors, at the operator's request, with nothing depending on the
// link — so blocking is free and there is no failure mode to design around.
// Its output informs a human decision; it never moves the fleet by itself.
#define SURVEY_STEP_KHZ 1000
#define SURVEY_LO_HZ 2400000000UL
#define SURVEY_HI_HZ 2483000000UL
#define SURVEY_SAMPLES 24
// The sweep is ~9 s of blocking, which is longer than TALLY_SIGNAL_LOST_MS (3 s)
// and TX_WATCHDOG_MS (5 s): left alone it made every camera declare signal-lost
// and start channel-scanning, and ended in a false "no successful TX" alarm plus
// a forced re-init — a diagnostic that broke the thing it was diagnosing. So the
// sweep returns to the working channel periodically and sends one heartbeat.
#define SURVEY_PUMP_MS 350

static void bandSurvey(Stream *io) {
  if (!radio.isConnected()) {
    hublogf("[SURVEY] radio DEAD — fix the module first\n");
    return;
  }
  if (radio.txActive() || !g_burst.idle()) {
    hublogf("[SURVEY] TX in flight — try again in a moment\n");
    return;
  }
  // A survey while a switch is being announced would interleave off-channel
  // sweeping with the countdown the whole fleet is timing against.
  if (g_chanSwitch.pending() || g_oldChanFreq) {
    hublogf("[SURVEY] a channel switch is in progress — try again after it\n");
    return;
  }
  (void)io; // the report goes to hublogf: both consoles AND the flash log
  uint32_t steps = (SURVEY_HI_HZ - SURVEY_LO_HZ) / (SURVEY_STEP_KHZ * 1000UL) + 1;
  // TWO passes (a floor pass, then a busy pass against that floor), plus the
  // heartbeat pump. The old estimate counted one pass and understated by half.
  uint32_t perStep = 6 + SURVEY_SAMPLES * 2;
  hublogf("[SURVEY] sweeping %lu.%lu-%lu.%lu MHz in %u kHz steps, 2 passes "
          "(~%lus; heartbeat keeps beating)\n",
          SURVEY_LO_HZ / 1000000UL, (SURVEY_LO_HZ % 1000000UL) / 100000UL,
          SURVEY_HI_HZ / 1000000UL, (SURVEY_HI_HZ % 1000000UL) / 100000UL,
          (unsigned)SURVEY_STEP_KHZ,
          (unsigned long)(steps * perStep * 2 / 1000 + steps * perStep * 2 / SURVEY_PUMP_MS / 10));

  // Two passes: the first establishes each step's floor, the second counts how
  // often energy sits well above it. One pass cannot do both — busy-ness is only
  // meaningful relative to a floor, which is why a single-pass survey reports
  // every channel as 0% busy.
  static int8_t floorDbm[128];
  static uint8_t busyPct[128];
  if (steps > 128)
    steps = 128;
  uint32_t lastPump = millis();
  for (uint8_t pass = 0; pass < 2; pass++) {
    for (uint32_t i = 0; i < steps; i++) {
      if (millis() - lastPump > SURVEY_PUMP_MS) {
        radio.setFrequency(g_chanFreq); // pump must transmit on the REAL channel
        radio.startReceive();
        // Drain the WHOLE burst before going off-channel again: a cut noticed by
        // the pump schedules copies up to 740 ms out, and a 300 ms guard left
        // the later ones waiting for the next pump slot.
        uint32_t guard = millis();
        do {
          fleetPump();
        } while ((!g_burst.idle() || radio.txActive()) &&
                 millis() - guard < kTallyBurstOffsets[TALLY_BURST_COPIES_MAX - 1] +
                                        2 * TALLY_FRAME_AIRTIME_MS + 100);
        lastPump = millis();
      }
      uint32_t f = SURVEY_LO_HZ + i * SURVEY_STEP_KHZ * 1000UL;
      radio.setFrequency(f);
      radio.startReceive();
      delay(6);
      int8_t lo = 127;
      uint8_t busy = 0;
      for (uint8_t k = 0; k < SURVEY_SAMPLES; k++) {
        int8_t r = radio.getRssiInst();
        if (r != 0) {
          if (r < lo)
            lo = r;
          if (pass == 1 && r > floorDbm[i] + CHAN_BUSY_ABOVE_FLOOR)
            busy++;
        }
        delay(2);
      }
      if (pass == 0)
        floorDbm[i] = (lo == 127) ? -100 : lo;
      else
        busyPct[i] = (uint8_t)((uint16_t)busy * 100 / SURVEY_SAMPLES);
    }
  }

  // Restore the working channel BEFORE printing: the print is slow and the fleet
  // should not stay deaf for it.
  radio.setFrequency(g_chanFreq);
  radio.startReceive();

  hublogf("[SURVEY]  MHz   floor  busy  bar (busy%% over the floor)\n");
  for (uint32_t i = 0; i < steps; i++) {
    uint32_t f = SURVEY_LO_HZ + i * SURVEY_STEP_KHZ * 1000UL;
    char bar[21];
    uint8_t n = (uint8_t)(busyPct[i] * 20 / 100);
    for (uint8_t k = 0; k < 20; k++)
      bar[k] = (k < n) ? '#' : ((k % 5 == 0) ? '.' : ' ');
    bar[20] = 0;
    // Mark the configured channels so the operator can see at a glance whether
    // the plan matches the room.
    const char *mark = "  ";
    for (uint8_t c = 0; c < TALLY_CHAN_COUNT; c++)
      if (f / 500000UL == g_chanList[c] / 500000UL)
        mark = (c == g_chanIdx) ? "<*" : "<c";
    hublogf("[SURVEY] %4lu.%lu %5d %4u%%  %s %s\n", f / 1000000UL,
            (f % 1000000UL) / 100000UL, (int)floorDbm[i], busyPct[i], bar, mark);
    // ~90 lines to two consoles plus a LittleFS write is seconds of blocking on
    // its own — the sweep is not the only part that can starve the heartbeat.
    if ((i & 0x07) == 0)
      fleetPump();
  }

  // Recommend a set: quietest steps first, spaced so two picks cannot sit inside
  // one 20 MHz WiFi channel, and kept away from the band edges by half a
  // bandwidth. This is a RECOMMENDATION for TALLY_CHAN_LIST, not an action —
  // hub and slaves must agree on the table, so changing it is a deliberate
  // reflash of the whole fleet, never something a survey does behind your back.
  hublogf("[SURVEY] --- recommended TALLY_CHAN_LIST for this room ---\n");
  uint8_t picked[TALLY_CHAN_COUNT];
  uint8_t nPicked = 0;
  for (uint8_t want = 0; want < TALLY_CHAN_COUNT; want++) {
    uint32_t bestI = 0xFFFFFFFF;
    int bestCost = 1 << 30;
    for (uint32_t i = 0; i < steps; i++) {
      uint32_t f = SURVEY_LO_HZ + i * SURVEY_STEP_KHZ * 1000UL;
      if (f < 2400500000UL || f > 2482500000UL)
        continue; // half-bandwidth margin from both band edges
      bool tooClose = false;
      for (uint8_t p = 0; p < nPicked; p++) {
        uint32_t pf = SURVEY_LO_HZ + picked[p] * SURVEY_STEP_KHZ * 1000UL;
        uint32_t d = (f > pf) ? (f - pf) : (pf - f);
        if (d < 20000000UL)
          tooClose = true; // inside one WiFi channel width of an earlier pick
      }
      if (tooClose)
        continue;
      // Busy-ness dominates, floor breaks ties. The two must be SCALED: floors
      // span ~50 dB while busy spans 0-100, so summing them raw made the ranking
      // arbitrary and could hand back a deep-floor but heavily occupied step —
      // and put it at index 0, the channel every slave boots on.
      int cost = (int)floorDbm[i] * 10 + (int)busyPct[i] * 6;
      if (cost < bestCost) {
        bestCost = cost;
        bestI = i;
      }
    }
    if (bestI == 0xFFFFFFFF)
      break;
    picked[nPicked++] = (uint8_t)bestI;
  }
  for (uint8_t p = 0; p < nPicked; p++) {
    uint32_t f = SURVEY_LO_HZ + picked[p] * SURVEY_STEP_KHZ * 1000UL;
    hublogf("[SURVEY]   %luUL,   // %lu.%lu MHz  floor=%d busy=%u%%\n",
            (unsigned long)f, f / 1000000UL, (f % 1000000UL) / 100000UL,
            (int)floorDbm[picked[p]], busyPct[picked[p]]);
  }
  hublogf("[SURVEY] index 0 is the home channel: every slave boots there, so put\n"
          "[SURVEY] the quietest pick first. Reflash hub AND all cameras\n"
          "[SURVEY] together — a mismatched table strands them.\n");
}

// AFA site survey: sample the ambient noise floor on every candidate channel
// and print a comparison. Blocks the loop ~150ms/channel — acceptable for a
// hand-typed diagnostic (slaves tolerate 6 missed heartbeats).
static void noiseSurvey() {
  if (!radio.isConnected()) {
    hublogf("[NOISE] radio DEAD — fix the module first\n");
    return;
  }
  if (radio.txActive()) {
    // Retuning mid-TX kills the packet on air and miscounts the drop
    hublogf("[NOISE] TX in flight — try again in a moment\n");
    return;
  }
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    radio.setFrequency(g_chanList[i]);
    radio.startReceive();
    delay(25); // PLL settle + RSSI integration
    // Floor + busy share, NOT a mean of dBm: averaging dBm is a geometric mean
    // of power and barely sees the bursts that actually erase frames (a 37 dB
    // error in the worst case — audit finding 3).
    int8_t floorDbm = 127;
    int8_t peak = -127; // loudest sample = worst interferer
    const int N = 32;
    int8_t samp[32];
    for (int k = 0; k < N; k++) {
      int8_t r = radio.getRssiInst();
      samp[k] = r;
      if (r < floorDbm)
        floorDbm = r;
      if (r > peak)
        peak = r;
      delay(3);
    }
    int busy = 0;
    for (int k = 0; k < N; k++)
      if (samp[k] > floorDbm + CHAN_BUSY_ABOVE_FLOOR)
        busy++;
    hublogf("[NOISE] ch%u %lu.%lu MHz: floor=%d dBm busy=%d%% peak=%d dBm%s\n", i,
            g_chanList[i] / 1000000UL, (g_chanList[i] % 1000000UL) / 100000UL,
            (int)floorDbm, busy * 100 / N, (int)peak,
            g_chanList[i] == g_chanFreq ? "  <- current" : "");
  }
  // Back to the working channel
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  hublogf("[NOISE] floor below -100 = clean; busy%% is what breaks frames — a\n"
          "[NOISE] high floor with busy~0 is harmless, LoRa decodes below noise\n");
}



// ===== Serial console (both ports): status / ping / reinit / help =====
// io = the port the command arrived on (log dumps go only there); nullptr
// for internally-generated commands (the 10s status heartbeat).
static void handleSerialCommand(const String &cmd, Stream *io = nullptr) {
  if (cmd == "status") {
    // "q" is now the transmit scheduler's state: copies of the CURRENT state
    // still to send, plus any queued one-shots (locator / old-channel beacon).
    uint8_t qDepth =
        (uint8_t)(g_burst.pending() + ((g_osHead + ONESHOT_QUEUE_SIZE -
                                       g_osTail) %
                                      ONESHOT_QUEUE_SIZE));
    char ipbuf[20];
    if (WiFi.status() == WL_CONNECTED)
      snprintf(ipbuf, sizeof(ipbuf), "%u.%u.%u.%u", WiFi.localIP()[0],
               WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    else
      strcpy(ipbuf, "DOWN");
    uint16_t reachMask = 0;
    for (uint8_t id = 1; id <= 16; id++)
      if (g_camReachable[id])
        reachMask |= (1U << (id - 1));
    hublogf("[STATUS] up=%lus radio=%s(0x%02X) pwr=%d(%u%%) ch=%lu.%lu tx=%lu "
            "drop=%lu rxerr=%lu q=%u wifi=%s atem=%s auto=%s prog=0x%04X "
            "prev=0x%04X reach=0x%04X\n",
            (unsigned long)(millis() / 1000), radio.isConnected() ? "OK" : "DEAD",
            radio.getChipStatus(), (int)g_txPower, (unsigned)pwrPercent(g_txPower),
            g_chanFreq / 1000000UL,
            (g_chanFreq % 1000000UL) / 100000UL, (unsigned long)g_loraTxCount,
            (unsigned long)g_loraDropCount, (unsigned long)radio.getRxErrors(),
            qDepth, ipbuf,
            atemPhase == ATEM_RUNNING      ? "RUNNING"
            : atemPhase == ATEM_CONNECTING ? "CONNECTING"
                                           : "IDLE",
            g_autoRf ? "on" : "OFF", g_progMask, g_prevMask, reachMask);
  } else if (cmd == "ping") {
    if (locatorPingsLeft == 0 && (int32_t)(millis() - locatorNextMs) >= 0) {
      hublogf("[CMD] locator ping -> cam 1\n");
      locatorPingsLeft = 3;
      locatorNextMs = millis();
    }
  } else if (cmd == "reinit") {
    if (radio.txActive()) {
      hublogf("[CMD] TX in flight — reinit would kill it; try again\n");
      return;
    }
    hublogf("[CMD] radio re-init: %s (%s)\n",
            radioInit() ? "OK" : "FAILED", radio.initErrorStr());
  } else if (cmd.startsWith("power")) {
    // Range/stability testing: change chip TX power live, no reflash.
    // The 27S PA adds ~14 dB on top of the chip dBm figure.
    String arg = cmd.substring(5);
    arg.trim();
    if (!arg.length()) {
      hublogf("[PWR] chip=%d dBm (%u%%). The 27S PA SATURATES at chip 0 dBm —\n"
              "[PWR] above 0 adds current, not range. Set: power <-18..12>\n",
              (int)g_txPower, (unsigned)pwrPercent(g_txPower));
    } else if (arg[0] != '-' && (arg[0] < '0' || arg[0] > '9')) {
      hublogf("[PWR] usage: power <-18..12>\n");
    } else {
      int p = arg.toInt();
      if (p < -18 || p > 12) {
        hublogf("[PWR] out of range: chip is -18..12 dBm\n");
      } else {
        autoRfManualOverride("power");
        g_txPower = (int8_t)p;
        radio.setTxPower(g_txPower);
        hublogf("[PWR] chip=%d dBm (%u%%)%s\n", p, (unsigned)pwrPercent(g_txPower),
                p > 0 ? " — WARNING: the PA is already saturated at 0 dBm, so"
                        " this only draws more current and sags the rail"
                      : "");
      }
    }
  } else if (cmd.startsWith("chan")) {
    String arg = cmd.substring(4);
    arg.trim();
    if (!arg.length()) {
      for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
        hublogf("[CHAN] %u: %lu.%lu MHz%s\n", i, g_chanList[i] / 1000000UL,
                (g_chanList[i] % 1000000UL) / 100000UL,
                g_chanList[i] == g_chanFreq ? "  <- current" : "");
      hublogf("Switch fleet: chan <0..%u>\n", TALLY_CHAN_COUNT - 1);
    } else if (arg[0] < '0' || arg[0] > '9') {
      hublogf("[CHAN] usage: chan <0..%u>\n", TALLY_CHAN_COUNT - 1);
    } else {
      uint8_t i = (uint8_t)arg.toInt();
      if (i >= TALLY_CHAN_COUNT) {
        hublogf("[CHAN] no such channel (0..%u)\n", TALLY_CHAN_COUNT - 1);
      } else if (i == g_chanIdx) {
        hublogf("[CHAN] already on ch%u\n", i);
      } else if (g_chanSwitch.pending() || g_oldChanFreq) {
        hublogf("[CHAN] a switch is already in progress — wait for it\n");
      } else {
        // The plan rides in the next ~10 heartbeats (decorrelated by 500ms),
        // then the hub retunes and beacons the new channel on the old one.
        autoRfManualOverride("chan");
        requestChannelSwitch(i, "manual");
      }
    }
  } else if (cmd == "noise") {
    noiseSurvey();
  } else if (cmd == "survey") {
    bandSurvey(io);
  } else if (cmd == "pintest") {
    // Wiring bring-up: drive each ESP->E28 control line HIGH alone for 8s so
    // a multimeter on the module pad verifies the PHYSICAL mapping wire by
    // wire — continuity beeps can't tell "the wire lands on the wrong pad"
    // (mirrored module numbering, neighbouring header pin). Pure GPIO, works
    // with the radio dead. pinMode() detaches the SPI pins from the
    // peripheral, so we reboot at the end to restore a clean radio state.
    static const struct {
      const char *name;
      int8_t gpio;
      uint8_t pad;
    } kLines[] = {
        {"NSS ", E28_PIN_NSS, 6},  {"SCK ", E28_PIN_SCK, 5},
        {"MOSI", E28_PIN_MOSI, 4}, {"RXEN", E28_PIN_RXEN, 8},
        {"TXEN", E28_PIN_TXEN, 9},
    };
    hublogf("[PINTEST] 5 lines x 8s: the named line is 3.3V on its E28 pad, "
            "all other lines 0V. Probe the named pad each step.\n");
    for (auto &l : kLines) {
      pinMode(l.gpio, OUTPUT);
      digitalWrite(l.gpio, LOW);
    }
    for (auto &l : kLines) {
      digitalWrite(l.gpio, HIGH);
      hublogf("[PINTEST] %s HIGH -> expect 3.3V on E28 pad %u (from GPIO %d); "
              "BUSY=%d DIO1=%d\n",
              l.name, l.pad, (int)l.gpio, digitalRead(E28_PIN_BUSY),
              digitalRead(E28_PIN_DIO1));
      delay(8000);
      digitalWrite(l.gpio, LOW);
    }
    hublogf("[PINTEST] done — rebooting to restore SPI state\n");
    delay(300);
    ESP.restart();
  } else if (cmd == "log") {
    // Field-log dump to the asking console only (up to ~512KB @115200 ≈ 45s;
    // fleetPump keeps heartbeats/ATEM/telemetry running between chunks)
    TallyLog.dump(io ? *io : Serial, fleetPump);
  } else if (cmd == "logclear") {
    TallyLog.clear();
    hublogf("[LOG] cleared\n");
  } else if (cmd == "cams") {
    // Per-camera return-channel report: walk-test readout in one command
    bool any = false;
    for (uint8_t id = 1; id <= 16; id++) {
      if (g_camLastSeen[id] == 0)
        continue;
      any = true;
      // dl = how loudly the SLAVE hears us (the safety-critical direction),
      // ul = how loudly we hear the slave, missed = its heartbeat gradient.
      hublogf("[CAM %2u] %-8s dl=%4d ul=%4d dBm missed=%-2u last seen %lus ago\n",
              id,
              !g_camReachable[id] ? "SILENT"
              : camComplaining(id) ? "POOR"
                                   : "ok",
              (int)g_camRssi[id], (int)g_camRxRssi[id], g_camMissed[id],
              (unsigned long)((millis() - g_camLastSeen[id]) / 1000));
    }
    if (!any)
      hublogf("[CAM] no telemetry received from any slave yet\n");
  } else if (cmd.startsWith("auto")) {
    String arg = cmd.substring(4);
    arg.trim();
    if (arg == "on") {
      g_autoRf = true;
      hublogf("[AUTO] on — resuming automatic channel selection\n");
    } else if (arg == "off") {
      g_autoRf = false;
      hublogf("[AUTO] off — manual chan only\n");
    } else {
      autoRfReport();
    }
  } else if (cmd == "help") {
    hublogf("Commands: status, cams, ping, power [n], chan [i], auto [on|off], "
            "noise, survey, pintest, log, logclear, reinit, help\n");
  } else if (cmd.length()) {
    hublogf("Unknown command '%s' — try 'help'\n", cmd.c_str());
  }
}

static void pollSerialCommands() {
  // Non-blocking line accumulator instead of readStringUntil(): the Stream
  // timeout is per-CHARACTER (timedRead restarts its clock every byte), so
  // any timeout either stalls the loop on line noise (1s default) or splits
  // hand-typed commands at normal inter-keystroke gaps (50ms). Accumulating
  // available bytes and acting only on a complete newline does neither.
  static String lineBuf[2];
  Stream *ports[2] = {&Serial, &Serial0};
  for (int i = 0; i < 2; i++) {
    while (ports[i]->available()) {
      char c = (char)ports[i]->read();
      if (c == '\n' || c == '\r') {
        lineBuf[i].trim();
        if (lineBuf[i].length())
          handleSerialCommand(lineBuf[i], ports[i]);
        lineBuf[i] = "";
      } else if (lineBuf[i].length() < 100) { // bound against a noise flood
        lineBuf[i] += c;
      }
    }
  }
}

void loop() {
  processTx();
  chanMeasureTick();   // passive channel stats (diagnostics + LBT reference)
  serviceTelemetry();  // receive slave telemetry in idle windows
  sweepReachability(); // log cameras going online/offline
  beaconTick();        // after a switch: recover slaves left on the old channel
  autoRfTick();        // channel switch decisions (downlink signal only)
  TallyLog.tick();     // periodic flash-log flush

  // TX watchdog: a radio that reports connected while every send fails would
  // stop the heartbeat silently — only the slaves would alarm. Force a re-init.
  if (radio.isConnected() && millis() - g_lastTxOkMs > TX_WATCHDOG_MS) {
    hublogf("[E28] no successful TX for %lums — forcing re-init\n",
            (unsigned long)(millis() - g_lastTxOkMs));
    g_lastTxOkMs = millis(); // don't re-trigger every pass
    radioInit();
  }

  // Radio recovery: re-init every 10s while disconnected (begin() bails out
  // in ~150ms when the module is absent, so this stays affordable)
  if (!radio.isConnected()) {
    static uint32_t lastRadioRetry = 0;
    if (millis() - lastRadioRetry > 10000) {
      lastRadioRetry = millis();
      if (radioInit())
        hublogf("[E28] recovered (status=0x%02X)\n", radio.getChipStatus());
      else
        hublogf("[E28] recovery failed: %s\n", radio.initErrorStr());
    }
  }

  // WiFi reconnect (non-blocking)
#ifndef NO_WIFI
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > WIFI_RETRY_MS) {
      lastRetry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
#endif

  // === Tally source (ATEM poll / test stream) — shared with fleetPump()
  tallySourcePoll();

  // === STATE_ALL broadcast: on change + heartbeat (see heartbeatTick())
  heartbeatTick();

  pollSerialCommands();

  // Periodic status heartbeat on serial (10s) — one line tells whether the
  // radio/queue/WiFi/ATEM are alive without touching the device
  static uint32_t lastStatusLog = 0;
  if (millis() - lastStatusLog > 10000) {
    lastStatusLog = millis();
    handleSerialCommand("status");
  }

  // Locator / Ping (Button 0 or serial "ping") — works without ATEM.
  // Non-blocking: 3 pings 100ms apart; after the last one locatorNextMs
  // holds a ~2s cooldown that gates retriggering (see the triggers above).
  if (digitalRead(0) == LOW && locatorPingsLeft == 0 &&
      (int32_t)(millis() - locatorNextMs) >= 0) {
    drawCenteredMsg("LOCATOR", "Ping sent -> Cam 1");
    locatorPingsLeft = 3;
    locatorNextMs = millis();
  }

  if (locatorPingsLeft > 0 && (int32_t)(millis() - locatorNextMs) >= 0) {
    TallyPacket pkt = TallyProtocol::createPingPacket(1);
    enqueueOneShot(pkt);
    locatorPingsLeft--;
    // 100ms to the next ping; after the 3rd, a ~2s cooldown gates retrigger
    locatorNextMs = millis() + (locatorPingsLeft ? 100 : 2000);
  }

  // ==== OLED: tally grid when ATEM is live, debug screen otherwise ====
  // The grid (production view) redraws only when the masks/connection change,
  // plus a slow refresh — a full 128x64 I2C blit is ~23ms and used to run every
  // 500ms unconditionally, which also delayed checkTxDone() mid-transmit.
  static uint32_t lastDraw = 0;
  static uint16_t drawnProg = 0xFFFF, drawnPrev = 0xFFFF;
  static bool drawnConnected = false;

  // ⚡ Bolt: Prevent slow, blocking I2C screen updates during high-priority
  // non-blocking UI sequences (like LOCATOR). Also skip the blit while a TX is
  // in flight: a ~23ms full-frame I2C blit would otherwise defer checkTxDone()
  // (PA-off) and the next packet by that much. The redraw runs next pass.
  // And skip it while the FIRST copy of a cut is still waiting for processTx()
  // below: this block sits between heartbeatTick() (which notices the cut and
  // marks the grid dirty) and the transmit, so the very redraw the cut caused
  // used to add its 23 ms to the one latency the whole design exists to
  // minimise. The grid is painted one pass later, after the copy is on air.
  bool uiActive = (locatorPingsLeft > 0) || radio.txActive() ||
                  g_burst.latencyCritical();
  bool connected = (atemPhase == ATEM_RUNNING);

  if (!uiActive) {
    if (connected) {
      bool dirty = (g_progMask != drawnProg) || (g_prevMask != drawnPrev) ||
                   !drawnConnected;
      if (dirty || millis() - lastDraw > 2000) {
        lastDraw = millis();
        drawnProg = g_progMask;
        drawnPrev = g_prevMask;
        drawnConnected = true;
        display.clearDisplay();
        drawStatusBar(WiFi.localIP().toString(), WiFi.status() == WL_CONNECTED,
                      radio.isConnected(), true);
        drawTallyGrid(g_progMask, g_prevMask);
        display.display();
      }
    } else if (millis() - lastDraw > 500) {
      // Debug screen is a transient bring-up view; its blinker/counters justify
      // the faster cadence.
      lastDraw = millis();
      drawnConnected = false;
      drawLoRaDebug();
    }
  }

  // Second queue pass: with one pass per ~10ms loop, TxDone detection (and
  // PA-off) lagged the ~96ms airtime by up to a full tick; this halves it
  processTx();

  delay(10);
}
