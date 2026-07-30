#ifndef AUTORF_H
#define AUTORF_H

// ===== AutoRF v4: automatic channel selection =====
// Zero-operator channel management. Deliberately narrow in scope: channel
// choice is a SECONDARY defence. The primary one is unconditional time
// diversity in the transmit scheduler (see main.cpp) — it needs no measurement,
// has no failure mode, and works against the burst interference that dominates
// a concert. Channel hopping cannot help at all against the FHSS interferers
// (2.4 GHz wireless DMX, audience BLE) that sweep the whole band.
//
// What the v3 implementation got wrong, and what changed here:
//
//  1. The CURRENT channel's noise was never re-measured — the scout skipped it,
//     so it stayed frozen at its boot-survey value forever. Worse, the boot
//     survey ran before WiFi came up, so the current channel was measured with
//     the hub's own WiFi radio OFF and an empty venue, while alternatives were
//     re-measured with WiFi on and an audience present. That bias systematically
//     suppressed switching exactly when it was needed.
//     -> The current channel is now measured PASSIVELY AND CONTINUOUSLY. The hub
//        is in RX ~80% of the time anyway; sampling costs one SPI read.
//
//  2. Noise was an arithmetic mean of dBm values. That is a geometric mean of
//     POWER, which barely sees peaks: 3-of-16 samples at -50 dBm plus 13 at
//     -105 averaged to -94.7 dBm when the true mean power was -57 dBm — a 37 dB
//     error. It could not tell a harmless raised floor (LoRa decodes below
//     noise) from WiFi hammering 19% of the time (which erases frames).
//     -> floor = a percentile-style minimum tracker; busyFrac = the share of
//        samples well above that floor. Two numbers, physically distinct.
//
//  3. The scout blocked the loop for ~242 ms every 60 s, spiking tally latency.
//     -> Non-blocking state machine, ONE channel per visit, and only inside the
//        idle window between heartbeats.
//
//  4. Uplink ADR: deleted. It saved ~1% duty cycle and cost a feedback loop
//     whose power reductions made cameras look unreachable — which the channel
//     decision then read as "bad channel". Slaves transmit at a fixed power.
//
//  5. The switch trigger needed >=3 cameras to fire, and otherwise fell back to
//     the hub's own RX error count — which measures the UPLINK and is polluted
//     by foreign energy. -> Trigger on the downlink signal (cameras reporting
//     missed heartbeats) with an absolute count, so it works with one camera.
//
// Tally correctness outranks RF elegance: every trigger is sustained,
// hysteresised, dwell-limited and rate-limited, and a failing decision degrades
// to "stay + log", never to flapping. Everything lives in RAM (zero flash
// wear). A manual `chan` command turns AutoRF off until `auto on`.

// ---- Tunables: starting values. Calibrate from field logs — see
// ---- ARCHITECTURE_RF_V4.md §14 for what to measure.
#define AUTORF_EVAL_MS 10000UL            // decision cadence
#define AUTORF_SCOUT_EVERY_MS 20000UL     // one alternative channel per visit
#define AUTORF_DEGRADE_SUSTAIN_MS 30000UL // trigger must hold this long
#define AUTORF_BUSY_WEIGHT 60.0f          // dB-equivalent cost of 100% busy
#define AUTORF_BUSY_ABOVE_FLOOR 10        // "busy" = this many dB over floor
#define AUTORF_HYSTERESIS_DB 6            // alternative must beat current by
#define AUTORF_POOR_COUNT_TRIG 2          // cameras missing heartbeats
#define AUTORF_POOR_FRAC_TRIG 0.30f       // ...or this share of a bigger fleet
#define AUTORF_ONAIR_OVERRIDE_FRAC 0.60f  // switch even on-air past this
// First switch allowed one minute after boot, not ten: a venue's spectrum
// collapses when the audience arrives, which is often inside the first window.
#define AUTORF_FIRST_SWITCH_MS 60000UL
#define AUTORF_MIN_DWELL_MS (10UL * 60 * 1000)
#define AUTORF_MAX_SWITCHES_HOUR 4
#define AUTORF_BLOCK_LOG_MS (5UL * 60 * 1000)

static bool g_autoRf = true; // manual `chan` flips this off

// Per-channel quality. floor/busyFrac are the only inputs to the score.
struct ChanStat {
  int8_t floorDbm;  // tracked minimum (the real noise floor)
  float busyFrac;   // share of samples > floor + AUTORF_BUSY_ABOVE_FLOOR
  bool valid;
};
static ChanStat g_chan[TALLY_CHAN_COUNT];

// Accumulators for the window currently being measured.
static int16_t g_accMin = 127;
static uint16_t g_accN = 0;
static uint16_t g_accBusy = 0;

static uint32_t g_lastEval = 0;
static uint32_t g_degradeSince = 0;
static uint32_t g_lastAutoSwitch = 0;
static uint32_t g_switchHourStart = 0;
static uint8_t g_switchesThisHour = 0;
static uint32_t g_lastBlockLog = 0;
static bool g_autoRfInited = false;

static void autoRfInit() {
  if (g_autoRfInited)
    return;
  g_autoRfInited = true;
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    g_chan[i].floorDbm = -100; // unknown until measured
    g_chan[i].busyFrac = 0.0f;
    g_chan[i].valid = false;
  }
  // Dwell clock is pre-aged so the first switch is allowed after
  // AUTORF_FIRST_SWITCH_MS rather than a full dwell.
  g_lastAutoSwitch = millis() - (AUTORF_MIN_DWELL_MS - AUTORF_FIRST_SWITCH_MS);
  g_switchHourStart = millis();
}

// Fold one RSSI sample into the current window. Called for whichever channel
// the radio is tuned to right now.
static void autoRfAddSample(ChanStat &cs, int8_t rssi) {
  g_accN++;
  if (rssi < g_accMin)
    g_accMin = rssi;
  if (cs.valid && rssi > cs.floorDbm + AUTORF_BUSY_ABOVE_FLOOR)
    g_accBusy++;
}

// Close the window into a channel's statistics.
static void autoRfCommit(ChanStat &cs) {
  if (g_accN == 0)
    return;
  int8_t f = (int8_t)g_accMin;
  // Slow-track the floor: jump down instantly (a quieter reading is the truth),
  // creep up (so one quiet gap in heavy traffic can't hide the interference).
  if (!cs.valid || f < cs.floorDbm) {
    cs.floorDbm = f; // a quieter reading is the truth: take it at once
  } else {
    // Creep up by at least 1 dB: a plain (diff >> 2) is 0 for differences under
    // 4 dB, which pinned the floor at any transient dip forever and then
    // inflated busyFrac against a floor that no longer existed.
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

// ---- Non-blocking scout state (declared here so the passive sampler below
// ---- can tell when the radio is parked on another channel) ----
enum ScoutPhase : uint8_t { SC_IDLE, SC_SETTLE, SC_SAMPLE };
static ScoutPhase g_scPhase = SC_IDLE;

// ---- Passive measurement of the CURRENT channel (closes findings 1 and 2) ----
// Free: the hub sits in RX between frames, and this is one SPI read. No
// blocking, no deafness, no schedule.
static void autoRfPassiveSample() {
  if (!radio.isConnected() || radio.txActive() || g_txOffChannel)
    return;
  // While a scout visit owns the radio these samples belong to ANOTHER channel
  // — folding them into the current channel's window would corrupt exactly the
  // measurement this rewrite exists to get right.
  if (g_scPhase != SC_IDLE)
    return;
  if (g_chanIdx >= TALLY_CHAN_COUNT)
    return;
  int8_t r = radio.getRssiInst();
  if (r == 0) // driver's documented "no reading"
    return;
  autoRfAddSample(g_chan[g_chanIdx], r);
}

// ---- Non-blocking scout of ONE alternative channel (closes finding 3) ----
// A visit is ~30 ms of the ~450 ms idle window after a heartbeat, spread across
// loop passes, and aborts the instant anything needs to go out.
static uint8_t g_scIdx = 0;      // channel being visited
static uint8_t g_scNext = 0;     // round-robin cursor
static uint8_t g_scTaken = 0;    // samples taken this visit
static uint32_t g_scAtMs = 0;
static int16_t g_scMin = 127;
static uint16_t g_scBusy = 0;
#define SCOUT_SAMPLES 8
#define SCOUT_SETTLE_MS 6
#define SCOUT_STEP_MS 3

static void autoRfScoutAbort() {
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  g_scPhase = SC_IDLE;
  g_scoutActive = false;
}

static void autoRfScoutTick() {
  uint32_t now = millis();

  if (g_scPhase != SC_IDLE) {
    // processTx() reclaimed the radio and already restored the frequency —
    // drop the partial visit without touching the radio again.
    if (!g_scoutActive) {
      g_scPhase = SC_IDLE;
      return;
    }
    // Anything to transmit, or the radio died -> get back on channel at once.
    if (!radio.isConnected() || !txIdle()) {
      autoRfScoutAbort();
      return;
    }
    if (g_scPhase == SC_SETTLE) {
      if (now - g_scAtMs < SCOUT_SETTLE_MS)
        return;
      g_scPhase = SC_SAMPLE;
      g_scAtMs = now;
      return;
    }
    // SC_SAMPLE
    if (now - g_scAtMs < SCOUT_STEP_MS)
      return;
    g_scAtMs = now;
    int8_t r = radio.getRssiInst();
    if (r != 0) {
      if (r < g_scMin)
        g_scMin = r;
      ChanStat &cs = g_chan[g_scIdx];
      if (cs.valid && r > cs.floorDbm + AUTORF_BUSY_ABOVE_FLOOR)
        g_scBusy++;
      g_scTaken++;
    }
    if (g_scTaken < SCOUT_SAMPLES)
      return;
    // Visit complete: fold it in through the shared accumulator path.
    g_accMin = g_scMin;
    g_accN = g_scTaken;
    g_accBusy = g_scBusy;
    autoRfCommit(g_chan[g_scIdx]);
    autoRfScoutAbort();
    return;
  }

  // Start a visit? Only in the calm part of a heartbeat cycle.
  static uint32_t lastVisit = 0;
  if (now - lastVisit < AUTORF_SCOUT_EVERY_MS)
    return;
  if (!radio.isConnected() || !txIdle() || g_switchPending || g_oldChanFreq)
    return;
  if (TALLY_CHAN_COUNT < 2)
    return;
  // Pick the next channel that isn't the current one.
  uint8_t idx = g_scNext;
  for (uint8_t n = 0; n < TALLY_CHAN_COUNT; n++) {
    if (idx != g_chanIdx)
      break;
    idx = (uint8_t)((idx + 1) % TALLY_CHAN_COUNT);
  }
  if (idx == g_chanIdx)
    return;
  g_scNext = (uint8_t)((idx + 1) % TALLY_CHAN_COUNT);
  lastVisit = now;

  // Save the current window before retuning, so samples aren't mixed between
  // channels.
  autoRfCommit(g_chan[g_chanIdx]);

  g_scIdx = idx;
  g_scTaken = 0;
  g_scMin = 127;
  g_scBusy = 0;
  g_scPhase = SC_SETTLE;
  g_scAtMs = now;
  g_scoutActive = true; // processTx() may revoke this at any moment
  radio.setFrequency(g_chanList[idx]);
  radio.startReceive();
}

// Channel cost, lower is better. busyFrac is the concert-relevant term: a
// channel with a -100 floor that is busy 40% of the time is far worse than one
// with a -85 floor that is quiet.
static float autoRfCost(uint8_t ch) {
  const ChanStat &cs = g_chan[ch];
  if (!cs.valid)
    return 1e9f; // never move TO an unmeasured channel
  return (float)cs.floorDbm + AUTORF_BUSY_WEIGHT * cs.busyFrac;
}

static void autoRfTrySwitch(uint8_t idx, const char *reason) {
  uint32_t now = millis();
  const char *block = nullptr;
  if (now - g_lastAutoSwitch < AUTORF_MIN_DWELL_MS)
    block = "min-dwell";
  else if (g_switchesThisHour >= AUTORF_MAX_SWITCHES_HOUR)
    block = "rate-limit";
  else if (g_switchPending || g_oldChanFreq)
    block = "switch-in-progress";
  if (block) {
    if (now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
      g_lastBlockLog = now;
      hublogf("[AUTO] want ch%u (%s) but blocked: %s — staying\n", idx, reason,
              block);
    }
    return;
  }
  g_lastAutoSwitch = now;
  g_switchesThisHour++;
  g_degradeSince = 0;
  requestChannelSwitch(idx, reason);
}

static void autoRfTick() {
  autoRfInit();
  if (!g_autoRf)
    return;

  autoRfPassiveSample(); // every pass, ~free
  autoRfScoutTick();

  uint32_t now = millis();
  if (now - g_switchHourStart > 3600000UL) {
    g_switchHourStart = now;
    g_switchesThisHour = 0;
  }
  if (now - g_lastEval < AUTORF_EVAL_MS)
    return;
  g_lastEval = now;

  // Close the current channel's window so the decision uses fresh numbers.
  if (g_scPhase == SC_IDLE && g_chanIdx < TALLY_CHAN_COUNT)
    autoRfCommit(g_chan[g_chanIdx]);

  // ---- Downlink health: the direction that matters ----
  uint8_t seen = 0, poor = 0;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    seen++;
    if (camPoor(id))
      poor++;
  }
  float poorFrac = seen ? (float)poor / seen : 0.0f;
  // Absolute count OR fraction, so a 1-2 camera rig is covered too (v3 needed
  // three cameras before this signal existed at all).
  bool degraded = (poor >= AUTORF_POOR_COUNT_TRIG) ||
                  (seen >= 3 && poorFrac >= AUTORF_POOR_FRAC_TRIG) ||
                  (seen == 1 && poor == 1);
  if (degraded) {
    if (!g_degradeSince)
      g_degradeSince = now;
  } else {
    g_degradeSince = 0;
  }

  if (g_switchPending || g_oldChanFreq)
    return; // a switch is already in flight

  // ---- Pick the best alternative ----
  uint8_t best = 0xFF;
  float bestCost = 1e9f;
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    if (i == g_chanIdx)
      continue;
    float c = autoRfCost(i);
    if (c < bestCost) {
      bestCost = c;
      best = i;
    }
  }
  if (best == 0xFF)
    return;
  float curCost = autoRfCost(g_chanIdx);

  bool sustained =
      g_degradeSince && (now - g_degradeSince >= AUTORF_DEGRADE_SUSTAIN_MS);
  if (!sustained)
    return;
  if (bestCost + AUTORF_HYSTERESIS_DB >= curCost)
    return; // nowhere better to go — staying beats a pointless outage

  // A switch costs a brief outage. Don't spend it on a live camera unless the
  // degradation is already severe enough that staying is worse.
  if (g_progMask != 0 && poorFrac < AUTORF_ONAIR_OVERRIDE_FRAC) {
    if (now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
      g_lastBlockLog = now;
      hublogf("[AUTO] want ch%u but a camera is ON AIR and only %d%% poor — "
              "staying\n",
              best, (int)(poorFrac * 100));
    }
    return;
  }
  autoRfTrySwitch(best, "downlink degraded");
}

// Boot survey: pick the cleanest channel to START on. Blocking (~40ms/channel)
// but this runs once, before the first heartbeat. Called AFTER WiFi is up so
// the baseline includes the hub's own WiFi radio — measuring with it off was
// half of finding 2.
static void autoRfBootSurvey() {
  autoRfInit();
  if (!radio.isConnected())
    return;
  // Two passes per channel: busy samples are counted relative to the channel's
  // floor, which the first pass is still establishing. Without the second pass
  // every channel would report busy=0 and the survey would compare floors only.
  for (uint8_t pass = 0; pass < 2; pass++) {
    for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
      radio.setFrequency(g_chanList[i]);
      radio.startReceive();
      delay(SCOUT_SETTLE_MS);
      g_accMin = 127;
      g_accN = 0;
      g_accBusy = 0;
      for (int k = 0; k < 12; k++) {
        int8_t r = radio.getRssiInst();
        if (r != 0)
          autoRfAddSample(g_chan[i], r);
        delay(SCOUT_STEP_MS);
      }
      autoRfCommit(g_chan[i]);
    }
  }
  uint8_t best = 0;
  for (uint8_t i = 1; i < TALLY_CHAN_COUNT; i++)
    if (autoRfCost(i) < autoRfCost(best))
      best = i;
  g_chanIdx = best;
  g_chanFreq = g_chanList[best];
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  hublogf("[AUTO] boot survey: ch%u cleanest (floor=%d dBm busy=%d%%)\n", best,
          (int)g_chan[best].floorDbm, (int)(g_chan[best].busyFrac * 100));
}

static void autoRfManualOverride(const char *what) {
  if (!g_autoRf)
    return;
  g_autoRf = false;
  hublogf("[AUTO] manual %s — AutoRF OFF until 'auto on'\n", what);
}

static void autoRfReport() {
  autoRfInit();
  hublogf("[AUTO] mode=%s cur=ch%u hubpwr=%d dBm (fixed) switches=%u/%u per hour\n",
          g_autoRf ? "ON" : "OFF (manual)", g_chanIdx, (int)g_txPower,
          g_switchesThisHour, AUTORF_MAX_SWITCHES_HOUR);
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    if (!g_chan[i].valid) {
      hublogf("[AUTO] ch%u: not measured yet%s\n", i,
              i == g_chanIdx ? " <- current" : "");
      continue;
    }
    hublogf("[AUTO] ch%u: floor=%d dBm busy=%d%% cost=%d%s\n", i,
            (int)g_chan[i].floorDbm, (int)(g_chan[i].busyFrac * 100),
            (int)autoRfCost(i), i == g_chanIdx ? " <- current" : "");
  }
  bool any = false;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    any = true;
    hublogf("[AUTO] cam %u: dl=%d ul=%d dBm missed=%u%s\n", id,
            (int)g_camRssi[id], (int)g_camRxRssi[id], g_camMissed[id],
            camPoor(id) ? " POOR" : "");
  }
  if (!any)
    hublogf("[AUTO] no cameras seen yet\n");
}

#endif // AUTORF_H
