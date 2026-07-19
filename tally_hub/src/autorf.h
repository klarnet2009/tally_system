#ifndef AUTORF_H
#define AUTORF_H

// ===== AutoRF: automatic channel selection + adaptive power =====
// Zero-operator RF management, built from three field-proven references:
//  - Semtech/ChirpStack ADR (LoRaWAN): link-margin control loop with an
//    "installation margin" and slow stepwise convergence.
//  - ExpressLRS Dynamic Power (same SX1280 silicon): LQ (link quality) is
//    the reliability signal, RSSI the margin signal — lower power ONLY when
//    LQ is good; raise fast on link loss / sudden drops.
//  - Bluetooth AFH: channels are classified by OUR OWN error rate, not by
//    foreign energy; unused channels are re-scouted periodically (a bad
//    channel is never blacklisted forever).
// Everything lives in RAM (zero flash wear): boot defaults are the config
// constants, the loops reconverge within minutes. Any manual `chan`/`power`
// command switches AutoRF OFF until `auto on` — human intent always wins.
//
// Tally correctness outranks RF elegance: every trigger is sustained,
// hysteresised, dwell-limited and rate-limited; a failing switch decision
// degrades to "stay + alarm log", never to flapping.

// ---- Tunables: starting values, calibrate from field logs (DEBUGGING.md) --
#define AUTORF_EVAL_MS 10000UL          // decision eval period
#define AUTORF_SCOUT_MS 60000UL         // background noise scout period
#define AUTORF_DEGRADE_SUSTAIN_MS 30000UL // trigger must hold this long
#define AUTORF_RXERR_TRIG 6.0f          // rxerr/min EMA = degraded channel
#define AUTORF_POOR_FRAC_TRIG 0.30f     // >=30% of seen cams offline/poor
#define AUTORF_HYSTERESIS_DB 6          // alt channel must beat current by
#define AUTORF_OPPORTUNISTIC_DB 10      // idle-time improvement threshold
#define AUTORF_MIN_DWELL_MS (10UL * 60 * 1000)     // min time between switches
#define AUTORF_OPPORT_DWELL_MS (30UL * 60 * 1000)  // stricter for opportunistic
#define AUTORF_MAX_SWITCHES_HOUR 4
#define AUTORF_BLOCK_LOG_MS (5UL * 60 * 1000)      // "switch blocked" rate
#define AUTORF_PWR_EVAL_MS 30000UL      // hub power eval period
#define AUTORF_CAM_PWR_EVAL_MS 60000UL  // per-cam power eval period
#define AUTORF_TARGET_DBM (-102)        // uplink telemetry target at the hub
#define AUTORF_WEAK_HIGH (-99)          // downlink weakest above -> lower
#define AUTORF_WEAK_LOW (-105)          // downlink weakest below -> raise
                                        // (SF9 floor -114, margin 9-12 dB)
#define AUTORF_HUB_PWR_MAX 4            // auto cap: 3V3 rail brownout limit
#define AUTORF_PWR_STEP 3
#define AUTORF_PWR_HOLD_MS (10UL * 60 * 1000) // no down-steps after OFFLINE
#define AUTORF_RESTORE_WINDOW_MS (2UL * 60 * 1000) // power-down -> OFFLINE
#define AUTORF_CRASH_DROP_DB 10         // sudden per-cam downlink drop

static bool g_autoRf = true; // manual `chan`/`power` flips this off

static bool g_autoRfInited = false;
static int8_t g_noise[TALLY_CHAN_COUNT]; // avg noise floor per channel (dBm)
static uint32_t g_lastScout = 0;
static uint32_t g_lastEval = 0;
static float g_rxErrEma = 0; // channel errors/min, EMA over eval periods
static uint32_t g_rxErrPrev = 0;
static uint32_t g_degradeSince = 0; // 0 = not degraded
static uint32_t g_lastAutoSwitch = 0;
static uint32_t g_switchHourStart = 0;
static uint8_t g_switchesThisHour = 0;
static uint32_t g_lastBlockLog = 0;
static uint32_t g_pwrHoldUntil = 0;    // down-steps frozen until then
static int8_t g_camPower[17];          // hub-assigned telemetry power per cam
static int8_t g_camPowerPrev[17];      // for the blind-restore path
static int8_t g_camPowerFloor[17];     // never auto-assign below this
static uint32_t g_camPwrEvalAt[17];
static uint32_t g_camPwrCmdAt[17];     // last power command to this cam
static int8_t g_camDlPrev[17];         // downlink rssi last eval (crash detect)

static void autoRfInit() {
  if (g_autoRfInited)
    return;
  g_autoRfInited = true;
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
    g_noise[i] = -100; // unknown = assume middling until the first scout
  for (uint8_t id = 0; id <= 16; id++) {
    g_camPower[id] = TALLY_TX_POWER;
    g_camPowerPrev[id] = TALLY_TX_POWER;
    g_camPowerFloor[id] = -18;
  }
  g_lastAutoSwitch = millis(); // dwell clock starts at boot
  g_switchHourStart = millis();
}

// Sample the ambient noise floor on one channel (~120ms: retune, settle,
// 16 RSSI-inst reads). Same recipe as the manual `noise` survey.
static int8_t autoRfSampleNoise(uint32_t freqHz) {
  radio.setFrequency(freqHz);
  radio.startReceive();
  delay(25); // PLL settle + RSSI integration
  int32_t sum = 0;
  for (int k = 0; k < 16; k++) {
    sum += radio.getRssiInst();
    delay(6);
  }
  return (int8_t)(sum / 16);
}

static uint8_t autoRfCurIdx() { // 0xFF when g_chanFreq matches no list entry
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
    if (g_chanList[i] == g_chanFreq)
      return i;
  return 0xFF;
}

// Boot-time: pick the cleanest channel to START on (~400ms, before the first
// heartbeat). Slaves boot-scan from home and converge as usual.
static void autoRfBootSurvey() {
  autoRfInit();
  if (!radio.isConnected())
    return;
  uint8_t best = 0;
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    g_noise[i] = autoRfSampleNoise(g_chanList[i]);
    if (g_noise[i] < g_noise[best])
      best = i;
  }
  g_chanFreq = g_chanList[best];
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  hublogf("[AUTO] boot survey: ch%u cleanest (%d dBm) — starting there\n", best,
          (int)g_noise[best]);
}

// Background scout (~250ms deaf = at most one delayed heartbeat, slaves
// tolerate six): re-check the channels we're NOT on. Only when the link is
// idle — never mid-burst, never mid-switch.
static void autoRfScout() {
  if (!radio.isConnected() || radio.txActive())
    return;
  if (g_loraQueueHead != g_loraQueueTail || g_pendingChanFreq)
    return;
  uint8_t cur = autoRfCurIdx();
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
    if (i != cur)
      g_noise[i] = autoRfSampleNoise(g_chanList[i]);
  radio.setFrequency(g_chanFreq);
  radio.startReceive();
  hublogf("[AUTO] scout: noise ch0=%d ch1=%d ch2=%d dBm (current=%d)\n",
          (int)g_noise[0], (int)g_noise[1], (int)g_noise[2], (int)cur);
}

// Channel score (lower = better): noise floor +, for the current channel,
// the penalty from OUR OWN link errors (AFH principle: our packet errors
// outrank foreign energy — interference that doesn't hurt us is not a
// reason to move).
static float autoRfScore(uint8_t ch, uint8_t cur, float linkPenalty) {
  float s = g_noise[ch];
  if (ch == cur)
    s += linkPenalty;
  return s;
}

// Enqueue a coordinated switch via the existing announce/drain mechanics
// (6 announcements on the old channel, hub retunes after the queue drains).
// Honors dwell + hourly rate limit; logs the block reason when vetoed.
static void autoRfTrySwitch(uint8_t idx, const char *reason) {
  uint32_t now = millis();
  const char *block = nullptr;
  if (now - g_lastAutoSwitch < AUTORF_MIN_DWELL_MS)
    block = "min-dwell";
  else if (g_switchesThisHour >= AUTORF_MAX_SWITCHES_HOUR)
    block = "rate-limit";
  else if (g_loraQueueHead != g_loraQueueTail || radio.txActive())
    block = "tx-busy";
  if (block) {
    if (now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
      g_lastBlockLog = now;
      hublogf("[AUTO] want ch%u (%s) but blocked: %s — staying\n", idx, reason,
              block);
    }
    return;
  }
  TallyPacket pkt = TallyProtocol::createSetChannelPacket(idx, g_chanList[idx]);
  for (int k = 0; k < 6; k++)
    enqueueLora(pkt);
  g_pendingChanFreq = g_chanList[idx];
  g_lastAutoSwitch = now;
  g_switchesThisHour++;
  g_degradeSince = 0;
  hublogf("[AUTO] switching to ch%u (%lu.%lu MHz): %s\n", idx,
          g_chanList[idx] / 1000000UL,
          (g_chanList[idx] % 1000000UL) / 100000UL, reason);
}

static void autoRfSetHubPower(int8_t p) {
  g_txPower = p;
  radio.setTxPower(p);
}

// ExpressLRS rule: a lost camera raises the hub power one step (bounded,
// once per hold window so a flapping cam can't ratchet), and freezes
// down-steps for a while. A cam that went OFFLINE shortly after we LOWERED
// its power gets a blind restore (the downlink is strong, it will hear) and
// a floor so we never hide it again.
static void autoRfOnCamChange(uint8_t id, bool online) {
  if (!g_autoRf || id < 1 || id > 16)
    return;
  uint32_t now = millis();
  if (online) {
    // Re-announce the assignment: covers slaves that rebooted to defaults
    TallyPacket pkt =
        TallyProtocol::createSetPowerPacket(id, g_camPower[id]);
    for (int k = 0; k < 3; k++)
      enqueueLora(pkt);
    return;
  }
  if (now - g_camPwrCmdAt[id] < AUTORF_RESTORE_WINDOW_MS &&
      g_camPower[id] < g_camPowerPrev[id]) {
    g_camPower[id] = g_camPowerPrev[id];
    g_camPowerFloor[id] = g_camPowerPrev[id];
    TallyPacket pkt =
        TallyProtocol::createSetPowerPacket(id, g_camPower[id]);
    for (int k = 0; k < 6; k++)
      enqueueLora(pkt);
    hublogf("[AUTO] cam %u vanished after power-down — restored %d dBm, floor set\n",
            id, (int)g_camPower[id]);
  }
  if (now > g_pwrHoldUntil && g_txPower < AUTORF_HUB_PWR_MAX) {
    autoRfSetHubPower(g_txPower + AUTORF_PWR_STEP);
    hublogf("[AUTO] cam %u OFFLINE — hub power up to %d dBm, hold 10min\n", id,
            (int)g_txPower);
  }
  g_pwrHoldUntil = now + AUTORF_PWR_HOLD_MS;
}

// The two ADR loops + channel state machine. Call every loop pass; all
// heavy work is timer-gated. No-op while g_autoRf is false (manual mode).
static void autoRfTick() {
  autoRfInit();
  if (!g_autoRf)
    return;
  uint32_t now = millis();

  // Hourly switch budget
  if (now - g_switchHourStart > 3600000UL) {
    g_switchHourStart = now;
    g_switchesThisHour = 0;
  }

  // Background scout (also keeps "unused" channels honestly re-classified)
  if (now - g_lastScout > AUTORF_SCOUT_MS) {
    g_lastScout = now;
    autoRfScout();
  }

  if (now - g_lastEval < AUTORF_EVAL_MS)
    return;
  float dtMin = (now - g_lastEval) / 60000.0f;
  g_lastEval = now;

  // --- Channel error rate (EMA of rxerr/min) ---
  uint32_t rxNow = radio.getRxErrors();
  float inst = (rxNow - g_rxErrPrev) / dtMin;
  g_rxErrPrev = rxNow;
  g_rxErrEma = 0.7f * g_rxErrEma + 0.3f * inst;

  // --- Fleet-wide poor fraction (seen cams that are offline or LINK_POOR) ---
  uint8_t seen = 0, poor = 0;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    seen++;
    if (!g_camReachable[id] || g_camLinkPoor[id])
      poor++;
  }
  float poorFrac = seen ? (float)poor / seen : 0.0f;
  uint8_t cur = autoRfCurIdx();
  float linkPenalty = 2.0f * g_rxErrEma + 20.0f * poorFrac;

  bool degraded = g_rxErrEma > AUTORF_RXERR_TRIG ||
                  (seen >= 3 && poorFrac >= AUTORF_POOR_FRAC_TRIG);
  if (degraded) {
    if (!g_degradeSince)
      g_degradeSince = now;
  } else {
    g_degradeSince = 0;
  }

  // --- Channel decision ---
  uint8_t best = 0xFF;
  float bestScore = 1e9, curScore = (cur != 0xFF) ? autoRfScore(cur, cur, linkPenalty) : 1e9;
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    if (i == cur)
      continue;
    float s = autoRfScore(i, cur, linkPenalty);
    if (s < bestScore) {
      bestScore = s;
      best = i;
    }
  }
  if (best != 0xFF) {
    bool sustained = g_degradeSince && now - g_degradeSince >= AUTORF_DEGRADE_SUSTAIN_MS;
    if (sustained && bestScore + AUTORF_HYSTERESIS_DB < curScore) {
      autoRfTrySwitch(best, "channel degraded");
    } else if (!degraded && bestScore + AUTORF_OPPORTUNISTIC_DB < curScore &&
               atemPhase != ATEM_RUNNING &&
               now - g_lastAutoSwitch > AUTORF_OPPORT_DWELL_MS) {
      autoRfTrySwitch(best, "opportunistic (idle, much cleaner)");
    }
  }

  // --- Downlink power (hub): keep the WEAKEST online cam inside
  // [AUTORF_WEAK_LOW, AUTORF_WEAK_HIGH]. LQ gate: ExpressLRS lowers power
  // only when the link is healthy — so do we; up-steps are always allowed.
  int8_t weakest = 0;
  uint8_t online = 0;
  bool lqGood = g_rxErrEma < AUTORF_RXERR_TRIG;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camReachable[id])
      continue;
    if (g_camLinkPoor[id])
      lqGood = false;
    if (!online || g_camRssi[id] < weakest)
      weakest = g_camRssi[id];
    online++;
  }
  if (online && now > g_pwrHoldUntil) {
    if (weakest < AUTORF_WEAK_LOW && g_txPower < AUTORF_HUB_PWR_MAX) {
      autoRfSetHubPower(g_txPower + AUTORF_PWR_STEP);
      hublogf("[AUTO] weakest cam %d dBm — hub power up to %d dBm\n",
              (int)weakest, (int)g_txPower);
    } else if (lqGood && weakest > AUTORF_WEAK_HIGH && g_txPower > -18) {
      autoRfSetHubPower(g_txPower - AUTORF_PWR_STEP);
      hublogf("[AUTO] weakest cam %d dBm, link clean — hub power down to %d dBm\n",
              (int)weakest, (int)g_txPower);
    }
  }

  // --- Sudden per-cam downlink crash (moved behind a wall): react within
  // one eval instead of waiting for the OFFLINE timeout.
  for (uint8_t id = 1; id <= 16; id++) {
    if (g_camReachable[id] && g_camDlPrev[id] != 0 &&
        g_camDlPrev[id] - g_camRssi[id] >= AUTORF_CRASH_DROP_DB &&
        g_txPower < AUTORF_HUB_PWR_MAX) {
      autoRfSetHubPower(g_txPower + AUTORF_PWR_STEP);
      hublogf("[AUTO] cam %u downlink crashed %d dB — hub power up to %d dBm\n",
              id, (int)(g_camDlPrev[id] - g_camRssi[id]), (int)g_txPower);
    }
    if (g_camLastSeen[id])
      g_camDlPrev[id] = g_camRssi[id];
  }

  // --- Uplink power (per-cam telemetry): converge each cam's RSSI at the
  // hub onto AUTORF_TARGET_DBM, 3dB steps, bounded by its floor.
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camReachable[id] || now - g_camLastSeen[id] > 5000)
      continue; // fresh telemetry only
    if (now - g_camPwrEvalAt[id] < AUTORF_CAM_PWR_EVAL_MS)
      continue;
    g_camPwrEvalAt[id] = now;
    int err = AUTORF_TARGET_DBM - g_camRxRssi[id];
    if (err > -3 && err < 3)
      continue; // inside the deadband
    int np = g_camPower[id] + (err > 0 ? AUTORF_PWR_STEP : -AUTORF_PWR_STEP);
    if (np < g_camPowerFloor[id])
      np = g_camPowerFloor[id];
    if (np < -18)
      np = -18;
    if (np > 12)
      np = 12;
    if (np == g_camPower[id])
      continue;
    g_camPowerPrev[id] = g_camPower[id];
    g_camPower[id] = (int8_t)np;
    g_camPwrCmdAt[id] = now;
    TallyPacket pkt = TallyProtocol::createSetPowerPacket(id, (int8_t)np);
    for (int k = 0; k < 3; k++)
      enqueueLora(pkt);
    hublogf("[AUTO] cam %u uplink %d dBm -> set telemetry power %d dBm\n", id,
            (int)g_camRxRssi[id], np);
  }
}

// Human intent always wins: a manual chan/power command turns AutoRF off
// until `auto on` (logged, never silent).
static void autoRfManualOverride(const char *what) {
  if (!g_autoRf)
    return;
  g_autoRf = false;
  hublogf("[AUTO] manual %s — AutoRF OFF until 'auto on'\n", what);
}

// `auto` serial report: mode, scores, per-cam assignments.
static void autoRfReport() {
  autoRfInit();
  uint8_t cur = autoRfCurIdx();
  hublogf("[AUTO] mode=%s cur=ch%u rxerr=%.1f/min switches=%u/%u per hour\n",
          g_autoRf ? "ON" : "OFF (manual)", cur, (double)g_rxErrEma,
          g_switchesThisHour, AUTORF_MAX_SWITCHES_HOUR);
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
    hublogf("[AUTO] ch%u: noise=%d dBm%s\n", i, (int)g_noise[i],
            i == cur ? " <- current" : "");
  bool any = false;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    any = true;
    hublogf("[AUTO] cam %u: pwr=%d dBm (floor %d) dl=%d ul=%d dBm%s\n", id,
            (int)g_camPower[id], (int)g_camPowerFloor[id], (int)g_camRssi[id],
            (int)g_camRxRssi[id], g_camLinkPoor[id] ? " POOR" : "");
  }
  if (!any)
    hublogf("[AUTO] no cameras seen yet\n");
}

#endif // AUTORF_H
