#ifndef AUTORF_H
#define AUTORF_H

// ===== Automatic channel escape =====
//
// Scope is deliberately SMALL, and got smaller on review. Channel choice is a
// secondary defence: the primary one is unconditional time diversity in the
// transmit scheduler (main.cpp), which needs no measurement and has no failure
// mode. Channel hopping cannot help at all against the FHSS interferers
// (2.4 GHz wireless DMX, audience BLE) that sweep the whole band.
//
// The v4 first draft still built a full measurement-driven optimiser here: a
// scout that retuned the radio to sample other channels, a cost function mixing
// noise floor and busy fraction, hysteresis against the current channel's score.
// It was deleted, because by this file's own argument it was the least valuable
// part of the system while carrying the most complexity — and it produced every
// bug found in review, including one (transmitting on the scout's frequency)
// that would have taken the whole fleet down. Complexity on a safety-critical
// path has to earn its place, and that did not.
//
// What remains is the policy that actually matters:
//
//   "If the slaves are missing heartbeats for long enough, this channel is bad —
//    move to another one."
//
//  - The trigger is the DIRECT signal (slaves' own missed-heartbeat gradient,
//    reported in telemetry), not an inference from measured energy. It measures
//    the downlink, which is the safety-critical direction. The hub's own RX
//    error count is NOT used: it measures the uplink and is polluted by foreign
//    energy.
//  - Target selection is a rotation with a short penalty memory, not a score.
//    We do not claim to know which channel is better — we know the current one
//    is bad. If the next is also bad the same trigger fires again and moves on;
//    with a sustained trigger plus a dwell this converges in a couple of
//    migrations. No measurement can mislead it, because none is consulted.
//  - The radio NEVER leaves the working channel to measure. Passive stats live
//    in main.cpp and are diagnostics plus the listen-before-talk reference only.
//
// Every trigger is sustained, dwell-limited and rate-limited, and a blocked
// decision degrades to "stay + log", never to flapping. A manual `chan` command
// turns this off until `auto on`.

#define AUTORF_EVAL_MS 10000UL            // decision cadence
// Two-tier escape. Partial degradation waits 30s: a switch costs a brief outage
// and must not be spent on a transient. But when almost the whole fleet is down
// we are already failing, so waiting is pure loss — escape after 10s and cut the
// announcement short too, because slaves that cannot hear us gain nothing from a
// longer countdown (the old-channel beacon and their scan cover them).
#define AUTORF_DEGRADE_SUSTAIN_MS 30000UL // partial degradation
#define AUTORF_SEVERE_SUSTAIN_MS 10000UL  // most of the fleet down
#define AUTORF_SEVERE_FRAC 0.80f
#define AUTORF_SEVERE_BEATS 4 // shortened announcement when severe
#define AUTORF_POOR_COUNT_TRIG 2          // cameras missing heartbeats
#define AUTORF_POOR_FRAC_TRIG 0.30f       // ...or this share of a bigger fleet
#define AUTORF_ONAIR_OVERRIDE_FRAC 0.60f  // switch even on-air past this
// First switch allowed one minute after boot, not ten: a venue's spectrum
// collapses when the audience arrives, which is often inside the first window.
#define AUTORF_FIRST_SWITCH_MS 60000UL
#define AUTORF_MIN_DWELL_MS (10UL * 60 * 1000)
#define AUTORF_MAX_SWITCHES_HOUR 4
#define AUTORF_BLOCK_LOG_MS (5UL * 60 * 1000)
// A channel abandoned for being bad is skipped while this lasts, so the fleet
// can't ping-pong between two bad channels.
#define AUTORF_PENALTY_MS (20UL * 60 * 1000)

static bool g_autoRf = true; // manual `chan` flips this off

static uint32_t g_lastEval = 0;
static uint32_t g_degradeSince = 0;
static uint32_t g_lastAutoSwitch = 0;
static uint32_t g_switchHourStart = 0;
static uint8_t g_switchesThisHour = 0;
static uint32_t g_lastBlockLog = 0;
static uint32_t g_chanPenaltyAt[TALLY_CHAN_COUNT] = {0}; // 0 = never abandoned
static bool g_autoRfInited = false;

static void autoRfInit() {
  if (g_autoRfInited)
    return;
  g_autoRfInited = true;
  chanMeasureInit();
  // Dwell clock pre-aged so the first switch is allowed after
  // AUTORF_FIRST_SWITCH_MS rather than a full dwell. Unsigned modular
  // arithmetic makes the apparent underflow behave correctly.
  g_lastAutoSwitch = millis() - (AUTORF_MIN_DWELL_MS - AUTORF_FIRST_SWITCH_MS);
  g_switchHourStart = millis();
}

// Where to go: the least recently abandoned channel that isn't the current one.
// No score, no measurement — see the header note.
static uint8_t autoRfPickTarget() {
  uint8_t best = 0xFF;
  uint32_t bestAge = 0;
  uint32_t now = millis();
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    if (i == g_chanIdx)
      continue;
    // Never-abandoned sorts first, then the oldest abandonment.
    uint32_t age =
        (g_chanPenaltyAt[i] == 0) ? 0xFFFFFFFFUL : (now - g_chanPenaltyAt[i]);
    if (best == 0xFF || age > bestAge) {
      bestAge = age;
      best = i;
    }
  }
  if (best == 0xFF)
    return 0xFF;
  // Everything else was abandoned recently: staying beats a pointless outage.
  if (g_chanPenaltyAt[best] != 0 && bestAge < AUTORF_PENALTY_MS)
    return 0xFF;
  return best;
}

static void autoRfTrySwitch(uint8_t idx, const char *reason, bool severe) {
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
  g_chanPenaltyAt[g_chanIdx] = now; // remember this one was bad
  g_lastAutoSwitch = now;
  g_switchesThisHour++;
  g_degradeSince = 0;
  requestChannelSwitch(idx, reason,
                       severe ? AUTORF_SEVERE_BEATS : TALLY_CHAN_ANNOUNCE_BEATS);
}

static void autoRfTick() {
  autoRfInit();

  uint32_t now = millis();
  if (now - g_switchHourStart > 3600000UL) {
    g_switchHourStart = now;
    g_switchesThisHour = 0;
  }
  if (now - g_lastEval < AUTORF_EVAL_MS)
    return;
  g_lastEval = now;

  // Close the measurement window so `auto`/`noise` show fresh numbers — even in
  // manual mode, because the stats are diagnostics and should keep working.
  chanCommit(g_chan[g_chanIdx < TALLY_CHAN_COUNT ? g_chanIdx : 0]);

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
  // Severe = almost nobody is hearing us. Escapes on the fast tier.
  bool severe = seen >= 1 && poorFrac >= AUTORF_SEVERE_FRAC;
  // Published for the transmit scheduler: while degraded a change is sent with
  // more copies. Useful even with automatic switching turned off.
  g_linkDegraded = degraded;

  if (degraded) {
    if (!g_degradeSince)
      g_degradeSince = now;
  } else {
    g_degradeSince = 0;
  }

  if (!g_autoRf)
    return; // manual mode: measurement and the degraded flag still run
  if (g_switchPending || g_oldChanFreq)
    return; // a switch is already in flight
  uint32_t needSustain =
      severe ? AUTORF_SEVERE_SUSTAIN_MS : AUTORF_DEGRADE_SUSTAIN_MS;
  if (!g_degradeSince || now - g_degradeSince < needSustain)
    return;

  uint8_t target = autoRfPickTarget();
  if (target == 0xFF) {
    if (now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
      g_lastBlockLog = now;
      hublogf("[AUTO] channel degraded but every alternative was tried "
              "recently — staying\n");
    }
    return;
  }

  // A switch costs a brief outage. Don't spend it on a live camera unless the
  // degradation is already severe enough that staying is worse.
  if (g_progMask != 0 && poorFrac < AUTORF_ONAIR_OVERRIDE_FRAC) {
    if (now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
      g_lastBlockLog = now;
      hublogf("[AUTO] want ch%u but a camera is ON AIR and only %d%% poor — "
              "staying\n",
              target, (int)(poorFrac * 100));
    }
    return;
  }
  autoRfTrySwitch(target, severe ? "downlink SEVERE" : "downlink degraded",
                  severe);
}

static void autoRfManualOverride(const char *what) {
  if (!g_autoRf)
    return;
  g_autoRf = false;
  hublogf("[AUTO] manual %s — automatic channel escape OFF until 'auto on'\n",
          what);
}

static void autoRfReport() {
  autoRfInit();
  hublogf("[AUTO] mode=%s cur=ch%u hubpwr=%d dBm (fixed) switches=%u/%u per hour"
          " degraded=%s\n",
          g_autoRf ? "ON" : "OFF (manual)", g_chanIdx, (int)g_txPower,
          g_switchesThisHour, AUTORF_MAX_SWITCHES_HOUR,
          g_linkDegraded ? "YES" : "no");
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    // Informational: only channels we have actually been ON have stats, because
    // the radio never leaves the working channel to measure. Use `noise` for a
    // deliberate survey of all of them.
    if (!g_chan[i].valid) {
      hublogf("[AUTO] ch%u: not measured (never been on it)%s\n", i,
              i == g_chanIdx ? " <- current" : "");
      continue;
    }
    bool penalised = g_chanPenaltyAt[i] &&
                     (millis() - g_chanPenaltyAt[i] < AUTORF_PENALTY_MS);
    hublogf("[AUTO] ch%u: floor=%d dBm busy=%d%%%s%s\n", i,
            (int)g_chan[i].floorDbm, (int)(g_chan[i].busyFrac * 100),
            i == g_chanIdx ? " <- current" : "",
            penalised ? "  (abandoned recently)" : "");
  }
  bool any = false;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    any = true;
    hublogf("[AUTO] cam %u: dl=%d ul=%d dBm missed=%u shows=%d tag=%02X%s\n", id,
            (int)g_camRssi[id], (int)g_camRxRssi[id], g_camMissed[id],
            (int)g_camShown[id], g_camTag[id], camPoor(id) ? " POOR" : "");
  }
  if (!any)
    hublogf("[AUTO] no cameras seen yet\n");
}

#endif // AUTORF_H
