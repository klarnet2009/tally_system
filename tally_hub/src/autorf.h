#ifndef AUTORF_H
#define AUTORF_H

// ===== Automatic channel escape: hub adapter =====
//
// The POLICY lives in lib/TallyProtocol/TallyEscape.h, where it is unit-tested
// on the host with a controllable clock. This file is only the adapter: it
// collects what the hub knows about each camera, hands it to the policy, and
// carries out the verdict.
//
// That split is deliberate. The audit found four of its five defects in this
// logic while it lived here in file-scope globals, where no test could reach it
// — including a trigger that read "camera switched off" as "channel is bad" and
// migrated the whole fleet through a blackout every few minutes during a break.
// Policy that decides when to move a live production belongs somewhere a test
// can interrogate it.
//
// Channel choice remains a SECONDARY defence. The primary one is unconditional
// time diversity in the transmit scheduler; channel hopping cannot help at all
// against the FHSS interferers (2.4 GHz wireless DMX, audience BLE) that sweep
// the whole band.

#define AUTORF_EVAL_MS 2000UL // decision cadence
// Must stay well under the policy's shortest sustain, or the sustain cannot be
// honoured: with a 10s cadence and a 10s sustain the trigger needed TWO evals
// and the "10 second" fast escape actually took up to 20s.
static_assert(AUTORF_EVAL_MS * 2 <= ESC_SEVERE_SUSTAIN_MS,
              "eval cadence too coarse to honour the severe sustain");

#define AUTORF_STATS_MS 10000UL // channel-stat window (diagnostics + LBT)
#define AUTORF_BLOCK_LOG_MS (60UL * 1000)

static bool g_autoRf = true; // manual `chan` flips this off
static uint32_t g_lastEval = 0;
static uint32_t g_lastStats = 0;
static uint32_t g_lastBlockLog = 0;
static EscTier g_lastTier = ESC_NONE;
static bool g_escapeInited = false;

static void autoRfInit() {
  if (g_escapeInited)
    return;
  g_escapeInited = true;
  chanMeasureInit();
  g_escape.begin(millis());
  // Pre-age the log gate the same way the policy pre-ages its dwell, so a switch
  // vetoed inside the first minutes — exactly the window ESC_FIRST_SWITCH_MS
  // exists to allow — still explains itself. Left at 0 it stayed silent.
  g_lastBlockLog = millis() - AUTORF_BLOCK_LOG_MS;
}

static void autoRfTick() {
  autoRfInit();
  uint32_t now = millis();

  // Close the stats window on its own slower timer: the decision cadence is now
  // fast, and committing every 2s would shorten the busyFrac time constant.
  if (now - g_lastStats >= AUTORF_STATS_MS) {
    g_lastStats = now;
    chanCommit(g_chan[g_chanIdx < TALLY_CHAN_COUNT ? g_chanIdx : 0]);
  }

  if (now - g_lastEval < AUTORF_EVAL_MS)
    return;
  g_lastEval = now;

  EscCam cams[17];
  for (uint8_t id = 1; id <= 16; id++) {
    cams[id].everSeen = (g_camLastSeen[id] != 0);
    cams[id].reachable = g_camReachable[id];
    cams[id].missed = g_camMissed[id];
  }

  TallyEscape::Inputs in;
  in.now = now;
  in.curChan = g_chanIdx;
  in.onAir = (g_progMask != 0);
  in.switchInFlight = g_chanSwitch.pending() || (g_oldChanFreq != 0);
  const ChanStat &cs = g_chan[g_chanIdx < TALLY_CHAN_COUNT ? g_chanIdx : 0];
  in.busyValid = cs.valid;
  in.busyFrac = cs.busyFrac;

  TallyEscape::Verdict v = g_escape.evaluate(in, cams);

  // Published to the transmit scheduler: extra burst copies while the fleet
  // struggles. Useful even with automatic switching turned off.
  g_linkDegraded = v.degraded;

  if (v.tier != g_lastTier) {
    g_lastTier = v.tier;
    static const char *kTier[] = {"clear", "DEGRADED", "SEVERE", "BLACKOUT"};
    hublogf("[AUTO] link %s (talking=%u complaining=%u silent=%u)\n",
            kTier[v.tier], v.talking, v.complaining, v.silent);
  }

  if (!g_autoRf)
    return; // manual mode: measurement and the degraded flag still run

  if (v.blocked && now - g_lastBlockLog > AUTORF_BLOCK_LOG_MS) {
    g_lastBlockLog = now;
    hublogf("[AUTO] would escape but blocked: %s — staying\n", v.blocked);
  }
  if (v.wantSwitch)
    requestChannelSwitch(v.target, v.reason);
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
  static const char *kTier[] = {"clear", "DEGRADED", "SEVERE", "BLACKOUT"};
  hublogf("[AUTO] mode=%s cur=ch%u hubpwr=%d dBm (fixed) link=%s switches=%u/%u"
          " per hour\n",
          g_autoRf ? "ON" : "OFF (manual)", g_chanIdx, (int)g_txPower,
          kTier[g_lastTier], g_escape.switchesThisHour(),
          ESC_MAX_SWITCHES_HOUR);
  uint32_t now = millis();
  for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
    // Informational only: the radio never leaves the working channel to measure,
    // so stats exist only for channels the hub has actually been on. Use
    // `survey` for a deliberate sweep of the whole band.
    if (!g_chan[i].valid) {
      hublogf("[AUTO] ch%u: not measured (never been on it)%s\n", i,
              i == g_chanIdx ? " <- current" : "");
      continue;
    }
    hublogf("[AUTO] ch%u: floor=%d dBm busy=%d%%%s%s\n", i,
            (int)g_chan[i].floorDbm, (int)(g_chan[i].busyFrac * 100),
            i == g_chanIdx ? " <- current" : "",
            g_escape.penalised(now, i) ? "  (tried/abandoned recently)" : "");
  }
  bool any = false;
  for (uint8_t id = 1; id <= 16; id++) {
    if (!g_camLastSeen[id])
      continue;
    any = true;
    hublogf("[AUTO] cam %u: %s dl=%d ul=%d dBm missed=%u shows=%d tag=%02X\n", id,
            g_camReachable[id] ? "talking" : "SILENT", (int)g_camRssi[id],
            (int)g_camRxRssi[id], g_camMissed[id], (int)g_camShown[id],
            g_camTag[id]);
  }
  if (!any)
    hublogf("[AUTO] no cameras seen yet\n");
}

#endif // AUTORF_H
