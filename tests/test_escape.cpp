// Channel-escape policy. Four of the five audit defects lived in this logic
// while it sat in the hub's globals where no test could reach it. Every case
// below is one of those defects, pinned.
#include "Arduino.h"
#include "harness.h"
#include "TallyEscape.h"

static EscCam cams[17];

static void clearCams() {
  for (int i = 0; i < 17; i++) cams[i] = EscCam();
}
// A camera that is talking to us and reporting a healthy link.
static void healthy(uint8_t id) {
  cams[id].everSeen = true; cams[id].reachable = true; cams[id].missed = 0;
}
// Talking to us, but telling us it is missing heartbeats.
static void complaining(uint8_t id) {
  cams[id].everSeen = true; cams[id].reachable = true;
  cams[id].missed = ESC_POOR_MISSED;
}
// Was seen once, now silent (off / flat battery / out of range / dead channel).
static void silent(uint8_t id) {
  cams[id].everSeen = true; cams[id].reachable = false; cams[id].missed = 0;
}

static TallyEscape::Inputs in(uint32_t now, bool onAir = false) {
  TallyEscape::Inputs i;
  i.now = now; i.curChan = 0; i.onAir = onAir;
  i.busyValid = true; i.busyFrac = 0.0f;
  return i;
}

// Hold a tier long enough to pass its sustain, returning the final verdict.
static TallyEscape::Verdict hold(TallyEscape &e, uint32_t start, uint32_t dur,
                                 bool onAir = false, float busy = 0.0f) {
  TallyEscape::Verdict v;
  for (uint32_t t = 0; t <= dur; t += 1000) {
    TallyEscape::Inputs i = in(start + t, onAir);
    i.busyFrac = busy;
    v = e.evaluate(i, cams);
  }
  return v;
}

int main() {
  printf("== TallyEscape ==\n");
  const uint32_t T0 = 100000; // past ESC_FIRST_SWITCH_MS

  CASE("DEFECT: cameras switched off must NOT trigger an escape");
  {
    // The whole fleet powered down for a break. everSeen stays true forever, so
    // the old code read this as 100% failure and migrated the fleet repeatedly.
    TallyEscape e; e.begin(0);
    clearCams();
    for (uint8_t id = 1; id <= 4; id++) silent(id);
    TallyEscape::Verdict v = hold(e, T0, 120000, false, 0.0f); // quiet air
    CHECK_EQ(v.talking, 0);
    CHECK_EQ(v.silent, 4);
    CHECK_EQ(v.tier, ESC_NONE);
    CHECK(!v.degraded);
    CHECK(!v.wantSwitch);
  }

  CASE("a talking camera reporting missed beats IS the trigger");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1);
    TallyEscape::Verdict v = hold(e, T0, ESC_DEGRADED_SUSTAIN_MS + 2000);
    CHECK_EQ(v.talking, 1);
    CHECK_EQ(v.complaining, 1);
    CHECK(v.wantSwitch);
    CHECK(v.target != 0);
  }

  CASE("DEFECT: ONE complaining camera must not reach the severe tier");
  {
    // frac is trivially 1.0 with a single camera, which reached the one tier
    // allowed to override the on-air guard — so a single unit behind a wall
    // migrated the whole fleet every 90s. It must be DEGRADED: slower sustain,
    // and it respects the guard.
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1);
    TallyEscape::Verdict v = hold(e, T0, ESC_SEVERE_SUSTAIN_MS + 1000);
    CHECK_EQ(v.tier, ESC_DEGRADED);
    CHECK(!v.wantSwitch); // 6s is not enough for the degraded tier
    // ...and it must not push past a live camera
    TallyEscape e2; e2.begin(0);
    v = hold(e2, T0, ESC_DEGRADED_SUSTAIN_MS + 2000, true, 0.0f);
    CHECK(!v.wantSwitch);
    CHECK(v.blocked != nullptr);
  }

  CASE("DEFECT: a two-camera fleet at 50% failure must still raise a tier");
  {
    // The old `talking >= 3` gate on the fraction branch left a 2-camera rig with
    // NO tier at all: no extra burst copies, no log line, no escape — while the
    // docs promised a trigger at 30%.
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); healthy(2);
    TallyEscape::Verdict v = e.evaluate(in(T0), cams);
    CHECK_EQ(v.tier, ESC_DEGRADED);
    CHECK(v.degraded);
    v = hold(e, T0, ESC_DEGRADED_SUSTAIN_MS + 2000);
    CHECK(v.wantSwitch);
  }

  CASE("DEFECT: no switch at all inside the post-boot settling window");
  {
    // The floor used to be a pre-aged dwell of 540s, which already exceeded the
    // 90s severe dwell — so the severe tier was exempt and could migrate the
    // fleet ~10s after a power-cycle, on telemetry whose missed-beat window still
    // held gaps from that very reboot.
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); complaining(3);
    TallyEscape::Verdict v = hold(e, 1000, ESC_SEVERE_SUSTAIN_MS + 3000);
    CHECK_EQ(v.tier, ESC_SEVERE);
    CHECK(!v.wantSwitch);
    CHECK(v.blocked != nullptr);
    // ...and allowed once the window has passed
    v = hold(e, ESC_FIRST_SWITCH_MS + 2000, ESC_SEVERE_SUSTAIN_MS + 2000);
    CHECK(v.wantSwitch);
  }

  CASE("3 of 4 complaining is severe: one healthy camera does not hold us back");
  {
    TallyEscape e; e.begin(0);
    clearCams();
    complaining(1); complaining(2); complaining(3); healthy(4);
    TallyEscape::Verdict v = hold(e, T0, ESC_SEVERE_SUSTAIN_MS + 2000);
    CHECK_EQ(v.tier, ESC_SEVERE); // 0.75 >= ESC_SEVERE_FRAC
    CHECK(v.wantSwitch);
  }

  CASE("severe escapes on the fast sustain, partial does not");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    // 2/4 = 0.5: below ESC_SEVERE_FRAC -> must NOT fire on the fast sustain
    TallyEscape::Verdict v = hold(e, T0, ESC_SEVERE_SUSTAIN_MS + 1000);
    CHECK_EQ(v.tier, ESC_DEGRADED);
    CHECK(!v.wantSwitch);
    v = hold(e, T0, ESC_DEGRADED_SUSTAIN_MS + 2000);
    CHECK(v.wantSwitch);
  }
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); complaining(3); complaining(4);
    TallyEscape::Verdict v = hold(e, T0, ESC_SEVERE_SUSTAIN_MS + 1000);
    CHECK_EQ(v.tier, ESC_SEVERE);
    CHECK(v.wantSwitch); // 4/4 -> fast tier
  }

  CASE("a transient does not escape: the tier clock resets when it clears");
  {
    TallyEscape e; e.begin(0);
    // 2 of 4 complaining = partial degradation (not severe), 30s sustain
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    hold(e, T0, 20000); // 20s of degradation, short of the 30s sustain
    clearCams(); healthy(1); healthy(2); healthy(3); healthy(4);
    e.evaluate(in(T0 + 21000), cams); // clears
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    TallyEscape::Verdict v = hold(e, T0 + 22000, 20000); // 20s again
    CHECK(!v.wantSwitch); // must not have inherited the first 20s
  }

  CASE("escalating degraded -> severe restarts the clock, does not inherit it");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    hold(e, T0, 20000); // degraded (2/4) for 20s
    clearCams(); complaining(1); complaining(2); complaining(3); complaining(4);
    // now severe; must serve its OWN 6s, not fire instantly on the inherited 20s
    TallyEscape::Verdict v = e.evaluate(in(T0 + 21000), cams);
    CHECK_EQ(v.tier, ESC_SEVERE);
    CHECK(!v.wantSwitch);
    v = hold(e, T0 + 21000, ESC_SEVERE_SUSTAIN_MS + 1000);
    CHECK(v.wantSwitch);
  }

  CASE("BLACKOUT needs corroboration: silent fleet + BUSY air escapes");
  {
    TallyEscape e; e.begin(0);
    clearCams(); silent(1); silent(2);
    // quiet air -> nothing, however long we wait
    TallyEscape::Verdict v = hold(e, T0, ESC_BLACKOUT_SUSTAIN_MS + 5000, false, 0.0f);
    CHECK_EQ(v.tier, ESC_NONE);
    // same silence, but the channel is measurably hammered -> escape
    TallyEscape e2; e2.begin(0);
    v = hold(e2, T0, ESC_BLACKOUT_SUSTAIN_MS + 5000, false, 0.9f);
    CHECK_EQ(v.tier, ESC_BLACKOUT);
    CHECK(v.wantSwitch);
  }

  CASE("the on-air guard holds only DEGRADED: severe AND blackout both move");
  {
    // Blackout while the ATEM says someone is live: nobody is talking to us, so
    // there is no reachable camera whose outage staying could prevent. The old
    // guard kept the fleet on a dead channel for as long as anyone was on air —
    // at a concert, that is the whole show.
    TallyEscape e; e.begin(0);
    clearCams(); silent(1); silent(2);
    TallyEscape::Verdict v =
        hold(e, T0, ESC_BLACKOUT_SUSTAIN_MS + 5000, true, 0.9f);
    CHECK_EQ(v.tier, ESC_BLACKOUT);
    CHECK(v.wantSwitch);
    CHECK(v.blocked == nullptr);

    TallyEscape e2; e2.begin(0);
    clearCams(); complaining(1); complaining(2); complaining(3);
    v = hold(e2, T0, ESC_SEVERE_SUSTAIN_MS + 2000, true, 0.0f);
    CHECK(v.wantSwitch); // 3/3 = 1.0 >= override fraction
  }

  CASE("a mildly degraded fleet does not switch while a camera is on air");
  {
    TallyEscape e; e.begin(0);
    clearCams();
    complaining(1); healthy(2); healthy(3); // 1/3 = 0.33: degraded, not severe
    TallyEscape::Verdict v =
        hold(e, T0, ESC_DEGRADED_SUSTAIN_MS + 2000, true, 0.0f);
    CHECK_EQ(v.tier, ESC_DEGRADED);
    CHECK(!v.wantSwitch);
    CHECK(v.blocked != nullptr);
  }

  CASE("rotation walks the list and skips recently abandoned channels");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    uint32_t t = T0;
    uint8_t cur = 0;
    for (int step = 0; step < TALLY_CHAN_COUNT - 1; step++) {
      TallyEscape::Inputs i = in(t);
      i.curChan = cur;
      TallyEscape::Verdict v;
      for (uint32_t k = 0; k <= ESC_DEGRADED_SUSTAIN_MS + 2000; k += 1000) {
        i.now = t + k;
        v = e.evaluate(i, cams);
      }
      CHECK(v.wantSwitch);
      CHECK(v.target != cur);
      CHECK(!e.penalised(i.now, v.target)); // never rotate into a fresh reject
      e.noteSwitching(i.now, cur);
      cur = v.target;
      t = i.now + ESC_DWELL_MS + 1000;
    }
    // Every channel has now been tried in this ONE episode: the policy must
    // settle rather than keep spending outages that cannot help.
    TallyEscape::Inputs i = in(t);
    i.curChan = cur;
    TallyEscape::Verdict v;
    for (uint32_t k = 0; k <= ESC_DEGRADED_SUSTAIN_MS + 2000; k += 1000) {
      i.now = t + k;
      v = e.evaluate(i, cams);
    }
    CHECK(!v.wantSwitch);
    CHECK(v.blocked != nullptr);
  }

  CASE("severe uses the short dwell so it is not stuck behind a 10-min wait");
  {
    TallyEscape e; e.begin(0);
    // 2 of 4 = 0.5: degraded (>= 0.30) but not severe (< 0.65)
    clearCams(); complaining(1); complaining(2); healthy(3); healthy(4);
    TallyEscape::Verdict v = hold(e, T0, ESC_DEGRADED_SUSTAIN_MS + 2000);
    CHECK(v.wantSwitch);
    e.noteSwitching(T0 + ESC_DEGRADED_SUSTAIN_MS + 2000, 0);
    uint32_t t = T0 + ESC_DEGRADED_SUSTAIN_MS + 2000;

    // 2 minutes later, severe: the 90s severe dwell has passed, the 10-min one
    // has not. It must be allowed to move.
    clearCams(); complaining(1); complaining(2); complaining(3);
    TallyEscape::Inputs i = in(t + 120000);
    i.curChan = 1;
    for (uint32_t k = 0; k <= ESC_SEVERE_SUSTAIN_MS + 2000; k += 1000) {
      i.now = t + 120000 + k;
      v = e.evaluate(i, cams);
    }
    CHECK_EQ(v.tier, ESC_SEVERE);
    CHECK(v.wantSwitch);
  }

  CASE("the degraded flag is published for every tier (extra burst copies)");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); // 1 of 1 talking -> degraded (not severe)
    TallyEscape::Verdict v = e.evaluate(in(T0), cams);
    CHECK(v.degraded); // immediately, without waiting for any sustain
    CHECK(!v.wantSwitch);
  }

  CASE("first switch is allowed ~1 min after boot, not a full dwell");
  {
    TallyEscape e; e.begin(0);
    clearCams(); complaining(1); complaining(2); complaining(3);
    // severe from t=0; sustain is 6s, so by 61s the only thing that could still
    // block is the dwell — and the pre-aged clock must have released it.
    TallyEscape::Verdict v = hold(e, 1000, 61000);
    CHECK(v.wantSwitch);
  }

  printf("== TallyChanSwitch ==\n");

  CASE("DEFECT: N beats must put N usable countdowns on air, never N-1");
  {
    TallyChanSwitch s;
    s.request(2, 100, 10);
    CHECK(s.pending());
    CHECK_EQ(s.countdown(100), 10); // announced at the beat it was requested
    CHECK_EQ(s.countdown(105), 5);
    CHECK_EQ(s.countdown(109), 1);  // the last useful announcement
    CHECK(!s.expired(109));
    CHECK_EQ(s.countdown(110), 0);
    CHECK(s.expired(110));
  }

  CASE("DEFECT: beats=1 must not silently become zero announcements");
  {
    TallyChanSwitch s;
    s.request(1, 50, 1);
    // clamped to 2, so at least one frame carries a live countdown
    CHECK(s.countdown(50) >= 1);
    CHECK(!s.expired(50));
  }

  CASE("beats is clamped to the 4-bit wire field");
  {
    TallyChanSwitch s;
    s.request(1, 0, 200);
    CHECK(s.countdown(0) <= 15);
  }

  CASE("the countdown survives the heartbeat counter wrapping at 256");
  {
    TallyChanSwitch s;
    s.request(3, 250, 10); // due at (250+10) mod 256 = 4
    CHECK_EQ(s.countdown(250), 10);
    CHECK_EQ(s.countdown(255), 5);
    CHECK_EQ(s.countdown(0), 4);
    CHECK_EQ(s.countdown(3), 1);
    CHECK(s.expired(4));
  }

  CASE("cleared switch announces nothing");
  {
    TallyChanSwitch s;
    s.request(1, 0, 5);
    s.clear();
    CHECK(!s.pending());
    CHECK_EQ(s.countdown(0), 0);
  }

  return testSummary("TallyEscape/TallyChanSwitch");
}
