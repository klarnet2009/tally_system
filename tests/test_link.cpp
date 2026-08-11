// Link-supervisor tests. Every rule here is a function of millis(), so on real
// hardware they can only be exercised by waiting in real time — which is exactly
// why they were never tested before.
#include "Arduino.h"
#include "harness.h"
#include "TallyLink.h"

static int g_stateCalls, g_locatorCalls, g_linkCalls, g_chanCalls;
static TallyState g_lastState;
static bool g_lastLost;
static uint8_t g_lastChanIdx;
static uint32_t g_lastChanAt;

static void onState(TallyState s) { g_stateCalls++; g_lastState = s; }
static void onLocator() { g_locatorCalls++; }
static void onLink(bool lost) { g_linkCalls++; g_lastLost = lost; }
static void onChan(uint8_t idx, uint32_t at) {
  g_chanCalls++; g_lastChanIdx = idx; g_lastChanAt = at;
}

static void feed(TallyLink &l, const TallyPacket &p) {
  uint8_t buf[TALLY_PACKET_SIZE];
  TallyProtocol::serialize(p, buf);
  l.onPacket(buf, TALLY_PACKET_SIZE);
}

static void resetCounters() {
  g_stateCalls = g_locatorCalls = g_linkCalls = g_chanCalls = 0;
}

// A link that has already heard the hub once, at t=1000.
static void primed(TallyLink &l, uint8_t camId = 1) {
  testSetMillis(1000);
  l.begin(camId, onState, onLocator, onLink);
  l.setChannelCallback(onChan);
  resetCounters();
  feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 0));
}

int main() {
  printf("== TallyLink ==\n");

  CASE("state callback fires only on a CHANGE, and reports the new state");
  {
    TallyLink l; primed(l);
    feed(l, TallyProtocol::createStateAllPacket(1u << 0, 0, true, 1));
    CHECK_EQ(g_stateCalls, 1);
    CHECK_EQ(g_lastState, STATE_PROGRAM);
    feed(l, TallyProtocol::createStateAllPacket(1u << 0, 0, true, 2));
    CHECK_EQ(g_stateCalls, 1); // idempotent re-send must not re-fire
    feed(l, TallyProtocol::createStateAllPacket(0, 1u << 0, true, 3));
    CHECK_EQ(g_stateCalls, 2);
    CHECK_EQ(g_lastState, STATE_PREVIEW);
  }

  CASE("signal-lost asserts only after the full timeout, and clears on a frame");
  {
    TallyLink l; primed(l);
    testAdvance(TALLY_SIGNAL_LOST_MS);
    l.tick();
    CHECK(!l.signalLost()); // boundary: not yet
    testAdvance(1);
    l.tick();
    CHECK(l.signalLost());
    CHECK_EQ(g_linkCalls, 1);
    CHECK(g_lastLost);
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 9));
    CHECK(!l.signalLost());
    CHECK_EQ(g_linkCalls, 2);
    CHECK(!g_lastLost);
  }

  CASE("peer telemetry must NOT count as link liveness (a dead hub stays dead)");
  {
    TallyLink l; primed(l);
    testAdvance(TALLY_SIGNAL_LOST_MS + 1);
    // another camera's telemetry arrives — valid frame, but not from the hub
    feed(l, TallyProtocol::createTelemetryPacket(4, -90, 0, 0, true,
                                                STATE_OFF, 0x22));
    l.tick();
    CHECK(l.signalLost());
  }

  CASE("source-stale asserts when the hub flags its source frozen");
  {
    TallyLink l; primed(l);
    // hub keeps beating but with sourceLive cleared
    for (uint32_t t = 0; t < TALLY_SOURCE_GRACE_MS + 200; t += 100) {
      testAdvance(100);
      feed(l, TallyProtocol::createStateAllPacket(0, 0, false, 0));
      l.tick();
    }
    CHECK(!l.signalLost());  // radio is fine
    CHECK(l.sourceStale());  // but the colour must not be trusted
    CHECK(!l.trustworthy());
  }

  CASE("a brief source dropout inside the grace window does NOT flicker");
  {
    TallyLink l; primed(l);
    testAdvance(500);
    feed(l, TallyProtocol::createStateAllPacket(0, 0, false, 1)); // one bad beat
    l.tick();
    CHECK(!l.sourceStale());
    testAdvance(500);
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 2)); // recovered
    l.tick();
    CHECK(l.trustworthy());
  }

  CASE("locator fires for our id and for broadcast, not for another camera");
  {
    TallyLink l; primed(l, 3);
    feed(l, TallyProtocol::createPingPacket(3));
    CHECK_EQ(g_locatorCalls, 1);
    feed(l, TallyProtocol::createPingPacket(TALLY_BROADCAST_ID));
    CHECK_EQ(g_locatorCalls, 2);
    feed(l, TallyProtocol::createPingPacket(4));
    CHECK_EQ(g_locatorCalls, 2);
  }

  CASE("missed-heartbeat gradient counts gaps, and is not charged for boot");
  {
    TallyLink l;
    testSetMillis(1000);
    l.begin(1, onState, onLocator, onLink);
    resetCounters();
    testAdvance(10 * TALLY_REFRESH_MS); // long silence BEFORE the first frame
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 0));
    CHECK_EQ(l.missedBeats(), 0); // boot gap must not read as degradation
    testAdvance(2 * TALLY_REFRESH_MS + 1); // a real gap
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 1));
    CHECK_EQ(l.missedBeats(), 1);
    testAdvance(TALLY_REFRESH_MS); // on-time beat: no new gap
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 2));
    CHECK_EQ(l.missedBeats(), 1);
  }

  CASE("everHeard is false until the hub is actually heard (boot scan gate)");
  {
    TallyLink l;
    testSetMillis(1000);
    l.begin(1, onState, onLocator, onLink);
    CHECK(!l.everHeard());
    feed(l, TallyProtocol::createTelemetryPacket(2, 0, 0, 0, true, STATE_OFF, 1));
    CHECK(!l.everHeard()); // a peer is not the hub
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 0));
    CHECK(l.everHeard());
  }

  CASE("channel plan resolves the countdown to an instant, late not early");
  {
    TallyLink l; primed(l);
    uint32_t t0 = millis();
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 5, 2, 4));
    CHECK_EQ(g_chanCalls, 1);
    CHECK_EQ(g_lastChanIdx, 2);
    // 4 beats out, plus the deliberate margin that makes slaves switch AFTER
    // the hub rather than before it
    CHECK_EQ(g_lastChanAt,
             t0 + 4 * TALLY_REFRESH_MS + TALLY_CHAN_SWITCH_MARGIN_MS);
  }

  CASE("later announcements refine the instant; losing any subset still works");
  {
    TallyLink l; primed(l);
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 1, 1, 8));
    uint32_t firstEstimate = g_lastChanAt;
    testAdvance(3 * TALLY_REFRESH_MS);
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 4, 1, 5));
    // the refined estimate lands within a beat of the original
    long drift = (long)g_lastChanAt - (long)firstEstimate;
    CHECK(drift > -(long)TALLY_REFRESH_MS && drift < (long)TALLY_REFRESH_MS);
  }

  CASE("heartbeat cycle counter and arrival time are exposed for TX slotting");
  {
    TallyLink l; primed(l);
    testAdvance(250);
    uint32_t at = millis();
    feed(l, TallyProtocol::createStateAllPacket(0, 0, true, 42));
    CHECK(l.hbSeen());
    CHECK_EQ(l.lastHbCount(), 42);
    CHECK_EQ(l.lastHbAtMs(), at);
  }

  CASE("garbage and foreign frames are rejected without touching the link");
  {
    TallyLink l; primed(l);
    uint8_t junk[TALLY_PACKET_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7};
    CHECK(!l.onPacket(junk, sizeof(junk)));
    CHECK(!l.onPacket(junk, 3)); // too short
    CHECK_EQ(g_stateCalls, 0);
  }

  return testSummary("TallyLink");
}
