// Burst scheduler + listen-before-talk. These encode the two rules that matter
// most on a concert floor: a copy never carries a stale state, and a busy
// channel can delay a frame but can never silence the link.
#include "Arduino.h"
#include "harness.h"
#include "TallyBurst.h"

int main() {
  printf("== TallyBurst / TallyLbt ==\n");
  static const uint16_t offs[TALLY_BURST_COPIES_MAX] = TALLY_BURST_OFFSETS_MS;

  CASE("offsets are strictly increasing and IRREGULAR (no aliasing comb)");
  {
    for (int i = 1; i < TALLY_BURST_COPIES_MAX; i++)
      CHECK(offs[i] > offs[i - 1]);
    // An even comb could line up with a periodic interferer (WiFi beacons are
    // 102.4ms apart) and lose every copy — assert the gaps really do differ.
    bool anyDifferent = false;
    for (int i = 2; i < TALLY_BURST_COPIES_MAX; i++)
      if ((offs[i] - offs[i - 1]) != (offs[1] - offs[0]))
        anyDifferent = true;
    CHECK(anyDifferent);
    // The span must exceed a heartbeat so one long burst of interference cannot
    // cover the whole set.
    CHECK(offs[TALLY_BURST_COPIES_MIN - 1] > 300);
  }

  CASE("a change schedules MIN copies, degraded schedules MAX, never fewer");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MIN);
    b.onChange(1000, true);
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MAX);
    CHECK(TALLY_BURST_COPIES_MAX >= TALLY_BURST_COPIES_MIN);
  }

  CASE("copies come due exactly on the offset schedule");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    for (int i = 0; i < TALLY_BURST_COPIES_MIN; i++) {
      testSetMillis(1000 + offs[i]);
      CHECK(b.dueNow(millis()));
      if (i + 1 < TALLY_BURST_COPIES_MIN) {
        // and NOT due again immediately after sending
        b.markSent(millis());
        CHECK(!b.dueNow(millis()));
        testSetMillis(1000 + offs[i + 1] - 1);
        CHECK(!b.dueNow(millis()));
      } else {
        b.markSent(millis());
      }
    }
    CHECK(b.idle());
    CHECK_EQ(b.pending(), 0);
  }

  CASE("only the FIRST copy of a burst is latency-critical");
  {
    TallyBurst b; b.reset(0);
    b.onChange(0, false);
    CHECK(b.latencyCritical());
    b.markSent(0);
    CHECK(!b.latencyCritical());
  }

  CASE("a lone heartbeat is NOT latency-critical (it may yield to a busy air)");
  {
    TallyBurst b; b.reset(0);
    CHECK(b.scheduleHeartbeat(0));
    CHECK_EQ(b.pending(), 1);
    CHECK(!b.latencyCritical());
  }

  CASE("a second cut mid-burst RESTARTS the schedule — the core stale-state fix");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    testSetMillis(1000);
    b.markSent(millis());          // copy 1 of state A sent
    testSetMillis(1050);           // 50ms later: a new cut (state B)
    b.onChange(millis(), false);
    // Full copy count again, and due immediately: the remaining copies now
    // describe B, not A. v3's packet FIFO would have finished sending A first.
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MIN);
    CHECK(b.dueNow(millis()));
    CHECK(b.latencyCritical());
  }

  CASE("a heartbeat never interrupts a burst in progress");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    b.markSent(1000);
    CHECK(!b.scheduleHeartbeat(1100));
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MIN - 1); // untouched
  }

  CASE("abandon drops the rest (radio went away) and leaves the object usable");
  {
    TallyBurst b; b.reset(0);
    b.onChange(0, true);
    b.abandon();
    CHECK(b.idle());
    CHECK(b.scheduleHeartbeat(10));
  }

  CASE("millis() rollover: a burst spanning the wrap still comes due");
  {
    uint32_t nearWrap = 0xFFFFFF00u;
    TallyBurst b; b.reset(nearWrap);
    b.onChange(nearWrap, false);
    b.markSent(nearWrap);
    // second copy is due at nearWrap + offs[1], which wraps past zero
    testSetMillis((uint32_t)(nearWrap + offs[1]));
    CHECK(b.dueNow(millis()));
  }

  CASE("LBT: a clear channel transmits immediately");
  {
    TallyLbt g;
    CHECK(g.clear(1000, false, false));
    CHECK(!g.deferring());
  }

  CASE("LBT: a busy channel defers, then sends anyway at the cap");
  {
    TallyLbt g;
    CHECK(!g.clear(1000, false, true));
    CHECK(g.deferring());
    CHECK(!g.clear(1000 + TALLY_LBT_MAX_DEFER_MS - 1, false, true));
    CHECK(g.clear(1000 + TALLY_LBT_MAX_DEFER_MS, false, true));
    // a permanently busy channel must NOT be able to silence the link
    CHECK(!g.deferring());
  }

  CASE("LBT: the first copy of a cut is never delayed, busy or not");
  {
    TallyLbt g;
    CHECK(g.clear(1000, true, true));
    CHECK(!g.deferring());
  }

  CASE("LBT: separate instances do not consume each other's patience");
  {
    // This is the defect the split fixes: with one shared clock, a beacon that
    // had been deferring for 12ms let the NEXT frame through instantly, straight
    // into a busy channel.
    TallyLbt state, oneShot;
    CHECK(!oneShot.clear(1000, false, true));
    testSetMillis(1000 + TALLY_LBT_MAX_DEFER_MS);
    CHECK(oneShot.clear(millis(), false, true)); // its own patience ran out
    CHECK(!state.clear(millis(), false, true));  // the burst still has all of its
  }

  CASE("LBT: a channel that clears mid-deferral resets the patience budget");
  {
    TallyLbt g;
    CHECK(!g.clear(1000, false, true));
    CHECK(g.clear(1005, false, false)); // cleared -> send
    CHECK(!g.deferring());
    CHECK(!g.clear(1006, false, true)); // fresh budget, not the old one
    CHECK(!g.clear(1006 + TALLY_LBT_MAX_DEFER_MS - 1, false, true));
  }

  return testSummary("TallyBurst/TallyLbt");
}
