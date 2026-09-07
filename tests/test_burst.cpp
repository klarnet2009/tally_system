// Burst scheduler + listen-before-talk. These encode the two rules that matter
// most on a concert floor: a copy never carries a stale state, and a busy
// channel can delay a frame but can never silence the link.
#include "Arduino.h"
#include "harness.h"
#include "TallyBurst.h"

int main() {
  printf("== TallyBurst / TallyLbt ==\n");
  // Anchor the shared clock before each group: the rollover case below leaves it
  // near the wrap, and a later case that reads millis() would silently inherit it.
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

  testSetMillis(0);
  CASE("a change schedules MIN copies, degraded schedules MAX, never fewer");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MIN);
    b.onChange(1000, true);
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MAX);
    CHECK(TALLY_BURST_COPIES_MAX >= TALLY_BURST_COPIES_MIN);
  }

  testSetMillis(0);
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

  testSetMillis(0);
  CASE("only the FIRST copy of a burst is latency-critical");
  {
    TallyBurst b; b.reset(0);
    b.onChange(0, false);
    CHECK(b.latencyCritical());
    b.markSent(0);
    CHECK(!b.latencyCritical());
  }

  testSetMillis(0);
  CASE("a lone heartbeat is NOT latency-critical (it may yield to a busy air)");
  {
    TallyBurst b; b.reset(0);
    CHECK(b.scheduleHeartbeat(0));
    CHECK_EQ(b.pending(), 1);
    CHECK(!b.latencyCritical());
  }

  testSetMillis(0);
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

  testSetMillis(0);
  CASE("a heartbeat never interrupts a burst in progress");
  {
    TallyBurst b; b.reset(1000);
    b.onChange(1000, false);
    b.markSent(1000);
    CHECK(!b.scheduleHeartbeat(1100));
    CHECK_EQ(b.pending(), TALLY_BURST_COPIES_MIN - 1); // untouched
  }

  testSetMillis(0);
  CASE("abandon drops the rest (radio went away) and leaves the object usable");
  {
    TallyBurst b; b.reset(0);
    b.onChange(0, true);
    b.abandon();
    CHECK(b.idle());
    CHECK(b.scheduleHeartbeat(10));
  }

  testSetMillis(0);
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

  testSetMillis(0);
  CASE("LBT: a clear channel transmits immediately");
  {
    TallyLbt g;
    CHECK(g.clear(1000, false, false));
    CHECK(!g.deferring());
  }

  testSetMillis(0);
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

  testSetMillis(0);
  CASE("LBT: the first copy of a cut is never delayed, busy or not");
  {
    TallyLbt g;
    CHECK(g.clear(1000, true, true));
    CHECK(!g.deferring());
  }

  testSetMillis(0);
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

  testSetMillis(0);
  CASE("LBT: a channel that clears mid-deferral resets the patience budget");
  {
    TallyLbt g;
    CHECK(!g.clear(1000, false, true));
    CHECK(g.clear(1005, false, false)); // cleared -> send
    CHECK(!g.deferring());
    CHECK(!g.clear(1006, false, true)); // fresh budget, not the old one
    CHECK(!g.clear(1006 + TALLY_LBT_MAX_DEFER_MS - 1, false, true));
  }

  testSetMillis(0);
  CASE("burst flag: every copy EXCEPT the last, so something releases the hold");
  {
    // The last copy is deliberately unflagged. It is what clears the receiver's
    // telemetry hold and re-anchors its slot — and nothing follows it, so
    // anchoring there cannot aim an uplink at a later copy. Without this,
    // sustained cutting starved the lone heartbeat and the hold latched on
    // forever, stopping the entire fleet's telemetry.
    TallyBurst b; b.reset(0);
    b.onChange(0, false);
    for (int i = 0; i < TALLY_BURST_COPIES_MIN - 1; i++) {
      CHECK(b.flagAsBurst());
      b.markSent(0);
    }
    CHECK(!b.flagAsBurst()); // final copy releases
    b.markSent(0);
    CHECK(b.idle());
  }

  testSetMillis(0);
  CASE("a lone heartbeat is never flagged as a burst copy");
  {
    TallyBurst b; b.reset(0);
    b.scheduleHeartbeat(0);
    CHECK(!b.flagAsBurst());
  }

  printf("== telemetry slot ==\n");

  auto slotIn = [](uint32_t now, uint8_t cam, uint8_t hb, uint32_t hbAt) {
    TallySlotInputs i;
    i.now = now; i.camId = cam; i.hbSeen = true; i.hbCount = hb; i.hbAtMs = hbAt;
    return i;
  };

  CASE("a camera transmits only in its own cycle, at its own bank offset");
  {
    // cam 1 -> slot 0, bank 0; cam 9 -> slot 0, bank 1
    CHECK(tallySlotDue(slotIn(1000 + TALLY_TLM_OFFSET_MS, 1, 8, 1000)));
    CHECK(!tallySlotDue(slotIn(1000 + TALLY_TLM_OFFSET_MS, 1, 9, 1000)));
    CHECK(tallySlotDue(
        slotIn(1000 + TALLY_TLM_OFFSET_MS + TALLY_TLM_BANK_MS, 9, 8, 1000)));
    // ...and cam 9 must NOT fire in cam 1's window
    CHECK(!tallySlotDue(slotIn(1000 + TALLY_TLM_OFFSET_MS, 9, 8, 1000)));
  }

  CASE("the slot opens and closes; a late frame must not run into bank 1");
  {
    uint32_t open = 1000 + TALLY_TLM_OFFSET_MS;
    CHECK(!tallySlotDue(slotIn(open - 1, 1, 8, 1000)));
    CHECK(tallySlotDue(slotIn(open, 1, 8, 1000)));
    CHECK(tallySlotDue(slotIn(open + TALLY_TLM_LATE_MS, 1, 8, 1000)));
    CHECK(!tallySlotDue(slotIn(open + TALLY_TLM_LATE_MS + 1, 1, 8, 1000)));
    // the closing bound must leave a whole frame before bank 1 opens
    CHECK(TALLY_TLM_LATE_MS + 51 <= TALLY_TLM_BANK_MS);
  }

  CASE("a burst in flight silences telemetry regardless of the slot");
  {
    TallySlotInputs i = slotIn(1000 + TALLY_TLM_OFFSET_MS, 1, 8, 1000);
    CHECK(tallySlotDue(i));
    i.burstInFlight = true;
    CHECK(!tallySlotDue(i));
  }

  CASE("one transmission per cycle, and nothing before the first heartbeat");
  {
    TallySlotInputs i = slotIn(1000 + TALLY_TLM_OFFSET_MS, 1, 8, 1000);
    i.sentThisCycle = true;
    CHECK(!tallySlotDue(i));
    i.sentThisCycle = false;
    i.hbSeen = false;
    CHECK(!tallySlotDue(i));
  }

  CASE("an out-of-range camera id never transmits");
  {
    CHECK(!tallySlotDue(slotIn(1000 + TALLY_TLM_OFFSET_MS, 0, 8, 1000)));
    CHECK(!tallySlotDue(slotIn(1000 + TALLY_TLM_OFFSET_MS, 17, 8, 1000)));
  }

  CASE("every camera 1..16 gets exactly one slot per 8 cycles, none colliding");
  {
    // Walk 8 cycles and check that at most two cameras are due at any instant,
    // and never two in the same bank.
    for (uint8_t hb = 0; hb < TALLY_TLM_CYCLES; hb++) {
      uint8_t dueBank0 = 0, dueBank1 = 0;
      for (uint8_t cam = 1; cam <= 16; cam++) {
        uint32_t bank = (uint32_t)(cam - 1) / TALLY_TLM_CYCLES;
        uint32_t at = 1000 + TALLY_TLM_OFFSET_MS + bank * TALLY_TLM_BANK_MS;
        if (tallySlotDue(slotIn(at, cam, hb, 1000))) {
          if (bank == 0) dueBank0++; else dueBank1++;
        }
      }
      CHECK_EQ(dueBank0, 1);
      CHECK_EQ(dueBank1, 1);
    }
  }

  CASE("a heartbeat scheduled inside the fleet's uplink window waits it out");
  {
    // A burst's final copy went out at t=1000: the fleet anchors on it and one
    // camera transmits telemetry inside [1000, 1000 + TALLY_TLM_WINDOW_MS).
    TallyBurst b; b.reset(0);
    b.onChange(1000, false);
    while (!b.idle()) b.markSent(1000);
    const uint32_t anchorTx = 1000;
    // The steady timer fires 100 ms later. The COUNTER advances (caller's job);
    // the FRAME must not go out until the window has closed.
    CHECK(b.scheduleHeartbeat(1100, anchorTx + TALLY_TLM_WINDOW_MS));
    CHECK(!b.dueNow(1100));
    CHECK(!b.dueNow(anchorTx + TALLY_TLM_WINDOW_MS - 1));
    CHECK(b.dueNow(anchorTx + TALLY_TLM_WINDOW_MS));
    CHECK(!b.latencyCritical()); // still an ordinary heartbeat: LBT may defer it
    CHECK(!b.flagAsBurst());     // and it anchors the next slot
    // A window that has already closed schedules immediately.
    TallyBurst c; c.reset(0);
    CHECK(c.scheduleHeartbeat(5000, 4000));
    CHECK(c.dueNow(5000));
    // The single-argument form is "now".
    TallyBurst d; d.reset(0);
    CHECK(d.scheduleHeartbeat(7000));
    CHECK(d.dueNow(7000));
    // Wrap-safe: notBefore may sit past the millis() rollover.
    TallyBurst e; e.reset(0);
    CHECK(e.scheduleHeartbeat(0xFFFFFF00u, 0xFFFFFF00u + 300));
    CHECK(!e.dueNow(0xFFFFFF00u));
    CHECK(e.dueNow(0xFFFFFF00u + 300));
    // A burst still running is never interrupted by a held heartbeat either.
    TallyBurst f; f.reset(0);
    f.onChange(2000, false);
    CHECK(!f.scheduleHeartbeat(2100, 2500));
    CHECK_EQ(f.pending(), TALLY_BURST_COPIES_MIN);
  }

  CASE("untilNext() reports the gap to the next copy, 0 when due or idle");
  {
    // The old-channel beacon uses this to slip between copies without ever
    // pushing one back — and never ahead of a cut's first copy.
    TallyBurst b; b.reset(0);
    CHECK_EQ(b.untilNext(0), 0u); // idle
    b.onChange(1000, false);
    CHECK_EQ(b.untilNext(1000), 0u); // first copy due now
    CHECK(b.latencyCritical());
    b.markSent(1000);
    CHECK_EQ(b.untilNext(1000), (unsigned)offs[1]);
    CHECK_EQ(b.untilNext(1000 + offs[1] - 10), 10u);
    CHECK_EQ(b.untilNext(1000 + offs[1] + 5), 0u); // overdue counts as due
    // wrap-safe
    TallyBurst c; c.reset(0);
    c.onChange(0xFFFFFFF0u, false);
    c.markSent(0xFFFFFFF0u);
    CHECK_EQ(c.untilNext(0xFFFFFFF0u), (unsigned)offs[1]);
    CHECK_EQ(c.untilNext(0xFFFFFFF0u + offs[1]), 0u); // past the rollover
  }

  CASE("the uplink window fits inside a heartbeat period with margin");
  {
    // If it did not, the held heartbeat would collide with the next one.
    CHECK(TALLY_TLM_WINDOW_MS < TALLY_REFRESH_MS);
    // ...and it really covers a bank-1 camera using its full late tolerance
    // plus its own frame, measured from the anchoring frame's START.
    CHECK(TALLY_TLM_WINDOW_MS >= TALLY_FRAME_AIRTIME_MS + TALLY_TLM_OFFSET_MS +
                                     TALLY_TLM_BANK_MS + TALLY_TLM_LATE_MS +
                                     TALLY_FRAME_AIRTIME_MS);
  }

  return testSummary("TallyBurst/TallyLbt");
}
