#ifndef TALLY_BURST_H
#define TALLY_BURST_H

#include <Arduino.h>

#include "TallyConfig.h"

// Transmit-scheduling arithmetic, extracted from the hub's main loop so it can
// be exercised on the host with a controllable clock. It touches no radio and no
// globals: it only answers "is a copy due, and is it the latency-critical one?".
//
// The invariant this class exists to protect: a copy carries whatever the state
// is AT TRANSMIT TIME, never a queued snapshot. So the scheduler stores a repeat
// COUNT and never a packet — v3's packet FIFO could put an already-superseded
// state on air (up to ~290 ms of visibly wrong colour after a fast second cut).
class TallyBurst {
public:
  void reset(uint32_t now) {
    _idx = 0;
    _copies = 0;
    _startMs = now;
    _nextAtMs = now;
  }

  // A tally change. Supersedes any copies still pending: because every copy
  // re-reads the live state, the ones already sent were correct for their moment
  // and the ones not yet sent now carry the new state.
  // `degraded` only ever ADDS copies — a scheme that can reduce margin is a
  // scheme where a measurement error costs a wrong light.
  void onChange(uint32_t now, bool degraded) {
    _idx = 0;
    _copies = degraded ? TALLY_BURST_COPIES_MAX : TALLY_BURST_COPIES_MIN;
    _startMs = now;
    _nextAtMs = now;
  }

  // Periodic re-send. Never interrupts a burst in progress: the burst's copies
  // carry the same state, so they already serve as the heartbeat.
  // `notBefore` holds the FRAME back until the fleet's uplink window after the
  // previous anchoring frame has closed (TALLY_TLM_WINDOW_MS). Only the frame:
  // the cycle counter belongs to the caller and advances on its own steady
  // timer, so this delay can never freeze it — the failure mode that made an
  // earlier "gate the counter on the frame" design deadlock channel switches.
  // Returns true when a heartbeat copy was actually scheduled.
  bool scheduleHeartbeat(uint32_t now, uint32_t notBefore) {
    if (!idle())
      return false;
    _idx = 0;
    _copies = 1;
    _startMs = ((int32_t)(notBefore - now) > 0) ? notBefore : now;
    _nextAtMs = _startMs;
    return true;
  }
  bool scheduleHeartbeat(uint32_t now) { return scheduleHeartbeat(now, now); }

  bool idle() const { return _idx >= _copies; }
  // Does the copy about to be sent carry TALLY_FLAG_BURST?
  //
  // Every copy of a multi-copy group EXCEPT THE LAST. The exception is
  // load-bearing: the flag tells a receiver to hold telemetry off and not to
  // re-anchor its slot, and something has to release both. A lone heartbeat used
  // to do that, but heartbeats are suppressed while a burst runs — so cuts
  // arriving faster than the burst span starved them completely, latching the
  // hold on and freezing the anchor until the hub declared the whole fleet
  // offline. The final copy is the natural release point: nothing follows it, so
  // anchoring on it cannot aim an uplink at a later copy.
  bool flagAsBurst() const { return _copies > 1 && _idx + 1 < _copies; }
  uint8_t pending() const { return idle() ? 0 : (uint8_t)(_copies - _idx); }

  // A copy is due for transmission now.
  bool dueNow(uint32_t now) const {
    return !idle() && (int32_t)(now - _nextAtMs) >= 0;
  }
  // Milliseconds until the next copy is due (0 when due or idle). Lets a lower-
  // priority transmit (the old-channel beacon) take the air between copies
  // without ever pushing one back.
  uint32_t untilNext(uint32_t now) const {
    if (idle())
      return 0;
    int32_t d = (int32_t)(_nextAtMs - now);
    return d > 0 ? (uint32_t)d : 0;
  }

  // The first copy of a multi-copy burst is the one whose latency IS the point;
  // it must never be delayed for collision avoidance.
  bool latencyCritical() const { return _idx == 0 && _copies > 1; }

  void markSent(uint32_t /*now*/) {
    if (idle())
      return;
    _idx++;
    if (!idle())
      _nextAtMs = _startMs + kOffsets()[_idx];
  }

  // Abandon the remaining copies (radio went away).
  void abandon() { _idx = _copies; }

private:
  // Table and its invariants live in TallyConfig.h next to the values, where the
  // static_asserts check the ARRAY rather than a hand-maintained #define.
  static const uint16_t *kOffsets() { return kTallyBurstOffsets; }
  uint8_t _idx = 0;
  uint8_t _copies = 0;
  uint32_t _startMs = 0;
  uint32_t _nextAtMs = 0;
};

// Listen-before-talk gate. One instance PER TRANSMIT CLASS: sharing a single
// deferral clock between the state burst and the one-shot queue let one frame
// consume the other's patience, so a copy could be pushed into a busy channel
// immediately. Separate instances make that structurally impossible.
class TallyLbt {
public:
  // true = transmit now. Defers at most TALLY_LBT_MAX_DEFER_MS, because a
  // permanently busy channel must not be allowed to silence the link.
  bool clear(uint32_t now, bool latencyCritical, bool channelBusy) {
    if (latencyCritical || !channelBusy) {
      _deferSince = 0;
      return true;
    }
    if (_deferSince == 0)
      _deferSince = now;
    if (now - _deferSince >= TALLY_LBT_MAX_DEFER_MS) {
      _deferSince = 0;
      return true; // out of patience: send anyway
    }
    return false;
  }
  void reset() { _deferSince = 0; }
  bool deferring() const { return _deferSince != 0; }

private:
  uint32_t _deferSince = 0;
};

// ===== Telemetry slot decision =====
// Pure, so the hub/slave coupling that produced every regression of the last
// three review passes is finally testable: cycle number vs slot anchor, the
// burst hold, the late-window, and the once-per-cycle rule all interact here and
// nowhere else.
struct TallySlotInputs {
  uint32_t now = 0;
  uint8_t camId = 0;
  bool hbSeen = false;
  uint8_t hbCount = 0;      // cycle number from the latest hub frame
  uint32_t hbAtMs = 0;      // arrival of the frame that ANCHORS the slot
  bool burstInFlight = false;
  bool sentThisCycle = false; // already transmitted for hbCount
};

static inline bool tallySlotDue(const TallySlotInputs &in) {
  if (!in.hbSeen || in.camId < 1 || in.camId > 16)
    return false;
  if (in.burstInFlight)
    return false; // never transmit into a change burst
  if (in.sentThisCycle)
    return false;
  uint8_t slot = (uint8_t)((in.camId - 1) % TALLY_TLM_CYCLES);
  if ((uint8_t)(in.hbCount % TALLY_TLM_CYCLES) != slot)
    return false; // not our cycle
  uint32_t bank = (uint32_t)(in.camId - 1) / TALLY_TLM_CYCLES;
  uint32_t dueAt = in.hbAtMs + TALLY_TLM_OFFSET_MS + bank * TALLY_TLM_BANK_MS;
  if ((int32_t)(in.now - dueAt) < 0)
    return false; // slot has not opened
  if (in.now - dueAt > TALLY_TLM_LATE_MS)
    return false; // too late: would run into the next bank's slot
  return true;
}

#endif // TALLY_BURST_H
