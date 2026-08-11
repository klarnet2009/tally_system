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
  // Returns true when a heartbeat copy was actually scheduled.
  bool scheduleHeartbeat(uint32_t now) {
    if (!idle())
      return false;
    _idx = 0;
    _copies = 1;
    _startMs = now;
    _nextAtMs = now;
    return true;
  }

  bool idle() const { return _idx >= _copies; }
  uint8_t pending() const { return idle() ? 0 : (uint8_t)(_copies - _idx); }

  // A copy is due for transmission now.
  bool dueNow(uint32_t now) const {
    return !idle() && (int32_t)(now - _nextAtMs) >= 0;
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
  static const uint16_t *kOffsets() {
    static const uint16_t offs[] = TALLY_BURST_OFFSETS_MS;
    // Sized from the initializer, then checked — declaring it
    // [TALLY_BURST_COPIES_MAX] instead let C++ zero-fill any missing entries, so
    // raising the copy count without extending the list gave the extra copies
    // offset 0: they fired back-to-back, one interference burst erased both, and
    // the time diversity the class exists for vanished with no compile error and
    // no runtime symptom.
    static_assert(sizeof(offs) / sizeof(offs[0]) == TALLY_BURST_COPIES_MAX,
                  "TALLY_BURST_OFFSETS_MS must have TALLY_BURST_COPIES_MAX entries");
    static_assert(TALLY_BURST_OFFSETS_FIRST_IS_ZERO,
                  "the first burst copy must be due immediately");
    return offs;
  }
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

#endif // TALLY_BURST_H
