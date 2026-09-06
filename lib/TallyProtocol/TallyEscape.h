#ifndef TALLY_ESCAPE_H
#define TALLY_ESCAPE_H

#include <Arduino.h>

#include "TallyConfig.h"

// Channel-escape policy and coordinated-switch bookkeeping, extracted from the
// hub so both can be exercised on the host with a controllable clock.
//
// This extraction is the point, not a side effect: four of the five defects the
// audit found lived in this logic while it sat in main.cpp/autorf.h, where no
// test could reach it. Everything here is pure — no radio, no globals, no I/O.
//
// ---- Why the trigger looks the way it does ----
// The first version counted "camera unreachable" as evidence that the CHANNEL
// was bad. It is not: a camera can be unreachable because it is switched off,
// out of battery, out of range, or because the channel died. Since
// g_camLastSeen was never cleared, a fleet powered down for a break read as
// 100% failure and migrated the whole system through a blackout every few
// minutes, mid-show, overriding the on-air guard on the way.
//
// So the primary signal is only cameras that ARE talking to us AND report
// missing heartbeats. That is unambiguous: the frame proves the uplink works,
// and its missed-beat count is a direct measurement of the downlink — the
// safety-critical direction.
//
// The one genuinely ambiguous case, "everything went silent at once", still
// needs an answer or a dead channel would deadlock the fleet (slaves scan, hub
// stays put). It gets its own tier: a long sustain, no on-air override, and it
// requires CORROBORATION from the channel-busy measurement. That is the correct
// use of a measurement that must never drive a decision on its own — a quiet
// channel with silent cameras means the cameras are off, not that the air is
// hostile.
#define ESC_POOR_MISSED 3          // a talking camera missing this many beats
#define ESC_DEGRADED_FRAC 0.30f    // share of talking cameras complaining
#define ESC_DEGRADED_MIN_CAMS 2    // ...or this many, whatever the fleet size
// 0.65 so 2-of-3 and 3-of-4 both count: at that point the fleet is effectively
// down, and "severe" is also exactly the condition allowed to override the
// on-air guard — one threshold, one meaning, no second constant to drift.
#define ESC_SEVERE_FRAC 0.65f
// ...but severe ALSO needs at least two cameras agreeing. With one camera the
// fraction is trivially 1.0, so a single unit behind a wall or at the range edge
// would reach the one tier that overrides the on-air guard and migrate the whole
// fleet every 90 s. One complaining camera is a camera problem until a second
// camera corroborates it; it still raises DEGRADED, which respects the guard.
#define ESC_SEVERE_MIN_CAMS 2
#define ESC_BLACKOUT_BUSY 0.25f    // corroboration for "everyone went silent"

#define ESC_DEGRADED_SUSTAIN_MS 30000UL // partial: do not spend an outage on a blip
#define ESC_SEVERE_SUSTAIN_MS 6000UL    // almost all complaining: waiting is loss
#define ESC_BLACKOUT_SUSTAIN_MS 60000UL // ambiguous: be very sure

// Dwell protects against churn from a NOISY trigger. The trigger above is much
// harder to fool, so severe degradation may move quickly — otherwise a venue
// whose APs cover the first channels keeps the fleet on them for half an hour
// while stepping through the list.
#define ESC_DWELL_MS (10UL * 60 * 1000)
#define ESC_DWELL_SEVERE_MS 90000UL
#define ESC_MAX_SWITCHES_HOUR 8
#define ESC_FIRST_SWITCH_MS 60000UL // a venue fills up inside 10 minutes
// Within ONE episode of degradation each channel is tried at most once; when
// they have all been tried we STAY and say so, because migrating again cannot
// help and every migration costs an outage. A purely time-based penalty could
// not express this: with a 20-minute window and a 10-minute dwell the penalty
// expired mid-rotation, so a fleet on four hostile channels circled forever.
// The episode set resets when degradation actually clears; the short time guard
// below only stops us bouncing straight back in a NEW episode.
#define ESC_RECENT_MS (5UL * 60 * 1000)

// One camera's facts, as the hub knows them.
struct EscCam {
  bool everSeen = false;  // has ever sent telemetry (never cleared, by design)
  bool reachable = false; // telemetry arrived recently
  uint8_t missed = 0;     // its own missed-heartbeat gradient, 0..15
};

enum EscTier : uint8_t { ESC_NONE, ESC_DEGRADED, ESC_SEVERE, ESC_BLACKOUT };

class TallyEscape {
public:
  struct Inputs {
    uint32_t now = 0;
    uint8_t curChan = 0;
    bool onAir = false;          // any camera in PROGRAM right now
    bool switchInFlight = false; // an announced switch is already running
    bool busyValid = false;      // channel-busy measurement is meaningful
    float busyFrac = 0.0f;
  };
  struct Verdict {
    // Published to the transmit scheduler: extra burst copies while the fleet
    // struggles. True for every tier, including blackout.
    bool degraded = false;
    EscTier tier = ESC_NONE;
    bool wantSwitch = false;   // policy says move
    uint8_t target = 0xFF;     // where to (valid when wantSwitch)
    const char *reason = "";
    const char *blocked = nullptr; // wanted to move but a limit said no
    // Diagnostics
    uint8_t talking = 0;
    uint8_t complaining = 0;
    uint8_t silent = 0;
  };

  void begin(uint32_t now) {
    // Two independent gates, because one cannot express the intent:
    //  - _earliestSwitch is an ABSOLUTE floor. Encoding the floor as a pre-aged
    //    dwell alone was wrong: pre-aging by (ESC_DWELL_MS - ESC_FIRST_SWITCH_MS)
    //    = 540 s already exceeded the 90 s severe dwell, so the severe tier was
    //    exempt from the floor entirely and could migrate the fleet ~10 s after a
    //    power-cycle — on telemetry whose missed-beat window still held gaps from
    //    that very reboot.
    //  - _lastSwitch is aged by a FULL dwell so the dwell never blocks the first
    //    switch; the floor above is what governs it. Unsigned modular arithmetic
    //    makes the apparent underflow behave correctly.
    _earliestSwitch = now + ESC_FIRST_SWITCH_MS;
    _lastSwitch = now - ESC_DWELL_MS;
    _hourStart = now;
    _switchesThisHour = 0;
    _tierSince = 0;
    _tier = ESC_NONE;
    _triedMask = 0;
    for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++)
      _penaltyAt[i] = 0;
  }

  Verdict evaluate(const Inputs &in, const EscCam *cams /* [17] */) {
    Verdict v;
    if (in.now - _hourStart > 3600000UL) {
      _hourStart = in.now;
      _switchesThisHour = 0;
    }

    // ---- Classify ----
    for (uint8_t id = 1; id <= 16; id++) {
      if (!cams[id].everSeen)
        continue;
      if (cams[id].reachable) {
        v.talking++;
        if (cams[id].missed >= ESC_POOR_MISSED)
          v.complaining++;
      } else {
        v.silent++;
      }
    }

    float frac = v.talking ? (float)v.complaining / (float)v.talking : 0.0f;
    EscTier tier = ESC_NONE;
    if (v.complaining >= ESC_SEVERE_MIN_CAMS && frac >= ESC_SEVERE_FRAC) {
      tier = ESC_SEVERE;
    } else if (v.complaining >= ESC_DEGRADED_MIN_CAMS ||
               (v.talking >= 1 && frac >= ESC_DEGRADED_FRAC)) {
      // Absolute count OR fraction. The fraction alone misses a large fleet: on
      // 10 cameras, two losing frames is 20% and would raise nothing at all — no
      // extra burst copies for the cameras that are actually suffering, and no
      // gradient for the operator. Removing the old `talking >= 3` gate was
      // right; dropping the absolute trigger with it was not.
      tier = ESC_DEGRADED;
    } else if (v.talking == 0 && v.silent >= 1 && in.busyValid &&
               in.busyFrac >= ESC_BLACKOUT_BUSY) {
      // Everyone went quiet AND the air is genuinely busy. A quiet channel with
      // silent cameras is a break, not interference — and gets no tier at all.
      tier = ESC_BLACKOUT;
    }

    v.tier = tier;
    v.degraded = (tier != ESC_NONE);

    if (tier == ESC_NONE) {
      _tierSince = 0;
      _tier = ESC_NONE;
      _triedMask = 0; // the episode is over: everything may be tried again
      return v;
    }
    // A tier change restarts the clock: escalating from degraded to severe must
    // not inherit the slower tier's elapsed time, and de-escalating must not
    // keep the faster one's.
    if (tier != _tier) {
      _tier = tier;
      _tierSince = in.now;
    }

    uint32_t needed = ESC_DEGRADED_SUSTAIN_MS;
    if (tier == ESC_SEVERE)
      needed = ESC_SEVERE_SUSTAIN_MS;
    else if (tier == ESC_BLACKOUT)
      needed = ESC_BLACKOUT_SUSTAIN_MS;
    if (in.now - _tierSince < needed)
      return v;

    if (in.switchInFlight) {
      v.blocked = "switch-in-progress";
      return v;
    }

    // A switch costs a live camera an outage only if that camera hears the
    // announcements badly — which is what DEGRADED means, so a mildly degraded
    // fleet stays put while anyone is on air. SEVERE moves anyway: staying is
    // clearly worse. BLACKOUT is not held back either: nobody is talking to us,
    // so there is no reachable camera whose outage the guard could protect. A
    // slave that still hears us follows the coordinated switch with no gap; one
    // that does not is already dark, and the beacon plus its scan are the way
    // back. Holding blackout here kept the fleet on a dead channel for as long
    // as the ATEM said someone was live — at a concert, the whole show.
    if (in.onAir && tier == ESC_DEGRADED) {
      v.blocked = "camera on air";
      return v;
    }

    uint8_t target = pickTarget(in.now, in.curChan);
    if (target == 0xFF) {
      v.blocked = "every channel already tried this episode";
      return v;
    }

    if ((int32_t)(in.now - _earliestSwitch) < 0) {
      v.blocked = "settling after boot";
      return v;
    }
    uint32_t dwell = (tier == ESC_SEVERE) ? ESC_DWELL_SEVERE_MS : ESC_DWELL_MS;
    if (in.now - _lastSwitch < dwell) {
      v.blocked = "min-dwell";
      return v;
    }
    if (_switchesThisHour >= ESC_MAX_SWITCHES_HOUR) {
      v.blocked = "rate-limit";
      return v;
    }

    v.wantSwitch = true;
    v.target = target;
    v.reason = (tier == ESC_SEVERE)     ? "downlink SEVERE"
               : (tier == ESC_BLACKOUT) ? "fleet silent + air busy"
                                        : "downlink degraded";
    return v;
  }

  // Call when a switch has actually been requested.
  void noteSwitching(uint32_t now, uint8_t fromChan) {
    if (fromChan < TALLY_CHAN_COUNT) {
      _penaltyAt[fromChan] = now ? now : 1; // 0 means "never", keep it reserved
      _triedMask |= (uint16_t)(1u << fromChan);
    }
    _lastSwitch = now;
    _switchesThisHour++;
    _tierSince = 0;
    _tier = ESC_NONE;
  }

  // Diagnostics
  bool penalised(uint32_t now, uint8_t ch) const {
    if (ch >= TALLY_CHAN_COUNT)
      return false;
    if (_triedMask & (uint16_t)(1u << ch))
      return true; // already tried in this episode
    return _penaltyAt[ch] != 0 && (now - _penaltyAt[ch]) < ESC_RECENT_MS;
  }
  uint8_t switchesThisHour() const { return _switchesThisHour; }
  EscTier tier() const { return _tier; }

private:
  // Least-recently-abandoned channel that isn't the current one. No score and
  // no measurement: we do not claim to know which channel is better, only that
  // this one is bad. If the next is bad too, the same trigger moves on.
  uint8_t pickTarget(uint32_t now, uint8_t cur) const {
    uint8_t best = 0xFF;
    uint32_t bestAge = 0;
    for (uint8_t i = 0; i < TALLY_CHAN_COUNT; i++) {
      if (i == cur)
        continue;
      if (penalised(now, i))
        continue; // tried this episode, or abandoned in the last few minutes
      uint32_t age = (_penaltyAt[i] == 0) ? 0xFFFFFFFFUL : (now - _penaltyAt[i]);
      if (best == 0xFF || age > bestAge) {
        bestAge = age;
        best = i;
      }
    }
    return best; // 0xFF = nowhere left to go; staying beats another outage
  }

  uint32_t _penaltyAt[TALLY_CHAN_COUNT] = {0};
  uint16_t _triedMask = 0; // channels tried in the CURRENT degradation episode
  uint32_t _lastSwitch = 0;
  uint32_t _earliestSwitch = 0;
  uint32_t _hourStart = 0;
  uint32_t _tierSince = 0;
  EscTier _tier = ESC_NONE;
  uint8_t _switchesThisHour = 0;
};

// ===== Coordinated switch: countdown derived, never decremented =====
// The first implementation decremented a counter in the heartbeat tick while the
// frame that carries it is serialized LATER, in the transmit pump. So N
// announced beats put only N-1 usable countdowns on air, and beats==1 put 0 —
// which the protocol defines as "no switch pending", meaning the hub retuned
// with ZERO announcements and abandoned the fleet on the old channel.
//
// Deriving the countdown from a target beat number removes the ordering hazard
// entirely: there is nothing to decrement, so nothing can be read at the wrong
// moment.
class TallyChanSwitch {
public:
  void request(uint8_t targetChan, uint8_t hbNow, uint8_t beats) {
    if (beats < 2)
      beats = 2; // 1 would derive countdown 0 = "no switch pending"
    // 14, not 15: the hub requests against the NEXT cycle, so the first
    // countdown on air is beats+1 and 16 would read as "already expired" —
    // retuning with zero announcements, the exact failure this class prevents.
    if (beats > 14)
      beats = 14;
    _target = targetChan;
    _dueHb = (uint8_t)(hbNow + beats);
    _pending = true;
  }
  bool pending() const { return _pending; }
  uint8_t target() const { return _target; }

  // What this frame should announce. 0 = nothing pending (also what a receiver
  // reads as "no switch"), so a live announcement is always >= 1.
  uint8_t countdown(uint8_t hbNow) const {
    if (!_pending)
      return 0;
    uint8_t left = (uint8_t)(_dueHb - hbNow); // modular: survives the wrap
    if (left > 15)
      return 0; // the deadline has passed
    return left;
  }
  bool expired(uint8_t hbNow) const { return _pending && countdown(hbNow) == 0; }
  void clear() { _pending = false; }

private:
  bool _pending = false;
  uint8_t _target = 0;
  uint8_t _dueHb = 0;
};

#endif // TALLY_ESCAPE_H
