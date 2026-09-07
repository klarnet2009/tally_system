#ifndef TALLY_LINK_H
#define TALLY_LINK_H

#include "TallyProtocol.h"

// Shared receiver core for all slave firmwares: packet dispatch and link
// supervision live here once, so the firmwares can't drift on protocol
// behaviour. Firmwares supply only the presentation callbacks (LED / NeoPixel /
// buzzer) and the radio retune.
//
// Two independent "don't trust the light" conditions are tracked:
//  - signalLost(): no hub frame for TALLY_SIGNAL_LOST_MS (radio link dead).
//  - sourceStale(): frames arrive but the hub flagged its tally source (ATEM)
//    as frozen/disconnected for longer than a short grace. Either one means the
//    displayed colour may be wrong — the slave shows a distinct indication.
class TallyLink {
public:
    typedef void (*StateCallback)(TallyState newState);
    typedef void (*LocatorCallback)();
    typedef void (*LinkCallback)(bool lost);
    // A coordinated switch was announced inside the heartbeat: retune to
    // `freqHz` (from our channel table via `chanIdx`) at `atMs`. The callback
    // owns the radio; TallyLink stays radio-agnostic.
    typedef void (*ChannelCallback)(uint8_t chanIdx, uint32_t atMs);

    void begin(uint8_t cameraId, StateCallback onState,
               LocatorCallback onLocator, LinkCallback onLink);

    // Optional: fired when a heartbeat carries a channel-switch countdown.
    // Called on EVERY announcing frame — each one refines the switch instant,
    // so losing any subset still leaves a usable estimate. Idempotent.
    void setChannelCallback(ChannelCallback cb) { _onChannel = cb; }

    // Feed a raw RX buffer. Returns true for a valid frame on our network
    // (callers may count failures for diagnostics).
    bool onPacket(const uint8_t* buf, uint8_t len);

    // Run the signal-lost / source-stale timers; call every loop pass.
    void tick();

    // Reset the link timer without a packet (e.g. after a radio recovery).
    void noteAlive();

    // Bench/test override (serial commands); the next differing STATE_ALL
    // still fires the state callback.
    void forceState(TallyState s) { _state = s; }

    TallyState state() const { return _state; }
    bool signalLost() const { return _signalLost; }
    bool sourceStale() const { return _sourceStale; }
    // No STATE frame for TALLY_STATE_STALE_MS although the link is alive (some
    // hub frame — possibly only a PING — kept the signal-lost timer fed).
    bool stateStale() const { return _stateStale; }
    // True only while the displayed colour may be trusted as current. Requires
    // the hub to have been heard at least once: before that _signalLost is still
    // false (its timer has simply not run out yet) but there is nothing to
    // trust — the slave has never been told a state. Both presentation gates
    // (paint a solid colour, allow a tone) hang off this one predicate.
    bool trustworthy() const {
        return _everHeard && !_signalLost && !_sourceStale && !_stateStale;
    }
    // Missed-heartbeat gradient (0..15) over a 30s window, reported to the hub
    // so it can tell "this camera struggles" apart from "the channel is bad",
    // and surface degradation to the operator BEFORE it becomes an outage.
    uint8_t missedBeats() const { return _rxGaps > 15 ? 15 : _rxGaps; }
    uint32_t msSinceLastRx() const;

    // Have we EVER heard the hub? A slave that never has (fresh boot, battery
    // swap while the fleet runs on an escape channel) should start scanning
    // immediately — there is no "spurious deaf spell" to sit through.
    bool everHeard() const { return _everHeard; }

    // ---- Telemetry slotting time base ----
    // The heartbeat carries a cycle counter, so the fleet shares a clock with no
    // extra protocol. A camera transmits only in its own cycle, at a fixed offset
    // after that HEARTBEAT arrived — burst copies update the cycle number but
    // deliberately NOT the anchor time, because they land at arbitrary offsets
    // from a cut and would drag the slot onto the hub's next copy.
    uint8_t lastHbCount() const { return _lastHbCount; }
    uint32_t lastHbAtMs() const { return _lastHbAtMs; }
    bool hbSeen() const { return _hbSeen; }
    // A change burst is in flight: the hub says so with a wire flag, and a slave
    // must not transmit telemetry into it. Latched on a burst copy and cleared by
    // the next lone heartbeat, so it covers the whole burst.
    bool burstInFlight() const { return _burstInFlight; }

private:
    uint8_t _cameraId = 1;
    TallyState _state = STATE_OFF;
    bool _signalLost = false;
    bool _sourceStale = false;
    bool _stateStale = false;
    bool _everHeard = false;
    uint32_t _lastRxMs = 0;
    uint32_t _lastStateMs = 0;    // last CMD_STATE_ALL, not just any hub frame
    uint32_t _lastSourceLiveMs = 0;
    uint8_t _rxGaps = 0;          // heartbeat gaps in the current 30s window
    uint32_t _poorWindowStart = 0;
    uint8_t _lastHbCount = 0;
    uint32_t _lastHbAtMs = 0;
    bool _burstInFlight = false;
    uint32_t _burstSinceMs = 0;
    bool _hbSeen = false;
    StateCallback _onState = nullptr;
    LocatorCallback _onLocator = nullptr;
    LinkCallback _onLink = nullptr;
    ChannelCallback _onChannel = nullptr;
};

#endif // TALLY_LINK_H
