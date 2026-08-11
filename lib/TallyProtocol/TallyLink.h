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
    // True whenever the displayed colour must not be trusted as current.
    bool trustworthy() const { return !_signalLost && !_sourceStale; }
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
    // The heartbeat carries a cycle counter, so the fleet shares a clock with
    // no extra protocol. A camera transmits only in its own cycle, at a fixed
    // offset after that heartbeat ARRIVED here.
    uint8_t lastHbCount() const { return _lastHbCount; }
    uint32_t lastHbAtMs() const { return _lastHbAtMs; }
    bool hbSeen() const { return _hbSeen; }
    // How many hub frames carried the CURRENT cycle number. The hub advances the
    // cycle only when it schedules a heartbeat, so >1 means a change burst is in
    // flight — and a slave must not transmit telemetry into it. This is the only
    // way a receiver can tell a burst from a lone heartbeat.
    uint8_t framesThisCycle() const { return _framesThisHb; }

private:
    uint8_t _cameraId = 1;
    TallyState _state = STATE_OFF;
    bool _signalLost = false;
    bool _sourceStale = false;
    bool _everHeard = false;
    uint32_t _lastRxMs = 0;
    uint32_t _lastSourceLiveMs = 0;
    uint8_t _rxGaps = 0;          // heartbeat gaps in the current 30s window
    uint32_t _poorWindowStart = 0;
    uint8_t _lastHbCount = 0;
    uint32_t _lastHbAtMs = 0;
    uint8_t _framesThisHb = 0;
    bool _hbSeen = false;
    StateCallback _onState = nullptr;
    LocatorCallback _onLocator = nullptr;
    LinkCallback _onLink = nullptr;
    ChannelCallback _onChannel = nullptr;
};

#endif // TALLY_LINK_H
