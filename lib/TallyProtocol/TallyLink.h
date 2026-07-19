#ifndef TALLY_LINK_H
#define TALLY_LINK_H

#include "TallyProtocol.h"

// Shared receiver core for all slave firmwares: packet dispatch and link
// supervision live here once, so v1 and v2 can't drift on protocol behaviour.
// Firmwares supply only the presentation callbacks (LED / NeoPixel / buzzer).
//
// Two independent "don't trust the light" conditions are tracked:
//  - signalLost(): no valid packet for TALLY_SIGNAL_LOST_MS (radio link dead).
//  - sourceStale(): packets arrive but the hub flagged its tally source (ATEM)
//    as frozen/disconnected for longer than a short grace. Either one means the
//    displayed colour may be wrong — the slave shows a distinct indication.
class TallyLink {
public:
    typedef void (*StateCallback)(TallyState newState);
    typedef void (*LocatorCallback)();
    typedef void (*LinkCallback)(bool lost);
    typedef void (*ChannelCallback)(uint32_t freqHz, uint8_t chanIdx);
    typedef void (*PowerCallback)(int8_t powerDbm);

    void begin(uint8_t cameraId, StateCallback onState,
               LocatorCallback onLocator, LinkCallback onLink);

    // Optional (separate setter so v1's begin() signature stays stable):
    // fired on a valid CMD_SET_CHANNEL from the hub. The callback owns the
    // radio retune; TallyLink stays presentation/radio-agnostic.
    void setChannelCallback(ChannelCallback cb) { _onChannel = cb; }
    // Fired on a CMD_SET_POWER addressed at us (or broadcast): the hub's
    // AutoRF assigns our telemetry TX power. RAM-only by design.
    void setPowerCallback(PowerCallback cb) { _onPower = cb; }

    // Feed a raw RX buffer. Returns true for a valid packet on our network
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
    // We're missing heartbeats: >=3 gaps (>2x the hub's refresh) inside a
    // 30s counting window. Reported to the hub via TALLY_TLM_LINK_POOR so it
    // can tell "this camera struggles" apart from "the channel is bad".
    bool linkPoor() const { return _rxGaps >= 3; }
    uint32_t msSinceLastRx() const;

private:
    uint8_t _cameraId = 1;
    TallyState _state = STATE_OFF;
    bool _signalLost = false;
    bool _sourceStale = false;
    uint32_t _lastRxMs = 0;
    uint32_t _lastSourceLiveMs = 0;
    uint8_t _rxGaps = 0;          // heartbeat gaps in the current 30s window
    uint32_t _poorWindowStart = 0;
    StateCallback _onState = nullptr;
    LocatorCallback _onLocator = nullptr;
    LinkCallback _onLink = nullptr;
    ChannelCallback _onChannel = nullptr;
    PowerCallback _onPower = nullptr;
};

#endif // TALLY_LINK_H
