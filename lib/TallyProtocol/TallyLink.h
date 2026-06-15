#ifndef TALLY_LINK_H
#define TALLY_LINK_H

#include "TallyProtocol.h"

// Shared receiver core for all slave firmwares: packet dispatch and link
// supervision live here once, so v1 and v2 can't drift on protocol behaviour.
// Firmwares supply only the presentation callbacks (LED / NeoPixel / buzzer).
class TallyLink {
public:
    typedef void (*StateCallback)(TallyState newState);
    typedef void (*LocatorCallback)();
    typedef void (*LinkCallback)(bool lost);

    void begin(uint8_t cameraId, StateCallback onState,
               LocatorCallback onLocator, LinkCallback onLink);

    // Feed a raw RX buffer. Returns true for a valid packet on our network
    // (callers may count failures for diagnostics).
    bool onPacket(const uint8_t* buf, uint8_t len);

    // Run the signal-lost timer; call every loop pass.
    void tick();

    // Reset the link timer without a packet (e.g. after a radio recovery).
    void noteAlive();

    // Bench/test override (serial commands); the next differing STATE_ALL
    // still fires the state callback.
    void forceState(TallyState s) { _state = s; }

    TallyState state() const { return _state; }
    bool signalLost() const { return _signalLost; }
    uint32_t msSinceLastRx() const;

private:
    uint8_t _cameraId = 1;
    TallyState _state = STATE_OFF;
    bool _signalLost = false;
    uint32_t _lastRxMs = 0;
    StateCallback _onState = nullptr;
    LocatorCallback _onLocator = nullptr;
    LinkCallback _onLink = nullptr;
};

#endif // TALLY_LINK_H
