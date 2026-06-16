#include "TallyLink.h"

void TallyLink::begin(uint8_t cameraId, StateCallback onState,
                      LocatorCallback onLocator, LinkCallback onLink) {
    _cameraId = cameraId;
    _onState = onState;
    _onLocator = onLocator;
    _onLink = onLink;
    _lastRxMs = millis();
    _lastSourceLiveMs = millis();
}

bool TallyLink::onPacket(const uint8_t* buf, uint8_t len) {
    TallyPacket pkt;
    if (!TallyProtocol::deserialize(buf, len, pkt)) {
        return false;
    }

    // Any valid packet from our hub proves the radio link is alive
    _lastRxMs = millis();
    if (_signalLost) {
        _signalLost = false;
        if (_onLink) _onLink(false);
    }

    uint8_t code = TallyProtocol::cmdCode(pkt);
    if (code == CMD_PING) {
        if (pkt.aux == _cameraId || pkt.aux == TALLY_BROADCAST_ID) {
            if (_onLocator) _onLocator();
        }
    } else if (code == CMD_STATE_ALL) {
        // Track the hub's tally-source freshness (set in tick()'s grace timer)
        if (TallyProtocol::sourceLive(pkt))
            _lastSourceLiveMs = millis();

        TallyState ts = TallyProtocol::stateForCamera(pkt, _cameraId);
        if (ts != _state) {
            _state = ts;
            if (_onState) _onState(ts);
        }
    }
    // CMD_TELEMETRY is a slave->hub frame; a slave ignores it.
    return true;
}

void TallyLink::tick() {
    if (!_signalLost && millis() - _lastRxMs > TALLY_SIGNAL_LOST_MS) {
        _signalLost = true;
        if (_onLink) _onLink(true);
    }
    // Source-stale: the hub kept sending (link alive) but flagged its tally
    // source frozen for longer than the grace window. Riding out brief ATEM
    // reconnects, this avoids flicker while still catching a real freeze.
    _sourceStale = (millis() - _lastSourceLiveMs > TALLY_SOURCE_GRACE_MS);
}

void TallyLink::noteAlive() {
    _lastRxMs = millis();
}

uint32_t TallyLink::msSinceLastRx() const {
    return millis() - _lastRxMs;
}
