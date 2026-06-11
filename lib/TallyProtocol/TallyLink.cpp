#include "TallyLink.h"

void TallyLink::begin(uint8_t cameraId, StateCallback onState,
                      LocatorCallback onLocator, LinkCallback onLink) {
    _cameraId = cameraId;
    _onState = onState;
    _onLocator = onLocator;
    _onLink = onLink;
    _lastRxMs = millis();
}

bool TallyLink::onPacket(const uint8_t* buf, uint8_t len) {
    TallyPacket pkt;
    if (!TallyProtocol::deserialize(buf, len, pkt)) {
        return false;
    }

    // Any valid packet from our hub proves the link is alive
    _lastRxMs = millis();
    if (_signalLost) {
        _signalLost = false;
        if (_onLink) _onLink(false);
    }

    if (pkt.command == CMD_PING) {
        if (pkt.payload[0] == _cameraId ||
            pkt.payload[0] == TALLY_BROADCAST_ID) {
            if (_onLocator) _onLocator();
        }
    } else if (pkt.command == CMD_STATE_ALL) {
        TallyState ts = TallyProtocol::stateForCamera(pkt, _cameraId);
        if (ts != _state) {
            _state = ts;
            if (_onState) _onState(ts);
        }
    }
    return true;
}

void TallyLink::tick() {
    if (!_signalLost && millis() - _lastRxMs > TALLY_SIGNAL_LOST_MS) {
        _signalLost = true;
        if (_onLink) _onLink(true);
    }
}

void TallyLink::noteAlive() {
    _lastRxMs = millis();
}

uint32_t TallyLink::msSinceLastRx() const {
    return millis() - _lastRxMs;
}
