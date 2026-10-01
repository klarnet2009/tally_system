#include "TallyLink.h"

void TallyLink::begin(uint8_t cameraId, StateCallback onState,
                      LocatorCallback onLocator, LinkCallback onLink) {
    _cameraId = cameraId;
    _onState = onState;
    _onLocator = onLocator;
    _onLink = onLink;
    _lastRxMs = millis();
    _lastSourceLiveMs = millis();
    _poorWindowStart = millis();
}

bool TallyLink::onPacket(const uint8_t* buf, uint8_t len) {
    // ⚡ Bolt: Fast-path early return to avoid deserialization overhead for unaddressed ping packets
    if (len >= 3 && (buf[1] >> 4) == CMD_PING) {
        uint8_t target = buf[2];
        if (target != _cameraId && target != TALLY_BROADCAST_ID) {
            return false;
        }
    }

    TallyPacket pkt;
    if (!TallyProtocol::deserialize(buf, len, pkt)) {
        return false;
    }

    uint8_t code = TallyProtocol::cmd(pkt);

    if (code == CMD_TELEMETRY) {
        // Slave->hub frame: valid, but NOT link liveness. Only hub-originated
        // frames may refresh the link timer — counting peer telemetry masked
        // a dead hub behind beating slaves, and >=2 slaves stranded on a
        // wrong channel kept each other "alive" and never rescanned (livelock).
        return true;
    }

    // Hub-originated frame: the hub is alive. Count missed-heartbeat gaps for
    // the degradation gradient FIRST, while _lastRxMs still holds the old time.
    // Gated on _everHeard so the boot-to-first-packet interval isn't counted as
    // a gap (it would report a healthy link as degraded for the first 30s).
    uint32_t now = millis();
    if (_everHeard && now - _lastRxMs > 2 * TALLY_REFRESH_MS)
        _rxGaps++;
    _lastRxMs = now;
    _everHeard = true;
    if (_signalLost) {
        _signalLost = false;
        if (_onLink) _onLink(false);
    }

    if (code == CMD_PING) {
        uint8_t target = TallyProtocol::pingTarget(pkt);
        if (target == _cameraId || target == TALLY_BROADCAST_ID) {
            if (_onLocator) _onLocator();
        }
        return true;
    }

    if (code != CMD_STATE_ALL)
        return true;

    // ---- STATE_ALL: tally state, source freshness, channel plan, time base --
    _lastHbCount = TallyProtocol::hbCount(pkt);
    _hbSeen = true;
    if (TallyProtocol::isBurstCopy(pkt)) {
      _burstInFlight = true; // held until the burst's final (unflagged) copy
      _burstSinceMs = now;
    } else {
      _burstInFlight = false;
      // Anchors the telemetry slot. Safe on the final copy of a burst as well as
      // on a lone heartbeat, because in both cases nothing follows it.
      _lastHbAtMs = now;
    }

    if (TallyProtocol::sourceLive(pkt))
        _lastSourceLiveMs = now;

    // Channel plan: the countdown is in heartbeats. Convert to an absolute
    // instant and add a margin so clock skew makes us switch slightly LATE
    // (still covered by the hub's old-channel beacon) rather than early, which
    // would cost us the remaining announcements. Every announcing frame
    // refines the estimate, so losing any subset still leaves a usable one.
    uint8_t cd = TallyProtocol::chanCountdown(pkt);
    if (cd > 0 && _onChannel) {
        uint32_t at = now + (uint32_t)cd * TALLY_REFRESH_MS +
                      TALLY_CHAN_SWITCH_MARGIN_MS;
        _onChannel(TallyProtocol::chanIdx(pkt), at);
    }

    TallyState ts = TallyProtocol::stateForCamera(pkt, _cameraId);
    if (ts != _state) {
        _state = ts;
        if (_onState) _onState(ts);
    }
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
    // If the burst's releasing copy was itself lost, the hold must not stick:
    // no burst spans anywhere near two heartbeats.
    if (_burstInFlight && millis() - _burstSinceMs > 2 * TALLY_REFRESH_MS)
      _burstInFlight = false;
    // Gradient window: fixed 30s buckets (a hint for the hub, not a
    // measurement — boundary resets are fine under its sustained trigger).
    if (millis() - _poorWindowStart > 30000) {
        _poorWindowStart = millis();
        _rxGaps = 0;
    }
}

void TallyLink::noteAlive() {
    _lastRxMs = millis();
}

uint32_t TallyLink::msSinceLastRx() const {
    return millis() - _lastRxMs;
}
