#include "TallyProtocol.h"

TallyProtocol::TallyProtocol() {
}

static TallyPacket makeFrame(uint8_t code, uint8_t aux, uint8_t p0, uint8_t p1,
                             uint8_t p2, uint8_t p3) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = TALLY_CMD_BYTE(code);
    packet.netId = TALLY_NET_ID;
    packet.aux = aux;
    packet.payload[0] = p0;
    packet.payload[1] = p1;
    packet.payload[2] = p2;
    packet.payload[3] = p3;
    packet.crc = TallyProtocol::calculateCRC(packet);
    return packet;
}

TallyPacket TallyProtocol::createStateAllPacket(uint16_t progMask, uint16_t prevMask,
                                                bool sourceLive) {
    uint8_t flags = sourceLive ? TALLY_FLAG_SOURCE_LIVE : 0;
    return makeFrame(CMD_STATE_ALL, flags,
                     (uint8_t)(progMask & 0xFF), (uint8_t)(progMask >> 8),
                     (uint8_t)(prevMask & 0xFF), (uint8_t)(prevMask >> 8));
}

TallyPacket TallyProtocol::createPingPacket(uint8_t cameraId) {
    return makeFrame(CMD_PING, cameraId, 0, 0, 0, 0);
}

TallyPacket TallyProtocol::createTelemetryPacket(uint8_t cameraId, uint16_t battMv,
                                                 int8_t rssi, uint8_t flags) {
    return makeFrame(CMD_TELEMETRY, cameraId,
                     (uint8_t)(battMv & 0xFF), (uint8_t)(battMv >> 8),
                     (uint8_t)rssi, flags);
}

TallyState TallyProtocol::stateForCamera(const TallyPacket& packet, uint8_t cameraId) {
    if (cameraId < 1 || cameraId > 16) {
        return STATE_OFF;
    }
    uint16_t progMask = (uint16_t)packet.payload[0] | ((uint16_t)packet.payload[1] << 8);
    uint16_t prevMask = (uint16_t)packet.payload[2] | ((uint16_t)packet.payload[3] << 8);
    uint16_t bit = 1U << (cameraId - 1);

    bool onAir = (progMask & bit) != 0;
    bool preview = (prevMask & bit) != 0;
    if (onAir && preview) return STATE_BOTH;
    if (onAir) return STATE_PROGRAM;
    if (preview) return STATE_PREVIEW;
    return STATE_OFF;
}

void TallyProtocol::serialize(const TallyPacket& packet, uint8_t* buffer) {
    memcpy(buffer, &packet, TALLY_PACKET_SIZE);
}

bool TallyProtocol::deserialize(const uint8_t* buffer, uint8_t len, TallyPacket& packet) {
    if (len < TALLY_PACKET_SIZE) {
        return false;
    }

    // Cheapest noise reject before the memcpy; everything else (netId, version,
    // command whitelist, CRC) is validate()'s job — one authority, so a new
    // command can't be whitelisted in one place and forgotten in the other.
    if (buffer[0] != TALLY_START_BYTE) {
        return false;
    }

    memcpy(&packet, buffer, TALLY_PACKET_SIZE);

    return validate(packet);
}

bool TallyProtocol::validate(const TallyPacket& packet) {
    if (packet.start != TALLY_START_BYTE) {
        return false;
    }
    if (packet.netId != TALLY_NET_ID) {
        return false;
    }
    // Fail closed on a protocol-version mismatch: an old/new node rejects the
    // frame and shows signal-lost rather than decoding a different layout.
    if (TALLY_CMD_VERSION(packet.command) != TALLY_PROTOCOL_VERSION) {
        return false;
    }
    uint8_t code = TALLY_CMD_CODE(packet.command);
    if (code != CMD_PING && code != CMD_STATE_ALL && code != CMD_TELEMETRY) {
        return false;
    }
    if (packet.crc != calculateCRC(packet)) {
        return false;
    }
    return true;
}

uint8_t TallyProtocol::calculateCRC(const TallyPacket& packet) {
    // serialize()/deserialize() memcpy the struct as the wire format, so its
    // size must equal TALLY_PACKET_SIZE.
    static_assert(sizeof(TallyPacket) == TALLY_PACKET_SIZE, "TallyPacket layout != wire size");
    // CRC-8/CCITT (poly 0x07, init 0x00) over the 8 header+payload bytes.
    // Far stronger than the old XOR: catches the multi-bit patterns that flip a
    // tally bitmask to a wrong-but-XOR-valid value.
    const uint8_t* data = (const uint8_t*)&packet;
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < TALLY_PACKET_SIZE - 1; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
