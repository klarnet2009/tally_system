#include "TallyProtocol.h"

TallyProtocol::TallyProtocol() {
}

// ⚡ Bolt: Precomputed lookup table for CRC-8/CCITT (poly 0x07) to replace nested bitwise shifts in fast-path calculations
static const uint8_t CRC8_LUT[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
    0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
    0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
    0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
    0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
    0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
    0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
    0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
    0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
    0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
    0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
    0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

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

    // ⚡ Bolt: Fast-path CRC calculation using precomputed LUT to bypass O(N^2) inner bitwise loop overhead
    for (uint8_t i = 0; i < TALLY_PACKET_SIZE - 1; i++) {
        crc = CRC8_LUT[crc ^ data[i]];
    }
    return crc;
}
