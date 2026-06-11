#include "TallyProtocol.h"

TallyProtocol::TallyProtocol() {
}

TallyPacket TallyProtocol::createStateAllPacket(uint16_t progMask, uint16_t prevMask) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_STATE_ALL;
    packet.netId = TALLY_NET_ID;
    packet.payload[0] = (uint8_t)(progMask & 0xFF);
    packet.payload[1] = (uint8_t)(progMask >> 8);
    packet.payload[2] = (uint8_t)(prevMask & 0xFF);
    packet.payload[3] = (uint8_t)(prevMask >> 8);
    packet.crc = calculateCRC(packet);
    return packet;
}

TallyPacket TallyProtocol::createPingPacket(uint8_t cameraId) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_PING;
    packet.netId = TALLY_NET_ID;
    packet.payload[0] = cameraId;
    packet.payload[1] = 0;
    packet.payload[2] = 0;
    packet.payload[3] = 0;
    packet.crc = calculateCRC(packet);
    return packet;
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

    // Fast-path rejections before memcpy + CRC: noise, foreign networks,
    // unknown commands
    if (buffer[0] != TALLY_START_BYTE) {
        return false;
    }
    if (buffer[2] != TALLY_NET_ID) {
        return false;
    }
    if (buffer[1] != CMD_PING && buffer[1] != CMD_STATE_ALL) {
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
    // Command whitelist (a range check would silently break when command
    // values become non-contiguous)
    if (packet.command != CMD_PING && packet.command != CMD_STATE_ALL) {
        return false;
    }
    if (packet.crc != calculateCRC(packet)) {
        return false;
    }
    return true;
}

uint8_t TallyProtocol::calculateCRC(const TallyPacket& packet) {
    // serialize()/deserialize() memcpy the struct as the wire format, so its
    // size must equal TALLY_PACKET_SIZE; the unrolled CRC below assumes 8 bytes.
    static_assert(sizeof(TallyPacket) == TALLY_PACKET_SIZE, "TallyPacket layout != wire size");
    static_assert(TALLY_PACKET_SIZE == 8, "Packet size changed, update CRC unrolling");
    const uint8_t* data = (const uint8_t*)&packet;
    return data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
}
