#include "TallyProtocol.h"

TallyProtocol::TallyProtocol() {
}

TallyPacket TallyProtocol::createSetStatePacket(uint8_t cameraId, TallyState state, uint8_t brightness) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_SET_STATE;
    packet.cameraId = cameraId;
    packet.state = state;
    packet.brightness = brightness;
    packet.reserved[0] = 0;
    packet.reserved[1] = 0;
    packet.crc = calculateCRC(packet);
    return packet;
}

TallyPacket TallyProtocol::createPingPacket(uint8_t cameraId) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_PING;
    packet.cameraId = cameraId;
    packet.state = STATE_OFF;
    packet.brightness = 0;
    packet.reserved[0] = 0;
    packet.reserved[1] = 0;
    packet.crc = calculateCRC(packet);
    return packet;
}

TallyPacket TallyProtocol::createHeartbeatPacket() {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_HEARTBEAT;
    packet.cameraId = TALLY_BROADCAST_ID;
    packet.state = STATE_OFF;
    packet.brightness = 0;
    packet.reserved[0] = 0;
    packet.reserved[1] = 0;
    packet.crc = calculateCRC(packet);
    return packet;
}

TallyPacket TallyProtocol::createAckPacket(uint8_t cameraId) {
    TallyPacket packet;
    packet.start = TALLY_START_BYTE;
    packet.command = CMD_ACK;
    packet.cameraId = cameraId;
    packet.state = STATE_OFF;
    packet.brightness = 0;
    packet.reserved[0] = 0;
    packet.reserved[1] = 0;
    packet.crc = calculateCRC(packet);
    return packet;
}

void TallyProtocol::serialize(const TallyPacket& packet, uint8_t* buffer) {
    // ⚡ Bolt: Rely on compiler intrinsics for fixed-size memory copies instead of manual loops
    memcpy(buffer, &packet, TALLY_PACKET_SIZE);
}

bool TallyProtocol::deserialize(const uint8_t* buffer, uint8_t len, TallyPacket& packet) {
    if (len < TALLY_PACKET_SIZE) {
        return false;
    }
    
    // ⚡ Bolt: Fast-path early return to skip memcpy and CRC overhead for noisy/invalid packets
    if (buffer[0] != TALLY_START_BYTE) {
        return false;
    }

    // ⚡ Bolt: Fast-path command validation before expensive memcpy and CRC calculation
    if (buffer[1] < CMD_SET_STATE || buffer[1] > CMD_HEARTBEAT) {
        return false;
    }

    // ⚡ Bolt: Rely on compiler intrinsics for fixed-size memory copies instead of manual loops
    memcpy(&packet, buffer, TALLY_PACKET_SIZE);

    return validate(packet);
}

bool TallyProtocol::validate(const TallyPacket& packet) {
    // Check start byte
    if (packet.start != TALLY_START_BYTE) {
        return false;
    }
    
    // ⚡ Bolt: Fast-path command validation before expensive CRC calculation
    // Validate command
    if (packet.command < CMD_SET_STATE || packet.command > CMD_HEARTBEAT) {
        return false;
    }
    
    // Check CRC
    if (packet.crc != calculateCRC(packet)) {
        return false;
    }
    
    return true;
}

uint8_t TallyProtocol::calculateCRC(const TallyPacket& packet) {
    static_assert(TALLY_PACKET_SIZE == 8, "Packet size changed, update CRC unrolling");
    const uint8_t* data = (const uint8_t*)&packet;
    
    // ⚡ Bolt: Manually unroll the XOR loop for 7 bytes to eliminate branching and loop overhead
    return data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
}

bool TallyProtocol::parseSerialCommand(const char* cmd, uint8_t& cameraId, TallyState& state) {
    // Expected format: T<id><state>
    // T1P = Camera 1 Preview
    // T1R = Camera 1 Program (Red)
    // T1O = Camera 1 Off
    // T2P = Camera 2 Preview
    // etc.
    
    // ⚡ Bolt: Fast-path O(1) length check avoids O(N) strlen traversal on potentially long inputs
    if (cmd == nullptr || cmd[0] == '\0' || cmd[1] == '\0' || cmd[2] == '\0') {
        return false;
    }
    
    // Check for 'T' prefix
    if (cmd[0] != 'T' && cmd[0] != 't') {
        return false;
    }
    
    // Parse camera ID (supports 1-9 for simple case)
    if (cmd[1] < '1' || cmd[1] > '9') {
        return false;
    }
    cameraId = cmd[1] - '0';
    
    // Parse state
    char stateChar = cmd[2];
    switch (stateChar) {
        case 'P':
        case 'p':
            state = STATE_PREVIEW;
            break;
        case 'R':
        case 'r':
            state = STATE_PROGRAM;
            break;
        case 'O':
        case 'o':
            state = STATE_OFF;
            break;
        case 'B':
        case 'b':
            state = STATE_BOTH;
            break;
        default:
            return false;
    }
    
    return true;
}
