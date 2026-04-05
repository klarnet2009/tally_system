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

    memcpy(&packet, buffer, TALLY_PACKET_SIZE);
    return validate(packet);
}

bool TallyProtocol::validate(const TallyPacket& packet) {
    // Check start byte
    if (packet.start != TALLY_START_BYTE) {
        return false;
    }
    
    // Check CRC
    if (packet.crc != calculateCRC(packet)) {
        return false;
    }
    
    // Validate command
    if (packet.command < CMD_SET_STATE || packet.command > CMD_HEARTBEAT) {
        return false;
    }
    
    return true;
}

uint8_t TallyProtocol::calculateCRC(const TallyPacket& packet) {
    const uint8_t* data = (const uint8_t*)&packet;
    uint8_t crc = 0;
    
    // XOR all bytes except the CRC byte itself
    for (uint8_t i = 0; i < TALLY_PACKET_SIZE - 1; i++) {
        crc ^= data[i];
    }
    
    return crc;
}

bool TallyProtocol::parseSerialCommand(const char* cmd, uint8_t& cameraId, TallyState& state) {
    // Expected format: T<id><state>
    // T1P = Camera 1 Preview
    // T1R = Camera 1 Program (Red)
    // T1O = Camera 1 Off
    // T2P = Camera 2 Preview
    // etc.
    
    if (cmd == nullptr || strlen(cmd) < 3) {
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
