#ifndef TALLY_PROTOCOL_H
#define TALLY_PROTOCOL_H

#include <Arduino.h>

// Protocol constants
#define TALLY_START_BYTE    0xAA
#define TALLY_PACKET_SIZE   8
#define TALLY_BROADCAST_ID  0xFF

// Commands
enum TallyCommand : uint8_t {
    CMD_SET_STATE   = 0x01,
    CMD_PING        = 0x02,
    CMD_CONFIG      = 0x03,
    CMD_ACK         = 0x04,
    CMD_HEARTBEAT   = 0x05
};

// Camera states
enum TallyState : uint8_t {
    STATE_OFF       = 0x00,
    STATE_PREVIEW   = 0x01,  // Green
    STATE_PROGRAM   = 0x02,  // Red
    STATE_BOTH      = 0x03   // Both preview and program (rare)
};

// Packet structure (8 bytes)
#pragma pack(push, 1)
struct TallyPacket {
    uint8_t start;       // Start marker: 0xAA
    uint8_t command;     // Command type
    uint8_t cameraId;    // Camera ID (1-254, 0xFF = broadcast)
    uint8_t state;       // Tally state
    uint8_t brightness;  // LED brightness (0-255)
    uint8_t reserved[2]; // Reserved for future use
    uint8_t crc;         // XOR checksum
};
#pragma pack(pop)

class TallyProtocol {
public:
    TallyProtocol();
    
    // Create packets
    static TallyPacket createSetStatePacket(uint8_t cameraId, TallyState state, uint8_t brightness = 255);
    static TallyPacket createPingPacket(uint8_t cameraId);
    static TallyPacket createHeartbeatPacket();
    static TallyPacket createAckPacket(uint8_t cameraId);
    
    // Serialize/deserialize
    static void serialize(const TallyPacket& packet, uint8_t* buffer);
    static bool deserialize(const uint8_t* buffer, uint8_t len, TallyPacket& packet);
    
    // Validation
    static bool validate(const TallyPacket& packet);
    static uint8_t calculateCRC(const TallyPacket& packet);
    
    // Parse serial commands (format: T1P, T1R, T1O, T2P, etc.)
    static bool parseSerialCommand(const char* cmd, uint8_t& cameraId, TallyState& state);
};

#endif // TALLY_PROTOCOL_H
