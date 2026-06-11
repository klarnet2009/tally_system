#ifndef TALLY_PROTOCOL_H
#define TALLY_PROTOCOL_H

#include <Arduino.h>

#include "TallyConfig.h"

// Protocol constants
#define TALLY_START_BYTE    0xAA
#define TALLY_PACKET_SIZE   8
#define TALLY_BROADCAST_ID  0xFF

// Commands
enum TallyCommand : uint8_t {
    CMD_PING        = 0x02, // payload[0] = target camera ID (0xFF = all)
    CMD_STATE_ALL   = 0x06  // payload = progLo, progHi, prevLo, prevHi
};

// Camera states
enum TallyState : uint8_t {
    STATE_OFF       = 0x00,
    STATE_PREVIEW   = 0x01,  // Green
    STATE_PROGRAM   = 0x02,  // Red
    STATE_BOTH      = 0x03   // Both preview and program
};

// Packet structure (8 bytes). One CMD_STATE_ALL broadcast carries the tally
// state of up to 16 cameras; its periodic re-send doubles as the link
// heartbeat (slaves reset their signal-lost timer on any valid packet).
#pragma pack(push, 1)
struct TallyPacket {
    uint8_t start;      // Start marker: 0xAA
    uint8_t command;    // Command type
    uint8_t netId;      // Network ID (TALLY_NET_ID) — rejects foreign systems
    uint8_t payload[4]; // Per-command payload
    uint8_t crc;        // XOR checksum
};
#pragma pack(pop)

class TallyProtocol {
public:
    TallyProtocol();

    // Create packets
    static TallyPacket createStateAllPacket(uint16_t progMask, uint16_t prevMask);
    static TallyPacket createPingPacket(uint8_t cameraId);

    // Extract this camera's state from a CMD_STATE_ALL packet
    static TallyState stateForCamera(const TallyPacket& packet, uint8_t cameraId);

    // Serialize/deserialize
    static void serialize(const TallyPacket& packet, uint8_t* buffer);
    static bool deserialize(const uint8_t* buffer, uint8_t len, TallyPacket& packet);

    // Validation
    static bool validate(const TallyPacket& packet);
    static uint8_t calculateCRC(const TallyPacket& packet);
};

#endif // TALLY_PROTOCOL_H
