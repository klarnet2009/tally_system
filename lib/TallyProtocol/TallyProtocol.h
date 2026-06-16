#ifndef TALLY_PROTOCOL_H
#define TALLY_PROTOCOL_H

#include <Arduino.h>

#include "TallyConfig.h"

// ===== Protocol v3 =====
// 9-byte frame. The command byte carries a version nibble so a mixed-firmware
// fleet fails CLOSED (an old node rejects a v3 packet and shows signal-lost
// rather than silently decoding garbage as a tally). CRC is a real CRC-8 (not
// XOR). One STATE_ALL broadcast carries 16 cameras + a "source live" flag; its
// periodic re-send is the link heartbeat. Slaves send TELEMETRY back so the hub
// knows which cameras are actually reachable.
#define TALLY_START_BYTE        0xAA
#define TALLY_PACKET_SIZE       9
#define TALLY_BROADCAST_ID      0xFF
#define TALLY_PROTOCOL_VERSION  0x3 // bumped from the XOR/8-byte v2 wire format

// command byte = (version << 4) | code
#define TALLY_CMD_BYTE(code) ((uint8_t)((TALLY_PROTOCOL_VERSION << 4) | ((code) & 0x0F)))
#define TALLY_CMD_CODE(b)    ((uint8_t)((b) & 0x0F))
#define TALLY_CMD_VERSION(b) ((uint8_t)((b) >> 4))

enum TallyCmd : uint8_t {
    CMD_PING        = 0x2, // aux = target camera ID (0xFF = all)
    CMD_STATE_ALL   = 0x6, // aux = flags; payload = progLo,progHi,prevLo,prevHi
    CMD_TELEMETRY   = 0x7  // slave -> hub: aux = camId; payload = batt,rssi,flags
};

// STATE_ALL aux-byte flags
#define TALLY_FLAG_SOURCE_LIVE  0x01 // hub's tally source (ATEM) is fresh, not frozen

// TELEMETRY payload flags
#define TALLY_TLM_NO_BATTERY    0x01 // slave has no battery-sense ADC wired

// Camera states
enum TallyState : uint8_t {
    STATE_OFF       = 0x00,
    STATE_PREVIEW   = 0x01,  // Green
    STATE_PROGRAM   = 0x02,  // Red
    STATE_BOTH      = 0x03   // Both preview and program
};

// 9-byte wire frame. `aux` and `payload` are interpreted per command.
#pragma pack(push, 1)
struct TallyPacket {
    uint8_t start;      // 0xAA
    uint8_t command;    // (version << 4) | code
    uint8_t netId;      // rejects foreign systems
    uint8_t aux;        // STATE_ALL: flags | PING/TELEMETRY: camId
    uint8_t payload[4]; // per-command
    uint8_t crc;        // CRC-8 over bytes 0..7
};
#pragma pack(pop)

class TallyProtocol {
public:
    TallyProtocol();

    // Create packets (hub -> slaves)
    static TallyPacket createStateAllPacket(uint16_t progMask, uint16_t prevMask,
                                            bool sourceLive);
    static TallyPacket createPingPacket(uint8_t cameraId);
    // Slave -> hub telemetry
    static TallyPacket createTelemetryPacket(uint8_t cameraId, uint16_t battMv,
                                             int8_t rssi, uint8_t flags);

    // Accessors
    static uint8_t cmdCode(const TallyPacket& p) { return TALLY_CMD_CODE(p.command); }
    static TallyState stateForCamera(const TallyPacket& packet, uint8_t cameraId);
    static bool sourceLive(const TallyPacket& packet) { return packet.aux & TALLY_FLAG_SOURCE_LIVE; }
    static uint16_t telemetryBattMv(const TallyPacket& p) {
        return (uint16_t)p.payload[0] | ((uint16_t)p.payload[1] << 8);
    }
    static int8_t telemetryRssi(const TallyPacket& p) { return (int8_t)p.payload[2]; }
    static uint8_t telemetryFlags(const TallyPacket& p) { return p.payload[3]; }

    // Serialize/deserialize
    static void serialize(const TallyPacket& packet, uint8_t* buffer);
    static bool deserialize(const uint8_t* buffer, uint8_t len, TallyPacket& packet);

    // Validation
    static bool validate(const TallyPacket& packet);
    static uint8_t calculateCRC(const TallyPacket& packet);
};

#endif // TALLY_PROTOCOL_H
