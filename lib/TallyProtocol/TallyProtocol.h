#ifndef TALLY_PROTOCOL_H
#define TALLY_PROTOCOL_H

#include <Arduino.h>

#include "TallyConfig.h"

// ===== Protocol v4 =====
// 8-byte frame. Deliberately NOT 9: the LoRa payload symbol count quantizes,
// so 4..8 bytes all cost 43.2 ms at SF9/BW406/CR4-6 while 9 bytes costs
// 50.7 ms — the 8th byte is free, and it buys the heartbeat counter that makes
// collision-free telemetry slotting possible.
//
// What v3 carried and v4 drops:
//  - start byte:   the LoRa PHY sync word already frames the packet.
//  - app CRC-8:    the PHY CRC-16 is computed and checked in HARDWARE, and the
//                  driver discards CrcError receptions before they reach this
//                  layer. A second software CRC cost airtime, not safety.
//                  (Deliberate decision to trust the PHY CRC.)
//  - SET_POWER:    uplink ADR is gone — it bought ~1% duty cycle and
//                  contaminated the channel decision (ARCHITECTURE_RF_V4 §9).
//  - SET_CHANNEL:  channel switches now ride inside the STATE_ALL heartbeat, so
//                  ~10 decorrelated frames carry the plan instead of 6
//                  back-to-back frames sharing one interference burst.
//
// Byte 0 pins version AND network, so a mixed-firmware fleet fails CLOSED: a v3
// node rejects the frame and shows signal-lost rather than decoding a different
// layout as a tally colour.
#define TALLY_PACKET_SIZE 8
#define TALLY_PROTOCOL_VERSION 0x4

// byte 0 = [version:4][netId:4]
#define TALLY_VERNET_BYTE                                                      \
  ((uint8_t)((TALLY_PROTOCOL_VERSION << 4) | (TALLY_NET_ID & 0x0F)))
// byte 1 = [command:4][flags:4]
#define TALLY_CMDFLAGS_BYTE(cmd, flags)                                        \
  ((uint8_t)((((cmd) & 0x0F) << 4) | ((flags) & 0x0F)))

#define TALLY_BROADCAST_ID 0xFF

enum TallyCmd : uint8_t {
  CMD_STATE_ALL = 0x1, // hub -> fleet: masks + source flag + channel plan
  CMD_PING      = 0x2, // hub -> one/all: locator
  CMD_TELEMETRY = 0x3  // slave -> hub: liveness, RSSI, missed-beat gradient
};

// STATE_ALL flag nibble
#define TALLY_FLAG_SOURCE_LIVE 0x1 // hub's tally source (ATEM) is fresh
// TELEMETRY flag nibble
#define TALLY_FLAG_NO_BATTERY 0x1 // slave has no battery-sense divider wired

// Camera states
enum TallyState : uint8_t {
  STATE_OFF     = 0x00,
  STATE_PREVIEW = 0x01, // Green
  STATE_PROGRAM = 0x02, // Red
  STATE_BOTH    = 0x03  // Both preview and program
};

// 8-byte wire frame. `data` is interpreted per command:
//
//  STATE_ALL: [0]=prog_lo [1]=prog_hi [2]=prev_lo [3]=prev_hi
//             [4]=[chanIdx:4][countdown:4]  (countdown 0 = no switch pending)
//             [5]=heartbeat counter (mod 256)
//  PING:      [0]=target camera id (0xFF = all)
//  TELEMETRY: [0]=camId [1]=rssi(int8) [2]=missed heartbeats (0..15)
//             [3]=batt_lo [4]=batt_hi
#pragma pack(push, 1)
struct TallyPacket {
  uint8_t verNet;
  uint8_t cmdFlags;
  uint8_t data[6];
};
#pragma pack(pop)

class TallyProtocol {
public:
  // ---- Build (hub -> fleet) ----
  // chanIdx/chanCountdown announce a coordinated switch; countdown 0 = none.
  static TallyPacket createStateAllPacket(uint16_t progMask, uint16_t prevMask,
                                          bool sourceLive, uint8_t hbCount,
                                          uint8_t chanIdx = 0,
                                          uint8_t chanCountdown = 0);
  static TallyPacket createPingPacket(uint8_t cameraId);
  // ---- Build (slave -> hub) ----
  static TallyPacket createTelemetryPacket(uint8_t cameraId, int8_t rssi,
                                           uint8_t missedBeats, uint16_t battMv,
                                           bool noBattery);

  // ---- Read ----
  static uint8_t cmd(const TallyPacket &p) { return (uint8_t)(p.cmdFlags >> 4); }
  static uint8_t flags(const TallyPacket &p) {
    return (uint8_t)(p.cmdFlags & 0x0F);
  }

  static TallyState stateForCamera(const TallyPacket &p, uint8_t cameraId);
  static bool sourceLive(const TallyPacket &p) {
    return (p.cmdFlags & TALLY_FLAG_SOURCE_LIVE) != 0;
  }
  static uint16_t progMask(const TallyPacket &p) {
    return (uint16_t)p.data[0] | ((uint16_t)p.data[1] << 8);
  }
  static uint16_t prevMask(const TallyPacket &p) {
    return (uint16_t)p.data[2] | ((uint16_t)p.data[3] << 8);
  }
  static uint8_t chanIdx(const TallyPacket &p) {
    return (uint8_t)(p.data[4] >> 4);
  }
  static uint8_t chanCountdown(const TallyPacket &p) {
    return (uint8_t)(p.data[4] & 0x0F);
  }
  static uint8_t hbCount(const TallyPacket &p) { return p.data[5]; }
  static uint8_t pingTarget(const TallyPacket &p) { return p.data[0]; }

  static uint8_t telemetryCamId(const TallyPacket &p) { return p.data[0]; }
  static int8_t telemetryRssi(const TallyPacket &p) { return (int8_t)p.data[1]; }
  static uint8_t telemetryMissed(const TallyPacket &p) { return p.data[2]; }
  static uint16_t telemetryBattMv(const TallyPacket &p) {
    return (uint16_t)p.data[3] | ((uint16_t)p.data[4] << 8);
  }
  static bool telemetryNoBattery(const TallyPacket &p) {
    return (p.cmdFlags & TALLY_FLAG_NO_BATTERY) != 0;
  }

  // ---- Wire ----
  static void serialize(const TallyPacket &p, uint8_t *buffer);
  static bool deserialize(const uint8_t *buffer, uint8_t len, TallyPacket &p);
  static bool validate(const TallyPacket &p);
};

#endif // TALLY_PROTOCOL_H
