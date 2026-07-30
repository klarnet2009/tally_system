#include "TallyProtocol.h"

static TallyPacket makeFrame(uint8_t cmd, uint8_t flags, uint8_t d0, uint8_t d1,
                             uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5) {
  TallyPacket p;
  p.verNet = TALLY_VERNET_BYTE;
  p.cmdFlags = TALLY_CMDFLAGS_BYTE(cmd, flags);
  p.data[0] = d0;
  p.data[1] = d1;
  p.data[2] = d2;
  p.data[3] = d3;
  p.data[4] = d4;
  p.data[5] = d5;
  return p;
}

TallyPacket TallyProtocol::createStateAllPacket(uint16_t progMask,
                                                uint16_t prevMask,
                                                bool sourceLive,
                                                uint8_t hbCount,
                                                uint8_t chanIdx,
                                                uint8_t chanCountdown) {
  uint8_t flags = sourceLive ? TALLY_FLAG_SOURCE_LIVE : 0;
  uint8_t chanByte = (uint8_t)(((chanIdx & 0x0F) << 4) | (chanCountdown & 0x0F));
  return makeFrame(CMD_STATE_ALL, flags, (uint8_t)(progMask & 0xFF),
                   (uint8_t)(progMask >> 8), (uint8_t)(prevMask & 0xFF),
                   (uint8_t)(prevMask >> 8), chanByte, hbCount);
}

TallyPacket TallyProtocol::createPingPacket(uint8_t cameraId) {
  return makeFrame(CMD_PING, 0, cameraId, 0, 0, 0, 0, 0);
}

TallyPacket TallyProtocol::createTelemetryPacket(uint8_t cameraId, int8_t rssi,
                                                 uint8_t missedBeats,
                                                 uint16_t battMv,
                                                 bool noBattery) {
  uint8_t flags = noBattery ? TALLY_FLAG_NO_BATTERY : 0;
  if (missedBeats > 15)
    missedBeats = 15; // a hint for the hub, not a measurement — clamp it
  return makeFrame(CMD_TELEMETRY, flags, cameraId, (uint8_t)rssi, missedBeats,
                   (uint8_t)(battMv & 0xFF), (uint8_t)(battMv >> 8), 0);
}

TallyState TallyProtocol::stateForCamera(const TallyPacket &p,
                                         uint8_t cameraId) {
  if (cameraId < 1 || cameraId > 16) {
    return STATE_OFF;
  }
  uint16_t bit = 1U << (cameraId - 1);
  bool onAir = (progMask(p) & bit) != 0;
  bool preview = (prevMask(p) & bit) != 0;
  if (onAir && preview) return STATE_BOTH;
  if (onAir) return STATE_PROGRAM;
  if (preview) return STATE_PREVIEW;
  return STATE_OFF;
}

void TallyProtocol::serialize(const TallyPacket &p, uint8_t *buffer) {
  memcpy(buffer, &p, TALLY_PACKET_SIZE);
}

bool TallyProtocol::deserialize(const uint8_t *buffer, uint8_t len,
                                TallyPacket &p) {
  if (len < TALLY_PACKET_SIZE) {
    return false;
  }
  // Cheapest reject first: byte 0 pins version AND network, so a single
  // comparison rejects both foreign systems and other firmware generations.
  if (buffer[0] != TALLY_VERNET_BYTE) {
    return false;
  }
  memcpy(&p, buffer, TALLY_PACKET_SIZE);
  return validate(p);
}

bool TallyProtocol::validate(const TallyPacket &p) {
  // serialize()/deserialize() memcpy the struct as the wire format, so its
  // size must equal TALLY_PACKET_SIZE.
  static_assert(sizeof(TallyPacket) == TALLY_PACKET_SIZE,
                "TallyPacket layout != wire size");
  // Version + network, fail-closed (see header).
  if (p.verNet != TALLY_VERNET_BYTE) {
    return false;
  }
  // Integrity is the PHY's job: the SX1280 checks CRC-16 in hardware and the
  // driver drops CrcError/HeaderError receptions, so a frame arriving here is
  // already known intact. All that remains is rejecting an unknown command.
  uint8_t c = cmd(p);
  if (c != CMD_STATE_ALL && c != CMD_PING && c != CMD_TELEMETRY) {
    return false;
  }
  return true;
}
