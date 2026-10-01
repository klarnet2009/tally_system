#include "TallyProtocol.h"
#include "crc8_lut.h"

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
  p.crc = TallyProtocol::calculateCRC(p);
  return p;
}

TallyPacket TallyProtocol::createStateAllPacket(uint16_t progMask,
                                                uint16_t prevMask,
                                                bool sourceLive,
                                                uint8_t hbCount,
                                                uint8_t chanIdx,
                                                uint8_t chanCountdown,
                                                bool burstCopy) {
  uint8_t flags = sourceLive ? TALLY_FLAG_SOURCE_LIVE : 0;
  if (burstCopy)
    flags |= TALLY_FLAG_BURST;
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
                                                 uint16_t battMv, bool noBattery,
                                                 TallyState shown,
                                                 uint8_t deviceTag) {
  uint8_t flags = noBattery ? TALLY_FLAG_NO_BATTERY : 0;
  flags |= (uint8_t)(((uint8_t)shown << TALLY_TLM_STATE_SHIFT) &
                     TALLY_TLM_STATE_MASK);
  if (missedBeats > 15)
    missedBeats = 15; // a hint for the hub, not a measurement — clamp it
  return makeFrame(CMD_TELEMETRY, flags, cameraId, (uint8_t)rssi, missedBeats,
                   (uint8_t)(battMv & 0xFF), (uint8_t)(battMv >> 8), deviceTag);
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
  // EXACT length, not a minimum: every frame we emit is this size, so a longer
  // reception is by definition not ours. Free, and it rejects foreign traffic
  // that would otherwise have to be caught by the checksum alone.
  if (len != TALLY_PACKET_SIZE) {
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
  uint8_t c = cmd(p);
  if (c != CMD_STATE_ALL && c != CMD_PING && c != CMD_TELEMETRY) {
    return false;
  }
  // Not an integrity check — the PHY already did that in hardware. This is an
  // AUTHENTICITY check: it is what stops a foreign SX1280 frame that happens to
  // start with our version+netId byte from being decoded as a tally state.
  if (p.crc != calculateCRC(p)) {
    return false;
  }
  return true;
}

uint8_t TallyProtocol::calculateCRC(const TallyPacket &p) {
  static_assert(sizeof(TallyPacket) == TALLY_PACKET_SIZE,
                "TallyPacket layout != wire size");
  // CRC-8/CCITT (poly 0x07, init 0x00) over every byte but the CRC itself.
  // ⚡ Bolt: Using a 256-byte precomputed Lookup Table (LUT) for O(1) performance boost
  const uint8_t *data = (const uint8_t *)&p;
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < TALLY_PACKET_SIZE - 1; i++) {
    crc = crc8_lut[crc ^ data[i]];
  }
  return crc;
}
