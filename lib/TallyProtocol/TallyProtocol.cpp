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

static const uint8_t CRC8_LUT[256] = {
  0x00,  0x07,  0x0e,  0x09,  0x1c,  0x1b,  0x12,  0x15,
  0x38,  0x3f,  0x36,  0x31,  0x24,  0x23,  0x2a,  0x2d,
  0x70,  0x77,  0x7e,  0x79,  0x6c,  0x6b,  0x62,  0x65,
  0x48,  0x4f,  0x46,  0x41,  0x54,  0x53,  0x5a,  0x5d,
  0xe0,  0xe7,  0xee,  0xe9,  0xfc,  0xfb,  0xf2,  0xf5,
  0xd8,  0xdf,  0xd6,  0xd1,  0xc4,  0xc3,  0xca,  0xcd,
  0x90,  0x97,  0x9e,  0x99,  0x8c,  0x8b,  0x82,  0x85,
  0xa8,  0xaf,  0xa6,  0xa1,  0xb4,  0xb3,  0xba,  0xbd,
  0xc7,  0xc0,  0xc9,  0xce,  0xdb,  0xdc,  0xd5,  0xd2,
  0xff,  0xf8,  0xf1,  0xf6,  0xe3,  0xe4,  0xed,  0xea,
  0xb7,  0xb0,  0xb9,  0xbe,  0xab,  0xac,  0xa5,  0xa2,
  0x8f,  0x88,  0x81,  0x86,  0x93,  0x94,  0x9d,  0x9a,
  0x27,  0x20,  0x29,  0x2e,  0x3b,  0x3c,  0x35,  0x32,
  0x1f,  0x18,  0x11,  0x16,  0x03,  0x04,  0x0d,  0x0a,
  0x57,  0x50,  0x59,  0x5e,  0x4b,  0x4c,  0x45,  0x42,
  0x6f,  0x68,  0x61,  0x66,  0x73,  0x74,  0x7d,  0x7a,
  0x89,  0x8e,  0x87,  0x80,  0x95,  0x92,  0x9b,  0x9c,
  0xb1,  0xb6,  0xbf,  0xb8,  0xad,  0xaa,  0xa3,  0xa4,
  0xf9,  0xfe,  0xf7,  0xf0,  0xe5,  0xe2,  0xeb,  0xec,
  0xc1,  0xc6,  0xcf,  0xc8,  0xdd,  0xda,  0xd3,  0xd4,
  0x69,  0x6e,  0x67,  0x60,  0x75,  0x72,  0x7b,  0x7c,
  0x51,  0x56,  0x5f,  0x58,  0x4d,  0x4a,  0x43,  0x44,
  0x19,  0x1e,  0x17,  0x10,  0x05,  0x02,  0x0b,  0x0c,
  0x21,  0x26,  0x2f,  0x28,  0x3d,  0x3a,  0x33,  0x34,
  0x4e,  0x49,  0x40,  0x47,  0x52,  0x55,  0x5c,  0x5b,
  0x76,  0x71,  0x78,  0x7f,  0x6a,  0x6d,  0x64,  0x63,
  0x3e,  0x39,  0x30,  0x37,  0x22,  0x25,  0x2c,  0x2b,
  0x06,  0x01,  0x08,  0x0f,  0x1a,  0x1d,  0x14,  0x13,
  0xae,  0xa9,  0xa0,  0xa7,  0xb2,  0xb5,  0xbc,  0xbb,
  0x96,  0x91,  0x98,  0x9f,  0x8a,  0x8d,  0x84,  0x83,
  0xde,  0xd9,  0xd0,  0xd7,  0xc2,  0xc5,  0xcc,  0xcb,
  0xe6,  0xe1,  0xe8,  0xef,  0xfa,  0xfd,  0xf4,  0xf3,
};

uint8_t TallyProtocol::calculateCRC(const TallyPacket &p) {
  static_assert(sizeof(TallyPacket) == TALLY_PACKET_SIZE,
                "TallyPacket layout != wire size");
  // CRC-8/CCITT (poly 0x07, init 0x00) over every byte but the CRC itself.

  // ⚡ Bolt Optimization:
  // Replaced nested bitwise loop with a precomputed Lookup Table (LUT).
  // Impact: Reduces time complexity from O(8n) to O(n). Trades 256 bytes of
  // flash memory for significantly lower CPU overhead during the high-frequency
  // packet validation path, critical for the overall system performance.

  const uint8_t *data = (const uint8_t *)&p;
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < TALLY_PACKET_SIZE - 1; i++) {
    crc = CRC8_LUT[crc ^ data[i]];
  }
  return crc;
}
