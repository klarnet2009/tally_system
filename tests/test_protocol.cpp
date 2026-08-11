// Protocol tests: bit packing and validation. A packing bug here silently shows
// the wrong colour on a camera, and nothing downstream can detect it.
#include "Arduino.h"
#include "harness.h"
#include "TallyProtocol.h"

int main() {
  printf("== TallyProtocol ==\n");

  CASE("frame size matches the wire constant");
  CHECK_EQ(sizeof(TallyPacket), TALLY_PACKET_SIZE);

  CASE("STATE_ALL round-trips masks, source flag, hb counter, channel plan");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0xBEEF, 0x1234, true, 77,
                                                        2, 9);
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(p, buf);
    TallyPacket q;
    CHECK(TallyProtocol::deserialize(buf, sizeof(buf), q));
    CHECK_EQ(TallyProtocol::cmd(q), CMD_STATE_ALL);
    CHECK_EQ(TallyProtocol::progMask(q), 0xBEEF);
    CHECK_EQ(TallyProtocol::prevMask(q), 0x1234);
    CHECK(TallyProtocol::sourceLive(q));
    CHECK_EQ(TallyProtocol::hbCount(q), 77);
    CHECK_EQ(TallyProtocol::chanIdx(q), 2);
    CHECK_EQ(TallyProtocol::chanCountdown(q), 9);
  }

  CASE("source-live flag clears independently of the channel plan");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0, 0, false, 0, 3, 15);
    CHECK(!TallyProtocol::sourceLive(p));
    CHECK_EQ(TallyProtocol::chanIdx(p), 3);
    CHECK_EQ(TallyProtocol::chanCountdown(p), 15);
  }

  CASE("countdown 0 means no switch pending");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(1, 2, true, 5);
    CHECK_EQ(TallyProtocol::chanCountdown(p), 0);
  }

  CASE("stateForCamera decodes all four states and rejects bad ids");
  {
    // cam1 program only, cam2 preview only, cam3 both, cam4 off
    uint16_t prog = (1u << 0) | (1u << 2);
    uint16_t prev = (1u << 1) | (1u << 2);
    TallyPacket p = TallyProtocol::createStateAllPacket(prog, prev, true, 0);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 1), STATE_PROGRAM);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 2), STATE_PREVIEW);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 3), STATE_BOTH);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 4), STATE_OFF);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 0), STATE_OFF);   // out of range
    CHECK_EQ(TallyProtocol::stateForCamera(p, 17), STATE_OFF);  // out of range
  }

  CASE("camera 16 (the top bit) is not lost");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(1u << 15, 0, true, 0);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 16), STATE_PROGRAM);
    CHECK_EQ(TallyProtocol::stateForCamera(p, 15), STATE_OFF);
  }

  CASE("PING carries its target, broadcast included");
  {
    TallyPacket p = TallyProtocol::createPingPacket(7);
    CHECK_EQ(TallyProtocol::cmd(p), CMD_PING);
    CHECK_EQ(TallyProtocol::pingTarget(p), 7);
    TallyPacket b = TallyProtocol::createPingPacket(TALLY_BROADCAST_ID);
    CHECK_EQ(TallyProtocol::pingTarget(b), TALLY_BROADCAST_ID);
  }

  CASE("TELEMETRY round-trips rssi, missed count, shown state, tag, battery");
  {
    TallyPacket p = TallyProtocol::createTelemetryPacket(
        5, -97, 4, 3700, false, STATE_BOTH, 0xA5);
    CHECK_EQ(TallyProtocol::cmd(p), CMD_TELEMETRY);
    CHECK_EQ(TallyProtocol::telemetryCamId(p), 5);
    CHECK_EQ(TallyProtocol::telemetryRssi(p), -97); // int8 survives the byte
    CHECK_EQ(TallyProtocol::telemetryMissed(p), 4);
    CHECK_EQ(TallyProtocol::telemetryShown(p), STATE_BOTH);
    CHECK_EQ(TallyProtocol::telemetryTag(p), 0xA5);
    CHECK_EQ(TallyProtocol::telemetryBattMv(p), 3700);
    CHECK(!TallyProtocol::telemetryNoBattery(p));
  }

  CASE("shown state and no-battery flag share the nibble without collision");
  {
    for (int s = 0; s <= 3; s++) {
      TallyPacket p = TallyProtocol::createTelemetryPacket(
          1, 0, 0, 0, true, (TallyState)s, 1);
      CHECK_EQ(TallyProtocol::telemetryShown(p), s);
      CHECK(TallyProtocol::telemetryNoBattery(p));
    }
  }

  CASE("missed-beat count is clamped, never wrapped into the next field");
  {
    TallyPacket p =
        TallyProtocol::createTelemetryPacket(1, 0, 200, 0, true, STATE_OFF, 1);
    CHECK_EQ(TallyProtocol::telemetryMissed(p), 15);
    CHECK_EQ(TallyProtocol::telemetryTag(p), 1); // neighbour untouched
  }

  CASE("FAIL CLOSED: a wrong protocol version is rejected, not decoded");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0xFFFF, 0, true, 0);
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(p, buf);
    buf[0] = (uint8_t)((0x3 << 4) | (TALLY_NET_ID & 0x0F)); // pretend v3
    TallyPacket q;
    CHECK(!TallyProtocol::deserialize(buf, sizeof(buf), q));
  }

  CASE("FAIL CLOSED: a foreign network id is rejected");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0xFFFF, 0, true, 0);
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(p, buf);
    buf[0] = (uint8_t)((TALLY_PROTOCOL_VERSION << 4) |
                       ((TALLY_NET_ID + 1) & 0x0F));
    TallyPacket q;
    CHECK(!TallyProtocol::deserialize(buf, sizeof(buf), q));
  }

  CASE("an unknown command is rejected");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0, 0, true, 0);
    p.cmdFlags = TALLY_CMDFLAGS_BYTE(0xE, 0);
    CHECK(!TallyProtocol::validate(p));
  }

  CASE("a short buffer is rejected before it is read");
  {
    TallyPacket p = TallyProtocol::createStateAllPacket(0, 0, true, 0);
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(p, buf);
    TallyPacket q;
    CHECK(!TallyProtocol::deserialize(buf, TALLY_PACKET_SIZE - 1, q));
  }

  return testSummary("TallyProtocol");
}
