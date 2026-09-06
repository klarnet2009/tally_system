// Host test shim for the SPI bus: a SCRIPTABLE slave. Every NSS-framed
// transaction the driver performs is recorded byte-for-byte (MOSI), and a
// responder installed by the test computes each MISO byte from the frame so
// far — enough to emulate the SX1280's command/register model.
//
// This exists because the driver's two worst bugs (hardware CRC configured OFF
// via an SX126x field encoding; the SF-dependent register the datasheet requires
// never written) were invisible to every logic test and to the bench: both ends
// were wrong identically, so the link "worked". Only the bytes on the bus tell.
#ifndef TEST_SPI_SHIM_H
#define TEST_SPI_SHIM_H

#include <cstdint>
#include <functional>
#include <vector>

#define MSBFIRST 1
#define SPI_MODE0 0

struct SPISettings {
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

struct SPIClass {
  typedef std::vector<uint8_t> Frame;
  std::vector<Frame> frames; // completed transactions, oldest first
  Frame cur;                 // transaction in progress (NSS low)
  int8_t nssPin = -1;        // learned from begin(); NSS edges frame the bus
  // MISO byte for the byte just clocked; receives the frame INCLUDING it.
  std::function<uint8_t(const Frame &)> respond;

  void begin(int8_t, int8_t, int8_t, int8_t ss) { nssPin = ss; }
  void beginTransaction(const SPISettings &) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t b) {
    cur.push_back(b);
    return respond ? respond(cur) : 0;
  }
  void transferBytes(const uint8_t *tx, uint8_t *rx, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
      uint8_t r = transfer(tx ? tx[i] : 0);
      if (rx)
        rx[i] = r;
    }
  }
  // Called by the GPIO shim on NSS edges.
  void nssLow() { cur.clear(); }
  void nssHigh() {
    if (!cur.empty())
      frames.push_back(cur);
    cur.clear();
  }
  // Test helpers.
  void reset() { frames.clear(); cur.clear(); }
  // Last completed frame starting with `opcode`, or nullptr.
  const Frame *last(uint8_t opcode) const {
    for (size_t i = frames.size(); i-- > 0;)
      if (!frames[i].empty() && frames[i][0] == opcode)
        return &frames[i];
    return nullptr;
  }
  int count(uint8_t opcode) const {
    int n = 0;
    for (const Frame &f : frames)
      if (!f.empty() && f[0] == opcode)
        n++;
    return n;
  }
};
extern SPIClass SPI;

#endif // TEST_SPI_SHIM_H
