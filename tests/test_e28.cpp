// SX1280 driver tests against a scripted SPI slave (tests/SPI.h). These check
// the BYTES the driver puts on the bus, because that is where its two worst
// defects lived — and neither could be seen by any logic test or any bench:
//
//  - SetPacketParams carried the SX126x field encodings (CRC 0x01, IQ 0x00). On
//    the SX1280, CRC ON is 0x20, so every frame ever sent had NO hardware CRC —
//    while the protocol layer argued it could drop its own CRC "because the PHY
//    checks it". Both ends were misconfigured identically, so the link worked.
//  - The datasheet (§14.4.1) requires an SF-dependent write to register 0x0925
//    and bit 0 of 0x093C after SetModulationParams. Neither was ever written, so
//    the SF9 sensitivity the whole link budget rests on was never configured.
#include "Arduino.h"
#include "SPI.h"
#include "harness.h"

#include "E28_SX1280.h"
#include "TallyRadio.h"

#include <map>

// ---- a minimal SX1280 behind the bus ----
static std::map<uint16_t, uint8_t> g_regs; // register file
static uint16_t g_irq = 0;                  // GetIrqStatus value
static uint8_t g_rxLen = 0;                 // GetRxBufferStatus payload length
static uint8_t g_rxBuf[32];                 // ReadBuffer contents
static bool g_poisonSfReg = false;          // make 0x0925 read back garbage
static const uint8_t kStatus = 0x44;        // STDBY_RC, "command OK"

static uint8_t sx1280(const SPIClass::Frame &f) {
  const size_t i = f.size() - 1; // index of the byte being clocked now
  switch (f[0]) {
  case SX1280_CMD_GET_PACKET_TYPE: // [op][NOP][type]
    return i == 2 ? SX1280_PACKET_TYPE_LORA : kStatus;
  case SX1280_CMD_WRITE_REGISTER: // [op][MSB][LSB][data...]
    if (i >= 3)
      g_regs[(uint16_t)((f[1] << 8) | f[2]) + (uint16_t)(i - 3)] = f[i];
    return kStatus;
  case SX1280_CMD_READ_REGISTER: { // [op][MSB][LSB][NOP][data...]
    if (i < 4)
      return kStatus;
    uint16_t addr = (uint16_t)((f[1] << 8) | f[2]) + (uint16_t)(i - 4);
    if (g_poisonSfReg && addr == SX1280_REG_LORA_SF_CONFIG)
      return 0x00;
    auto it = g_regs.find(addr);
    return it == g_regs.end() ? 0 : it->second;
  }
  case SX1280_CMD_GET_IRQ_STATUS: // [op][NOP][MSB][LSB]
    return i == 2 ? (uint8_t)(g_irq >> 8) : i == 3 ? (uint8_t)g_irq : kStatus;
  case SX1280_CMD_CLR_IRQ_STATUS: // [op][MSB][LSB]
    if (i == 2)
      g_irq &= (uint16_t)~(((uint16_t)f[1] << 8) | f[2]);
    return kStatus;
  case SX1280_CMD_GET_RX_BUFFER_STATUS: // [op][NOP][len][offset]
    return i == 2 ? g_rxLen : i == 3 ? 0 : kStatus;
  case SX1280_CMD_READ_BUFFER: // [op][offset][NOP][data...]
    return i >= 3 && i - 3 < sizeof(g_rxBuf) ? g_rxBuf[i - 3] : kStatus;
  default:
    return kStatus;
  }
}

static void resetChip() {
  g_regs.clear();
  g_irq = 0;
  g_rxLen = 0;
  g_poisonSfReg = false;
  SPI.reset();
  SPI.respond = sx1280;
}

// Pin numbers are arbitrary; BUSY (5) reads LOW so every wait returns at once.
static bool bringUp(E28Radio &r) {
  return r.begin(1, 2, 3, 4, 5, 6, 7, 8, 9);
}

int main() {
  printf("== E28_SX1280 driver (scripted SPI) ==\n");
  testSetMillis(10000);

  CASE("DEFECT: SetPacketParams must use the SX1280 encodings — CRC ON is 0x20");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    tallyApplyRadioProfile(r, 0);
    uint8_t frame[TALLY_PACKET_SIZE] = {0};
    CHECK(r.startSend(frame, TALLY_PACKET_SIZE));
    const SPIClass::Frame *pp = SPI.last(SX1280_CMD_SET_PACKET_PARAMS);
    CHECK(pp != nullptr);
    if (pp) {
      CHECK_EQ(pp->size(), 8u);
      CHECK_EQ((*pp)[2], SX1280_LORA_HEADER_EXPLICIT);
      CHECK_EQ((*pp)[3], TALLY_PACKET_SIZE);
      CHECK_EQ((*pp)[4], 0x20); // literal on purpose: the datasheet value
      CHECK_EQ((*pp)[5], 0x40); // standard IQ — NOT 0x00, which is inverted
      CHECK((*pp)[4] != 0x01);  // the SX126x value that shipped for months
    }
    // The driver polls DIO1 before spending an SPI read on the IRQ register, so
    // a completed TX needs both the pin and the bit, like on the real chip.
    CHECK(!r.checkTxDone()); // still in flight: not timed out, DIO1 low
    g_irq = 0x0001;
    testSetPin(6, HIGH);
    CHECK(r.checkTxDone());
    CHECK(r.txSucceeded());
    testSetPin(6, LOW);
  }

  CASE("DEFECT: after SetModulationParams the SF register and FEC bit are written");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    // Constructor default is SF7 -> 0x37.
    CHECK_EQ(g_regs[SX1280_REG_LORA_SF_CONFIG], SX1280_LORA_SF_CONFIG_SF7_8);
    CHECK_EQ(g_regs[SX1280_REG_FREQ_ERR_CORR] & 0x01, 0x01);
    // The production profile is SF9 -> 0x32, re-written on every SF change.
    tallyApplyRadioProfile(r, 0);
    CHECK_EQ(g_regs[SX1280_REG_LORA_SF_CONFIG], SX1280_LORA_SF_CONFIG_SF9_12);
    const SPIClass::Frame *mp = SPI.last(SX1280_CMD_SET_MODULATION_PARAMS);
    CHECK(mp != nullptr);
    if (mp) {
      CHECK_EQ((*mp)[1], LORA_SF9);
      CHECK_EQ((*mp)[2], LORA_BW_0400);
      CHECK_EQ((*mp)[3], LORA_CR_4_6);
    }
    // The register write must FOLLOW the modulation command, never precede it.
    size_t lastMod = 0, lastSf = 0;
    for (size_t i = 0; i < SPI.frames.size(); i++) {
      const SPIClass::Frame &f = SPI.frames[i];
      if (f[0] == SX1280_CMD_SET_MODULATION_PARAMS)
        lastMod = i;
      if (f[0] == SX1280_CMD_WRITE_REGISTER && f.size() >= 4 &&
          f[1] == 0x09 && f[2] == 0x25)
        lastSf = i;
    }
    CHECK(lastSf > lastMod);
    // 0x093C is read-modify-write: other bits are preserved.
    g_regs[SX1280_REG_FREQ_ERR_CORR] = 0xF0;
    r.setSpreadingFactor(LORA_SF9);
    CHECK_EQ(g_regs[SX1280_REG_FREQ_ERR_CORR], 0xF1);
  }

  CASE("begin() fails with E28_ERR_SF_REG when the SF register does not take");
  {
    resetChip();
    g_poisonSfReg = true;
    E28Radio r;
    CHECK(!bringUp(r));
    CHECK_EQ(r.initError(), E28_ERR_SF_REG);
    CHECK(!r.isConnected());
    CHECK(strcmp(r.initErrorStr(), "?") != 0); // has a human-readable reason
  }

  CASE("begin() routes TxDone|RxDone|HeaderError|CrcError to DIO1");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    const SPIClass::Frame *irq = SPI.last(SX1280_CMD_SET_DIO_IRQ_PARAMS);
    CHECK(irq != nullptr);
    if (irq) {
      CHECK_EQ(((*irq)[1] << 8) | (*irq)[2], 0x0063); // IRQ mask
      CHECK_EQ(((*irq)[3] << 8) | (*irq)[4], 0x0063); // DIO1 mask
    }
  }

  CASE("DEFECT: a reception with CrcError set is dropped even though RxDone is too");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    r.startReceive();
    // On the SX1280 a CRC failure raises BOTH bits for the same reception.
    g_irq = 0x0002 | 0x0040;
    CHECK(!r.available());
    CHECK_EQ(r.getRxErrors(), 1u);
    CHECK_EQ(g_irq, 0); // the driver cleared the IRQs so RX can continue
    // A clean RxDone is still delivered.
    g_irq = 0x0002;
    CHECK(r.available());
    CHECK_EQ(r.getRxErrors(), 1u);
    // HeaderError alone counts too.
    g_irq = 0x0020;
    CHECK(!r.available());
    CHECK_EQ(r.getRxErrors(), 2u);
  }

  CASE("receive() re-checks the error bits: a CRC failure in the gap yields nothing");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    r.startReceive();
    g_irq = 0x0002;
    CHECK(r.available());
    g_irq |= 0x0040; // a second, corrupted reception landed in between
    uint8_t buf[TALLY_PACKET_SIZE];
    CHECK_EQ(r.receive(buf, TALLY_PACKET_SIZE), 0);
    CHECK_EQ(r.getRxErrors(), 1u);
  }

  CASE("receive() hands over exactly the FIFO payload of a clean reception");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    r.startReceive();
    g_rxLen = TALLY_PACKET_SIZE;
    for (uint8_t i = 0; i < TALLY_PACKET_SIZE; i++)
      g_rxBuf[i] = (uint8_t)(0xA0 + i);
    g_irq = 0x0002;
    uint8_t buf[TALLY_PACKET_SIZE] = {0};
    CHECK(r.available());
    CHECK_EQ(r.receive(buf, TALLY_PACKET_SIZE), TALLY_PACKET_SIZE);
    bool same = true;
    for (uint8_t i = 0; i < TALLY_PACKET_SIZE; i++)
      same = same && buf[i] == (uint8_t)(0xA0 + i);
    CHECK(same);
    CHECK_EQ(r.getRxErrors(), 0u);
  }

  CASE("a payload longer than the caller's buffer is dropped and COUNTED, not clipped");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    r.startReceive();
    g_rxLen = 24; // a foreign frame
    g_irq = 0x0002;
    uint8_t buf[TALLY_PACKET_SIZE];
    CHECK(r.available());
    CHECK_EQ(r.receive(buf, TALLY_PACKET_SIZE), 0);
    CHECK_EQ(r.getRxErrors(), 1u);
    CHECK_EQ(SPI.count(SX1280_CMD_READ_BUFFER), 0); // never even read it
  }

  CASE("the profile applies the fleet's frequency, SF9/BW406/CR4-6 and a 10-symbol preamble");
  {
    resetChip();
    E28Radio r;
    CHECK(bringUp(r));
    SPI.reset();
    tallyApplyRadioProfile(r, 0);
    const SPIClass::Frame *fq = SPI.last(SX1280_CMD_SET_RF_FREQUENCY);
    CHECK(fq != nullptr);
    if (fq) {
      uint32_t steps = ((uint32_t)(*fq)[1] << 16) | ((uint32_t)(*fq)[2] << 8) |
                       (*fq)[3];
      // steps = f * 2^18 / 52 MHz; invert and allow one step of rounding.
      uint64_t hz = (uint64_t)steps * 52000000ULL / 262144ULL;
      CHECK(hz + 250 >= TALLY_RF_FREQ_HZ && hz <= TALLY_RF_FREQ_HZ + 250);
    }
    // Preamble byte encodes mantissa * 2^exponent >= 10 symbols, checked via
    // the packet-params frame the next transmit emits.
    uint8_t frame[TALLY_PACKET_SIZE] = {0};
    CHECK(r.startSend(frame, TALLY_PACKET_SIZE));
    const SPIClass::Frame *pp = SPI.last(SX1280_CMD_SET_PACKET_PARAMS);
    CHECK(pp != nullptr);
    if (pp) {
      uint8_t pb = (*pp)[1];
      uint16_t symbols = (uint16_t)(pb & 0x0F) << (pb >> 4);
      CHECK(symbols >= TALLY_PREAMBLE_SYMBOLS);
      CHECK(symbols < 2 * TALLY_PREAMBLE_SYMBOLS); // rounded up, not doubled
    }
  }

  return testSummary("E28_SX1280");
}
