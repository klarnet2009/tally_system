#include "E28_SX1280.h"

E28Radio::E28Radio() {
  _sf = LORA_SF7;
  _bw = LORA_BW_0400;
  _cr = LORA_CR_4_5;
  _preambleByte = 0x0C; // 12 symbols
  _frequency = 2400000000UL; // SX1280 default; callers override via setFrequency
  _power = 1; // Low power mode: ~14dBm with PA (safe for USB power)
  _lastRSSI = 0;
  _lastSNR = 0;
  _connected = false;
  _txActive = false;
  _txSuccess = false;
  _txStartMs = 0;
  _rxMode = RX_NONE;
  _dcRxCount = 0;
  _dcSleepCount = 0;
  _dcPeriodBase = 0x02;
  _lastPktLen = 0xFFFF;
  _rxErrors = 0;
  _initError = E28_OK;
}

const char *E28Radio::initErrorStr() const {
  switch (_initError) {
  case E28_OK:             return "OK";
  case E28_ERR_BUSY_STUCK: return "BUSY stuck (power?)";
  case E28_ERR_MISO_LOW:   return "MISO low (no 3V3?)";
  case E28_ERR_MISO_HIGH:  return "MISO high (no module?)";
  case E28_ERR_READBACK:   return "cfg readback (MOSI/SCK?)";
  }
  return "?";
}

bool E28Radio::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t nss,
                     int8_t busy, int8_t dio1, int8_t rst, int8_t rxen,
                     int8_t txen) {
  _pinNSS = nss;
  _pinBUSY = busy;
  _pinDIO1 = dio1;
  _pinRESET = rst;
  _pinRXEN = rxen;
  _pinTXEN = txen;

  // Configure pins. Preload the NSS latch HIGH *before* pinMode: the ESP32
  // output latch defaults LOW, so OUTPUT-first would glitch NSS low for the
  // rest of this pin setup while SCK/MOSI still float — the chip can read
  // that as the start of a garbage SPI frame (a classic source of
  // works-sometimes warm-boot init failures). Write-then-pinMode brings the
  // pin up already deselected. Same trick for the RF switch pins.
  digitalWrite(_pinNSS, HIGH);
  pinMode(_pinNSS, OUTPUT);
  pinMode(_pinBUSY, INPUT);
  pinMode(_pinDIO1, INPUT);
  if (_pinRESET != -1)
    pinMode(_pinRESET, OUTPUT);
  if (_pinRXEN != -1) {
    digitalWrite(_pinRXEN, LOW);
    pinMode(_pinRXEN, OUTPUT);
  }
  if (_pinTXEN != -1) {
    digitalWrite(_pinTXEN, LOW);
    pinMode(_pinTXEN, OUTPUT);
  }

  // Initialize SPI with CALLER-provided pins. The SX1280 requires MODE0
  // (CPOL=0/CPHA=0); assert it explicitly instead of relying on the Arduino
  // default. The bus is dedicated to the radio, so configuring it once here
  // (the HW retains mode/bitorder/clock until changed) is sufficient.
  // 4 MHz, deliberately far below the chip's 18 MHz max: our frames are 9
  // bytes so SPI speed is irrelevant, while jumper-wire/module-socket wiring
  // plus GPIO-matrix routing erode setup/hold margin at 8 MHz+ — marginal
  // timing there shows up as exactly the "init sometimes fails" flakiness.
  SPI.begin(sck, miso, mosi, _pinNSS);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();

  // Reset the module
  reset();

  // Wait for chip to be ready
  delay(10);
  _connected = true; // assume present; bounded wait clears this on stuck BUSY
  _txActive = false; // a re-init (e.g. field recovery) abandons any in-flight TX
  _rxMode = RX_NONE;     // chip reset forgets its RX state
  _lastPktLen = 0xFFFF;  // and its packet params
  _initError = E28_OK;

  // Post-reset BUSY clears in <1ms on a live chip; a 100ms cap keeps a dead
  // module's begin() (called from recovery loops) from stalling for ~1s.
  // Early bail before the config sequence: with no module each command below
  // would burn the full 1s BUSY timeout (~6+ s per failed begin())
  bool busyOk = waitBusyFor(100);
  uint8_t probe = busyOk ? getChipStatus() : 0x00;

  if (!busyOk || probe == 0xFF || probe == 0x00) {
    if (_pinRESET == -1) {
      // Without a reset line the chip may be wedged in a previous-run mode
      // where the probe fails (sleep keeps BUSY high until woken). Fire a
      // blind SetStandby(RC) — deliberately bypassing the BUSY wait — then
      // re-probe once before declaring the module dead.
      digitalWrite(_pinNSS, LOW);
      SPI.transfer(SX1280_CMD_SET_STANDBY);
      SPI.transfer(0x00); // STDBY_RC
      digitalWrite(_pinNSS, HIGH);
      delay(2);
      _connected = true; // clean slate for the re-probe
      busyOk = waitBusyFor(100);
      probe = busyOk ? getChipStatus() : 0x00;
    }
    if (!busyOk) {
      _initError = E28_ERR_BUSY_STUCK;
      _connected = false;
      return false;
    }
    if (probe == 0xFF || probe == 0x00) {
      _initError = (probe == 0xFF) ? E28_ERR_MISO_HIGH : E28_ERR_MISO_LOW;
      _connected = false;
      return false;
    }
  }

  // Set standby mode
  standby();

  // Set packet type to LoRa
  uint8_t packetType = SX1280_PACKET_TYPE_LORA;
  writeCommand(SX1280_CMD_SET_PACKET_TYPE, &packetType, 1);

  // Apply the stored frequency (defaults to 2.4 GHz; setFrequency() persists
  // the caller's choice so it survives a re-init)
  setFrequency(_frequency);

  // Set default TX power
  setTxPower(_power);

  // Set modulation parameters
  setModulationParams();

  // Set buffer base addresses
  uint8_t bufferAddr[2] = {0x00, 0x00}; // TX base, RX base
  writeCommand(SX1280_CMD_SET_BUFFER_BASE_ADDR, bufferAddr, 2);

  // Configure DIO1 for TX/RX done + corrupted-reception interrupts.
  // HeaderError (bit5) matters under interference: a reception aborted on a
  // damaged header raises no RxDone, and a duty-cycled receiver that doesn't
  // see DIO1 would sit deaf until the timed safety net. Routing it to DIO1
  // makes the slave's unconditional rearmAfterIrq() heal the cycle instantly.
  uint8_t irqParams[8] = {
      0x00, 0x63, // IRQ mask: TxDone | RxDone | HeaderError | CrcError
      0x00, 0x63, // DIO1 mask
      0x00, 0x00, // DIO2 mask
      0x00, 0x00  // DIO3 mask
  };
  writeCommand(SX1280_CMD_SET_DIO_IRQ_PARAMS, irqParams, 8);

  // A BUSY timeout anywhere in the config block above flips _connected false
  // (and writeCommand then no-ops the rest). Treat that as a failed init so the
  // caller's retry re-resets the chip rather than running with half a config.
  if (!_connected) {
    _initError = E28_ERR_BUSY_STUCK;
    return false;
  }

  // Verify chip is connected via GetStatus
  uint8_t status = getChipStatus();
  // SPI returns 0xFF (all high) or 0x00 if no chip connected
  if (status == 0xFF || status == 0x00) {
    _initError = (status == 0xFF) ? E28_ERR_MISO_HIGH : E28_ERR_MISO_LOW;
    _connected = false;
    return false;
  }

  // Config readback: GetPacketType must return LORA. Status reads only prove
  // the MISO line works; this proves commands actually LAND on the chip
  // (MOSI/SCK integrity). Without it, a marginal MOSI wire yields a
  // "successful" init and a radio that is silently deaf/mute — the
  // hardest-to-diagnose flavour of "it didn't start". Failing here routes
  // into the caller's retry/recovery loop with an actionable reason.
  uint8_t pktType = 0xEE; // poison: stays 0xEE if the read never happens
  readCommand(SX1280_CMD_GET_PACKET_TYPE, &pktType, 1);
  if (pktType != SX1280_PACKET_TYPE_LORA) {
    _initError = E28_ERR_READBACK;
    _connected = false;
    return false;
  }

  _connected = true;
  _initError = E28_OK;
  return true;
}

void E28Radio::reset() {
  if (_pinRESET != -1) {
    digitalWrite(_pinRESET, LOW);
    delay(10);
    digitalWrite(_pinRESET, HIGH);
    delay(20);
  } else {
    // No reset line (hub wiring): a warm ESP32 reboot (flashing, EN button,
    // brownout restart) does NOT power-cycle the module, so the SX1280 may
    // still be in sleep/duty-cycle/TX from the previous run — historically
    // seen as "module not connected" right after reflashing. An NSS falling
    // edge wakes the chip from sleep (BUSY goes low when it's ready).
    digitalWrite(_pinNSS, LOW);
    delayMicroseconds(100);
    digitalWrite(_pinNSS, HIGH);
    delay(5);
  }
}

bool E28Radio::waitBusy() {
  // Short-circuit a module already declared gone: with BUSY physically stuck
  // (E28_ERR_BUSY_STUCK is a persistent power/wiring fault, not a glitch),
  // re-polling the same dead pin would burn another full second per call —
  // send() used to stack ~2s this way (standby()'s timeout + its own).
  if (!_connected)
    return false;

  // ⚡ Bolt: Fast-path early return to avoid millis() overhead in tight polling loops
  if (digitalRead(_pinBUSY) == LOW)
    return true;

  // Elapsed-time pattern: immune to millis() rollover (~49.7 days)
  uint32_t start = millis();
  while (digitalRead(_pinBUSY) == HIGH) {
    if (millis() - start > 1000) {
      _connected = false; // BUSY stuck = no module
      _initError = E28_ERR_BUSY_STUCK; // keep diagnostics truthful at runtime
      return false;
    }
    yield();
  }
  return true;
}

bool E28Radio::waitBusyFor(uint32_t timeoutMs) {
  if (digitalRead(_pinBUSY) == LOW)
    return true;

  uint32_t start = millis();
  while (digitalRead(_pinBUSY) == HIGH) {
    if (millis() - start > timeoutMs) {
      _connected = false; // BUSY stuck = no module
      _initError = E28_ERR_BUSY_STUCK;
      return false;
    }
    yield();
  }
  return true;
}

// Every SPI exchange as ONE NSS-framed transfer: header byte(s), then `len`
// body bytes — from txData (command write), or zeros while the body's RX
// bytes are captured into rxData (command read). One fixed buffer: the old
// <=12-byte "fast path" saved ~250 bytes of stack for a handful of cycles
// and was copy-pasted into five functions. false = the BUSY wait failed
// (module wedged/gone) and nothing was clocked — callers must bail; begin()
// re-validates _connected after its config block and re-resets on failure.
bool E28Radio::spiFrame(const uint8_t *header, uint8_t headerLen,
                        const uint8_t *txData, uint8_t *rxData, uint8_t len) {
  if (!waitBusy())
    return false;
  uint32_t totalLen = (uint32_t)headerLen + len; // worst case 3 + 255 = 258
  uint8_t txBuf[260];
  uint8_t rxBuf[260];
  memcpy(txBuf, header, headerLen);
  if (txData)
    memcpy(txBuf + headerLen, txData, len);
  else
    memset(txBuf + headerLen, 0, len);
  digitalWrite(_pinNSS, LOW);
  SPI.transferBytes(txBuf, rxBuf, totalLen);
  digitalWrite(_pinNSS, HIGH);
  if (rxData)
    memcpy(rxData, rxBuf + headerLen, len);
  // No trailing waitBusy: CPU execution (e.g. GPIO toggling) overlaps with
  // the radio's BUSY time — the next command's waitBusy is the guard.
  return true;
}

void E28Radio::writeCommand(uint8_t cmd, uint8_t *data, uint8_t len) {
  spiFrame(&cmd, 1, data, nullptr, len);
}

void E28Radio::readCommand(uint8_t cmd, uint8_t *data, uint8_t len) {
  uint8_t hdr[2] = {cmd, 0x00}; // NOP after the opcode
  spiFrame(hdr, 2, nullptr, data, len);
}

void E28Radio::setFrequency(uint32_t frequency) {
  _frequency = frequency; // remember so begin()/re-init re-applies it
  // Frequency = (rfFreq * Fxtal) / 2^18
  // Fxtal = 52 MHz for SX1280
  // 64-bit integer math: float loses precision at 2.4e9 (24-bit mantissa)
  uint32_t rfFreq = (uint32_t)((uint64_t)frequency * 262144ULL / 52000000ULL);

  uint8_t freqParams[3] = {(uint8_t)((rfFreq >> 16) & 0xFF),
                           (uint8_t)((rfFreq >> 8) & 0xFF),
                           (uint8_t)(rfFreq & 0xFF)};
  writeCommand(SX1280_CMD_SET_RF_FREQUENCY, freqParams, 3);
}

void E28Radio::setTxPower(int8_t power) {
  // Clamp power to valid range
  if (power < -18)
    power = -18;
  if (power > 12)
    power = 12;
  _power = power;

  // Power = -18 + power (0-31)
  uint8_t powerReg = (uint8_t)(power + 18);
  uint8_t rampTime = 0xE0; // 20us ramp time

  uint8_t txParams[2] = {powerReg, rampTime};
  writeCommand(SX1280_CMD_SET_TX_PARAMS, txParams, 2);
}

void E28Radio::setSpreadingFactor(uint8_t sf) {
  _sf = sf;
  setModulationParams();
}

void E28Radio::setBandwidth(uint8_t bw) {
  _bw = bw;
  setModulationParams();
}

void E28Radio::setCodingRate(uint8_t cr) {
  _cr = cr;
  setModulationParams();
}

void E28Radio::setModulationParams() {
  uint8_t modParams[3] = {_sf, _bw, _cr};
  writeCommand(SX1280_CMD_SET_MODULATION_PARAMS, modParams, 3);
}

void E28Radio::setPreambleLength(uint16_t symbols) {
  // SX1280 LoRa preamble byte: (exponent << 4) | mantissa,
  // length = mantissa * 2^exponent. Round up so the preamble is never
  // shorter than requested (matters for duty-cycle RX detection windows).
  uint8_t exp = 0;
  uint16_t mant = symbols;
  while (mant > 15 && exp < 15) {
    mant = (mant + 1) / 2;
    exp++;
  }
  if (mant > 15)
    mant = 15;
  _preambleByte = (uint8_t)((exp << 4) | mant);
  _lastPktLen = 0xFFFF; // packet params embed the preamble — force re-send
}

void E28Radio::setPacketParams(uint8_t payloadLen) {
  // Skip the 7-byte SPI command when nothing changed (every TX packet has
  // the same length, so this saves a command + BUSY round-trip per packet)
  if (_lastPktLen == payloadLen)
    return;
  _lastPktLen = payloadLen;

  uint8_t pktParams[7] = {
      _preambleByte,   // Preamble length (default 0x0C = 12 symbols)
      0x00,            // Header type: explicit
      payloadLen,      // Payload length
      0x01,            // CRC on
      0x00,            // Standard IQ
      0x00,       0x00 // Reserved
  };
  writeCommand(SX1280_CMD_SET_PACKET_PARAMS, pktParams, 7);
}

void E28Radio::clearIrqStatus() {
  uint8_t cmd = SX1280_CMD_CLR_IRQ_STATUS;
  uint8_t clr[2] = {0xFF, 0xFF}; // clear every IRQ bit
  spiFrame(&cmd, 1, clr, nullptr, 2);
}

uint16_t E28Radio::getIrqStatus() {
  uint8_t hdr[2] = {SX1280_CMD_GET_IRQ_STATUS, 0x00}; // NOP
  uint8_t irq[2] = {0};
  if (!spiFrame(hdr, 2, nullptr, irq, 2))
    return 0;
  return ((uint16_t)irq[0] << 8) | irq[1];
}

bool E28Radio::writeTxBuffer(uint8_t *data, uint8_t len) {
  uint8_t hdr[2] = {SX1280_CMD_WRITE_BUFFER, 0x00}; // Offset
  return spiFrame(hdr, 2, data, nullptr, len);
}

uint32_t E28Radio::txTimeoutMs(uint8_t payloadLen) const {
  // LoRa airtime from the live modulation config (register encodings):
  // SF = _sf>>4, CR index = _cr (0x01..0x04 = 4/5..4/8), preamble symbols =
  // mantissa << exponent. Explicit header, CRC on, no IQ invert, DE=0
  // (DE only matters at SF11/12 with BW<=125kHz, which this chip never runs).
  uint32_t bwHz;
  switch (_bw) {
  case LORA_BW_1600: bwHz = 1625000; break;
  case LORA_BW_0800: bwHz = 812500; break;
  case LORA_BW_0200: bwHz = 203125; break;
  case LORA_BW_0400:
  default:           bwHz = 406250; break;
  }
  const uint8_t sf = _sf >> 4;
  const uint16_t preambleSym =
      (uint16_t)(_preambleByte & 0x0F) << (_preambleByte >> 4);
  // Payload symbols: 8 + max(ceil((8PL - 4SF + 28 + 16) / 4SF) * (CR+4), 0)
  int32_t num = 8 * (int32_t)payloadLen - 4 * sf + 28 + 16;
  int32_t denom = 4 * sf;
  int32_t sym = (num > 0) ? ((num + denom - 1) / denom) * (_cr + 4) : 0;
  uint32_t payloadSym = 8 + (uint32_t)sym;
  // (preamble + 4.25 + payload) * Tsym, all in integer microseconds
  uint64_t tsymUs = ((uint64_t)1 << sf) * 1000000ULL / bwHz;
  uint64_t airtimeUs =
      (4ULL * preambleSym + 17 + 4ULL * payloadSym) * tsymUs / 4;
  return (uint32_t)(airtimeUs / 1000ULL) + 50; // +50ms margin
}

bool E28Radio::send(uint8_t *data, uint8_t len) {
  if (!_connected)
    return false;

  // Go to standby first (disables EN pins)
  standby();

  // Set packet length
  setPacketParams(len);

  if (!writeTxBuffer(data, len))
    return false;

  // Clear IRQ status
  clearIrqStatus();

  // Enable PA BEFORE starting TX
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, LOW);
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, HIGH);
  delayMicroseconds(50);

  // Start transmission (timeout = 0 for continuous)
  uint8_t txParams[3] = {0x00, 0x00, 0x00};
  writeCommand(SX1280_CMD_SET_TX, txParams, 3);
  if (!_connected) { // SET_TX never reached the chip — PA off, fail now
    if (_pinTXEN != -1)
      digitalWrite(_pinTXEN, LOW);
    return false;
  }

  // Wait for TX done (elapsed-time pattern: immune to millis() rollover)
  uint32_t txStart = millis();
  uint32_t txTimeout = txTimeoutMs(len); // airtime + margin (~146ms at SF9)
  while (!isTxDone()) { // TxDone bit
    if (millis() - txStart > txTimeout) {
      if (_pinTXEN != -1)
        digitalWrite(_pinTXEN, LOW);
      standby();
      return false;
    }
    yield();
  }

  clearIrqStatus();
  // Disable PA after TX done
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  standby();
  return true;
}

bool E28Radio::startSend(uint8_t *data, uint8_t len) {
  if (!_connected || _txActive)
    return false;

  standby();
  setPacketParams(len);

  if (!writeTxBuffer(data, len))
    return false;

  clearIrqStatus();

  // Enable PA BEFORE starting TX
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, LOW);
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, HIGH);
  delayMicroseconds(50);

  uint8_t txParams[3] = {0x00, 0x00, 0x00};
  writeCommand(SX1280_CMD_SET_TX, txParams, 3);
  // If a BUSY timeout swallowed SET_TX inside that writeCommand, the chip
  // never started transmitting — don't claim a TX in flight (_txActive would
  // block the queue ~100ms until checkTxDone()'s timeout) or leave the PA on.
  if (!_connected) {
    if (_pinTXEN != -1)
      digitalWrite(_pinTXEN, LOW);
    return false;
  }

  _txActive = true;
  _txStartMs = millis();
  _txLen = len;
  return true;
}

bool E28Radio::checkTxDone() {
  if (!_txActive)
    return true;

  // Airtime-derived cap mirrors the blocking send() (~96ms airtime at SF9,
  // so ~146ms with margin)
  bool done = isTxDone();
  if (!done && millis() - _txStartMs <= txTimeoutMs(_txLen))
    return false;

  // done == true: real TxDone; done == false: timeout (TX failed).
  // Callers count drops on !txSucceeded() so a stuck PA/antenna fault shows up.
  _txSuccess = done;

  // Teardown (same order as blocking send): IRQ clear, PA off, standby
  clearIrqStatus();
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  standby();
  _txActive = false;
  return true;
}

bool E28Radio::isTxDone() {
  // ⚡ Bolt: Fast-path hardware pin polling prevents SPI bus starvation during TxDone wait
  if (_pinDIO1 != -1 && digitalRead(_pinDIO1) == LOW) {
    return false;
  }
  return (getIrqStatus() & 0x0001) != 0;
}

void E28Radio::startReceive() {
  if (!_connected)
    return;

  // Arming RX abandons any in-flight async TX (standby below forces the PA
  // off); without this a TX interrupted by an RX arm would leave _txActive
  // wedged true and block startSend() forever
  _txActive = false;

  standby();
  setPacketParams(E28_MAX_PACKET_SIZE);
  clearIrqStatus();

  // Enable LNA
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, HIGH);
  delayMicroseconds(50);

  // Start continuous RX
  uint8_t rxParams[3] = {0xFF, 0xFF, 0xFF}; // Continuous RX
  writeCommand(SX1280_CMD_SET_RX, rxParams, 3);

  if (!_connected) // SET_RX never reached the chip — don't record a mode
    return;        // the chip isn't actually in (rearmAfterIrq trusts it)
  _rxMode = RX_CONTINUOUS;
}

void E28Radio::startReceiveDutyCycle(uint16_t rxCount, uint16_t sleepCount,
                                     uint8_t periodBase) {
  if (!_connected)
    return;

  _txActive = false; // same contract as startReceive(): RX arm aborts TX

  standby();
  setPacketParams(E28_MAX_PACKET_SIZE);
  clearIrqStatus();

  // Enable LNA
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, HIGH);
  delayMicroseconds(50);

  uint8_t params[5] = {periodBase, (uint8_t)(rxCount >> 8),
                       (uint8_t)(rxCount & 0xFF), (uint8_t)(sleepCount >> 8),
                       (uint8_t)(sleepCount & 0xFF)};
  writeCommand(SX1280_CMD_SET_RX_DUTY_CYCLE, params, 5);

  if (!_connected) // command swallowed — keep the bookkeeping truthful
    return;
  _rxMode = RX_DUTY_CYCLE;
  _dcRxCount = rxCount;
  _dcSleepCount = sleepCount;
  _dcPeriodBase = periodBase;
}

void E28Radio::rearmAfterIrq() {
  switch (_rxMode) {
  case RX_DUTY_CYCLE:
    // Mandatory full re-issue: any RxDone (even a CRC error) ends the cycle
    startReceiveDutyCycle(_dcRxCount, _dcSleepCount, _dcPeriodBase);
    break;
  case RX_CONTINUOUS:
    clearRxIrq(); // cheap: IRQ clear + SET_RX, no standby
    break;
  case RX_NONE:
    break;
  }
}

void E28Radio::restartReceive() {
  switch (_rxMode) {
  case RX_DUTY_CYCLE:
    startReceiveDutyCycle(_dcRxCount, _dcSleepCount, _dcPeriodBase);
    break;
  case RX_CONTINUOUS:
    startReceive();
    break;
  case RX_NONE:
    break;
  }
}

void E28Radio::clearRxIrq() {
  if (!_connected)
    return;

  // Fast RX re-arm: clear IRQ + restart continuous RX (no standby needed)
  clearIrqStatus();
  // Ensure LNA is enabled
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, HIGH);
  // Re-issue continuous RX command (forces back to RX if radio exited)
  uint8_t rxParams[3] = {0xFF, 0xFF, 0xFF};
  writeCommand(SX1280_CMD_SET_RX, rxParams, 3);
}

bool E28Radio::available() {
  if (!_connected)
    return false;

  // Read IRQ register directly (no DIO1 pin check — unreliable on some boards)
  uint16_t irq = getIrqStatus();
  // RxDone first: when a valid frame completes and a colliding/foreign frame
  // latches CrcError alongside it, draining the FIFO must win — receive()
  // clears every IRQ bit anyway. The old order wiped the pending RxDone
  // together with the error, silently abandoning a good frame.
  if (irq & 0x0002) // RxDone bit
    return true;
  // Corrupted reception: CRC fail on a full packet, or a header that didn't
  // survive interference. Counted so the error rate is visible on the
  // status lines.
  if (irq & 0x0060) { // CrcError | HeaderError
    _rxErrors++;
    clearIrqStatus();
  }
  return false;
}

uint8_t E28Radio::receive(uint8_t *buffer, uint8_t maxLen) {
  if (!_connected)
    return 0;

  uint8_t statusHdr[2] = {SX1280_CMD_GET_RX_BUFFER_STATUS, 0x00}; // NOP
  uint8_t bufStatus[2] = {0};
  if (!spiFrame(statusHdr, 2, nullptr, bufStatus, 2))
    return 0;

  uint8_t payloadLen = bufStatus[0];
  uint8_t bufferOffset = bufStatus[1];

  if (payloadLen > maxLen) {
    payloadLen = maxLen;
  }

  // Read data from buffer
  uint8_t readHdr[3] = {SX1280_CMD_READ_BUFFER, bufferOffset, 0x00}; // NOP
  if (!spiFrame(readHdr, 3, nullptr, buffer, payloadLen))
    return 0;

  // Read packet RSSI/SNR (LoRa packet status: byte0 = rssiSync, byte1 = snr)
  uint8_t pktStatus[5] = {0};
  readCommand(SX1280_CMD_GET_PACKET_STATUS, pktStatus, 5);
  // If the module dropped off the bus during that read, pktStatus is still
  // all-zero — keep the last real measurement instead of overwriting it with
  // a fake crisp 0 dBm / 0 dB that looks like a valid reading.
  if (_connected) {
    _lastRSSI = -(int8_t)(pktStatus[0] / 2); // RSSI = -rssiSync/2 dBm
    _lastSNR = (int8_t)pktStatus[1] / 4;     // SNR = snr/4 dB (two's complement)
  }

  // Clear IRQ
  clearIrqStatus();

  return payloadLen;
}

int8_t E28Radio::getRSSI() { return _lastRSSI; }

int8_t E28Radio::getSNR() { return _lastSNR; }

int8_t E28Radio::getRssiInst() {
  if (!_connected)
    return 0;
  uint8_t v = 0;
  readCommand(SX1280_CMD_GET_RSSI_INST, &v, 1);
  if (!_connected) // module died during the read — v is still 0, not a
    return 0;      // measurement; 0 stays the documented "no reading" value
  return -(int8_t)(v / 2); // same -x/2 dBm encoding as packet RSSI
}

void E28Radio::standby() {
  uint8_t stdbyConfig = SX1280_STANDBY_RC;
  writeCommand(SX1280_CMD_SET_STANDBY, &stdbyConfig, 1);

  // Standby = RF switch off
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, LOW);
}

uint8_t E28Radio::getChipStatus() {
  // Bounded BUSY wait (10ms cap). Deliberately not waitBusy(): this is the
  // recovery probe — it must not stall 1s per call or flip _connected when
  // the module is absent
  uint32_t start = millis();
  while (digitalRead(_pinBUSY) == HIGH && millis() - start <= 10)
    yield();

  digitalWrite(_pinNSS, LOW);
  // SX1280 drives its status onto MISO during every byte of the transaction,
  // including the opcode byte. Sample both and prefer a valid one: if the
  // first byte is marginal (MISO settling right after NSS falls), the NOP
  // byte still carries good status — avoids a false "module not connected"
  // verdict on a healthy chip. A truly absent module fails both (0x00/0xFF).
  uint8_t txBuf[2] = {SX1280_CMD_GET_STATUS, 0x00};
  uint8_t rxBuf[2] = {0, 0};
  SPI.transferBytes(txBuf, rxBuf, 2);
  digitalWrite(_pinNSS, HIGH);

  bool firstValid = (rxBuf[0] != 0x00 && rxBuf[0] != 0xFF);
  return firstValid ? rxBuf[0] : rxBuf[1];
}
