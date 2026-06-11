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
  _initError = E28_OK;
}

const char *E28Radio::initErrorStr() const {
  switch (_initError) {
  case E28_OK:             return "OK";
  case E28_ERR_BUSY_STUCK: return "BUSY stuck (power?)";
  case E28_ERR_MISO_LOW:   return "MISO low (no 3V3?)";
  case E28_ERR_MISO_HIGH:  return "MISO high (no module?)";
  }
  return "?";
}

bool E28Radio::begin() {
#ifndef E28_PIN_RXEN
#define E28_PIN_RXEN -1
#endif
#ifndef E28_PIN_TXEN
#define E28_PIN_TXEN -1
#endif
  return begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI, E28_PIN_NSS,
               E28_PIN_BUSY, E28_PIN_DIO1, E28_PIN_RESET, E28_PIN_RXEN,
               E28_PIN_TXEN);
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

  // Configure pins
  pinMode(_pinNSS, OUTPUT);
  pinMode(_pinBUSY, INPUT);
  pinMode(_pinDIO1, INPUT);
  if (_pinRESET != -1)
    pinMode(_pinRESET, OUTPUT);
  if (_pinRXEN != -1) {
    pinMode(_pinRXEN, OUTPUT);
    digitalWrite(_pinRXEN, LOW);
  }
  if (_pinTXEN != -1) {
    pinMode(_pinTXEN, OUTPUT);
    digitalWrite(_pinTXEN, LOW);
  }

  digitalWrite(_pinNSS, HIGH);

  // Initialize SPI with CALLER-provided pins
  SPI.begin(sck, miso, mosi, _pinNSS);
  SPI.setFrequency(8000000); // 8 MHz

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

  // Configure DIO1 for TX/RX done + CRC error interrupts
  uint8_t irqParams[8] = {
      0x00, 0x43, // IRQ mask: TxDone | RxDone | CrcError
      0x00, 0x43, // DIO1 mask
      0x00, 0x00, // DIO2 mask
      0x00, 0x00  // DIO3 mask
  };
  writeCommand(SX1280_CMD_SET_DIO_IRQ_PARAMS, irqParams, 8);

  // Verify chip is connected via GetStatus
  uint8_t status = getChipStatus();
  // SPI returns 0xFF (all high) or 0x00 if no chip connected
  if (status == 0xFF || status == 0x00) {
    _initError = (status == 0xFF) ? E28_ERR_MISO_HIGH : E28_ERR_MISO_LOW;
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

void E28Radio::waitBusy() {
  // ⚡ Bolt: Fast-path early return to avoid millis() overhead in tight polling loops
  if (digitalRead(_pinBUSY) == LOW) return;

  // Elapsed-time pattern: immune to millis() rollover (~49.7 days)
  uint32_t start = millis();
  while (digitalRead(_pinBUSY) == HIGH) {
    if (millis() - start > 1000) {
      _connected = false; // BUSY stuck = no module
      break;
    }
    yield();
  }
}

bool E28Radio::waitBusyFor(uint32_t timeoutMs) {
  if (digitalRead(_pinBUSY) == LOW)
    return true;

  uint32_t start = millis();
  while (digitalRead(_pinBUSY) == HIGH) {
    if (millis() - start > timeoutMs) {
      _connected = false; // BUSY stuck = no module
      return false;
    }
    yield();
  }
  return true;
}

void E28Radio::writeCommand(uint8_t cmd, uint8_t *data, uint8_t len) {
  waitBusy();

  digitalWrite(_pinNSS, LOW);
  if (len > 0) {
    // ⚡ Bolt: Fast-path for small writes (e.g. IRQ clears, config) to avoid large stack alloc/memcpy
    if (len <= 8) {
      uint8_t txBuf[9];
      txBuf[0] = cmd;
      // ⚡ Bolt: Rely on compiler intrinsics for memory copies instead of manual loops
      memcpy(&txBuf[1], data, len);
      SPI.writeBytes(txBuf, len + 1);
    } else {
      // ⚡ Bolt: Coalesce cmd and data into a single block transfer for larger payloads
      uint8_t txBuf[260];
      txBuf[0] = cmd;
      memcpy(&txBuf[1], data, len);
      SPI.writeBytes(txBuf, len + 1);
    }
  } else {
    SPI.transfer(cmd);
  }
  digitalWrite(_pinNSS, HIGH);

  // ⚡ Bolt: Removed redundant trailing waitBusy() to allow CPU execution (e.g. GPIO toggling) to overlap with radio BUSY time
}

void E28Radio::readCommand(uint8_t cmd, uint8_t *data, uint8_t len) {
  waitBusy();

  digitalWrite(_pinNSS, LOW);
  if (len > 0) {
    // ⚡ Bolt: Fast-path for small reads (e.g. IRQ checks) to avoid large stack alloc/memcpy
    if (len <= 8) {
      uint8_t txBuf[10] = {cmd, 0x00}; // NOP
      uint8_t rxBuf[10] = {0};
      uint32_t totalLen = len + 2;
      SPI.transferBytes(txBuf, rxBuf, totalLen);
      // ⚡ Bolt: Rely on compiler intrinsics for memory copies instead of manual loops
      memcpy(data, &rxBuf[2], len);
    } else {
      // ⚡ Bolt: Coalesce cmd, NOP, and read dummy bytes into a single block transfer
      uint8_t txBuf[260];
      uint8_t rxBuf[260];
      uint32_t totalLen = len + 2;

      memset(txBuf, 0, totalLen);
      txBuf[0] = cmd;
      txBuf[1] = 0x00; // NOP

      SPI.transferBytes(txBuf, rxBuf, totalLen);

      memcpy(data, &rxBuf[2], len);
    }
  } else {
    SPI.transfer(cmd);
    SPI.transfer(0x00); // NOP
  }
  digitalWrite(_pinNSS, HIGH);
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
  // ⚡ Bolt: Inline SPI transaction to bypass writeCommand wrapper overhead in high-frequency loops
  waitBusy();
  digitalWrite(_pinNSS, LOW);
  uint8_t txBuf[3] = {SX1280_CMD_CLR_IRQ_STATUS, 0xFF, 0xFF};
  SPI.writeBytes(txBuf, 3);
  digitalWrite(_pinNSS, HIGH);
  // ⚡ Bolt: Removed redundant trailing waitBusy() to overlap CPU execution with radio BUSY time
}

uint16_t E28Radio::getIrqStatus() {
  // ⚡ Bolt: Inline SPI transaction to bypass readCommand wrapper array creation and func call overhead
  waitBusy();
  digitalWrite(_pinNSS, LOW);
  uint8_t txBuf[4] = {SX1280_CMD_GET_IRQ_STATUS, 0x00, 0x00, 0x00};
  uint8_t rxBuf[4] = {0};
  SPI.transferBytes(txBuf, rxBuf, 4);
  digitalWrite(_pinNSS, HIGH);
  return ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
}

bool E28Radio::send(uint8_t *data, uint8_t len) {
  if (!_connected)
    return false;

  // Go to standby first (disables EN pins)
  standby();

  // Set packet length
  setPacketParams(len);

  // Write data to buffer
  waitBusy();
  digitalWrite(_pinNSS, LOW);
  if (len > 0) {
    // ⚡ Bolt: Fast-path for small payloads to avoid large stack alloc/memcpy overhead
    if (len <= 8) {
        uint8_t txBuf[10];
        uint32_t totalLen = len + 2;
        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        // ⚡ Bolt: Rely on compiler intrinsics for memory copies instead of manual loops
        memcpy(&txBuf[2], data, len);
        SPI.writeBytes(txBuf, totalLen);
    } else {
        // ⚡ Bolt: Coalesce command, offset and payload into a single SPI transfer
        // Use fixed stack buffer
        uint8_t txBuf[260];
        uint32_t totalLen = len + 2;

        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        memcpy(&txBuf[2], data, len);

        SPI.writeBytes(txBuf, totalLen);
    }
  } else {
    SPI.transfer(SX1280_CMD_WRITE_BUFFER);
    SPI.transfer(0x00); // Offset
  }
  digitalWrite(_pinNSS, HIGH);

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

  // Wait for TX done (elapsed-time pattern: immune to millis() rollover)
  uint32_t txStart = millis(); // 100ms TxDone timeout (packet is <5ms)
  while (!isTxDone()) { // TxDone bit
    if (millis() - txStart > 100) {
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

  // Write data to buffer
  waitBusy();
  digitalWrite(_pinNSS, LOW);
  if (len > 0) {
    // ⚡ Bolt: Fast-path for small payloads to avoid large stack alloc/memcpy overhead
    if (len <= 8) {
        uint8_t txBuf[10];
        uint32_t totalLen = len + 2;
        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        // ⚡ Bolt: Rely on compiler intrinsics for memory copies instead of manual loops
        memcpy(&txBuf[2], data, len);
        SPI.writeBytes(txBuf, totalLen);
    } else {
        // ⚡ Bolt: Coalesce command, offset and payload into a single SPI transfer
        // Use fixed stack buffer
        uint8_t txBuf[260];
        uint32_t totalLen = len + 2;

        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        memcpy(&txBuf[2], data, len);

        SPI.writeBytes(txBuf, totalLen);
    }
  } else {
    SPI.transfer(SX1280_CMD_WRITE_BUFFER);
    SPI.transfer(0x00); // Offset
  }
  digitalWrite(_pinNSS, HIGH);

  clearIrqStatus();

  // Enable PA BEFORE starting TX
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, LOW);
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, HIGH);
  delayMicroseconds(50);

  uint8_t txParams[3] = {0x00, 0x00, 0x00};
  writeCommand(SX1280_CMD_SET_TX, txParams, 3);

  _txActive = true;
  _txStartMs = millis();
  return true;
}

bool E28Radio::checkTxDone() {
  if (!_txActive)
    return true;

  // 100ms cap mirrors the blocking send(); a tally packet is on air <15ms
  bool done = isTxDone();
  if (!done && millis() - _txStartMs <= 100)
    return false;

  // done == true: real TxDone; done == false: 100ms timeout (TX failed).
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

  _rxMode = RX_CONTINUOUS;
}

void E28Radio::startReceiveDutyCycle(uint16_t rxCount, uint16_t sleepCount,
                                     uint8_t periodBase) {
  if (!_connected)
    return;

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
  // Check for CRC error first — discard bad packet silently
  if (irq & 0x0040) { // CrcError bit
    clearIrqStatus();
    return false;
  }
  return (irq & 0x0002) != 0; // RxDone bit
}

uint8_t E28Radio::receive(uint8_t *buffer, uint8_t maxLen) {
  if (!_connected)
    return 0;

  // Wait for SX1280 to finish internal processing
  waitBusy();

  // ⚡ Bolt: Inline GET_RX_BUFFER_STATUS to bypass readCommand wrapper overhead and redundant waitBusy() on hot RX path
  digitalWrite(_pinNSS, LOW);
  uint8_t txBufStatus[4] = {SX1280_CMD_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00};
  uint8_t rxBufStatus[4] = {0};
  SPI.transferBytes(txBufStatus, rxBufStatus, 4);
  digitalWrite(_pinNSS, HIGH);

  uint8_t payloadLen = rxBufStatus[2];
  uint8_t bufferOffset = rxBufStatus[3];

  if (payloadLen > maxLen) {
    payloadLen = maxLen;
  }

  // Read data from buffer
  waitBusy();
  digitalWrite(_pinNSS, LOW);
  if (payloadLen > 0) {
    // ⚡ Bolt: Fast-path for small payloads to avoid large stack alloc/memset/memcpy overhead
    if (payloadLen <= 8) {
        uint8_t txBuf[11] = {0};
        uint8_t rxBuf[11] = {0};
        uint32_t totalLen = payloadLen + 3;

        txBuf[0] = SX1280_CMD_READ_BUFFER;
        txBuf[1] = bufferOffset;
        txBuf[2] = 0x00; // NOP

        SPI.transferBytes(txBuf, rxBuf, totalLen);

        // ⚡ Bolt: Rely on compiler intrinsics for memory copies instead of manual loops
        memcpy(buffer, &rxBuf[3], payloadLen);
    } else {
        // ⚡ Bolt: Coalesce command, offset, NOP and read dummy bytes into a single SPI transfer
        // Use fixed stack buffer
        uint8_t txBuf[260];
        uint8_t rxBuf[260];
        uint32_t totalLen = payloadLen + 3;

        memset(txBuf, 0, totalLen);
        txBuf[0] = SX1280_CMD_READ_BUFFER;
        txBuf[1] = bufferOffset;
        txBuf[2] = 0x00; // NOP

        SPI.transferBytes(txBuf, rxBuf, totalLen);

        memcpy(buffer, &rxBuf[3], payloadLen);
    }
  } else {
    SPI.transfer(SX1280_CMD_READ_BUFFER);
    SPI.transfer(bufferOffset);
    SPI.transfer(0x00); // NOP
  }
  digitalWrite(_pinNSS, HIGH);

  // Read packet RSSI/SNR (LoRa packet status: byte0 = rssiSync, byte1 = snr)
  uint8_t pktStatus[5] = {0};
  readCommand(SX1280_CMD_GET_PACKET_STATUS, pktStatus, 5);
  _lastRSSI = -(int8_t)(pktStatus[0] / 2); // RSSI = -rssiSync/2 dBm
  _lastSNR = (int8_t)pktStatus[1] / 4;     // SNR = snr/4 dB (two's complement)

  // Clear IRQ
  clearIrqStatus();

  return payloadLen;
}

int8_t E28Radio::getRSSI() { return _lastRSSI; }

int8_t E28Radio::getSNR() { return _lastSNR; }

void E28Radio::sleep() {
  // Disable RF switch
  if (_pinTXEN != -1)
    digitalWrite(_pinTXEN, LOW);
  if (_pinRXEN != -1)
    digitalWrite(_pinRXEN, LOW);

  uint8_t sleepConfig = 0x00; // Cold start
  writeCommand(SX1280_CMD_SET_SLEEP, &sleepConfig, 1);
}

void E28Radio::standby() {
  uint8_t stdbyConfig = SX1280_STANDBY_RC;
  writeCommand(SX1280_CMD_SET_STANDBY, &stdbyConfig, 1);

  // We don't disable LNA/PA here immediately because standby is often
  // intermediate state before TX/RX. But for power saving, maybe we should?
  // Let's assume user calls startReceive() or send() after standby.
  // If we are just idling, sleep() is better.
  // But to be safe, maybe disable PA?
  // if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
  // if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);
  // Commented out to behave like standard transceivers that keep RF switch
  // settings? No, standard is Standby = RF Off.
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
  // ⚡ Bolt: Coalesce get status to avoid 2 separate single byte transfers
  uint8_t txBuf[2] = {SX1280_CMD_GET_STATUS, 0x00};
  uint8_t rxBuf[2] = {0, 0};
  SPI.transferBytes(txBuf, rxBuf, 2);
  digitalWrite(_pinNSS, HIGH);
  return rxBuf[0];
}

bool E28Radio::checkConnection() {
  uint8_t status = getChipStatus();
  _connected = (status != 0xFF && status != 0x00);
  return _connected;
}
