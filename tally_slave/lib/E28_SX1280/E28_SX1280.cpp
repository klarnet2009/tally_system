#include "E28_SX1280.h"

E28Radio::E28Radio() {
    _sf = LORA_SF7;
    _bw = LORA_BW_0400;
    _cr = LORA_CR_4_5;
    _power = 1;  // Low power mode: ~14dBm with PA (safe for USB power)
    _lastRSSI = 0;
    _lastSNR = 0;
    _connected = false;
}

bool E28Radio::begin(int8_t sck, int8_t miso, int8_t mosi,
                     int8_t nss, int8_t busy, int8_t dio1, 
                     int8_t rst, int8_t rxen, int8_t txen) {
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
    if (_pinRESET != -1) pinMode(_pinRESET, OUTPUT);
    if (_pinRXEN != -1) { pinMode(_pinRXEN, OUTPUT); digitalWrite(_pinRXEN, LOW); }
    if (_pinTXEN != -1) { pinMode(_pinTXEN, OUTPUT); digitalWrite(_pinTXEN, LOW); }
    
    digitalWrite(_pinNSS, HIGH);
    
    // Initialize SPI with CALLER-provided pins
    SPI.begin(sck, miso, mosi, _pinNSS);
    SPI.setFrequency(8000000);  // 8 MHz
    
    // Reset the module
    reset();
    
    // Wait for chip to be ready
    delay(10);
    waitBusy();
    
    // Set standby mode
    standby();
    
    // Set packet type to LoRa
    uint8_t packetType = SX1280_PACKET_TYPE_LORA;
    writeCommand(SX1280_CMD_SET_PACKET_TYPE, &packetType, 1);
    
    // Set default frequency (2.4 GHz)
    setFrequency(2400000000);
    
    // Set default TX power
    setTxPower(_power);
    
    // Set modulation parameters
    setModulationParams();
    
    // Set buffer base addresses
    uint8_t bufferAddr[2] = {0x00, 0x00};  // TX base, RX base
    writeCommand(SX1280_CMD_SET_BUFFER_BASE_ADDR, bufferAddr, 2);
    
    // Configure DIO1 for TX/RX done interrupts
    uint8_t irqParams[8] = {
        0x00, 0x03,  // IRQ mask: TxDone | RxDone
        0x00, 0x03,  // DIO1 mask
        0x00, 0x00,  // DIO2 mask
        0x00, 0x00   // DIO3 mask
    };
    writeCommand(SX1280_CMD_SET_DIO_IRQ_PARAMS, irqParams, 8);
    
    // Verify chip is connected via GetStatus
    uint8_t status = getChipStatus();
    // SPI returns 0xFF (all high) or 0x00 if no chip connected
    if (status == 0xFF || status == 0x00) {
        _connected = false;
        return false;
    }
    _connected = true;
    return true;
}

void E28Radio::reset() {
    if (_pinRESET != -1) {
        digitalWrite(_pinRESET, LOW);
        delay(10);
        digitalWrite(_pinRESET, HIGH);
        delay(20);
    } else {
        // Software reset if no pin
        delay(50); 
        // Maybe send a reset command? But SPI might be stuck if chip is?
        // Usually power-on reset is enough.
    }
}

void E28Radio::waitBusy() {
    uint32_t timeout = millis() + 1000;
    while (digitalRead(_pinBUSY) == HIGH) {
        if (millis() > timeout) {
            _connected = false; // BUSY stuck = no module
            break;
        }
        yield();
    }
}

void E28Radio::writeCommand(uint8_t cmd, uint8_t* data, uint8_t len) {
    waitBusy();
    
    digitalWrite(_pinNSS, LOW);
    if (len > 0) {
        // ⚡ Bolt: Coalesce cmd and data into a single block transfer
        // Use a fixed stack buffer to avoid heap fragmentation and allocation overhead
        uint8_t txBuf[260];
        txBuf[0] = cmd;
        memcpy(&txBuf[1], data, len);
        SPI.writeBytes(txBuf, len + 1);
    } else {
        SPI.transfer(cmd);
    }
    digitalWrite(_pinNSS, HIGH);
    
    waitBusy();
}

void E28Radio::readCommand(uint8_t cmd, uint8_t* data, uint8_t len) {
    waitBusy();
    
    digitalWrite(_pinNSS, LOW);
    if (len > 0) {
        // ⚡ Bolt: Coalesce cmd, NOP, and read dummy bytes into a single block transfer
        // Use fixed stack buffer to avoid heap overhead
        uint8_t txBuf[260];
        uint8_t rxBuf[260];
        uint32_t totalLen = len + 2;

        memset(txBuf, 0, totalLen);
        txBuf[0] = cmd;
        txBuf[1] = 0x00; // NOP

        SPI.transferBytes(txBuf, rxBuf, totalLen);
        memcpy(data, &rxBuf[2], len);
    } else {
        SPI.transfer(cmd);
        SPI.transfer(0x00);  // NOP
    }
    digitalWrite(_pinNSS, HIGH);
}

void E28Radio::setFrequency(uint32_t frequency) {
    // Frequency = (rfFreq * Fxtal) / 2^18
    // Fxtal = 52 MHz for SX1280
    uint32_t rfFreq = (uint32_t)((float)frequency / (52000000.0 / 262144.0));
    
    uint8_t freqParams[3] = {
        (uint8_t)((rfFreq >> 16) & 0xFF),
        (uint8_t)((rfFreq >> 8) & 0xFF),
        (uint8_t)(rfFreq & 0xFF)
    };
    writeCommand(SX1280_CMD_SET_RF_FREQUENCY, freqParams, 3);
}

void E28Radio::setTxPower(int8_t power) {
    // Clamp power to valid range
    if (power < -18) power = -18;
    if (power > 12) power = 12;
    _power = power;
    
    // Power = -18 + power (0-31)
    uint8_t powerReg = (uint8_t)(power + 18);
    uint8_t rampTime = 0xE0;  // 20us ramp time
    
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

void E28Radio::setPacketParams(uint8_t payloadLen) {
    uint8_t pktParams[7] = {
        0x0C,       // Preamble length (12 symbols)
        0x00,       // Header type: explicit
        payloadLen, // Payload length
        0x01,       // CRC on
        0x00,       // Standard IQ
        0x00, 0x00  // Reserved
    };
    writeCommand(SX1280_CMD_SET_PACKET_PARAMS, pktParams, 7);
}

void E28Radio::clearIrqStatus() {
    uint8_t clearAll[2] = {0xFF, 0xFF};
    writeCommand(SX1280_CMD_CLR_IRQ_STATUS, clearAll, 2);
}

uint16_t E28Radio::getIrqStatus() {
    uint8_t irqStatus[2];
    readCommand(SX1280_CMD_GET_IRQ_STATUS, irqStatus, 2);
    return ((uint16_t)irqStatus[0] << 8) | irqStatus[1];
}

bool E28Radio::send(uint8_t* data, uint8_t len) {
    // Go to standby first (disables EN pins)
    standby();
    
    // Set packet length
    setPacketParams(len);
    
    // Write data to buffer
    waitBusy();
    digitalWrite(_pinNSS, LOW);
    if (len > 0) {
        // ⚡ Bolt: Coalesce command, offset and payload into a single SPI transfer
        // Use fixed stack buffer
        uint8_t txBuf[260];
        uint32_t totalLen = len + 2;

        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        memcpy(&txBuf[2], data, len);

        SPI.writeBytes(txBuf, totalLen);
    } else {
        SPI.transfer(SX1280_CMD_WRITE_BUFFER);
        SPI.transfer(0x00);  // Offset
    }
    digitalWrite(_pinNSS, HIGH);
    
    // Clear IRQ status
    clearIrqStatus();
    
    // Enable PA BEFORE starting TX
    if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, HIGH);
    delayMicroseconds(50);
    
    // Start transmission (timeout = 0 for continuous)
    uint8_t txParams[3] = {0x00, 0x00, 0x00};
    writeCommand(SX1280_CMD_SET_TX, txParams, 3);
    
    // Wait for TX done
    uint32_t timeout = millis() + 5000;
    while (!(getIrqStatus() & 0x0001)) {  // TxDone bit
        if (millis() > timeout) {
            if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
            standby();
            return false;
        }
        yield();
    }
    
    clearIrqStatus();
    // Disable PA after TX done
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
    standby();
    return true;
}

bool E28Radio::sendAsync(uint8_t* data, uint8_t len) {
    standby();
    setPacketParams(len);
    
    // Write data to buffer
    waitBusy();
    digitalWrite(_pinNSS, LOW);
    if (len > 0) {
        // ⚡ Bolt: Coalesce command, offset and payload into a single SPI transfer
        // Use fixed stack buffer
        uint8_t txBuf[260];
        uint32_t totalLen = len + 2;

        txBuf[0] = SX1280_CMD_WRITE_BUFFER;
        txBuf[1] = 0x00; // Offset
        memcpy(&txBuf[2], data, len);

        SPI.writeBytes(txBuf, totalLen);
    } else {
        SPI.transfer(SX1280_CMD_WRITE_BUFFER);
        SPI.transfer(0x00); // Offset
    }
    digitalWrite(_pinNSS, HIGH);
    
    clearIrqStatus();
    
    // Enable PA BEFORE starting TX
    if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, HIGH);
    delayMicroseconds(50);
    
    uint8_t txParams[3] = {0x00, 0x00, 0x00};
    writeCommand(SX1280_CMD_SET_TX, txParams, 3);
    
    return true;
}

bool E28Radio::isTxDone() {
    return (getIrqStatus() & 0x0001) != 0;
}

void E28Radio::startReceive() {
    standby();
    setPacketParams(E28_MAX_PACKET_SIZE);
    clearIrqStatus();
    
    // Enable LNA
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
    if (_pinRXEN != -1) digitalWrite(_pinRXEN, HIGH);
    delayMicroseconds(50);

    // Start continuous RX
    uint8_t rxParams[3] = {0xFF, 0xFF, 0xFF};  // Continuous RX
    writeCommand(SX1280_CMD_SET_RX, rxParams, 3);
}

bool E28Radio::available() {
    return (getIrqStatus() & 0x0002) != 0;  // RxDone bit
}

uint8_t E28Radio::receive(uint8_t* buffer, uint8_t maxLen) {
    // Get RX buffer status
    uint8_t rxStatus[2];
    readCommand(SX1280_CMD_GET_RX_BUFFER_STATUS, rxStatus, 2);
    
    uint8_t payloadLen = rxStatus[0];
    uint8_t bufferOffset = rxStatus[1];
    
    if (payloadLen > maxLen) {
        payloadLen = maxLen;
    }
    
    // Read data from buffer
    waitBusy();
    digitalWrite(_pinNSS, LOW);
    if (payloadLen > 0) {
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
    } else {
        SPI.transfer(SX1280_CMD_READ_BUFFER);
        SPI.transfer(bufferOffset);
        SPI.transfer(0x00); // NOP
    }
    digitalWrite(_pinNSS, HIGH);
    
    // Get RSSI and SNR (from packet status register)
    // Simplified: just clear IRQ and return
    clearIrqStatus();
    
    return payloadLen;
}

int8_t E28Radio::getRSSI() {
    return _lastRSSI;
}

int8_t E28Radio::getSNR() {
    return _lastSNR;
}

void E28Radio::sleep() {
    // Disable RF switch
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
    if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);

    uint8_t sleepConfig = 0x00;  // Cold start
    writeCommand(SX1280_CMD_SET_SLEEP, &sleepConfig, 1);
}

void E28Radio::standby() {
    uint8_t stdbyConfig = SX1280_STANDBY_RC;
    writeCommand(SX1280_CMD_SET_STANDBY, &stdbyConfig, 1);
    
    // We don't disable LNA/PA here immediately because standby is often intermediate state 
    // before TX/RX. But for power saving, maybe we should?
    // Let's assume user calls startReceive() or send() after standby.
    // If we are just idling, sleep() is better.
    // But to be safe, maybe disable PA?
    // if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW); 
    // if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);
    // Commented out to behave like standard transceivers that keep RF switch settings?
    // No, standard is Standby = RF Off.
    if (_pinTXEN != -1) digitalWrite(_pinTXEN, LOW);
    if (_pinRXEN != -1) digitalWrite(_pinRXEN, LOW);
}

uint8_t E28Radio::getChipStatus() {
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
