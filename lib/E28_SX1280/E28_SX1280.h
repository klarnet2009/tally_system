#ifndef E28_SX1280_H
#define E28_SX1280_H

#include <Arduino.h>
#include <SPI.h>

// SX1280 Commands
#define SX1280_CMD_GET_STATUS 0xC0
#define SX1280_CMD_WRITE_REGISTER 0x18
#define SX1280_CMD_READ_REGISTER 0x19
#define SX1280_CMD_WRITE_BUFFER 0x1A
#define SX1280_CMD_READ_BUFFER 0x1B
#define SX1280_CMD_SET_SLEEP 0x84
#define SX1280_CMD_SET_STANDBY 0x80
#define SX1280_CMD_SET_FS 0xC1
#define SX1280_CMD_SET_TX 0x83
#define SX1280_CMD_SET_RX 0x82
#define SX1280_CMD_SET_PACKET_TYPE 0x8A
#define SX1280_CMD_GET_PACKET_TYPE 0x03
#define SX1280_CMD_SET_RF_FREQUENCY 0x86
#define SX1280_CMD_SET_TX_PARAMS 0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1280_CMD_SET_PACKET_PARAMS 0x8C
#define SX1280_CMD_SET_BUFFER_BASE_ADDR 0x8F
#define SX1280_CMD_GET_RX_BUFFER_STATUS 0x17
#define SX1280_CMD_GET_PACKET_STATUS 0x1D
#define SX1280_CMD_GET_RSSI_INST 0x1F
#define SX1280_CMD_SET_RX_DUTY_CYCLE 0x94
#define SX1280_CMD_CLR_IRQ_STATUS 0x97
#define SX1280_CMD_SET_DIO_IRQ_PARAMS 0x8D
#define SX1280_CMD_GET_IRQ_STATUS 0x15

// Packet types
#define SX1280_PACKET_TYPE_LORA 0x01

// Standby modes
#define SX1280_STANDBY_RC 0x00
#define SX1280_STANDBY_XOSC 0x01

// LoRa Spreading Factors
#define LORA_SF5 0x50
#define LORA_SF6 0x60
#define LORA_SF7 0x70
#define LORA_SF8 0x80
#define LORA_SF9 0x90
#define LORA_SF10 0xA0
#define LORA_SF11 0xB0
#define LORA_SF12 0xC0

// LoRa Bandwidths
#define LORA_BW_0200 0x34 // 203.125 kHz
#define LORA_BW_0400 0x26 // 406.25 kHz
#define LORA_BW_0800 0x18 // 812.5 kHz
#define LORA_BW_1600 0x0A // 1625 kHz

// LoRa Coding Rates
#define LORA_CR_4_5 0x01
#define LORA_CR_4_6 0x02
#define LORA_CR_4_7 0x03
#define LORA_CR_4_8 0x04

// Buffer size
#define E28_MAX_PACKET_SIZE 255

// Why the last begin() failed — surfaced on the hub OLED / serial logs so a
// field failure can be triaged without a logic analyzer.
enum E28InitError : uint8_t {
  E28_OK = 0,
  E28_ERR_BUSY_STUCK, // BUSY never went low: no power / BUSY miswired / chip hung
  E28_ERR_MISO_LOW,   // status reads 0x00: MISO stuck low — module unpowered/shorted
  E28_ERR_MISO_HIGH,  // status reads 0xFF: MISO stuck high — module absent/miswired
  E28_ERR_READBACK    // status reads fine but SetPacketType didn't land:
                      // commands don't reach the chip — MOSI/SCK wiring
};

class E28Radio {
public:
  E28Radio();

  // Initialize radio with custom pins
  bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t nss, int8_t busy,
             int8_t dio1, int8_t reset = -1, int8_t rxen = -1,
             int8_t txen = -1);

  // Configuration
  void setFrequency(uint32_t frequency); // Frequency in Hz
  void setTxPower(int8_t power);         // Power in dBm (-18 to +12)
  void setSpreadingFactor(uint8_t sf);   // SF5 to SF12
  void setBandwidth(uint8_t bw);         // Use LORA_BW_* constants
  void setCodingRate(uint8_t cr);        // Use LORA_CR_* constants
  void setPreambleLength(uint16_t symbols);

  // Transmission (blocking)
  bool send(uint8_t *data, uint8_t len);
  bool isTxDone();

  // Transmission (non-blocking): startSend() kicks the TX and returns;
  // poll checkTxDone() — it tears down (IRQ clear, PA off, standby) when done.
  // After checkTxDone() returns true, txSucceeded() tells real TxDone (true)
  // apart from the airtime-timeout/failure path (false).
  bool startSend(uint8_t *data, uint8_t len);
  bool checkTxDone();
  bool txActive() { return _txActive; }
  bool txSucceeded() const { return _txSuccess; }

  // Reception
  void startReceive();
  // Duty-cycled RX: radio autonomously alternates RX/sleep, DIO1 fires on
  // RxDone. periodBase 0x02 = 1ms units, so counts are milliseconds.
  // Caller MUST re-arm after every DIO1 event (chip exits the cycle on any
  // RxDone, even CRC errors) and avoid SPI polling between events (any NSS
  // edge during the sleep phase silently kills the cycle).
  void startReceiveDutyCycle(uint16_t rxCount, uint16_t sleepCount,
                             uint8_t periodBase = 0x02);
  // Re-arm using whichever RX mode was last started, so callers don't need
  // their own mode bookkeeping:
  //  - rearmAfterIrq(): after a DIO1 event. Cheap IRQ-clear for continuous
  //    RX; full re-issue for duty cycle (mandatory there).
  //  - restartReceive(): safety net / recovery. Full re-issue of the last
  //    mode, restoring RX no matter what state the chip fell into.
  void rearmAfterIrq();
  void restartReceive();
  void clearRxIrq(); // Lightweight: just clear IRQ, stay in continuous RX
  bool available();
  uint8_t receive(uint8_t *buffer, uint8_t maxLen);
  int8_t getRSSI();
  int8_t getSNR();
  // Instantaneous channel RSSI while in RX (no packet needed): the ambient
  // noise floor — the "how busy is this channel" meter for AFA site surveys.
  int8_t getRssiInst();
  // Corrupted receptions (PHY CRC / header errors) since boot: the on-site
  // interference meter. Rising fast while RX: stays quiet = channel is dirty.
  uint32_t getRxErrors() const { return _rxErrors; }

  // Power management
  void standby();
  bool isConnected() { return _connected; }
  uint8_t getChipStatus(); // Raw SX1280 status byte

  // Diagnostics: why the last begin() failed
  E28InitError initError() const { return _initError; }
  const char *initErrorStr() const;

private:
  int8_t _pinNSS;
  int8_t _pinBUSY;
  int8_t _pinDIO1;
  int8_t _pinRESET;
  int8_t _pinRXEN;
  int8_t _pinTXEN;

  uint8_t _sf;
  uint8_t _bw;
  uint8_t _cr;
  uint8_t _preambleByte;  // SX1280 encoding: (exponent << 4) | mantissa
  uint32_t _frequency;    // Hz; re-applied by begin() so it survives re-init
  int8_t _power;
  bool _connected;
  bool _txActive;
  bool _txSuccess;        // result of the last completed async TX
  uint32_t _txStartMs;
  uint8_t _txLen;         // payload length of the async TX in flight

  // Last-started RX mode, so rearmAfterIrq()/restartReceive() can re-issue it
  enum RxMode : uint8_t { RX_NONE, RX_CONTINUOUS, RX_DUTY_CYCLE };
  RxMode _rxMode;
  uint16_t _dcRxCount;
  uint16_t _dcSleepCount;
  uint8_t _dcPeriodBase;

  uint16_t _lastPktLen; // setPacketParams cache (0xFFFF = invalid)
  uint32_t _rxErrors;   // CRC/header-corrupted receptions since boot
  E28InitError _initError;

  int8_t _lastRSSI;
  int8_t _lastSNR;

  void reset();
  // false = module gone (prior latch, or BUSY stuck >1s just now). Callers
  // must bail on false — the compiler-visible return replaces the old
  // hand-written `waitBusy(); if (!_connected) return X;` pattern that every
  // call site had to remember to repeat.
  bool waitBusy();
  bool waitBusyFor(uint32_t timeoutMs); // bounded variant for probe paths
  // One NSS-framed SPI transfer: header byte(s), then `len` body bytes —
  // from txData (write), or zeros while RX is captured into rxData (read).
  // false = the BUSY wait failed; nothing was clocked, caller must bail.
  bool spiFrame(const uint8_t *header, uint8_t headerLen,
                const uint8_t *txData, uint8_t *rxData, uint8_t len);
  // TX-done timeout derived from the configured airtime (SF/BW/CR/preamble/
  // payload), plus margin — a hardcoded cap silently truncates frames the
  // moment airtime crosses it (the SF10 upgrade path blew through the old
  // 100ms).
  uint32_t txTimeoutMs(uint8_t payloadLen) const;
  void writeCommand(uint8_t cmd, uint8_t *data, uint8_t len);
  void readCommand(uint8_t cmd, uint8_t *data, uint8_t len);
  // Shared TX-FIFO fill (WRITE_BUFFER at offset 0) for send()/startSend() —
  // one copy, so a buffer-size/threshold tweak can't be applied to only one
  // of them. false = module dropped off the bus before/during the fill.
  bool writeTxBuffer(uint8_t *data, uint8_t len);
  void setModulationParams();
  void setPacketParams(uint8_t payloadLen);
  void clearIrqStatus();
  uint16_t getIrqStatus();
};

#endif // E28_SX1280_H
