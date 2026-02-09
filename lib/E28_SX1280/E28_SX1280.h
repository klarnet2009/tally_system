#ifndef E28_SX1280_H
#define E28_SX1280_H

#include <Arduino.h>
#include <SPI.h>

// Default pin configuration (override via config.h before including this header)
#ifndef E28_PIN_MISO
#define E28_PIN_MISO    19
#endif
#ifndef E28_PIN_MOSI
#define E28_PIN_MOSI    23
#endif
#ifndef E28_PIN_SCK
#define E28_PIN_SCK     18
#endif
#ifndef E28_PIN_NSS
#define E28_PIN_NSS     5
#endif
#ifndef E28_PIN_BUSY
#define E28_PIN_BUSY    4
#endif
#ifndef E28_PIN_DIO1
#define E28_PIN_DIO1    2
#endif
#ifndef E28_PIN_RESET
#define E28_PIN_RESET   14
#endif

// SX1280 Commands
#define SX1280_CMD_GET_STATUS           0xC0
#define SX1280_CMD_WRITE_REGISTER       0x18
#define SX1280_CMD_READ_REGISTER        0x19
#define SX1280_CMD_WRITE_BUFFER         0x1A
#define SX1280_CMD_READ_BUFFER          0x1B
#define SX1280_CMD_SET_SLEEP            0x84
#define SX1280_CMD_SET_STANDBY          0x80
#define SX1280_CMD_SET_FS               0xC1
#define SX1280_CMD_SET_TX               0x83
#define SX1280_CMD_SET_RX               0x82
#define SX1280_CMD_SET_PACKET_TYPE      0x8A
#define SX1280_CMD_GET_PACKET_TYPE      0x03
#define SX1280_CMD_SET_RF_FREQUENCY     0x86
#define SX1280_CMD_SET_TX_PARAMS        0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1280_CMD_SET_PACKET_PARAMS    0x8C
#define SX1280_CMD_SET_BUFFER_BASE_ADDR 0x8F
#define SX1280_CMD_GET_RX_BUFFER_STATUS 0x17
#define SX1280_CMD_CLR_IRQ_STATUS       0x97
#define SX1280_CMD_SET_DIO_IRQ_PARAMS   0x8D
#define SX1280_CMD_GET_IRQ_STATUS       0x15

// Packet types
#define SX1280_PACKET_TYPE_LORA         0x01

// Standby modes
#define SX1280_STANDBY_RC               0x00
#define SX1280_STANDBY_XOSC             0x01

// LoRa Spreading Factors
#define LORA_SF5                        0x50
#define LORA_SF6                        0x60
#define LORA_SF7                        0x70
#define LORA_SF8                        0x80
#define LORA_SF9                        0x90
#define LORA_SF10                       0xA0
#define LORA_SF11                       0xB0
#define LORA_SF12                       0xC0

// LoRa Bandwidths
#define LORA_BW_0200                    0x34  // 203.125 kHz
#define LORA_BW_0400                    0x26  // 406.25 kHz
#define LORA_BW_0800                    0x18  // 812.5 kHz
#define LORA_BW_1600                    0x0A  // 1625 kHz

// LoRa Coding Rates
#define LORA_CR_4_5                     0x01
#define LORA_CR_4_6                     0x02
#define LORA_CR_4_7                     0x03
#define LORA_CR_4_8                     0x04

// Buffer size
#define E28_MAX_PACKET_SIZE             255

class E28Radio {
public:
    E28Radio();
    
    // Initialize radio with custom pins
    bool begin(int8_t sck, int8_t miso, int8_t mosi,
               int8_t nss, int8_t busy, int8_t dio1, 
               int8_t reset = -1,
               int8_t rxen = -1,
               int8_t txen = -1);
    
    // Configuration
    void setFrequency(uint32_t frequency);      // Frequency in Hz
    void setTxPower(int8_t power);              // Power in dBm (-18 to +12)
    void setSpreadingFactor(uint8_t sf);        // SF5 to SF12
    void setBandwidth(uint8_t bw);              // Use LORA_BW_* constants
    void setCodingRate(uint8_t cr);             // Use LORA_CR_* constants
    
    // Transmission
    bool send(uint8_t* data, uint8_t len);
    bool sendAsync(uint8_t* data, uint8_t len);
    bool isTxDone();
    
    // Reception
    void startReceive();
    bool available();
    uint8_t receive(uint8_t* buffer, uint8_t maxLen);
    int8_t getRSSI();
    int8_t getSNR();
    
    // Power management
    void sleep();
    void standby();
    bool isConnected() { return _connected; }
    bool checkConnection();  // Re-poll SPI to verify module
    uint8_t getChipStatus(); // Raw SX1280 status byte
    
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
    int8_t _power;
    bool _connected;
    
    int8_t _lastRSSI;
    int8_t _lastSNR;
    
    void reset();
    void waitBusy();
    void writeCommand(uint8_t cmd, uint8_t* data, uint8_t len);
    void readCommand(uint8_t cmd, uint8_t* data, uint8_t len);
    void setModulationParams();
    void setPacketParams(uint8_t payloadLen);
    void clearIrqStatus();
    uint16_t getIrqStatus();
};

#endif // E28_SX1280_H
