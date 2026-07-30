#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <string.h>

#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define HIGH 1
#define MSBFIRST 1
#define SPI_MODE0 0

void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
void delay(int);
void delayMicroseconds(int);
uint32_t millis();
void yield();

// Mirror of the Arduino SPISettings used in real ESP32 builds (stub only).
class SPISettings {
public:
    SPISettings(uint32_t, int, int) {}
    SPISettings() {}
};

class SPIClass {
public:
    void begin(int, int, int, int);
    void setFrequency(uint32_t);
    void beginTransaction(SPISettings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t);
    void writeBytes(uint8_t*, uint32_t);
    void transferBytes(uint8_t*, uint8_t*, uint32_t);
};

extern SPIClass SPI;

class SerialMock {
public:
    void begin(int baud) {}
    void print(const char* str) {}
    void println(const char* str = "") {}
    void printf(const char* format, ...) {}
    void setTxTimeoutMs(unsigned long) {} // HWCDC API (real header: HWCDC.h)
    void setTimeout(unsigned long) {}
};
extern SerialMock Serial;

// Minimal ESP-class stub (real one lives in the ESP32 core's Esp.h)
class EspMock {
public:
    unsigned long long getEfuseMac() { return 0; }
    void restart() {}
};
extern EspMock ESP;

#endif
