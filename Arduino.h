#include <stdint.h>
#include <string.h>

#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define HIGH 1

void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
void delay(int);
void delayMicroseconds(int);
uint32_t millis();
void yield();

class SPIClass {
public:
    void begin(int, int, int, int);
    void setFrequency(uint32_t);
    uint8_t transfer(uint8_t);
    void writeBytes(uint8_t*, uint32_t);
    void transferBytes(uint8_t*, uint8_t*, uint32_t);
};

extern SPIClass SPI;
