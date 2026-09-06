#include "Arduino.h"
#include "SPI.h"
uint32_t g_testMillis = 0;
SerialShim Serial;
EspShim ESP;
SPIClass SPI;

static uint8_t g_pinLevel[64] = {0}; // every pin reads LOW unless the test says
void pinMode(int8_t, uint8_t) {}
void digitalWrite(int8_t pin, uint8_t level) {
  if (pin >= 0 && pin < 64)
    g_pinLevel[pin] = level;
  if (pin == SPI.nssPin) {
    if (level == LOW)
      SPI.nssLow();
    else
      SPI.nssHigh();
  }
}
int digitalRead(int8_t pin) {
  return (pin >= 0 && pin < 64) ? g_pinLevel[pin] : LOW;
}
void testSetPin(int8_t pin, uint8_t level) {
  if (pin >= 0 && pin < 64)
    g_pinLevel[pin] = level;
}
