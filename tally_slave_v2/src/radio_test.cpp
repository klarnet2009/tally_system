/**
 * RAINBOW COLOR TEST — SLAVE (ESP32-C3)
 *
 * Receives RGB color packets from hub and displays on NeoPixel.
 * Also sends acknowledgments back.
 *
 * Expected packet: [0xAA, 'C', R, G, B]
 */

#include "E28_SX1280.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// E28 pins for ESP32-C3
#define PIN_LORA_SCK 3
#define PIN_LORA_MOSI 4
#define PIN_LORA_MISO 5
#define PIN_LORA_NRESET 6
#define PIN_LORA_NSS 20
#define PIN_LORA_BUSY 10
#define PIN_LORA_DIO1 1
#define PIN_LORA_RXEN -1
#define PIN_LORA_TXEN -1

// NeoPixel
#define PIN_NEOPIXEL 7
#define NUM_PIXELS 1

Adafruit_NeoPixel led(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
E28Radio radio;

uint32_t rxCount = 0;
uint32_t txCount = 0;
uint8_t lastR = 0, lastG = 0, lastB = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=============================");
  Serial.println("  RAINBOW TEST — SLAVE");
  Serial.println("=============================");

  // NeoPixel
  led.begin();
  led.setBrightness(150);
  led.setPixelColor(0, 0);
  led.show();

  // Radio
  Serial.print("[E28] Init... ");
  bool ok = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                        PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                        PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (ok) {
    Serial.printf("OK (SPI:0x%02X)\n", radio.getChipStatus());
  } else {
    Serial.printf("FAILED (SPI:0x%02X)\n", radio.getChipStatus());
    // Blink red to indicate error
    while (1) {
      led.setPixelColor(0, led.Color(255, 0, 0));
      led.show();
      delay(200);
      led.setPixelColor(0, 0);
      led.show();
      delay(200);
    }
  }

  radio.startReceive();
  Serial.println("[E28] Listening for rainbow...\n");
}

void loop() {
  // === CHECK FOR COLOR PACKETS ===
  if (radio.available()) {
    uint8_t buf[8];
    uint8_t len = radio.receive(buf, sizeof(buf));

    if (len >= 5 && buf[0] == 0xAA && buf[1] == 'C') {
      // Color packet!
      lastR = buf[2];
      lastG = buf[3];
      lastB = buf[4];
      rxCount++;

      // Set NeoPixel
      led.setPixelColor(0, led.Color(lastR, lastG, lastB));
      led.show();

      // Log every 10th packet
      if (rxCount % 10 == 0) {
        Serial.printf("[COLOR] #%lu R:%d G:%d B:%d\n", rxCount, lastR, lastG,
                      lastB);
      }
    } else if (len > 0) {
      Serial.printf("[RX] Unknown pkt len=%d\n", len);
    }

    // Re-arm receiver
    // ⚡ Bolt: Fast RX re-arm to avoid standby/SPI overhead and prevent dropped packets
    radio.clearRxIrq();
  }

  // === STATUS EVERY 10s ===
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 10000) {
    lastStatus = millis();
    Serial.printf("[STATUS] Up:%lus RX:%lu R:%d G:%d B:%d SPI:0x%02X\n",
                  millis() / 1000, rxCount, lastR, lastG, lastB,
                  radio.getChipStatus());
  }

  // === TIMEOUT: blink white if no packets for 5s ===
  static uint32_t lastRxTime = 0;
  if (rxCount > 0)
    lastRxTime = millis();

  if (millis() - lastRxTime > 5000 && lastRxTime > 0) {
    static bool blinkState = false;
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      blinkState = !blinkState;
      if (blinkState) {
        led.setPixelColor(0, led.Color(30, 30, 30)); // dim white
      } else {
        led.setPixelColor(0, 0);
      }
      led.show();
    }
  }

  delay(1);
}
