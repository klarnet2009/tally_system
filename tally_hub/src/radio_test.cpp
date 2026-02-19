/**
 * RAINBOW COLOR TEST — HUB (ESP32-S3)
 *
 * Cycles through 6 rainbow colors every 300ms, sends RGB via LoRa.
 * Colors: Red → Yellow → Green → Cyan → Blue → Magenta
 *
 * Packet: [0xAA, 'C', R, G, B]
 */

#include "E28_SX1280.h"
#include "config.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>


#define WHITE SSD1306_WHITE

TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(128, 64, &I2Cbus, -1);
E28Radio radio;

uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t txFail = 0;

// 6 rainbow colors: R, Y, G, C, B, M
const uint8_t COLORS[][3] = {
    {255, 0, 0},   // Red
    {255, 255, 0}, // Yellow
    {0, 255, 0},   // Green
    {0, 255, 255}, // Cyan
    {0, 0, 255},   // Blue
    {255, 0, 255}, // Magenta
};
const char *COLOR_NAMES[] = {"RED",  "YELLOW", "GREEN",
                             "CYAN", "BLUE",   "MAGENTA"};
const uint8_t NUM_COLORS = 6;

uint8_t colorIdx = 0;
uint8_t curR = 0, curG = 0, curB = 0;

void setup() {
  // OLED
  I2Cbus.begin(OLED_I2C_SDA, OLED_I2C_SCL, 400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("RAINBOW");
  display.println("6 COLORS");
  display.display();
  delay(1000);

  // Radio
  bool ok = radio.begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI, E28_PIN_NSS,
                        E28_PIN_BUSY, E28_PIN_DIO1, E28_PIN_RESET, E28_PIN_RXEN,
                        E28_PIN_TXEN);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(ok ? "E28: OK" : "E28: FAIL");
  display.display();
  delay(1000);

  radio.startReceive();
}

void loop() {
  // === CHECK RX ===
  if (radio.available()) {
    uint8_t buf[8];
    uint8_t len = radio.receive(buf, sizeof(buf));
    if (len > 0)
      rxCount++;
    radio.startReceive();
  }

  // === SEND NEXT COLOR EVERY 300ms ===
  static uint32_t lastTx = 0;
  if (millis() - lastTx >= 300) {
    lastTx = millis();

    curR = COLORS[colorIdx][0];
    curG = COLORS[colorIdx][1];
    curB = COLORS[colorIdx][2];

    uint8_t pkt[5] = {0xAA, 'C', curR, curG, curB};
    bool sent = radio.send(pkt, 5);
    if (sent)
      txCount++;
    else
      txFail++;

    colorIdx = (colorIdx + 1) % NUM_COLORS;
    radio.startReceive();
  }

  // === UPDATE OLED EVERY 300ms ===
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw >= 300) {
    lastDraw = millis();

    display.clearDisplay();
    display.setTextColor(WHITE);

    // Row 0: Color name (big)
    display.setTextSize(2);
    display.setCursor(0, 0);
    uint8_t showIdx =
        (colorIdx + NUM_COLORS - 1) % NUM_COLORS; // show CURRENT (just sent)
    display.print(COLOR_NAMES[showIdx]);

    // Row 1: RGB
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.printf("R:%3d G:%3d B:%3d", curR, curG, curB);

    // Row 2: color bar
    display.fillRect(0, 30, 128, 8, WHITE);

    // Row 3: TX/RX
    display.setCursor(0, 42);
    display.printf("TX:%lu fail:%lu", txCount, txFail);

    // Row 4: Uptime
    display.setCursor(0, 54);
    display.printf("Up:%lus  RX:%lu", millis() / 1000, rxCount);

    display.display();
  }

  delay(1);
}
