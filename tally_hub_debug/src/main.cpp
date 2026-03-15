/*
 * E28 SX1280 — Debug Firmware
 * ============================
 * Пошаговая диагностика SPI-связи с модулем E28-2G4M27S.
 * Результаты отображаются на OLED и дублируются в Serial.
 *
 * Экраны (автоматически сменяются):
 *   1) Конфигурация пинов
 *   2) Состояние BUSY / RESET пина
 *   3) Raw SPI GetStatus (до reset)
 *   4) Hardware Reset + повторный GetStatus
 *   5) Полная инициализация radio.begin() — результат
 *   6) Циклический мониторинг: SPI status + BUSY + RSSI (loop)
 *
 * Boot-кнопка (GPIO0): перемотать экран дальше вручную
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ===== Pin config (same as tally_hub config.h) =====
#define E28_PIN_SCK 14
#define E28_PIN_MISO 13
#define E28_PIN_MOSI 11
#define E28_PIN_NSS 10
#define E28_PIN_BUSY 4
#define E28_PIN_DIO1 2
#define E28_PIN_RESET -1 // No hardware reset
#define E28_PIN_RXEN 1
#define E28_PIN_TXEN 3

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_ADDR 0x3C

#define BTN_BOOT 0

// ===== SX1280 Commands =====
#define SX1280_CMD_GET_STATUS 0xC0
#define SX1280_CMD_SET_STANDBY 0x80
#define SX1280_CMD_SET_PACKET_TYPE 0x8A
#define SX1280_CMD_SET_RF_FREQUENCY 0x86
#define SX1280_CMD_SET_TX_PARAMS 0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1280_CMD_SET_BUFFER_BASE_ADDR 0x8F
#define SX1280_CMD_SET_DIO_IRQ_PARAMS 0x8D
#define SX1280_STANDBY_RC 0x00

// Forward declarations
void waitForButton(uint32_t timeoutMs);

// ===== Globals =====
TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 oled(128, 64, &I2Cbus, -1);

static int currentScreen = 0;
static bool oledOk = false;

// ===== Helpers =====
void oledClear() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
}

void oledHeader(const char *title) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(BLACK, WHITE);
  oled.setCursor(0, 0);
  // Inverted header bar
  oled.fillRect(0, 0, 128, 10, WHITE);
  int tx = (128 - strlen(title) * 6) / 2;
  if (tx < 0)
    tx = 0;
  oled.setCursor(tx, 1);
  oled.print(title);
  oled.setTextColor(WHITE, BLACK);
  oled.setCursor(0, 13);
}

void oledShow() { oled.display(); }

// Raw SPI send without using E28Radio class — minimal test
uint8_t spiGetStatus() {
  digitalWrite(E28_PIN_NSS, LOW);
  // ⚡ Bolt: Coalesce get status to avoid 2 separate single byte transfers locking overhead
  uint8_t txBuf[2] = {SX1280_CMD_GET_STATUS, 0x00};
  uint8_t rxBuf[2] = {0, 0};
  SPI.transferBytes(txBuf, rxBuf, 2);
  digitalWrite(E28_PIN_NSS, HIGH);
  return rxBuf[0];
}

// Send command via raw SPI
void spiWriteCmd(uint8_t cmd, uint8_t *data, uint8_t len) {
  // Wait BUSY
  uint32_t t0 = millis();
  while (digitalRead(E28_PIN_BUSY) == HIGH) {
    if (millis() - t0 > 500)
      break;
    yield();
  }
  digitalWrite(E28_PIN_NSS, LOW);
  if (len > 0) {
    // ⚡ Bolt: Fully coalesce cmd and data into a single block transfer
    // Use fixed stack buffer to avoid heap fragmentation and allocation overhead
    uint8_t txBuf[260];
    txBuf[0] = cmd;
    memcpy(&txBuf[1], data, len);
    SPI.writeBytes(txBuf, len + 1);
  } else {
    SPI.transfer(cmd);
  }
  digitalWrite(E28_PIN_NSS, HIGH);
  // Wait BUSY again
  t0 = millis();
  while (digitalRead(E28_PIN_BUSY) == HIGH) {
    if (millis() - t0 > 500)
      break;
    yield();
  }
}

bool waitBusyTimeout(uint32_t ms) {
  uint32_t t0 = millis();
  while (digitalRead(E28_PIN_BUSY) == HIGH) {
    if (millis() - t0 > ms)
      return false; // timeout
    yield();
  }
  return true; // ok
}

// ===== Screen 1: Pin Configuration =====
void screenPins() {
  Serial.println("=== SCREEN 1: Pin Config ===");
  oledHeader("PIN CONFIG");

  oled.printf("SCK  =%2d  MISO=%2d\n", E28_PIN_SCK, E28_PIN_MISO);
  oled.printf("MOSI =%2d  NSS =%2d\n", E28_PIN_MOSI, E28_PIN_NSS);
  oled.printf("BUSY =%2d  DIO1=%2d\n", E28_PIN_BUSY, E28_PIN_DIO1);
  oled.printf("RESET=%2d\n", E28_PIN_RESET);
  oled.printf("RXEN =%2d  TXEN=%2d\n", E28_PIN_RXEN, E28_PIN_TXEN);
  oled.println();
  oled.println("Press BOOT -> next");
  oledShow();

  Serial.printf("  SCK=%d MISO=%d MOSI=%d NSS=%d\n", E28_PIN_SCK, E28_PIN_MISO,
                E28_PIN_MOSI, E28_PIN_NSS);
  Serial.printf("  BUSY=%d DIO1=%d RESET=%d\n", E28_PIN_BUSY, E28_PIN_DIO1,
                E28_PIN_RESET);
  Serial.printf("  RXEN=%d TXEN=%d\n", E28_PIN_RXEN, E28_PIN_TXEN);
}

// ===== Screen 2: BUSY / DIO1 Pin State =====
void screenPinStates() {
  Serial.println("=== SCREEN 2: Pin States ===");
  oledHeader("PIN STATES");

  int busy = digitalRead(E28_PIN_BUSY);
  int dio1 = digitalRead(E28_PIN_DIO1);
  int nss = digitalRead(E28_PIN_NSS);

  oled.printf("BUSY (GPIO%d): %s\n", E28_PIN_BUSY, busy ? "HIGH !" : "LOW ok");
  oled.printf("DIO1 (GPIO%d): %s\n", E28_PIN_DIO1, dio1 ? "HIGH" : "LOW");
  oled.printf("NSS  (GPIO%d): %s\n", E28_PIN_NSS, nss ? "HIGH ok" : "LOW !");
  oled.println();

  if (busy) {
    oled.println("!! BUSY=HIGH !!");
    oled.println("Module stuck or");
    oled.println("not connected");
  } else {
    oled.println("BUSY=LOW -> good");
    oled.println("SPI should work");
  }
  oledShow();

  Serial.printf("  BUSY=%d DIO1=%d NSS=%d\n", busy, dio1, nss);
  if (busy)
    Serial.println("  WARNING: BUSY stuck HIGH - module may be disconnected");
}

// ===== Screen 3: Raw SPI GetStatus (before reset) =====
void screenRawSPI() {
  Serial.println("=== SCREEN 3: Raw SPI ===");
  oledHeader("RAW SPI TEST");

  // Multiple reads to check consistency
  uint8_t s1 = spiGetStatus();
  delay(10);
  uint8_t s2 = spiGetStatus();
  delay(10);
  uint8_t s3 = spiGetStatus();

  oled.println("GetStatus x3:");
  oled.printf(" #1: 0x%02X\n", s1);
  oled.printf(" #2: 0x%02X\n", s2);
  oled.printf(" #3: 0x%02X\n", s3);
  oled.println();

  bool allSame = (s1 == s2) && (s2 == s3);
  bool allFF = (s1 == 0xFF && s2 == 0xFF && s3 == 0xFF);
  bool all00 = (s1 == 0x00 && s2 == 0x00 && s3 == 0x00);

  if (allFF) {
    oled.println("ALL 0xFF = no chip");
    oled.println("Check SPI wires!");
  } else if (all00) {
    oled.println("ALL 0x00 = shorted?");
    oled.println("Check GND/power");
  } else if (allSame) {
    uint8_t mode = (s1 >> 2) & 0x07;
    const char *modeStr = "??";
    if (mode == 0)
      modeStr = "SLEEP";
    else if (mode == 1)
      modeStr = "STDBY";
    else if (mode == 2)
      modeStr = "XOSC";
    else if (mode == 3)
      modeStr = "FS";
    else if (mode == 4)
      modeStr = "RX";
    else if (mode == 5)
      modeStr = "TX";
    oled.printf("Chip OK! Mode:%s\n", modeStr);
  } else {
    oled.println("Unstable reads!");
  }
  oledShow();

  Serial.printf("  Status bytes: 0x%02X 0x%02X 0x%02X\n", s1, s2, s3);
}

// ===== Screen 4: Manual Reset + Re-test =====
void screenReset() {
  Serial.println("=== SCREEN 4: Reset Test ===");
  oledHeader("RESET + RETEST");

  if (E28_PIN_RESET == -1) {
    oled.println("No HW RESET pin!");
    oled.println("Doing SW reset...");
    oled.println("(power cycle needed");
    oled.println(" for full reset)");
    oledShow();
    delay(1000);

    // Try standby command as "soft reset"
    uint8_t stdby = SX1280_STANDBY_RC;
    spiWriteCmd(SX1280_CMD_SET_STANDBY, &stdby, 1);
    delay(50);
  } else {
    oled.println("HW Reset...");
    oledShow();
    digitalWrite(E28_PIN_RESET, LOW);
    delay(50);
    digitalWrite(E28_PIN_RESET, HIGH);
    delay(100);
  }

  // Wait for BUSY to go LOW
  oledClear();
  oledHeader("RESET + RETEST");
  bool busyOk = waitBusyTimeout(1000);
  oled.printf("BUSY after rst: %s\n", busyOk ? "LOW ok" : "HIGH !!");

  // Re-read status
  uint8_t st = spiGetStatus();
  oled.printf("Status: 0x%02X\n", st);
  oled.println();

  if (st == 0xFF || st == 0x00) {
    oled.println("FAIL: chip not");
    oled.println("responding after");
    oled.println("reset attempt");
  } else {
    uint8_t mode = (st >> 2) & 0x07;
    oled.printf("Mode: %d ", mode);
    if (mode == 1)
      oled.println("(STDBY) OK!");
    else
      oled.printf("(expected 1)\n");
  }
  oledShow();

  Serial.printf("  BUSY after reset: %s\n", busyOk ? "OK" : "STUCK HIGH");
  Serial.printf("  Status: 0x%02X\n", st);
}

// ===== Screen 5: Full Init Sequence =====
void screenFullInit() {
  Serial.println("=== SCREEN 5: Full Init ===");
  oledHeader("FULL INIT TEST");

  int step = 0;
  bool ok = true;

  // Step 1: Standby
  oled.println("1. Standby...");
  oledShow();
  uint8_t stdby = SX1280_STANDBY_RC;
  spiWriteCmd(SX1280_CMD_SET_STANDBY, &stdby, 1);
  delay(10);
  uint8_t st = spiGetStatus();
  bool step1ok = (st != 0xFF && st != 0x00);
  oled.printf("   -> 0x%02X %s\n", st, step1ok ? "OK" : "FAIL");
  if (!step1ok)
    ok = false;
  oledShow();

  // Step 2: Set LoRa packet type
  oled.println("2. Pkt=LoRa...");
  oledShow();
  uint8_t pktType = 0x01; // LoRa
  spiWriteCmd(SX1280_CMD_SET_PACKET_TYPE, &pktType, 1);
  delay(5);
  st = spiGetStatus();
  bool step2ok = (st != 0xFF && st != 0x00);
  oled.printf("   -> 0x%02X %s\n", st, step2ok ? "OK" : "FAIL");
  if (!step2ok)
    ok = false;
  oledShow();

  // Step 3: Set Frequency 2.4GHz
  oled.println("3. Freq 2.4G...");
  oledShow();
  uint32_t rfFreq = (uint32_t)((float)2400000000UL / (52000000.0 / 262144.0));
  uint8_t freqP[3] = {(uint8_t)((rfFreq >> 16) & 0xFF),
                      (uint8_t)((rfFreq >> 8) & 0xFF),
                      (uint8_t)(rfFreq & 0xFF)};
  spiWriteCmd(SX1280_CMD_SET_RF_FREQUENCY, freqP, 3);
  delay(5);
  st = spiGetStatus();
  bool step3ok = (st != 0xFF && st != 0x00);
  oled.printf("   -> 0x%02X %s\n", st, step3ok ? "OK" : "FAIL");
  if (!step3ok)
    ok = false;
  oledShow();

  Serial.printf("  Init result: %s\n", ok ? "ALL OK" : "FAILED");
  Serial.printf("  Final status: 0x%02X\n", st);
}

// ===== Screen 6: Live Monitor (loop) =====
static uint32_t loopCount = 0;
void screenLiveMonitor() {
  oledHeader("LIVE MONITOR");

  uint8_t st = spiGetStatus();
  int busy = digitalRead(E28_PIN_BUSY);
  int dio1 = digitalRead(E28_PIN_DIO1);

  bool chipOk = (st != 0xFF && st != 0x00);
  uint8_t mode = (st >> 2) & 0x07;

  const char *modeStr = "??";
  if (mode == 0)
    modeStr = "SLP";
  else if (mode == 1)
    modeStr = "STBY";
  else if (mode == 2)
    modeStr = "XOSC";
  else if (mode == 3)
    modeStr = "FS";
  else if (mode == 4)
    modeStr = "RX";
  else if (mode == 5)
    modeStr = "TX";

  oled.printf("SPI: 0x%02X  %s\n", st, chipOk ? "OK" : "FAIL");
  oled.printf("Mode: %s  BUSY:%s\n", modeStr, busy ? "H" : "L");
  oled.printf("DIO1: %s\n", dio1 ? "HIGH" : "LOW");
  oled.println();

  if (chipOk) {
    oled.println(">>> MODULE OK <<<");
    oled.println("SPI is working");
  } else if (st == 0xFF) {
    oled.println("0xFF = no response");
    oled.println("Check: wiring,");
    oled.println("power, solder");
  } else {
    oled.println("0x00 = GND short?");
    oled.println("Check MISO line");
  }

  oled.setCursor(90, 56);
  oled.printf("#%lu", loopCount++);
  oledShow();

  if (loopCount % 20 == 0) {
    Serial.printf("[LIVE] SPI=0x%02X BUSY=%d DIO1=%d mode=%s\n", st, busy, dio1,
                  modeStr);
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("================================");
  Serial.println("E28 SX1280 DEBUG FIRMWARE v1.0");
  Serial.println("================================");

  // I2C & OLED
  I2Cbus.begin(OLED_SDA, OLED_SCL, 400000);
  delay(100);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    oled.setTextWrap(false);
    oled.clearDisplay();
    oled.display();
    Serial.println("OLED: OK");
  } else {
    Serial.println("OLED: FAILED!");
  }

  // Boot button
  pinMode(BTN_BOOT, INPUT_PULLUP);

  // Configure SPI pins
  pinMode(E28_PIN_NSS, OUTPUT);
  digitalWrite(E28_PIN_NSS, HIGH);
  pinMode(E28_PIN_BUSY, INPUT);
  pinMode(E28_PIN_DIO1, INPUT);

  // RXEN/TXEN off
  pinMode(E28_PIN_RXEN, OUTPUT);
  digitalWrite(E28_PIN_RXEN, LOW);
  pinMode(E28_PIN_TXEN, OUTPUT);
  digitalWrite(E28_PIN_TXEN, LOW);

  // Initialize SPI
  SPI.begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI, E28_PIN_NSS);
  SPI.setFrequency(8000000);
  Serial.println("SPI: initialized");

  // Run diagnostic screens sequentially
  Serial.println("\n--- Starting diagnostics ---\n");

  // Screen 1: Pin config
  screenPins();
  waitForButton(3000);

  // Screen 2: Pin states
  screenPinStates();
  waitForButton(3000);

  // Screen 3: Raw SPI
  screenRawSPI();
  waitForButton(4000);

  // Screen 4: Reset + retest
  screenReset();
  waitForButton(4000);

  // Screen 5: Full init
  screenFullInit();
  waitForButton(5000);

  Serial.println("\n--- Entering live monitor ---\n");
}

// Wait for button press or timeout
void waitForButton(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (digitalRead(BTN_BOOT) == LOW) {
      delay(200); // debounce
      while (digitalRead(BTN_BOOT) == LOW)
        yield();
      return;
    }
    delay(50);
  }
}

// ===== Loop: Live monitor =====
void loop() {
  screenLiveMonitor();
  delay(500);

  // Button press in live monitor: re-run all diagnostics
  if (digitalRead(BTN_BOOT) == LOW) {
    delay(200);
    while (digitalRead(BTN_BOOT) == LOW)
      yield();
    Serial.println("\n--- Re-running diagnostics ---\n");
    screenPins();
    waitForButton(3000);
    screenPinStates();
    waitForButton(3000);
    screenRawSPI();
    waitForButton(4000);
    screenReset();
    waitForButton(4000);
    screenFullInit();
    waitForButton(5000);
    Serial.println("\n--- Back to live monitor ---\n");
    loopCount = 0;
  }
}
