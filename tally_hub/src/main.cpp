#include <ATEMbase.h>
#include <ATEMmin.h>
#include <Arduino.h>
#include <SkaarhojPgmspace.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_system.h> // esp_reset_reason()

#include "AtemClientAdapter.h"
#include "E28_SX1280.h"
#include "TallyProtocol.h"
#include "TallyRadio.h"
#include "config.h"

static E28Radio radio;
static uint32_t g_loraTxCount = 0;   // TX packet counter
static uint32_t g_loraDropCount = 0; // Packets lost: queue overflow / radio / TX fail

// ===== Debug logging =====
// The S3 has two consoles: Serial = native USB-Serial-JTAG (GPIO19/20),
// Serial0 = UART0 via the devkit's USB bridge (GPIO43/44). Neither touches
// the E28 pins. Log to both so whichever cable is plugged in shows logs.
static void hublogf(const char *fmt, ...) {
  char buf[256]; // worst-case [STATUS] line is ~127 chars; headroom for growth
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  Serial0.print(buf);
}

// Locator state (file scope: triggered by the BOOT button and the serial
// "ping" command)
static uint32_t locatorStartTime = 0;
static uint8_t locatorStep = 0;

// Current tally masks broadcast to the slaves; the OLED grid is derived from
// these too, so there is one source of truth for "what's on air".
static uint16_t g_progMask = 0;
static uint16_t g_prevMask = 0;

// ATEM connection phase. atemPhase == ATEM_RUNNING is the single "connected"
// signal (no separate shadow flag to keep in sync).
enum AtemPhase : uint8_t { ATEM_IDLE, ATEM_CONNECTING, ATEM_RUNNING };
static AtemPhase atemPhase = ATEM_IDLE;

// begin() resets the chip to its 2.4 GHz default frequency, so the shared RF
// profile must be reapplied on every (re)init — including in-field recovery
static bool radioInit() {
  bool ok = radio.begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI, E28_PIN_NSS,
                        E28_PIN_BUSY, E28_PIN_DIO1, E28_PIN_RESET, E28_PIN_RXEN,
                        E28_PIN_TXEN);
  if (ok) {
    tallyApplyRadioProfile(radio);
    radio.startReceive(); // listen for slave telemetry between TX bursts
  }
  return ok;
}

// ===== Per-camera reachability (from slave CMD_TELEMETRY) =====
// The hub is otherwise blind to whether a slave is actually lit; this closes
// the loop. Indexed by camera ID 1..16.
static uint32_t g_camLastSeen[17] = {0};
static int8_t g_camRssi[17] = {0};
static bool g_camReachable[17] = {false};
// Slaves jitter their interval by camId*37ms (worst case ~2.6s for cam 16), so
// 4x the base leaves ~3 real beats of margin — 3x flapped OFFLINE/ONLINE on a
// high-ID camera after just two lost frames.
#define CAM_REACHABLE_MS (4 * TALLY_TELEMETRY_MS)

// ⚡ Bolt: Non-blocking transmission queue to prevent delay() stalls in main loop
#define LORA_QUEUE_SIZE 16
static TallyPacket g_loraQueue[LORA_QUEUE_SIZE];
static uint8_t g_loraQueueHead = 0;
static uint8_t g_loraQueueTail = 0;

static void enqueueLora(const TallyPacket &pkt) {
  uint8_t nextHead = (g_loraQueueHead + 1) % LORA_QUEUE_SIZE;
  if (nextHead != g_loraQueueTail) { // Queue not full
    g_loraQueue[g_loraQueueHead] = pkt;
    g_loraQueueHead = nextHead;
  } else {
    g_loraDropCount++;
  }
}

static void processLoraQueue() {
  static uint32_t lastTxDoneTime = 0;

  // Finish the in-flight transmission first (checkTxDone tears down PA/IRQ);
  // the loop never blocks on TX airtime (~15ms with the long preamble)
  if (radio.txActive()) {
    if (radio.checkTxDone()) {
      // A 100ms timeout (stuck PA/antenna fault) counts as a drop, not a TX,
      // so the OLED drop counter surfaces a failing radio instead of hiding it
      if (radio.txSucceeded())
        g_loraTxCount++;
      else
        g_loraDropCount++;
      lastTxDoneTime = millis();
      // Back to listening when the queue is drained (TX pulls us out of RX)
      if (g_loraQueueHead == g_loraQueueTail)
        radio.startReceive();
    }
    return;
  }

  if (g_loraQueueHead != g_loraQueueTail) {
    // Radio down: drain the packet as a drop instead of stalling on SPI
    if (!radio.isConnected()) {
      g_loraDropCount++;
      g_loraQueueTail = (g_loraQueueTail + 1) % LORA_QUEUE_SIZE;
      return;
    }
    // Minimum 2ms gap between packets, enforced non-blockingly
    if (millis() - lastTxDoneTime >= 2) {
      uint8_t buf[TALLY_PACKET_SIZE];
      TallyProtocol::serialize(g_loraQueue[g_loraQueueTail], buf);
      if (radio.startSend(buf, TALLY_PACKET_SIZE)) {
        g_loraQueueTail = (g_loraQueueTail + 1) % LORA_QUEUE_SIZE;
      }
    }
  }
}

// Receive slave telemetry during idle (non-TX) windows and update the
// reachability table. Cheap: one available() check per loop, a receive only
// when a frame is waiting.
static void serviceTelemetry() {
  if (radio.txActive() || !radio.isConnected())
    return;
  if (!radio.available())
    return;
  uint8_t buf[TALLY_PACKET_SIZE];
  uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);
  // Cheap re-arm (IRQ clear + SET_RX): the full startReceive() would bounce
  // through standby and could cut a frame already mid-air in continuous RX
  radio.rearmAfterIrq();
  if (len == 0)
    return;
  TallyPacket pkt;
  if (!TallyProtocol::deserialize(buf, len, pkt))
    return;
  if (TallyProtocol::cmdCode(pkt) == CMD_TELEMETRY) {
    uint8_t id = pkt.aux;
    if (id >= 1 && id <= 16) {
      g_camLastSeen[id] = millis();
      g_camRssi[id] = TallyProtocol::telemetryRssi(pkt);
    }
  }
}

// Mark cameras online/offline and log only the transitions (not every beat),
// so a dead/returning slave is visible without log spam.
static void sweepReachability() {
  for (uint8_t id = 1; id <= 16; id++) {
    bool reach = g_camLastSeen[id] != 0 &&
                 (millis() - g_camLastSeen[id] < CAM_REACHABLE_MS);
    if (reach != g_camReachable[id]) {
      g_camReachable[id] = reach;
      hublogf("[TLM] cam %u %s (rssi=%d)\n", id, reach ? "ONLINE" : "OFFLINE",
              g_camRssi[id]);
    }
  }
}

// ===== OLED / ATEM =====
TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(128, 64, &I2Cbus, -1);

#ifndef LORA_TEST_MODE
static IAtemClient *atem = nullptr;
static uint32_t lastPoll = 0;
static uint32_t lastAtemAttempt = 0;
#endif

String ipToStr(IPAddress ip) {
  char b[20];
  snprintf(b, sizeof(b), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(b);
}

// ===== Improved OLED UI =====
// Dual-color display: Yellow top 16px, Blue bottom 48px
#define HDR_H 16 // Yellow zone height

// Status bar icons (drawn in yellow zone)
static void drawWifiIcon(int x, int y, bool connected) {
  if (connected) {
    // WiFi arc
    display.drawPixel(x + 2, y, WHITE);
    display.drawLine(x + 1, y + 1, x + 3, y + 1, WHITE);
    display.drawLine(x, y + 2, x + 4, y + 2, WHITE);
    display.drawPixel(x + 2, y + 4, WHITE);
    display.drawPixel(x + 2, y + 5, WHITE);
  } else {
    display.drawLine(x, y, x + 4, y + 5, WHITE);
    display.drawLine(x + 4, y, x, y + 5, WHITE);
  }
}

static void drawLoRaIcon(int x, int y, bool connected) {
  if (connected) {
    // Antenna icon
    display.drawLine(x + 2, y, x + 2, y + 5, WHITE);
    display.drawPixel(x, y + 1, WHITE);
    display.drawPixel(x + 4, y + 1, WHITE);
    display.drawPixel(x + 1, y, WHITE);
    display.drawPixel(x + 3, y, WHITE);
  } else {
    // X mark
    display.drawLine(x, y, x + 4, y + 5, WHITE);
    display.drawLine(x + 4, y, x, y + 5, WHITE);
  }
}

void drawStatusBar(const String &ip, bool wifiOk, bool loraOk, bool atemOk) {
  display.fillRect(0, 0, 128, HDR_H, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);

  // Row 1 (y=0): Icons + label
  drawWifiIcon(1, 1, wifiOk);
  drawLoRaIcon(9, 1, loraOk);

  // ATEM dot
  if (atemOk)
    display.fillCircle(19, 3, 2, WHITE);
  else
    display.drawCircle(19, 3, 2, WHITE);

  display.setCursor(25, 0);
  display.print(atemOk ? "ATEM OK" : "NO ATEM");

  // Row 2 (y=8): IP + LoRa status
  display.setCursor(1, 8);
  display.print(ip);
  // LoRa debug status (right side)
  display.setCursor(86, 8);
  display.print(loraOk ? "LoRa OK" : "LoRa X");

  // Separator line at zone boundary
  display.drawFastHLine(0, 15, 128, WHITE);
}

void drawCenteredMsg(const char *l1, const char *l2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  int y1 = l2 ? 22 : 28;
  int x1 = (128 - strlen(l1) * 6) / 2;
  if (x1 < 0)
    x1 = 0;
  display.setCursor(x1, y1);
  display.print(l1);

  if (l2) {
    int x2 = (128 - strlen(l2) * 6) / 2;
    if (x2 < 0)
      x2 = 0;
    display.setCursor(x2, y1 + 12);
    display.print(l2);
  }
  display.display();
}

// Animated spinner: 8 dots around a circle, one filled at a time
static void drawSpinner(int cx, int cy, int r, uint8_t frame) {
  // ⚡ Bolt: 8-entry static float Lookup Tables for 45-degree multiples to avoid expensive trig calls
  static const float COS_LUT[8] = {1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f, 0.0f, 0.70710678f};
  static const float SIN_LUT[8] = {0.0f, 0.70710678f, 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f};

  for (int i = 0; i < 8; i++) {
    int dx = cx + (int)(COS_LUT[i] * r);
    int dy = cy + (int)(SIN_LUT[i] * r);
    if (i == (frame % 8)) {
      display.fillCircle(dx, dy, 2, WHITE); // Active dot
    } else {
      display.drawPixel(dx, dy, WHITE); // Dim dot
    }
  }
}

// Draw loading screen with spinner
static void drawLoadingScreen(const char *title, const char *detail,
                              uint8_t frame) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Title - centered in upper area
  int tx = (128 - strlen(title) * 6) / 2;
  if (tx < 0)
    tx = 0;
  display.setCursor(tx, 16);
  display.print(title);

  // Spinner in center
  drawSpinner(64, 38, 8, frame);

  // Detail text below spinner
  if (detail) {
    int dx = (128 - strlen(detail) * 6) / 2;
    if (dx < 0)
      dx = 0;
    display.setCursor(dx, 54);
    display.print(detail);
  }
  display.display();
}

static void drawCell(int x, int y, int w, int h, uint8_t camNum,
                     uint8_t status) {
  // Calculate centered position for camera number
  char numStr[4];
  snprintf(numStr, sizeof(numStr), "%d", camNum);

  if (status == 2) {
    // ====== PROGRAM (ON AIR) ======
    // Fully filled white cell — maximum visibility
    display.fillRoundRect(x + 1, y + 1, w - 2, h - 2, 3, WHITE);

    // Large black number centered
    display.setTextSize(2);
    display.setTextColor(BLACK, WHITE);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);

  } else if (status == 1) {
    // ====== PREVIEW ======
    // Thick border (double outline), white number
    display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, WHITE);
    display.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 2, WHITE);

    // Large white number centered
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);

  } else {
    // ====== OFF ======
    // Thin outline only
    display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 2, WHITE);

    // Small dim number centered
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 6) / 2;
    int ty = y + (h - 7) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);
  }
}

// Derived from the same masks we broadcast, so the OLED can never disagree
// with the slaves. The mono display can't show a distinct BOTH colour, so a
// camera that is both on-air and in preview renders as PROGRAM (filled) —
// on-air is the safety-critical state to surface.
void drawTallyGrid(uint16_t progMask, uint16_t prevMask) {
  const int cols = 4, rows = 2;
  const int cellW = 128 / cols;          // 32px
  const int cellH = (64 - HDR_H) / rows; // 27px
  const int yOff = HDR_H;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      int idx = r * cols + c;
      uint8_t human = TALLY_INPUTS[idx];
      uint8_t status = 0;
      if (human >= 1 && human <= 16) { // guard the bit shift against bad config
        uint16_t bit = 1U << (human - 1);
        status = (progMask & bit) ? 2 : ((prevMask & bit) ? 1 : 0);
      }
      int x = c * cellW;
      int y = yOff + r * cellH;
      drawCell(x, y, cellW, cellH, human, status);
    }
  }
}

// ===== LoRa Debug Screen =====
void drawLoRaDebug() {
  uint8_t st = radio.getChipStatus();
  // ⚡ Bolt: Eliminate redundant SPI transaction by deriving connection state directly from the status byte
  bool connected = (st != 0xFF && st != 0x00);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Yellow zone (top 16px)
  display.setCursor(1, 0);
  display.print(connected ? "LoRa: OK  " : "LoRa: ERR ");

  // Show pins
  display.print("B:");
  display.print(digitalRead(E28_PIN_BUSY) ? "1 " : "0 ");
  display.print("D:");
  display.print(digitalRead(E28_PIN_DIO1) ? "1" : "0");

  display.setCursor(1, 8);
  display.print("SPI:0x");
  if (st < 0x10)
    display.print("0");
  display.print(st, HEX);

  // Decode chip mode — SX1280 status byte: mode = bits [7:5]
  // (0x2=STDBY_RC 0x3=STDBY_XOSC 0x4=FS 0x5=RX 0x6=TX; 0/1/7 invalid)
  uint8_t mode = (st >> 5) & 0x07;
  const char *m = "??";
  if (mode == 2)
    m = "STB";
  else if (mode == 3)
    m = "XOS";
  else if (mode == 4)
    m = "FS ";
  else if (mode == 5)
    m = "RX ";
  else if (mode == 6)
    m = "TX ";
  display.print(" ");
  display.print(m);

  // Blinker to show it's alive
  static bool blinker = false;
  blinker = !blinker;
  display.setCursor(110, 8);
  display.print(blinker ? "*" : "o");

  display.drawFastHLine(0, 15, 128, WHITE);

  // Blue zone (bottom 48px) - 4 rows
  // Row 1: RSSI + SNR
  display.setCursor(1, 18);
  display.print("RSSI: ");
  display.print(radio.getRSSI());
  display.print("dBm");
  display.setCursor(75, 18);
  display.print("SNR:");
  display.print(radio.getSNR());

  // Row 2: TX / drop counters
  display.setCursor(1, 28);
  display.print("TX:");
  display.print(g_loraTxCount);
  display.print(" Drop:");
  display.print(g_loraDropCount);

  // Row 3: SPI Diagnosis
  display.setCursor(1, 38);
  if (st == 0xFF)
    display.print("ERR: MISO HIGH");
  else if (st == 0x00)
    display.print("ERR: MISO LOW");
  else
    display.print("SPI looks OK");

  // Row 4: Uptime & ATEM
  display.setCursor(1, 48);
  display.print("Up:");
  uint32_t sec = millis() / 1000;
  display.print(sec);
  display.print("s ");
  display.print(atemPhase == ATEM_RUNNING ? "[ATEM OK]" : "[NO ATEM]");

  display.display();
}

#ifndef LORA_TEST_MODE
// ===== ATEM connection: non-blocking state machine =====
// Never blocks the loop — the radio queue, locator and OLED keep running
// while the connection is (re)established in the background.
static uint32_t atemAttemptStart = 0;

static void atemTick() {
  // Resolve the configured IP once
  static bool ipParsed = false;
  static bool ipValid = false;
  static IPAddress target;
  if (!ipParsed) {
    ipParsed = true;
    String cfgIP = String(ATEM_IP_STR);
    if (cfgIP.length())
      ipValid = target.fromString(cfgIP);
  }
  if (!ipValid)
    return; // no ATEM configured — hub keeps broadcasting last-known masks

  switch (atemPhase) {
  case ATEM_IDLE:
    if (WiFi.status() == WL_CONNECTED &&
        millis() - lastAtemAttempt > ATEM_RETRY_MS) {
      lastAtemAttempt = millis();
      if (!atem)
        atem = CreateAtemClient();
      atem->begin(target);
      atem->connect();
      atemAttemptStart = millis();
      atemPhase = ATEM_CONNECTING;
      hublogf("[ATEM] connecting to %s...\n", ipToStr(target).c_str());
    }
    break;

  case ATEM_CONNECTING:
    atem->loop();
    if (atem->connected()) {
      atemPhase = ATEM_RUNNING;
      hublogf("[ATEM] connected\n");
    } else if (millis() - atemAttemptStart > ATEM_CONNECT_TIMEOUT_MS) {
      atemPhase = ATEM_IDLE; // next try after ATEM_RETRY_MS
      hublogf("[ATEM] connect timeout, retry in %ds\n", ATEM_RETRY_MS / 1000);
    }
    break;

  case ATEM_RUNNING:
    atem->loop();
    if (!atem->connected()) {
      atemPhase = ATEM_IDLE;
      lastAtemAttempt = millis();
      hublogf("[ATEM] connection lost\n");
    }
    break;
  }
}
#endif // LORA_TEST_MODE

void setup() {
  // Native USB CDC; no host attached must never block the loop
  Serial.begin();
  Serial.setTxTimeoutMs(0);
  Serial0.begin(115200); // UART bridge port (GPIO43/44)
  // readStringUntil() otherwise stalls the loop for its default 1s timeout on
  // a noise burst without a newline (e.g. a floating/glitching UART line)
  Serial.setTimeout(50);
  Serial0.setTimeout(50);

  I2Cbus.begin(OLED_I2C_SDA, OLED_I2C_SCL, 400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setTextWrap(false);
  display.clearDisplay();
  display.display();

  pinMode(0, INPUT_PULLUP); // Boot button for locator

  hublogf("\n=== Tally HUB (ESP32-S3 + E28-2G4M27S) ===\n");
  // Reset reason turns "module not connected after a warm reboot" from a
  // guess into a fact: ESP_RST_BROWNOUT after a TX storm = the PA/WiFi rail
  // is sagging (decouple it), vs ESP_RST_SW/POWERON = ordinary boot.
  hublogf("[BOOT] reset_reason=%d\n", (int)esp_reset_reason());
  hublogf("[CFG] netId=0x%02X freq=%lu preamble=%d power=%d refresh=%dms\n",
          TALLY_NET_ID, (unsigned long)TALLY_RF_FREQ_HZ,
          TALLY_PREAMBLE_SYMBOLS, TALLY_TX_POWER, TALLY_REFRESH_MS);
#ifdef LORA_TEST_MODE
  hublogf("[CFG] LORA_TEST_MODE active — ATEM disabled, cam1 toggle stream\n");
#endif

  // ==== E28 LoRa FIRST (with retry) ====
  // Before WiFi: the radio is the hub's core function, and WiFi's TX bursts
  // on the shared 3V3 rail are the last thing a 27dBm module needs while
  // it powers up (historically a source of flaky inits).
  bool radioOk = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    drawCenteredMsg("LoRa init...",
                    (String("Attempt ") + String(attempt) + "/5").c_str());
    radioOk = radioInit();
    if (radioOk)
      break;
    // Show WHY on both the OLED and serial, not just "FAILED"
    hublogf("[E28] attempt %d/5 failed: %s (status=0x%02X BUSY=%d DIO1=%d)\n",
            attempt, radio.initErrorStr(), radio.getChipStatus(),
            digitalRead(E28_PIN_BUSY), digitalRead(E28_PIN_DIO1));
    drawCenteredMsg("LoRa FAILED", radio.initErrorStr());
    delay(500);
  }
  if (radioOk)
    hublogf("[E28] init OK (status=0x%02X)\n", radio.getChipStatus());
  else
    hublogf("[E28] init FAILED after 5 attempts — recovery retries every 10s\n");

  // ==== Wi-Fi ====
  drawCenteredMsg("Wi-Fi: connecting...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // ждём Wi‑Fi с анимацией
  uint8_t wifiFrame = 0;
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_RETRY_MS) {
    drawLoadingScreen("Wi-Fi", WIFI_SSID, wifiFrame++);
    delay(150);
  }
  if (WiFi.status() != WL_CONNECTED) {
    hublogf("[WiFi] FAILED to join '%s' — retrying in background\n", WIFI_SSID);
    drawCenteredMsg("Wi-Fi: FAILED", "Retrying forever...");
  } else {
    hublogf("[WiFi] connected, IP %s\n", ipToStr(WiFi.localIP()).c_str());
    drawCenteredMsg("Wi-Fi: connected", ipToStr(WiFi.localIP()).c_str());
  }

  // Show LoRa debug screen for 3 seconds
  drawLoRaDebug();
  delay(3000);
}

// ===== Serial console (both ports): status / ping / reinit / help =====
static void handleSerialCommand(const String &cmd) {
  if (cmd == "status") {
    uint8_t qDepth =
        (g_loraQueueHead + LORA_QUEUE_SIZE - g_loraQueueTail) % LORA_QUEUE_SIZE;
    char ipbuf[20];
    if (WiFi.status() == WL_CONNECTED)
      snprintf(ipbuf, sizeof(ipbuf), "%u.%u.%u.%u", WiFi.localIP()[0],
               WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    else
      strcpy(ipbuf, "DOWN");
    uint16_t reachMask = 0;
    for (uint8_t id = 1; id <= 16; id++)
      if (g_camReachable[id])
        reachMask |= (1U << (id - 1));
    hublogf("[STATUS] up=%lus radio=%s(0x%02X) tx=%lu drop=%lu q=%u "
            "wifi=%s atem=%s prog=0x%04X prev=0x%04X reach=0x%04X\n",
            (unsigned long)(millis() / 1000), radio.isConnected() ? "OK" : "DEAD",
            radio.getChipStatus(), (unsigned long)g_loraTxCount,
            (unsigned long)g_loraDropCount, qDepth, ipbuf,
            atemPhase == ATEM_RUNNING      ? "RUNNING"
            : atemPhase == ATEM_CONNECTING ? "CONNECTING"
                                           : "IDLE",
            g_progMask, g_prevMask, reachMask);
  } else if (cmd == "ping") {
    if (locatorStep == 0) {
      hublogf("[CMD] locator ping -> cam 1\n");
      locatorStep = 1;
      locatorStartTime = millis();
    }
  } else if (cmd == "reinit") {
    hublogf("[CMD] radio re-init: %s (%s)\n",
            radioInit() ? "OK" : "FAILED", radio.initErrorStr());
  } else if (cmd == "help") {
    hublogf("Commands: status, ping, reinit, help\n");
  } else if (cmd.length()) {
    hublogf("Unknown command '%s' — try 'help'\n", cmd.c_str());
  }
}

static void pollSerialCommands() {
  Stream *ports[2] = {&Serial, &Serial0};
  for (Stream *port : ports) {
    if (port->available()) {
      String cmd = port->readStringUntil('\n');
      cmd.trim();
      handleSerialCommand(cmd);
    }
  }
}

void loop() {
  processLoraQueue();
  serviceTelemetry();  // receive slave telemetry in idle windows
  sweepReachability(); // log cameras going online/offline

  // Radio recovery: re-init every 10s while disconnected (begin() bails out
  // in ~150ms when the module is absent, so this stays affordable)
  if (!radio.isConnected()) {
    static uint32_t lastRadioRetry = 0;
    if (millis() - lastRadioRetry > 10000) {
      lastRadioRetry = millis();
      if (radioInit())
        hublogf("[E28] recovered (status=0x%02X)\n", radio.getChipStatus());
      else
        hublogf("[E28] recovery failed: %s\n", radio.initErrorStr());
    }
  }

  // WiFi reconnect (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > WIFI_RETRY_MS) {
      lastRetry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

#ifdef LORA_TEST_MODE
  // === TEST STREAM: cam 1 RED <-> GREEN every 1s (radio bring-up without ATEM)
  static uint32_t lastToggle = 0;
  static bool testRed = true;
  if (millis() - lastToggle > 1000) {
    lastToggle = millis();
    testRed = !testRed;
    g_progMask = testRed ? 0x0001 : 0x0000;
    g_prevMask = testRed ? 0x0000 : 0x0001;
  }
#else
  atemTick();

  if (atemPhase == ATEM_RUNNING && millis() - lastPoll > POLL_MS) {
    lastPoll = millis();
    g_progMask = 0;
    g_prevMask = 0;
    for (int i = 0; i < 8; i++) {
      uint8_t human = TALLY_INPUTS[i];
      if (human < 1 || human > 16)
        continue; // out-of-range entry would make the bit shift below UB
      uint8_t idx0 = human - 1;
      if (atem->isOnAir(idx0))
        g_progMask |= (1U << idx0);
      if (atem->isPreview(idx0))
        g_prevMask |= (1U << idx0);
    }
  }
#endif

  // === STATE_ALL broadcast: on change + every TALLY_REFRESH_MS (heartbeat).
  // Runs even with ATEM down — losing the switcher must not look like a dead
  // radio link to the slaves; they keep showing the last known masks.
  static uint32_t lastStateSend = 0;
  static uint16_t lastSentProg = 0xFFFF;
  static uint16_t lastSentPrev = 0xFFFF;
  bool masksChanged = (g_progMask != lastSentProg) || (g_prevMask != lastSentPrev);
  if (masksChanged || millis() - lastStateSend >= TALLY_REFRESH_MS) {
    lastSentProg = g_progMask;
    lastSentPrev = g_prevMask;
    lastStateSend = millis();
    // sourceLive: in production, true only while ATEM is actually connected
    // (frozen masks after an ATEM drop are flagged stale so slaves can warn);
    // in test mode the generated stream is always "live".
#ifdef LORA_TEST_MODE
    bool sourceLive = true;
#else
    bool sourceLive = (atemPhase == ATEM_RUNNING);
#endif
    TallyPacket pkt =
        TallyProtocol::createStateAllPacket(g_progMask, g_prevMask, sourceLive);
    // Burst-on-change: a tally transition ("camera goes ON AIR") is otherwise a
    // single fire-and-forget packet; one RF collision would show the wrong
    // light until the next heartbeat (up to TALLY_REFRESH_MS). Sending the
    // change 3x (queued, ~2ms apart) drops the residual loss from p to ~p^3 and
    // cuts worst-case wrong-light latency from ~500ms to tens of ms. Heartbeats
    // (no change) send once — slaves act on STATE_ALL idempotently.
    enqueueLora(pkt);
    if (masksChanged) {
      enqueueLora(pkt);
      enqueueLora(pkt);
    }
  }

  pollSerialCommands();

  // Periodic status heartbeat on serial (10s) — one line tells whether the
  // radio/queue/WiFi/ATEM are alive without touching the device
  static uint32_t lastStatusLog = 0;
  if (millis() - lastStatusLog > 10000) {
    lastStatusLog = millis();
    handleSerialCommand("status");
  }

  // Locator / Ping (Button 0 or serial "ping") — works without ATEM
  // ⚡ Bolt: Non-blocking locator logic using enqueueLora
  if (digitalRead(0) == LOW && locatorStep == 0) {
    drawCenteredMsg("LOCATOR", "Ping sent -> Cam 1");
    locatorStep = 1;
    locatorStartTime = millis();
  }

  if (locatorStep > 0) {
    uint32_t now = millis();
    uint32_t elapsed = now - locatorStartTime;

    // Steps 1, 3, 5: Enqueue ping (wait 50ms before steps 3 and 5)
    if ((locatorStep == 1) || (locatorStep % 2 != 0 && locatorStep <= 5 && elapsed >= 50)) {
      TallyPacket pkt = TallyProtocol::createPingPacket(1);
      enqueueLora(pkt);
      locatorStep++;
      locatorStartTime = now;
    }
    // Steps 2, 4, 6: Wait 100ms
    else if (locatorStep % 2 == 0 && locatorStep <= 6 && elapsed >= 100) {
      locatorStep++;
      locatorStartTime = now;
    }
    // Wait 2000ms after sequence before allowing another ping
    else if (locatorStep == 7 && elapsed >= 2000) {
      locatorStep = 0;
    }
  }

  // ==== OLED: tally grid when ATEM is live, debug screen otherwise ====
  // The grid (production view) redraws only when the masks/connection change,
  // plus a slow refresh — a full 128x64 I2C blit is ~23ms and used to run every
  // 500ms unconditionally, which also delayed checkTxDone() mid-transmit.
  static uint32_t lastDraw = 0;
  static uint16_t drawnProg = 0xFFFF, drawnPrev = 0xFFFF;
  static bool drawnConnected = false;

  // ⚡ Bolt: Prevent slow, blocking I2C screen updates during high-priority
  // non-blocking UI sequences (like LOCATOR). Also skip the blit while a TX is
  // in flight: a ~23ms full-frame I2C blit would otherwise defer checkTxDone()
  // (PA-off) and the next packet by that much. The redraw runs next pass.
  bool uiActive = (locatorStep > 0) || radio.txActive();
  bool connected = (atemPhase == ATEM_RUNNING);

  if (!uiActive) {
    if (connected) {
      bool dirty = (g_progMask != drawnProg) || (g_prevMask != drawnPrev) ||
                   !drawnConnected;
      if (dirty || millis() - lastDraw > 2000) {
        lastDraw = millis();
        drawnProg = g_progMask;
        drawnPrev = g_prevMask;
        drawnConnected = true;
        display.clearDisplay();
        drawStatusBar(ipToStr(WiFi.localIP()), WiFi.status() == WL_CONNECTED,
                      radio.isConnected(), true);
        drawTallyGrid(g_progMask, g_prevMask);
        display.display();
      }
    } else if (millis() - lastDraw > 500) {
      // Debug screen is a transient bring-up view; its blinker/counters justify
      // the faster cadence.
      lastDraw = millis();
      drawnConnected = false;
      drawLoRaDebug();
    }
  }

  // Second queue pass: with one pass per ~10ms loop, TxDone detection (and
  // PA-off) lagged the ~15ms airtime by up to a full tick; this halves it
  processLoraQueue();

  delay(10);
}
