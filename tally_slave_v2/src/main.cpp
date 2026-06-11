// ============================================================
//  SUFIDE Tally Slave v2 — ESP32-C3 SuperMini + E28-2G4M12SX
//  Hardware: WS2812B (GPIO 7), Buzzer (GPIO 8)
//  Pinout verified per HARDWARE_GUIDE.md (2026-02-10)
//
//  RX is interrupt-driven (DIO1) and, with -DPOWER_SAVE, duty-cycled:
//  the radio sleeps ~66% of the time and still catches every packet
//  thanks to the hub's 40-symbol preamble (see TallyConfig.h).
// ============================================================

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <E28_SX1280.h>
#include <TallyProtocol.h>
#include <TallyRadio.h>

// ===== CONFIGURATION =====
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID 1 // Camera ID this slave responds to (-DSLAVE_CAM_ID=n)
#endif

// ===== HARDWARE PINS (from HARDWARE_GUIDE.md) =====
// LoRa E28-2G4M12SX
#define PIN_LORA_SCK 3
#define PIN_LORA_MOSI 4
#define PIN_LORA_MISO 5
#define PIN_LORA_NRESET 6
#define PIN_LORA_NSS 20
#define PIN_LORA_BUSY 10
#define PIN_LORA_DIO1 1
#define PIN_LORA_RXEN -1 // Not present on 12dBm module
#define PIN_LORA_TXEN -1

// WS2812B RGB LED
#define PIN_LED 7
#define NUM_LEDS 1

// Passive Buzzer
#define PIN_BUZZER 8

// ===== COLORS =====
#define COLOR_OFF 0x000000
#define COLOR_PROGRAM 0xFF0000   // Red
#define COLOR_PREVIEW 0x00FF00   // Green
#define COLOR_BOTH 0xFFFF00      // Yellow
#define COLOR_PING 0x0000FF      // Blue (locator flash)
#define COLOR_LOST 0xFF4500      // Orange (signal lost)
#define COLOR_INIT_OK 0x00FF00   // Green
#define COLOR_INIT_FAIL 0xFF0000 // Red

// ===== GLOBALS =====
Adafruit_NeoPixel led(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
E28Radio radio;

uint32_t lastHeartbeat = 0;
uint32_t lastRxTime = 0; // Track last received packet time
bool loraConnected = false;
TallyState currentState = STATE_OFF;
uint32_t rxCount = 0;  // Total packets received since last heartbeat
uint32_t crcFails = 0; // CRC failures since last heartbeat
bool signalLost = false;

// DIO1 fires on RxDone/CrcError; the loop drains the event and re-arms RX
static volatile bool g_dio1Flag = false;
static void IRAM_ATTR onDio1() { g_dio1Flag = true; }

// Locator: non-blocking 5x (80ms blue+2400Hz / 40ms off) sequence
uint32_t locatorStart = 0;
bool locatorActive = false;
bool locatorPhaseOn = false;

// ===== HELPERS =====
void setColor(uint32_t color, uint8_t brightness = 80) {
  led.setBrightness(brightness);
  led.setPixelColor(0, color);
  led.show();
}

void flashColor(uint32_t color, int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    setColor(color, 120);
    delay(onMs);
    setColor(COLOR_OFF);
    delay(offMs);
  }
}

// Blocking beep — setup/bench use only, never from loop()
void beep(int freq, int durationMs) {
  tone(PIN_BUZZER, freq, durationMs);
  delay(durationMs);
  noTone(PIN_BUZZER);
}

void applyTallyColor() {
  switch (currentState) {
  case STATE_PROGRAM:
    setColor(COLOR_PROGRAM, 120);
    break;
  case STATE_PREVIEW:
    setColor(COLOR_PREVIEW, 80);
    break;
  case STATE_BOTH:
    setColor(COLOR_BOTH, 120);
    break;
  case STATE_OFF:
  default:
    setColor(COLOR_OFF);
    break;
  }
}

// Arm the receiver. Under POWER_SAVE the chip exits the duty cycle after
// every RxDone (even CRC errors), so this must be called after each DIO1
// event; without POWER_SAVE a cheap IRQ-clear keeps continuous RX going.
void rearmReceive(bool full = false) {
#ifdef POWER_SAVE
  (void)full;
  radio.startReceiveDutyCycle(TALLY_DC_RX_MS, TALLY_DC_SLEEP_MS);
#else
  if (full)
    radio.startReceive();
  else
    radio.clearRxIrq();
#endif
}

void startLocator() {
  locatorActive = true;
  locatorStart = millis();
  locatorPhaseOn = true;
  setColor(COLOR_PING, 200);
  tone(PIN_BUZZER, 2400);
}

void updateLocator() {
  if (!locatorActive)
    return;

  uint32_t elapsed = millis() - locatorStart;
  if (elapsed >= 600) { // 5 cycles x 120ms
    locatorActive = false;
    noTone(PIN_BUZZER);
    // Don't paint the (possibly stale) tally colour over a dead link — that
    // could show solid RED ("you're live") on data minutes old. Hand straight
    // back to the orange signal-lost indication.
    if (signalLost)
      setColor(COLOR_LOST, 60);
    else
      applyTallyColor();
    return;
  }

  bool on = (elapsed % 120) < 80; // 80ms on / 40ms off
  if (on != locatorPhaseOn) {
    locatorPhaseOn = on;
    if (on) {
      setColor(COLOR_PING, 200);
      tone(PIN_BUZZER, 2400);
    } else {
      setColor(COLOR_OFF);
      noTone(PIN_BUZZER);
    }
  }
}

// ===== SETUP =====
void setup() {
#ifdef POWER_SAVE
  // 80 MHz is the floor: USB-Serial-JTAG needs >=80 MHz. Light sleep is
  // deliberately NOT used — it powers down the USB-Serial-JTAG peripheral
  // (host drops the port) and the prebuilt Arduino core has CONFIG_PM_ENABLE
  // off, so automatic light sleep is unavailable anyway. The savings come
  // from the radio duty cycle + lower CPU clock + FreeRTOS idle.
  setCpuFrequencyMhz(80);
#endif

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=============================");
  Serial.println("  SUFIDE Tally Slave v2");
  Serial.printf("  Camera ID: %d  NetID: 0x%02X\n", SLAVE_CAM_ID, TALLY_NET_ID);
  Serial.println("=============================");

  led.begin();
  setColor(COLOR_OFF);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  Serial.print("[LoRa] Init... ");
  loraConnected = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                              PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                              PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (loraConnected) {
    Serial.println("OK");
    flashColor(COLOR_INIT_OK, 3, 150, 100);
    beep(1000, 100);

    tallyApplyRadioProfile(radio);

    attachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO1), onDio1, RISING);
    rearmReceive(true);
    lastRxTime = millis(); // Start signal timer
    Serial.println("[LoRa] Listening...");
  } else {
    Serial.println("FAILED");
    beep(400, 500);
    while (1) {
      setColor(COLOR_INIT_FAIL, 100);
      delay(100);
      setColor(COLOR_OFF);
      delay(100);
    }
  }
}

// Re-init the radio if a runtime fault (stuck BUSY) latched it disconnected.
// Without this a single glitch would leave the receiver permanently deaf,
// since every RX call is a guarded no-op while _connected is false.
void tryRadioRecover() {
  static uint32_t lastTry = 0;
  if (radio.isConnected())
    return;
  if (millis() - lastTry < 10000)
    return;
  lastTry = millis();
  Serial.println("[LoRa] Recovering...");
  if (radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS,
                  PIN_LORA_BUSY, PIN_LORA_DIO1, PIN_LORA_NRESET, PIN_LORA_RXEN,
                  PIN_LORA_TXEN)) {
    tallyApplyRadioProfile(radio);
    rearmReceive(true);
    lastRxTime = millis();
    Serial.println("[LoRa] Recovered");
  }
}

// ===== LOOP =====
void loop() {
  if (!loraConnected)
    return;

  tryRadioRecover();
  updateLocator();

  // === RX: interrupt-driven (no SPI polling — under POWER_SAVE any NSS
  // activity during the radio's sleep phase would silently kill the cycle)
  if (g_dio1Flag || digitalRead(PIN_LORA_DIO1) == HIGH) {
    g_dio1Flag = false;

    if (radio.available()) {
      uint8_t buf[TALLY_PACKET_SIZE];
      uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);

      if (len > 0) {
        TallyPacket pkt;
        if (TallyProtocol::deserialize(buf, len, pkt)) {
          lastRxTime = millis(); // Any valid packet proves the link is alive
          rxCount++;
          if (signalLost) {
            signalLost = false;
            Serial.println("[LINK] Signal restored");
            if (!locatorActive)
              applyTallyColor();
          }

          if (pkt.command == CMD_PING) {
            if (pkt.payload[0] == SLAVE_CAM_ID ||
                pkt.payload[0] == TALLY_BROADCAST_ID) {
              Serial.println("[LOCATOR] Alert!");
              startLocator();
            }
          } else if (pkt.command == CMD_STATE_ALL) {
            TallyState ts = TallyProtocol::stateForCamera(pkt, SLAVE_CAM_ID);
            if (ts != currentState) {
              currentState = ts;
              Serial.printf("[TALLY] State:%d RSSI:%d\n", ts, radio.getRSSI());
              if (!locatorActive && !signalLost)
                applyTallyColor();
            }
          }
        } else {
          crcFails++;
          if (crcFails <= 10) {
            Serial.printf("[CRC_FAIL] len=%d raw: ", len);
            for (int i = 0; i < len && i < 16; i++)
              Serial.printf("%02X ", buf[i]);
            Serial.println();
          }
        }
      }
    }
    // Unconditional re-arm: RxDone (even a CRC error) ends the duty cycle
    rearmReceive();
  }

  // === SIGNAL LOST DETECTION (hub broadcasts every TALLY_REFRESH_MS) ===
  if (!signalLost && millis() - lastRxTime > TALLY_SIGNAL_LOST_MS) {
    signalLost = true;
    Serial.println("[WARN] Signal lost!");
    tone(PIN_BUZZER, 800, 300); // non-blocking beep
  }

  if (signalLost && !locatorActive) {
    // Pulse orange every second
    static uint32_t lastPulse = 0;
    static bool pulseOn = false;
    if (millis() - lastPulse > 1000) {
      lastPulse = millis();
      pulseOn = !pulseOn;
      setColor(pulseOn ? COLOR_LOST : COLOR_OFF, 60);
    }
    // Short beep every 5 seconds
    static uint32_t lastLostBeep = 0;
    if (millis() - lastLostBeep > 5000) {
      lastLostBeep = millis();
      tone(PIN_BUZZER, 600, 100);
    }
  }

  // === RX SAFETY NET: timed full re-arm, NOT an SPI poll ===
  // Restores RX no matter what state the chip fell into (duty cycle ended,
  // missed interrupt, ...). Cheap and idempotent.
  static uint32_t lastRearm = 0;
  if (millis() - lastRxTime > TALLY_RX_REARM_MS &&
      millis() - lastRearm > TALLY_RX_REARM_MS) {
    lastRearm = millis();
    rearmReceive(true);
  }

  // Heartbeat: status log every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    lastHeartbeat = millis();
    uint32_t secSinceRx = (millis() - lastRxTime) / 1000;
    Serial.printf("[STATUS] Up:%lus State:%d LastRX:%lus ago RX:%lu CRC:%lu\n",
                  millis() / 1000, currentState, secSinceRx, rxCount, crcFails);
    rxCount = 0;
    crcFails = 0;
  }

  // Serial commands (for testing)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // red/green/off drive currentState (not just the LED) so the next
    // heartbeat doesn't leave a stale test colour stuck on the pixel
    if (cmd == "test") {
      Serial.println("[TEST] Locator alert...");
      startLocator();
    } else if (cmd == "red") {
      currentState = STATE_PROGRAM;
      applyTallyColor();
    } else if (cmd == "green") {
      currentState = STATE_PREVIEW;
      applyTallyColor();
    } else if (cmd == "off") {
      currentState = STATE_OFF;
      applyTallyColor();
    } else if (cmd == "beep") {
      beep(1000, 200);
    } else if (cmd == "help") {
      Serial.println("Commands: test, red, green, off, beep, help");
    }
  }

#ifdef POWER_SAVE
  delay(5); // FreeRTOS idle -> CPU clock-gating between events
#endif
}
