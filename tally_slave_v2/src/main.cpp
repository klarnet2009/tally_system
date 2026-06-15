// ============================================================
//  SUFIDE Tally Slave v2 — ESP32-C3 SuperMini + E28-2G4M12SX
//  Hardware: WS2812B (GPIO 7), Buzzer (GPIO 8)
//  Pinout verified per HARDWARE_GUIDE.md (2026-02-10), see pins.h
//
//  RX is interrupt-driven (DIO1) and, with -DPOWER_SAVE, duty-cycled:
//  the radio sleeps ~66% of the time and still catches every packet
//  thanks to the hub's 40-symbol preamble (see TallyConfig.h).
//  Protocol dispatch + link supervision live in TallyLink (shared with
//  slave v1); this file only renders states on the LED/buzzer.
// ============================================================

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <E28_SX1280.h>
#include <TallyLink.h>
#include <TallyProtocol.h>
#include <TallyRadio.h>

#include "pins.h"

// ===== CONFIGURATION =====
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID 1 // Camera ID this slave responds to (-DSLAVE_CAM_ID=n)
#endif

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
TallyLink tallyLink;

uint32_t lastHeartbeat = 0;
uint32_t rxCount = 0;  // Valid packets since last heartbeat
uint32_t rxFails = 0;  // Rejected packets (noise/CRC/foreign) since heartbeat

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
  switch (tallyLink.state()) {
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

// Arm the receiver in the mode this build uses; after this, the driver's
// rearmAfterIrq()/restartReceive() remember and re-issue the right thing.
void armReceive() {
#ifdef POWER_SAVE
  radio.startReceiveDutyCycle(TALLY_DC_RX_MS, TALLY_DC_SLEEP_MS);
#else
  radio.startReceive();
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
    if (tallyLink.signalLost())
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

// ===== TallyLink presentation callbacks =====
void onTallyState(TallyState ts) {
  Serial.printf("[TALLY] State:%d RSSI:%d\n", ts, radio.getRSSI());
  if (!locatorActive && !tallyLink.signalLost())
    applyTallyColor();
}

void onLocatorPing() {
  Serial.println("[LOCATOR] Alert!");
  startLocator();
}

void onLinkChange(bool lost) {
  if (lost) {
    Serial.println("[WARN] Signal lost!");
    tone(PIN_BUZZER, 800, 300); // non-blocking beep
  } else {
    Serial.println("[LINK] Signal restored");
    if (!locatorActive)
      applyTallyColor();
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
    armReceive();
    tallyLink.noteAlive();
    Serial.println("[LoRa] Recovered");
  } else {
    Serial.printf("[LoRa] Recovery failed: %s\n", radio.initErrorStr());
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
  bool ok = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                        PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                        PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (!ok) {
    Serial.printf("FAILED: %s\n", radio.initErrorStr());
    beep(400, 500);
    while (1) {
      setColor(COLOR_INIT_FAIL, 100);
      delay(100);
      setColor(COLOR_OFF);
      delay(100);
    }
  }

  Serial.println("OK");
  flashColor(COLOR_INIT_OK, 3, 150, 100);
  beep(1000, 100);

  tallyApplyRadioProfile(radio);

  tallyLink.begin(SLAVE_CAM_ID, onTallyState, onLocatorPing, onLinkChange);

  attachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO1), onDio1, RISING);
  armReceive();
  Serial.println("[LoRa] Listening...");
}

// ===== LOOP =====
void loop() {
  tryRadioRecover();
  updateLocator();

  // === RX: interrupt-driven (no SPI polling — under POWER_SAVE any NSS
  // activity during the radio's sleep phase would silently kill the cycle)
  if (g_dio1Flag || digitalRead(PIN_LORA_DIO1) == HIGH) {
    g_dio1Flag = false;

    // Bounded drain: a second packet can complete while we process the first
    // (continuous RX keeps receiving); re-checking available() before the
    // re-arm closes the window where its RxDone would be wiped by the IRQ
    // clear below and the payload silently abandoned in the FIFO. The cap
    // keeps a flooding neighbour from starving the rest of loop().
    for (int drain = 0; drain < 4 && radio.available(); drain++) {
      uint8_t buf[TALLY_PACKET_SIZE];
      uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);

      if (len > 0) {
        if (tallyLink.onPacket(buf, len)) {
          rxCount++;
        } else {
          rxFails++;
          if (rxFails <= 10) {
            Serial.printf("[RX_FAIL] len=%d raw: ", len);
            for (int i = 0; i < len && i < 16; i++)
              Serial.printf("%02X ", buf[i]);
            Serial.println();
          }
        }
      }
    }
    // Unconditional re-arm: RxDone (even a CRC error) ends the duty cycle
    radio.rearmAfterIrq();
  }

  // === LINK SUPERVISION (hub broadcasts every TALLY_REFRESH_MS) ===
  tallyLink.tick();

  if (tallyLink.signalLost() && !locatorActive) {
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
  if (tallyLink.msSinceLastRx() > TALLY_RX_REARM_MS &&
      millis() - lastRearm > TALLY_RX_REARM_MS) {
    lastRearm = millis();
    radio.restartReceive();
  }

  // Heartbeat: status log every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] Up:%lus State:%d LastRX:%lus ago RX:%lu Fail:%lu\n",
                  millis() / 1000, tallyLink.state(), tallyLink.msSinceLastRx() / 1000,
                  rxCount, rxFails);
    rxCount = 0;
    rxFails = 0;
  }

  // Serial commands (for testing)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // red/green/off drive the link state (not just the LED) so the next
    // heartbeat doesn't leave a stale test colour stuck on the pixel
    if (cmd == "test") {
      Serial.println("[TEST] Locator alert...");
      startLocator();
    } else if (cmd == "red") {
      tallyLink.forceState(STATE_PROGRAM);
      applyTallyColor();
    } else if (cmd == "green") {
      tallyLink.forceState(STATE_PREVIEW);
      applyTallyColor();
    } else if (cmd == "off") {
      tallyLink.forceState(STATE_OFF);
      applyTallyColor();
    } else if (cmd == "beep") {
      beep(1000, 200);
    } else if (cmd == "help") {
      Serial.println("Commands: test, red, green, off, beep, help");
    }
  }

  // FreeRTOS idle -> CPU clock-gating between events. Kept outside the
  // POWER_SAVE ifdef: RX is DIO1-driven either way, so a non-power-save
  // build gains nothing from spinning at 100% CPU.
  delay(5);
}
