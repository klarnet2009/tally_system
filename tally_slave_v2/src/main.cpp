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
#include <Preferences.h>
#include <E28_SX1280.h>
#include <TallyLink.h>
#include <TallyProtocol.h>
#include <TallyRadio.h>

#include "pins.h"

// ===== CONFIGURATION =====
// Compile-time fallback only; the live camera ID is stored in NVS and set in
// the field (BOOT button or serial "id N"), so ONE binary serves every camera.
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID 1
#endif

static uint8_t g_camId = SLAVE_CAM_ID;

// ===== COLORS =====
#define COLOR_OFF 0x000000
#define COLOR_PROGRAM 0xFF0000   // Red
#define COLOR_PREVIEW 0x00FF00   // Green
#define COLOR_BOTH 0xFFFF00      // Yellow
#define COLOR_PING 0x0000FF      // Blue (locator flash)
#define COLOR_LOST 0xFF4500      // Orange (radio signal lost)
#define COLOR_STALE 0xFFFFFF     // White (tally source frozen — don't trust)
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

// Buzzer policy: never sound while the camera is ON AIR — a live mic would
// capture the tone. -DTALLY_QUIET disables the buzzer entirely for
// sound-sensitive productions. Visual indications always fire.
static bool buzzerAllowed() {
#ifdef TALLY_QUIET
  return false;
#else
  TallyState s = tallyLink.state();
  return s != STATE_PROGRAM && s != STATE_BOTH;
#endif
}
static void buzzOn(int freq) {
  if (buzzerAllowed())
    tone(PIN_BUZZER, freq);
}
static void buzzPulse(int freq, int durMs) { // non-blocking
  if (buzzerAllowed())
    tone(PIN_BUZZER, freq, durMs);
}

void startLocator() {
  locatorActive = true;
  locatorStart = millis();
  locatorPhaseOn = true;
  setColor(COLOR_PING, 200);
  buzzOn(2400);
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
      buzzOn(2400);
    } else {
      setColor(COLOR_OFF);
      noTone(PIN_BUZZER);
    }
  }
}

// ===== TallyLink presentation callbacks =====
void onTallyState(TallyState ts) {
  Serial.printf("[TALLY] State:%d RSSI:%d\n", ts, radio.getRSSI());
  // Only paint a solid colour when the light is trustworthy; otherwise the
  // signal-lost / source-stale indications below own the LED.
  if (!locatorActive && tallyLink.trustworthy())
    applyTallyColor();
}

void onLocatorPing() {
  Serial.println("[LOCATOR] Alert!");
  startLocator();
}

// ===== Camera ID provisioning (NVS) =====
// One universal binary: the camera ID lives in NVS, not the firmware image.
// Set it in the field by holding BOOT at power-up (tap to count) or via the
// serial "id N" command. SLAVE_CAM_ID is only the first-boot default.
static uint8_t loadCamId() {
  Preferences p;
  p.begin("tally", true);
  uint8_t id = p.getUChar("camid", SLAVE_CAM_ID);
  p.end();
  if (id < 1 || id > 16)
    id = SLAVE_CAM_ID;
  return id;
}

static void saveCamId(uint8_t id) {
  Preferences p;
  p.begin("tally", false);
  p.putUChar("camid", id);
  p.end();
}

// If BOOT is held at power-up, enter set-ID mode: each tap increments the
// count (1..8, wraps), the LED flashes the count and the buzzer chirps; 3s of
// inactivity commits to NVS. Blocking, but this only runs at boot on request.
static uint8_t runSetIdModeIfRequested(uint8_t current) {
  pinMode(PIN_BOOT, INPUT_PULLUP);
  delay(20);
  if (digitalRead(PIN_BOOT) == HIGH)
    return current; // not held — normal boot

  Serial.println("[CFG] Set-ID mode: tap BOOT to count (1..8), idle 3s to save");
  while (digitalRead(PIN_BOOT) == LOW)
    delay(10); // wait for the initial hold to release

  uint8_t count = 0;
  uint32_t lastActivity = millis();
  while (millis() - lastActivity < 3000) {
    if (digitalRead(PIN_BOOT) == LOW) {
      delay(30); // debounce
      if (digitalRead(PIN_BOOT) == LOW) {
        count = (count >= 8) ? 1 : count + 1;
        flashColor(COLOR_PING, count, 130, 130); // blink the count back
        beep(1500, 80);
        lastActivity = millis();
        while (digitalRead(PIN_BOOT) == LOW)
          delay(10); // wait for release
      }
    }
    delay(10);
  }
  if (count >= 1 && count <= 16) {
    saveCamId(count);
    Serial.printf("[CFG] Saved camera ID = %d\n", count);
    flashColor(COLOR_INIT_OK, 2, 200, 150);
    return count;
  }
  return current;
}

void onLinkChange(bool lost) {
  if (lost) {
    Serial.println("[WARN] Signal lost!");
    buzzPulse(800, 300);
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

  led.begin();
  setColor(COLOR_OFF);
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  // Resolve camera ID from NVS (or the set-ID flow if BOOT is held)
  g_camId = loadCamId();
  g_camId = runSetIdModeIfRequested(g_camId);

  Serial.println("\n=============================");
  Serial.println("  SUFIDE Tally Slave v2");
  Serial.printf("  Camera ID: %d  NetID: 0x%02X\n", g_camId, TALLY_NET_ID);
#ifdef POWER_SAVE
  Serial.println("  RX: duty-cycle (POWER_SAVE)");
#else
  Serial.println("  RX: continuous");
#endif
  Serial.println("=============================");

  Serial.print("[LoRa] Init... ");
  bool ok = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                        PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                        PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (ok) {
    Serial.println("OK");
    flashColor(COLOR_INIT_OK, 3, 150, 100);
    beep(1000, 100);
    tallyApplyRadioProfile(radio);
  } else {
    // Non-terminal: instead of trapping the operator on a forever-blink that
    // needs a manual power-cycle, fall through to loop() — tryRadioRecover()
    // re-inits every 10s (the slave has a real reset line) and the signal-lost
    // indication shows "not working" until it heals. A warm-boot module that
    // came up slow/wedged then self-recovers.
    Serial.printf("FAILED: %s — retrying in loop()\n", radio.initErrorStr());
    beep(400, 200);
  }

  tallyLink.begin(g_camId, onTallyState, onLocatorPing, onLinkChange);

  // ISR only sets a flag, harmless even if the radio is down; if recovery
  // brings it up later RX still works (the loop also polls DIO1 level).
  attachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO1), onDio1, RISING);
  if (ok)
    armReceive();
  Serial.println(ok ? "[LoRa] Listening..." : "[LoRa] Waiting for radio...");
}

// ===== LOOP =====
void loop() {
  tryRadioRecover();
  updateLocator();

  // === RX: interrupt-driven (no SPI polling — under POWER_SAVE any NSS
  // activity during the radio's sleep phase would silently kill the cycle)
  if (g_dio1Flag || digitalRead(PIN_LORA_DIO1) == HIGH) {
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
    // Clear the flag AFTER re-arming: if DIO1 fires during the re-arm's
    // standby→SET_RX SPI sequence, the flag stays set and we re-enter next
    // loop (a harmless duplicate pass gated by available()) instead of
    // losing that packet to the re-arm's IRQ clear.
    g_dio1Flag = false;
  }

  // === LINK SUPERVISION (hub broadcasts every TALLY_REFRESH_MS) ===
  tallyLink.tick();

  // === "DON'T TRUST THE LIGHT" indications (priority just below locator) ===
  // signalLost  = radio link dead: orange pulse + periodic beep.
  // sourceStale = link alive but the hub's ATEM source is frozen: hold the
  //               last tally colour but blink WHITE over it, so the operator
  //               sees the colour may be stale without losing the held state.
  static bool wasUntrusted = false;
  if (!locatorActive) {
    if (tallyLink.signalLost()) {
      wasUntrusted = true;
      static uint32_t lastPulse = 0;
      static bool pulseOn = false;
      if (millis() - lastPulse > 1000) {
        lastPulse = millis();
        pulseOn = !pulseOn;
        setColor(pulseOn ? COLOR_LOST : COLOR_OFF, 60);
      }
      static uint32_t lastLostBeep = 0;
      if (millis() - lastLostBeep > 5000) {
        lastLostBeep = millis();
        buzzPulse(600, 100);
      }
    } else if (tallyLink.sourceStale()) {
      wasUntrusted = true;
      static uint32_t lastBlink = 0;
      static bool whiteOn = false;
      if (millis() - lastBlink > 400) {
        lastBlink = millis();
        whiteOn = !whiteOn;
        if (whiteOn)
          setColor(COLOR_STALE, 120);
        else
          applyTallyColor();
      }
    } else if (wasUntrusted) {
      // Just regained trust — repaint the true colour once.
      wasUntrusted = false;
      applyTallyColor();
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

  // === TELEMETRY (slave -> hub): periodic so the hub knows this camera is
  // reachable. Brief blocking TX of one frame, then re-arm RX. Jittered by
  // camId so slaves don't all transmit in lockstep on the shared channel.
  // (Not collision-free — a real fix would add CAD/LBT; fine for a small fleet
  // at this rate. battery mV is 0/unknown until a VBAT sense divider is wired.)
  static uint32_t lastTlm = 0;
  uint32_t tlmInterval = TALLY_TELEMETRY_MS + (uint32_t)g_camId * 37;
  if (radio.isConnected() && !locatorActive &&
      millis() - lastTlm > tlmInterval) {
    lastTlm = millis();
    TallyPacket t = TallyProtocol::createTelemetryPacket(
        g_camId, 0, radio.getRSSI(), TALLY_TLM_NO_BATTERY);
    uint8_t buf[TALLY_PACKET_SIZE];
    TallyProtocol::serialize(t, buf);
    radio.send(buf, TALLY_PACKET_SIZE); // blocking, ~one packet airtime
    radio.restartReceive();             // back to listening
  }

  // Heartbeat: status log every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] Up:%lus State:%d LastRX:%lus ago RX:%lu Fail:%lu\n",
                  (unsigned long)(millis() / 1000), (int)tallyLink.state(),
                  (unsigned long)(tallyLink.msSinceLastRx() / 1000),
                  (unsigned long)rxCount, (unsigned long)rxFails);
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
      tone(PIN_BUZZER, 1000, 200); // non-blocking; diagnostic, bypasses policy
    } else if (cmd.startsWith("id ")) {
      int n = cmd.substring(3).toInt();
      if (n >= 1 && n <= 16) {
        saveCamId((uint8_t)n);
        Serial.printf("[CFG] camId=%d saved — rebooting\n", n);
        delay(200);
        ESP.restart();
      } else {
        Serial.println("[CFG] id must be 1..16");
      }
    } else if (cmd == "help") {
      Serial.println("Commands: test, red, green, off, beep, id <1-16>, help");
    }
  }

  // FreeRTOS idle -> CPU clock-gating between events. Kept outside the
  // POWER_SAVE ifdef: RX is DIO1-driven either way, so a non-power-save
  // build gains nothing from spinning at 100% CPU.
  delay(5);
}
