// ============================================================
//  SUFIDE Tally Slave v2 — ESP32-C3 SuperMini + E28-2G4M12SX
//  Hardware: WS2812B (GPIO 7), Buzzer (GPIO 8)
//  Pinout verified per HARDWARE_GUIDE.md (2026-02-10), see pins.h
//
//  RX is interrupt-driven (DIO1) and, with -DPOWER_SAVE, duty-cycled:
//  the radio sleeps ~70% of the time and still catches every packet
//  thanks to the hub's 40-symbol preamble (see TallyConfig.h).
//  Protocol dispatch + link supervision live in TallyLink (shared with
//  slave v1); this file only renders states on the LED/buzzer.
// ============================================================

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Preferences.h>
#include <E28_SX1280.h>
#include <TallyLink.h>
#include <TallyLog.h>
#include <TallyProtocol.h>
#include <TallyRadio.h>

#include "pins.h"

// Serial + flash log in one call: field events must survive without a
// tethered laptop. Bench-only chatter (help text, command echo) stays on
// plain Serial so it doesn't churn the ring.
static void slogf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));
static void slogf(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  TallyLog.write(buf);
}

// ===== CONFIGURATION =====
// Compile-time fallback only; the live camera ID is stored in NVS and set in
// the field (BOOT button or serial "id N"), so ONE binary serves every camera.
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID 1
#endif

static uint8_t g_camId = SLAVE_CAM_ID;

// Runtime TX power (serial "power N", chip dBm; the 12SX has no external PA).
// Re-applied after every radio recovery so a live override sticks.
static int8_t g_txPower = TALLY_TX_POWER;

// AFA: current channel. Normally follows the hub's CMD_SET_CHANNEL; if we
// lose the link (missed announcement, reboot while the fleet runs on an
// escape channel), the scan in loop() walks this list until the hub is found.
static const uint32_t kChanList[] = TALLY_CHAN_LIST;
static uint8_t g_chanIdx = 0;
// The hub-announced frequency is authoritative (the packet carries it so a
// hub/slave channel-table mismatch still works); the index only aligns the
// scan start. Recovery must restore THIS, not a table entry.
static uint32_t g_chanFreq = TALLY_RF_FREQ_HZ;

// ===== COLORS =====
#define COLOR_OFF 0x000000
#define COLOR_PROGRAM 0xFF0000   // Red
#define COLOR_PREVIEW 0x00FF00   // Green
#define COLOR_BOTH 0xFFFF00      // Yellow
#define COLOR_PING 0x0000FF      // Blue (locator flash)
#define COLOR_LOST 0xFF4500      // Orange (radio signal lost)
#define COLOR_STALE 0xFFFFFF     // White (tally source frozen — don't trust)
#define COLOR_INIT_OK 0x00FF00   // Green

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
// capture the tone. Visual indications always fire.
static bool buzzerAllowed() {
  TallyState s = tallyLink.state();
  return s != STATE_PROGRAM && s != STATE_BOTH;
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
  // Going on air: cut any already-sounding tone NOW. buzzerAllowed() gates
  // only the START of a tone — a link-lost beep or locator tone already
  // running would otherwise leak up to ~300ms into a live mic.
  if (ts == STATE_PROGRAM || ts == STATE_BOTH)
    noTone(PIN_BUZZER);
  // Paint before logging: a LittleFS rotation stall inside slogf must not
  // sit between "camera went on air" and the LED turning on.
  // Only paint a solid colour when the light is trustworthy; otherwise the
  // signal-lost / source-stale indications below own the LED.
  if (!locatorActive && tallyLink.trustworthy())
    applyTallyColor();
  slogf("[TALLY] State:%d RSSI:%d\n", ts, radio.getRSSI());
}

void onLocatorPing() {
  slogf("[LOCATOR] Alert!\n");
  startLocator();
}

// Hub announced a coordinated channel switch (AFA). The packet's frequency is
// authoritative; the index only aligns our scan starting point.
void onChannelChange(uint32_t freqHz, uint8_t chanIdx) {
  if (freqHz < 2400300000UL || freqHz > 2483300000UL)
    return; // sanity: stay inside the 2.4 GHz ISM band whatever the packet says
  if (chanIdx < TALLY_CHAN_COUNT)
    g_chanIdx = chanIdx;
  g_chanFreq = freqHz; // remember for radio recovery — it is authoritative
  slogf("[CHAN] hub -> %lu.%lu MHz (ch%u)\n",
        (unsigned long)(freqHz / 1000000UL),
        (unsigned long)((freqHz % 1000000UL) / 100000UL), chanIdx);
  radio.setFrequency(freqHz);
  radio.restartReceive();
}

// Hub's AutoRF assigned our telemetry power (CMD_SET_POWER). Clamp to the
// chip range; the value lives in RAM, so a reboot falls back to the config
// default until the hub re-announces (hub re-sends on every ONLINE event).
void onPowerChange(int8_t dbm) {
  if (dbm < -18) dbm = -18;
  if (dbm > 12) dbm = 12;
  g_txPower = dbm;
  radio.setTxPower(dbm);
  slogf("[PWR] hub set telemetry power: %d dBm\n", (int)dbm);
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

// Bounded wait for BOOT release: a strap/wiring fault holding GPIO9 low must
// not wedge boot forever with zero indication. false = timed out — treat the
// line as faulted and bail.
static bool waitBootRelease(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_BOOT) == LOW) {
    if (millis() - t0 > timeoutMs)
      return false;
    delay(10);
  }
  return true;
}

// Set-ID mode: each tap increments the count (1..8, wraps), the LED flashes
// the count and the buzzer chirps; 3s of inactivity commits to NVS. Blocking,
// but this only runs at boot on request. `requested` comes from the boot-delay
// watcher in setup() — see the GPIO9 strap-pin note there.
static uint8_t runSetIdModeIfRequested(uint8_t current, bool requested) {
  // Trust ONLY the debounced boot watcher. A raw digitalRead fallback here
  // used to enter set-ID on any instantaneous LOW (a fresh tap landing after
  // the watch window, a bounce, strap-pin noise) that the 200ms-hold check
  // had just rejected — hanging boot for 3s+ and risking a wrong camera ID
  // being committed to NVS from stray taps.
  if (!requested)
    return current; // not held — normal boot

  Serial.println("[CFG] Set-ID mode: tap BOOT to count (1..8), idle 3s to save");
  if (!waitBootRelease(10000)) { // wait for the initial hold to release
    Serial.println("[CFG] BOOT stuck low — aborting set-ID, normal boot");
    return current;
  }

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
        if (!waitBootRelease(10000)) { // wait for release
          Serial.println("[CFG] BOOT stuck low — aborting set-ID, normal boot");
          return current;
        }
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
    slogf("[WARN] Signal lost!\n");
    buzzPulse(800, 300);
  } else {
    slogf("[LINK] Signal restored\n");
    if (!locatorActive)
      applyTallyColor();
  }
}

// Post-recovery hook for the shared tallyRadioRecover() (TallyRadio.h):
// restore v2's runtime radio state, then re-arm RX.
static void onRadioRecovered() {
  radio.setTxPower(g_txPower); // keep any live "power N" override
  radio.setFrequency(g_chanFreq); // the hub-announced channel, not our table
  armReceive();
}

// Re-init the radio if a runtime fault (stuck BUSY) latched it disconnected.
// Without this a single glitch would leave the receiver permanently deaf,
// since every RX call is a guarded no-op while _connected is false.
void tryRadioRecover() {
  tallyRadioRecover(radio, tallyLink, PIN_LORA_SCK, PIN_LORA_MISO,
                    PIN_LORA_MOSI, PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                    PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN,
                    onRadioRecovered);
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
  // Field units run on battery with NO USB host: HWCDC's default 100ms TX
  // timeout would otherwise stall every log line once the FIFO fills — and
  // onTallyState logs BEFORE painting the LED, so tally latency would spike
  // exactly in the deployed configuration. 1ms, NOT 0: HWCDC::write()'s
  // `tries--` has no zero-guard, so 0 underflows on the first no-progress
  // pass (host attached, nobody reading) into a ~49-day loop() freeze.
  Serial.setTxTimeoutMs(1);

  // CDC settle delay doubles as the set-ID entry window. GPIO9 is the C3's
  // boot strap: held LOW while power is APPLIED, the ROM enters download mode
  // and this firmware never runs. So the rule is: power on first, then press
  // and hold BOOT (>=200ms) within these ~2 seconds.
  pinMode(PIN_BOOT, INPUT_PULLUP);
  // millis()-based (like every other timer in this file), not iteration
  // counting: the 2s window and 200ms hold threshold stay true even if this
  // loop body ever grows another statement.
  bool bootHeld = false;
  bool lowRun = false;
  uint32_t lowStart = 0;
  uint32_t watchStart = millis();
  while (millis() - watchStart < 2000) { // CDC settle + set-ID entry window
    if (digitalRead(PIN_BOOT) == LOW) {
      if (!lowRun) {
        lowRun = true;
        lowStart = millis();
      } else if (millis() - lowStart >= 200) { // continuous hold, not a glitch
        bootHeld = true;
      }
    } else {
      lowRun = false;
    }
    delay(10);
  }

  led.begin();
  setColor(COLOR_OFF);
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  // Resolve camera ID from NVS (or the set-ID flow if BOOT was held)
  g_camId = loadCamId();
  g_camId = runSetIdModeIfRequested(g_camId, bootHeld);

  // Flash log before the banner so every boot is captured (2x128KB ring)
  TallyLog.begin(131072);

  Serial.println("\n=============================");
  Serial.println("  SUFIDE Tally Slave v2");
  Serial.printf("  Camera ID: %d  NetID: 0x%02X\n", g_camId, TALLY_NET_ID);
#ifdef POWER_SAVE
  Serial.println("  RX: duty-cycle (POWER_SAVE)");
#else
  Serial.println("  RX: continuous");
#endif
  Serial.println("=============================");
  slogf("[BOOT] camId=%d netId=0x%02X rx=%s log=%s\n", g_camId, TALLY_NET_ID,
#ifdef POWER_SAVE
        "duty-cycle",
#else
        "continuous",
#endif
        TallyLog.ok() ? "OK" : "UNAVAILABLE");

  Serial.print("[LoRa] Init... ");
  bool ok = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                        PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                        PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (ok) {
    slogf("[LoRa] init OK\n");
    flashColor(COLOR_INIT_OK, 3, 150, 100);
    beep(1000, 100);
    tallyApplyRadioProfile(radio);
  } else {
    // Non-terminal: instead of trapping the operator on a forever-blink that
    // needs a manual power-cycle, fall through to loop() — tryRadioRecover()
    // re-inits every 10s (the slave has a real reset line) and the signal-lost
    // indication shows "not working" until it heals. A warm-boot module that
    // came up slow/wedged then self-recovers.
    slogf("[LoRa] init FAILED: %s — retrying in loop()\n",
          radio.initErrorStr());
    beep(400, 200);
  }

  tallyLink.begin(g_camId, onTallyState, onLocatorPing, onLinkChange);
  tallyLink.setChannelCallback(onChannelChange);
  tallyLink.setPowerCallback(onPowerChange); // follow AutoRF power

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
  TallyLog.tick(); // periodic flash-log flush

  // === RX: interrupt-driven (no SPI polling — under POWER_SAVE any NSS
  // activity during the radio's sleep phase would silently kill the cycle)
  if (g_dio1Flag || digitalRead(PIN_LORA_DIO1) == HIGH) {
    // Clear the flag FIRST: an ISR firing during the drain/re-arm below sets
    // it again, so we re-enter next loop (a harmless duplicate pass gated by
    // available()) instead of losing that packet. Clearing it AFTER the
    // re-arm erased exactly the event it was meant to preserve.
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
            char hex[3 * 16 + 1] = {0};
            for (int i = 0; i < len && i < 16; i++)
              snprintf(hex + i * 3, 4, "%02X ", buf[i]);
            slogf("[RX_FAIL] len=%d raw: %s\n", len, hex);
          }
        }
      }
    }
    // Unconditional re-arm: RxDone (even a CRC error) ends the duty cycle
    radio.rearmAfterIrq();
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

  // === AFA CHANNEL SCAN: link lost -> maybe the hub escaped a jammed
  // channel (or we rebooted while the fleet runs elsewhere). Walk the list,
  // ~2s per channel (>=3 hub heartbeats each). Any valid packet ends the
  // scan by clearing signalLost; the current channel stays where we heard it.
  static uint32_t lastScanHop = 0;
  static bool scanning = false;
  if (!tallyLink.signalLost()) {
    scanning = false;
  } else if (radio.isConnected()) {
    if (!scanning) {
      // Full first dwell on the current channel: a spurious deaf spell must
      // not hop away instantly — the hub may still be right here.
      scanning = true;
      lastScanHop = millis();
    } else if (millis() - lastScanHop > 2000) {
      lastScanHop = millis();
      g_chanIdx = (uint8_t)((g_chanIdx + 1) % TALLY_CHAN_COUNT);
      g_chanFreq = kChanList[g_chanIdx]; // where we're listening now
      radio.setFrequency(g_chanFreq);
      radio.restartReceive();
      slogf("[SCAN] listening on ch%u (%lu.%lu MHz)\n", g_chanIdx,
            (unsigned long)(g_chanFreq / 1000000UL),
            (unsigned long)((g_chanFreq % 1000000UL) / 100000UL));
    }
  }

  // === TELEMETRY (slave -> hub): periodic so the hub knows this camera is
  // reachable. Shared beat (TallyRadio.h, one copy for v1/v2) — deferred
  // while the locator owns the LED. (Not collision-free — a real fix would
  // add CAD/LBT; fine for a small fleet at this rate.)
  tallyTelemetryTick(radio, g_camId, !locatorActive, tallyLink.linkPoor());

  // Heartbeat: status log every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    lastHeartbeat = millis();
    slogf("[STATUS] Up:%lus State:%d Ch:%u LastRX:%lus ago RX:%lu "
          "Fail:%lu RxErr:%lu\n",
          (unsigned long)(millis() / 1000), (int)tallyLink.state(), g_chanIdx,
          (unsigned long)(tallyLink.msSinceLastRx() / 1000),
          (unsigned long)rxCount, (unsigned long)rxFails,
          (unsigned long)radio.getRxErrors());
    rxCount = 0;
    rxFails = 0;
  }

  // Serial commands (for testing) — non-blocking line accumulator: the
  // Stream timeout is per-character, so readStringUntil() either stalls the
  // loop (1s default) or truncates hand-typed commands (any short timeout).
  static String cmdLine;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch != '\n' && ch != '\r') {
      if (cmdLine.length() < 100) // bound against a noise flood
        cmdLine += ch;
      continue;
    }
    String cmd = cmdLine;
    cmdLine = "";
    cmd.trim();
    if (!cmd.length())
      continue;

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
    } else if (cmd.startsWith("power")) {
      // Range testing: telemetry TX power live, no reflash (chip dBm, no PA)
      String arg = cmd.substring(5);
      arg.trim();
      if (!arg.length()) {
        Serial.printf("[PWR] chip=%d dBm. Set: power <-18..12>\n",
                      (int)g_txPower);
      } else {
        int p = arg.toInt();
        if ((arg[0] != '-' && (arg[0] < '0' || arg[0] > '9')) || p < -18 ||
            p > 12) {
          Serial.println("[PWR] usage: power <-18..12>");
        } else {
          g_txPower = (int8_t)p;
          radio.setTxPower(g_txPower);
          Serial.printf("[PWR] chip=%d dBm\n", p);
        }
      }
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
    } else if (cmd == "log") {
      TallyLog.dump(Serial);
    } else if (cmd == "logclear") {
      TallyLog.clear();
      Serial.println("[LOG] cleared");
    } else if (cmd == "help") {
      Serial.println("Commands: test, red, green, off, beep, power [n], "
                     "log, logclear, id <1-16>, help");
    }
  }

  // FreeRTOS idle -> CPU clock-gating between events. Kept outside the
  // POWER_SAVE ifdef: RX is DIO1-driven either way, so a non-power-save
  // build gains nothing from spinning at 100% CPU.
  delay(5);
}
