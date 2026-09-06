// ============================================================
//  SUFIDE Tally Slave v2 — ESP32-C3 SuperMini + E28-2G4M12SX
//  Hardware: WS2812B (GPIO 7), Buzzer (GPIO 8)
//  Pinout verified per HARDWARE_GUIDE.md (2026-02-10), see pins.h
//
//  RX is interrupt-driven (DIO1) and CONTINUOUS. Duty-cycled RX was removed
//  with the v4 architecture: it has an irreducible deaf window, and it was the
//  only reason the fleet needed a 40-symbol (55.8 ms) preamble. Reliability
//  outranks battery life here — see ARCHITECTURE_RF_V4.md §4.
//  Protocol dispatch + link supervision live in TallyLink; this file only
//  renders states on the LED/buzzer.
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
// 0 = not provisioned. A build may still bake in a first-boot default with
// -DSLAVE_CAM_ID=n, but the universal binary must NOT: a fresh unit that fell
// back to "camera 1" lit up with camera 1's tally on whichever camera it was
// actually clipped to, and sat in camera 1's telemetry slot as a second device.
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID 0
#endif

static uint8_t g_camId = SLAVE_CAM_ID;

// Runtime TX power (serial "power N", chip dBm; the 12SX has no external PA).
// Re-applied after every radio recovery so a live override sticks. The hub no
// longer assigns this — uplink ADR was deleted (ARCHITECTURE_RF_V4.md §9).
static int8_t g_txPower = TALLY_SLAVE_TX_POWER;

// AFA: current channel. Normally follows the switch plan carried inside the
// hub's heartbeat; if we lose the link anyway (missed every announcement,
// rebooted while the fleet runs on an escape channel), the scan in loop()
// walks this list until the hub is found.
static const uint32_t kChanList[] = TALLY_CHAN_LIST;
static uint8_t g_chanIdx = 0;
static uint32_t g_chanFreq = TALLY_RF_FREQ_HZ;
// Pending coordinated switch: the heartbeat's countdown resolved to an instant.
// Every announcing frame refines it, so losing any subset still leaves a
// usable estimate — that is the whole point of announcing in the heartbeat.
static bool g_switchPending = false;
static uint8_t g_switchToIdx = 0;
static uint32_t g_switchAtMs = 0;

// ===== COLORS =====
#define COLOR_OFF 0x000000
#define COLOR_PROGRAM 0xFF0000   // Red
#define COLOR_PREVIEW 0x00FF00   // Green
#define COLOR_BOTH 0xFFFF00      // Yellow
#define COLOR_PING 0x0000FF      // Blue (locator flash)
#define COLOR_LOST 0xFF4500      // Orange (radio signal lost)
#define COLOR_STALE 0xFFFFFF     // White (tally source frozen — don't trust)
#define COLOR_UNSET 0xFF00FF     // Magenta (camera ID not provisioned)
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

// Arm the receiver. Continuous RX only (see the file header).
void armReceive() { radio.startReceive(); }

// Buzzer policy: never sound while the camera MAY be on air — a live mic would
// capture the tone. "May be": while the light is not trustworthy (link lost,
// source frozen, hub never heard) the last known state says nothing about the
// current one, so the tone stays off and those indications are visual only.
// The old gate looked at the last known state alone, which made the link-lost
// beep sound every 5 s on a camera the director could have cut to meanwhile.
// Visual indications always fire.
static bool buzzerAllowed() {
  if (!tallyLink.trustworthy())
    return false;
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

// A heartbeat carried a channel-switch countdown. Record the target and the
// instant; the retune itself happens in loop() so it can never run from inside
// packet handling. Called on every announcing frame — later ones (smaller
// countdown) refine the estimate, and re-announcing the same target is a no-op.
void onChannelPlan(uint8_t chanIdx, uint32_t atMs) {
  if (chanIdx >= TALLY_CHAN_COUNT)
    return;
  if (chanIdx == g_chanIdx && !g_switchPending)
    return; // already where the hub wants us (e.g. an old-channel beacon)
  bool first = !g_switchPending || g_switchToIdx != chanIdx;
  g_switchPending = true;
  g_switchToIdx = chanIdx;
  g_switchAtMs = atMs;
  if (first)
    slogf("[CHAN] plan: -> ch%u in %lums\n", chanIdx,
          (unsigned long)(atMs - millis()));
}

// Perform the planned retune once its instant arrives.
static void channelPlanTick() {
  if (!g_switchPending)
    return;
  if ((int32_t)(millis() - g_switchAtMs) < 0)
    return;
  g_switchPending = false;
  g_chanIdx = g_switchToIdx;
  g_chanFreq = kChanList[g_chanIdx];
  radio.setFrequency(g_chanFreq);
  radio.restartReceive();
  slogf("[CHAN] now on ch%u (%lu.%lu MHz)\n", g_chanIdx,
        (unsigned long)(g_chanFreq / 1000000UL),
        (unsigned long)((g_chanFreq % 1000000UL) / 100000UL));
}

// ===== Camera ID provisioning (NVS) =====
// One universal binary: the camera ID lives in NVS, not the firmware image.
// Set it in the field by holding BOOT at power-up (tap to count) or via the
// serial "id N" command. SLAVE_CAM_ID is only the first-boot default.
// Returns 0 when the unit has never been provisioned (no NVS key and no build-
// time default). 0 is a real state the firmware shows (magenta blink, no tally
// colour, no telemetry), not an error to paper over with a guess.
static uint8_t loadCamId() {
  Preferences p;
  p.begin("tally", true);
  uint8_t id = p.getUChar("camid", SLAVE_CAM_ID);
  p.end();
  if (id > 16)
    id = 0; // corrupt value: unprovisioned, never "camera 1"
  return id;
}
static inline bool provisioned() { return g_camId >= 1 && g_camId <= 16; }

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
    // Visual only (orange pulse in loop()): with the link gone the camera's
    // real state is unknown, so buzzerAllowed() is false by construction.
    slogf("[WARN] Signal lost!\n");
  } else {
    slogf("[LINK] Signal restored\n");
    if (!locatorActive)
      applyTallyColor();
  }
}

// Post-recovery hook for the shared tallyRadioRecover() (TallyRadio.h):
// restore v2's runtime radio state, then re-arm RX.
static void onRadioRecovered() {
  radio.setTxPower(g_txPower);    // keep any live "power N" override
  radio.setFrequency(g_chanFreq); // the channel we are actually on
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
  if (provisioned())
    Serial.printf("  Camera ID: %d  NetID: 0x%X  proto v%d\n", g_camId,
                  TALLY_NET_ID, TALLY_PROTOCOL_VERSION);
  else
    Serial.printf("  Camera ID: NOT SET  NetID: 0x%X  proto v%d\n",
                  TALLY_NET_ID, TALLY_PROTOCOL_VERSION);
  Serial.println("  RX: continuous");
  Serial.println("=============================");
  slogf("[BOOT] camId=%d netId=0x%X proto=v%d rx=continuous log=%s\n", g_camId,
        TALLY_NET_ID, TALLY_PROTOCOL_VERSION,
        TallyLog.ok() ? "OK" : "UNAVAILABLE");
  if (!provisioned())
    slogf("[CFG] *** camera ID not set: magenta blink, no tally, no telemetry. "
          "Use 'id N' or hold BOOT at power-up\n");

  Serial.print("[LoRa] Init... ");
  bool ok = radio.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI,
                        PIN_LORA_NSS, PIN_LORA_BUSY, PIN_LORA_DIO1,
                        PIN_LORA_NRESET, PIN_LORA_RXEN, PIN_LORA_TXEN);

  if (ok) {
    slogf("[LoRa] init OK\n");
    flashColor(COLOR_INIT_OK, 3, 150, 100);
    beep(1000, 100);
    tallyApplyRadioProfile(radio, g_txPower);
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
  tallyLink.setChannelCallback(onChannelPlan);

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

  // === RX: interrupt-driven. Continuous RX, so a stray SPI read between
  // events is harmless (that constraint died with duty-cycle RX).
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
  // unprovisioned = no camera ID: magenta double-blink, never a tally colour.
  // signalLost    = radio link dead: orange pulse. Visual only — the camera's
  //                 real state is unknown, so no tone (buzzerAllowed()).
  // sourceStale   = link alive but the hub's ATEM source is frozen: hold the
  //                 last tally colour but blink WHITE over it, so the operator
  //                 sees the colour may be stale without losing the held state.
  static bool wasUntrusted = false;
  if (!locatorActive) {
    if (!provisioned()) {
      // This unit belongs to NO camera yet, so it must not show any tally
      // colour, trusted or not — magenta is a colour no tally state uses.
      wasUntrusted = true;
      static uint32_t lastStep = 0;
      static uint8_t step = 0;
      if (millis() - lastStep > 150) {
        lastStep = millis();
        step = (uint8_t)((step + 1) % 8); // blink, blink, pause
        setColor((step == 0 || step == 2) ? COLOR_UNSET : COLOR_OFF, 100);
      }
    } else if (tallyLink.signalLost()) {
      wasUntrusted = true;
      static uint32_t lastPulse = 0;
      static bool pulseOn = false;
      if (millis() - lastPulse > 1000) {
        lastPulse = millis();
        pulseOn = !pulseOn;
        setColor(pulseOn ? COLOR_LOST : COLOR_OFF, 60);
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

  // === Coordinated channel switch announced in the heartbeat ===
  channelPlanTick();

  // === AFA CHANNEL SCAN (last resort) ===
  // Reached only if we missed EVERY in-heartbeat announcement AND the hub's
  // old-channel beacon: walk the list, TALLY_SCAN_DWELL_MS per channel (>=2
  // hub heartbeats each). Any valid frame ends the scan by clearing
  // signalLost, and we stay on whichever channel we heard it.
  static uint32_t lastScanHop = 0;
  static bool scanning = false;
  if (!tallyLink.signalLost()) {
    scanning = false;
  } else if (radio.isConnected() && !g_switchPending) {
    if (!scanning) {
      scanning = true;
      // A slave that has NEVER heard the hub (fresh boot, battery swap while
      // the fleet runs on an escape channel) starts hopping immediately: there
      // is no "spurious deaf spell" to sit through, and every extra second is
      // a second of dark camera. Once we've had a link, take the full dwell
      // first — the hub may still be right here.
      lastScanHop = tallyLink.everHeard()
                        ? millis()
                        : millis() - TALLY_SCAN_DWELL_MS;
    } else if (millis() - lastScanHop > TALLY_SCAN_DWELL_MS) {
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

  // === TELEMETRY (slave -> hub) ===
  // Slotted off the heartbeat's cycle counter, so it is collision-free by
  // construction rather than thinned by jitter (TallyRadio.h). Deferred while
  // the locator owns the LED so a blocking send can't stutter the pattern.
  tallyTelemetryTick(radio, tallyLink, g_camId, !locatorActive);

  // Heartbeat: status log every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    lastHeartbeat = millis();
    slogf("[STATUS] Up:%lus State:%d Ch:%u LastRX:%lus ago RX:%lu "
          "Fail:%lu RxErr:%lu Pwr:%d%%\n",
          (unsigned long)(millis() / 1000), (int)tallyLink.state(), g_chanIdx,
          (unsigned long)(tallyLink.msSinceLastRx() / 1000),
          (unsigned long)rxCount, (unsigned long)rxFails,
          (unsigned long)radio.getRxErrors(),
          (int)((g_txPower + 18) * 100 / 30));
    if (tallyLink.missedBeats())
      slogf("[STATUS] missed heartbeats this window: %u\n",
            tallyLink.missedBeats());
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

  // FreeRTOS idle -> CPU clock-gating between events. RX is DIO1-driven, so
  // spinning at 100% CPU would buy nothing.
  delay(5);
}
