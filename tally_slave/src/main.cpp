#include <Arduino.h>
#include <E28_SX1280.h>
#include <TallyLink.h>
#include <TallyProtocol.h>
#include <TallyRadio.h>

// ===== CONFIGURATION =====
// Slave ID (override per device via -DSLAVE_CAM_ID=n build flag)
#ifndef SLAVE_CAM_ID
#define SLAVE_CAM_ID    1
#endif

// ESP32-C3 SuperMini Pins
#define SLAVE_PIN_SCK     4
#define SLAVE_PIN_MISO    5
#define SLAVE_PIN_MOSI    6
#define SLAVE_PIN_NSS     7
#define SLAVE_PIN_BUSY    3
#define SLAVE_PIN_DIO1    2
#define SLAVE_PIN_RESET   10

// No HS/PA control on 12dBm module
#define SLAVE_PIN_RXEN    -1
#define SLAVE_PIN_TXEN    -1

// Tally LED
#define PIN_LED         8   // Built-in LED on SuperMini (usually Active LOW)
#define LED_ON          LOW
#define LED_OFF         HIGH

// Protocol dispatch + link supervision live in TallyLink (shared with slave
// v2); this file only renders states on the single mono LED.
E28Radio radio;
TallyLink tallyLink;
uint32_t lastHeartbeat = 0;

// AFA: follow the hub's CMD_SET_CHANNEL, and scan this list when the link is
// lost (same behaviour as v2). The announced frequency is authoritative —
// recovery and scan hops track it here, not a fixed table entry.
static const uint32_t kChanList[] = TALLY_CHAN_LIST;
static uint32_t g_chanFreq = TALLY_RF_FREQ_HZ;

// Telemetry TX power. Boot default; the hub's AutoRF tunes it live via
// CMD_SET_POWER (RAM-only, re-applied after every radio recovery).
static int8_t g_txPower = TALLY_TX_POWER;

// ⚡ Bolt: State tracking for non-blocking LED updates
uint32_t locatorStartTime = 0;
uint32_t lastLedToggle = 0;
bool ledIsOn = false;

// ⚡ Bolt: Update LED non-blocking to prevent dropping LoRa packets
void updateLed() {
    uint32_t now = millis();

    // 1. High-priority Locator (Ping) Sequence (10Hz fast blink for 1 sec)
    if (locatorStartTime > 0) {
        if (now - locatorStartTime > 500) { // 5 blinks * 100ms
            locatorStartTime = 0; // End locator sequence, restore normal state
        } else {
            if (now - lastLedToggle >= 50) { // 50ms on / 50ms off = 100ms period
                lastLedToggle = now;
                ledIsOn = !ledIsOn;
                digitalWrite(PIN_LED, ledIsOn ? LED_ON : LED_OFF);
            }
            return; // Skip normal tally logic while locating
        }
    }

    // 2. Signal lost (radio dead): even slow blink — state is stale.
    if (tallyLink.signalLost()) {
        if (now - lastLedToggle >= 500) {
            lastLedToggle = now;
            ledIsOn = !ledIsOn;
            digitalWrite(PIN_LED, ledIsOn ? LED_ON : LED_OFF);
        }
        return;
    }

    // 2b. Source stale (link alive, ATEM frozen): a distinct double-blink
    // per second, so the operator can tell "switcher frozen" from both a
    // steady tally and the even signal-lost blink.
    if (tallyLink.sourceStale()) {
        uint32_t ph = now % 1000;
        bool on = (ph < 80) || (ph >= 160 && ph < 240);
        digitalWrite(PIN_LED, on ? LED_ON : LED_OFF);
        ledIsOn = on;
        return;
    }

    // 3. Normal Tally States (any non-OFF state lights the LED, incl. STATE_BOTH)
    if (tallyLink.state() != STATE_OFF) {
        if (!ledIsOn) {
            digitalWrite(PIN_LED, LED_ON);
            ledIsOn = true;
        }
    } else {
        if (ledIsOn) {
            digitalWrite(PIN_LED, LED_OFF);
            ledIsOn = false;
        }
    }
}

// ===== TallyLink presentation callbacks =====
void onTallyState(TallyState ts) {
    // updateLed() renders tallyLink.state() every loop pass; just log here
    Serial.printf("Tally Update: %d (RSSI: %d)\n", ts, radio.getRSSI());
}

void onLocatorPing() {
    // ⚡ Bolt: Start non-blocking locator sequence
    locatorStartTime = millis();
    lastLedToggle = millis();
    ledIsOn = true;
    digitalWrite(PIN_LED, LED_ON);
}

void onLinkChange(bool lost) {
    Serial.println(lost ? "[LINK] Signal lost!" : "[LINK] Signal restored");
}

// Hub announced a coordinated channel switch (AFA) — follow it, same as v2.
// Without this a "chan N" on the hub stranded every v1 unit on the old
// channel until the hub came back.
void onChannelChange(uint32_t freqHz, uint8_t chanIdx) {
    (void)chanIdx; // v1's scan walks the list from wherever it is
    if (freqHz < 2400300000UL || freqHz > 2483300000UL)
        return; // sanity: stay inside the 2.4 GHz ISM band
    g_chanFreq = freqHz; // remember for radio recovery — it is authoritative
    Serial.printf("[CHAN] hub -> %lu.%lu MHz\n",
                  (unsigned long)(freqHz / 1000000UL),
                  (unsigned long)((freqHz % 1000000UL) / 100000UL));
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
    Serial.printf("[PWR] hub set telemetry power: %d dBm\n", (int)dbm);
}

void setup() {
    Serial.begin(115200);
    // No USB host in the field: HWCDC's default 100ms TX timeout would stall
    // every log line once the FIFO fills. 1ms (NOT 0: HWCDC's `tries--` has no
    // zero-guard — 0 underflows to a ~49-day hang when a host is attached but
    // not reading) = one quick retry, then logs drop silently as intended.
    Serial.setTxTimeoutMs(1);
    delay(2000);
    Serial.println("\n=== Tally Slave (ESP-C3 + E28) ===");
    Serial.printf("Camera ID: %d\n", SLAVE_CAM_ID);

    // LED Setup
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_OFF);

    // LoRa Setup
    Serial.print("LoRa Init... ");
    // Init with explicit pins for C3
    bool ok = radio.begin(SLAVE_PIN_SCK, SLAVE_PIN_MISO, SLAVE_PIN_MOSI,
                          SLAVE_PIN_NSS, SLAVE_PIN_BUSY, SLAVE_PIN_DIO1,
                          SLAVE_PIN_RESET, SLAVE_PIN_RXEN, SLAVE_PIN_TXEN);
    if (ok) {
        Serial.println("OK");
        // Blink LED to confirm init
        for(int i=0; i<3; i++) { digitalWrite(PIN_LED, LED_ON); delay(100); digitalWrite(PIN_LED, LED_OFF); delay(100); }
        tallyApplyRadioProfile(radio);
    } else {
        // Non-terminal: fall through to loop() so tryRadioRecover() re-inits
        // every 10s instead of trapping in a forever-blink (the old while(1))
        // that needed a manual power-cycle. Signal-lost blink shows "not working".
        Serial.printf("FAILED: %s - retrying in loop()\n", radio.initErrorStr());
    }

    tallyLink.begin(SLAVE_CAM_ID, onTallyState, onLocatorPing, onLinkChange);
    tallyLink.setChannelCallback(onChannelChange); // follow AFA switches
    tallyLink.setPowerCallback(onPowerChange);     // follow AutoRF power

    // Set to RX mode
    if (ok)
        radio.startReceive();
}

// Post-recovery hook for the shared tallyRadioRecover() (TallyRadio.h):
// restore the hub-announced channel and power, then re-arm continuous RX.
static void onRadioRecovered() {
    radio.setTxPower(g_txPower); // keep any hub-assigned power
    radio.setFrequency(g_chanFreq); // not blindly the home channel
    radio.startReceive();
}

// Re-init the radio if a runtime fault (stuck BUSY) latched it disconnected,
// so a transient glitch can't leave the receiver permanently deaf.
void tryRadioRecover() {
    tallyRadioRecover(radio, tallyLink, SLAVE_PIN_SCK, SLAVE_PIN_MISO,
                      SLAVE_PIN_MOSI, SLAVE_PIN_NSS, SLAVE_PIN_BUSY,
                      SLAVE_PIN_DIO1, SLAVE_PIN_RESET, SLAVE_PIN_RXEN,
                      SLAVE_PIN_TXEN, onRadioRecovered);
}

void loop() {
    tryRadioRecover();

    // ⚡ Bolt: Execute non-blocking LED state machine
    updateLed();

    // The delay(2) at loop end caps this poll at ~500Hz instead of the old
    // full-speed busy-poll. v1 stays on a plain available() check (no DIO1
    // gating) because its DIO1 is unreliable on some boards, and continuous
    // RX (unlike v2's duty cycle) makes the periodic SPI read harmless.
    if (radio.available()) {
        // Bounded drain: re-check available() before re-arming so a packet
        // that completed mid-processing isn't wiped by the IRQ clear below
        for (int drain = 0; drain < 4; drain++) {
            // Restrict RX length to TALLY_PACKET_SIZE so receive() takes the
            // <=8-byte fast path instead of a 260-byte block transfer.
            uint8_t buf[TALLY_PACKET_SIZE];
            uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);

            if (len > 0) {
                // Invalid packets (noise/foreign netId/CRC) skipped silently —
                // a neighbouring LoRa system must not spam the serial console
                tallyLink.onPacket(buf, len);
            }

            if (!radio.available())
                break;
        }

        // ⚡ Bolt: Fast RX re-arm to avoid standby/SPI overhead and prevent dropped packets
        radio.rearmAfterIrq();
    }

    // Signal-lost timer (hub broadcasts at TALLY_REFRESH_MS)
    tallyLink.tick();

    // RX safety net (same as v2): timed full re-arm restores RX no matter
    // what state the chip fell into (undetected RX abort, ESD glitch) —
    // tryRadioRecover() only fires when the driver itself latched a fault.
    static uint32_t lastRearm = 0;
    if (tallyLink.msSinceLastRx() > TALLY_RX_REARM_MS &&
        millis() - lastRearm > TALLY_RX_REARM_MS) {
        lastRearm = millis();
        radio.restartReceive();
    }

    // AFA channel scan (same as v2): link lost -> maybe the hub escaped a
    // jammed channel (or we rebooted while the fleet runs elsewhere). Walk
    // the list, ~2s per channel, full first dwell on the current one.
    static uint32_t lastScanHop = 0;
    static bool scanning = false;
    static uint8_t scanIdx = 0;
    if (!tallyLink.signalLost()) {
        scanning = false;
    } else if (radio.isConnected()) {
        if (!scanning) {
            scanning = true;
            lastScanHop = millis();
        } else if (millis() - lastScanHop > 2000) {
            lastScanHop = millis();
            scanIdx = (scanIdx + 1) % TALLY_CHAN_COUNT;
            g_chanFreq = kChanList[scanIdx]; // where we're listening now
            radio.setFrequency(g_chanFreq);
            radio.restartReceive();
            Serial.printf("[SCAN] listening on %lu.%lu MHz\n",
                          (unsigned long)(g_chanFreq / 1000000UL),
                          (unsigned long)((g_chanFreq % 1000000UL) / 100000UL));
        }
    }

    // Telemetry (slave -> hub): without it the hub's reachability table
    // reports v1-based cameras OFFLINE forever. Shared beat (TallyRadio.h) —
    // deferred while the locator owns the LED, same as v2.
    tallyTelemetryTick(radio, SLAVE_CAM_ID, locatorStartTime == 0,
                       tallyLink.linkPoor());

    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }

    delay(2); // End full-speed busy-poll; +2ms worst-case tally latency
}
