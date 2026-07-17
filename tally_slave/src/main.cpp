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

void setup() {
    Serial.begin(115200);
    // No USB host in the field: HWCDC's default 100ms TX timeout would stall
    // every log line once the FIFO fills. Zero = drop logs when nobody reads.
    Serial.setTxTimeoutMs(0);
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

    // Set to RX mode
    if (ok)
        radio.startReceive();
}

// Re-init the radio if a runtime fault (stuck BUSY) latched it disconnected,
// so a transient glitch can't leave the receiver permanently deaf.
void tryRadioRecover() {
    static uint32_t lastTry = 0;
    if (radio.isConnected()) return;
    if (millis() - lastTry < 10000) return;
    lastTry = millis();
    Serial.println("[LoRa] Recovering...");
    if (radio.begin(SLAVE_PIN_SCK, SLAVE_PIN_MISO, SLAVE_PIN_MOSI,
                    SLAVE_PIN_NSS, SLAVE_PIN_BUSY, SLAVE_PIN_DIO1,
                    SLAVE_PIN_RESET, SLAVE_PIN_RXEN, SLAVE_PIN_TXEN)) {
        tallyApplyRadioProfile(radio);
        radio.startReceive();
        tallyLink.noteAlive();
        Serial.println("[LoRa] Recovered");
    } else {
        Serial.printf("[LoRa] Recovery failed: %s\n", radio.initErrorStr());
    }
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

    // Telemetry (slave -> hub), mirrors v2: without it the hub's reachability
    // table reports v1-based cameras OFFLINE forever. Brief blocking TX of one
    // frame, then back to RX. Jittered by camId against lockstep collisions.
    static uint32_t lastTlm = 0;
    uint32_t tlmInterval = TALLY_TELEMETRY_MS + (uint32_t)SLAVE_CAM_ID * 37;
    if (radio.isConnected() && millis() - lastTlm > tlmInterval) {
        lastTlm = millis();
        TallyPacket t = TallyProtocol::createTelemetryPacket(
            SLAVE_CAM_ID, 0, radio.getRSSI(), TALLY_TLM_NO_BATTERY);
        uint8_t tbuf[TALLY_PACKET_SIZE];
        TallyProtocol::serialize(t, tbuf);
        radio.send(tbuf, TALLY_PACKET_SIZE);
        radio.restartReceive(); // back to listening
    }

    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }

    delay(2); // End full-speed busy-poll; +2ms worst-case tally latency
}
