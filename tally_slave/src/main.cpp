#include <Arduino.h>
#include <E28_SX1280.h>
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

E28Radio radio;
uint32_t lastHeartbeat = 0;
bool connected = false;

// ⚡ Bolt: State tracking for non-blocking LED updates
TallyState currentTallyState = STATE_OFF;
uint32_t locatorStartTime = 0;
uint32_t lastLedToggle = 0;
bool ledIsOn = false;

// Link supervision: the hub re-broadcasts STATE_ALL every TALLY_REFRESH_MS,
// so radio silence longer than TALLY_SIGNAL_LOST_MS means the link is down
uint32_t lastRxTime = 0;
bool signalLost = false;

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

    // 2. Signal lost: slow blink so the operator knows the state is stale
    if (signalLost) {
        if (now - lastLedToggle >= 500) {
            lastLedToggle = now;
            ledIsOn = !ledIsOn;
            digitalWrite(PIN_LED, ledIsOn ? LED_ON : LED_OFF);
        }
        return;
    }

    // 3. Normal Tally States (any non-OFF state lights the LED, incl. STATE_BOTH)
    if (currentTallyState != STATE_OFF) {
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

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Tally Slave (ESP-C3 + E28) ===");
    Serial.printf("Camera ID: %d\n", SLAVE_CAM_ID);

    // LED Setup
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_OFF);

    // LoRa Setup
    Serial.print("LoRa Init... ");
    // Init with explicit pins for C3
    if (radio.begin(SLAVE_PIN_SCK, SLAVE_PIN_MISO, SLAVE_PIN_MOSI,
                    SLAVE_PIN_NSS, SLAVE_PIN_BUSY, SLAVE_PIN_DIO1,
                    SLAVE_PIN_RESET, SLAVE_PIN_RXEN, SLAVE_PIN_TXEN)) {
        Serial.println("OK");
        connected = true;
        // Blink LED to confirm init
        for(int i=0; i<3; i++) { digitalWrite(PIN_LED, LED_ON); delay(100); digitalWrite(PIN_LED, LED_OFF); delay(100); }
    } else {
        Serial.println("FAILED! Check Wiring.");
        connected = false;
        // Fast blink error
        while(1) { digitalWrite(PIN_LED, LED_ON); delay(50); digitalWrite(PIN_LED, LED_OFF); delay(50); }
    }

    tallyApplyRadioProfile(radio);

    // Set to RX mode
    radio.startReceive();
    lastRxTime = millis();
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
        lastRxTime = millis();
        Serial.println("[LoRa] Recovered");
    }
}

void loop() {
    if (!connected) return;

    tryRadioRecover();

    // ⚡ Bolt: Execute non-blocking LED state machine
    updateLed();

    // The delay(2) at loop end caps this poll at ~500Hz instead of the old
    // full-speed busy-poll. v1 stays on a plain available() check (no DIO1
    // gating) because its DIO1 is unreliable on some boards, and continuous
    // RX (unlike v2's duty cycle) makes the periodic SPI read harmless.
    if (radio.available()) {
        // Restrict RX length to TALLY_PACKET_SIZE so receive() takes the <=8-byte
        // fast path instead of a 260-byte block transfer on malformed packets.
        uint8_t buf[TALLY_PACKET_SIZE];
        uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);

        // deserialize() already rejects foreign netId before any memcpy/CRC,
        // so no separate pre-filter is needed here.
        if (len > 0) {
            TallyPacket pkt;
            if (TallyProtocol::deserialize(buf, len, pkt)) {
                // Any valid packet from our hub proves the link is alive
                lastRxTime = millis();
                if (signalLost) {
                    signalLost = false;
                    Serial.println("[LINK] Signal restored");
                }

                if (pkt.command == CMD_PING) {
                    if (pkt.payload[0] == SLAVE_CAM_ID ||
                        pkt.payload[0] == TALLY_BROADCAST_ID) {
                        // ⚡ Bolt: Start non-blocking locator sequence
                        locatorStartTime = millis();
                        lastLedToggle = millis();
                        ledIsOn = true;
                        digitalWrite(PIN_LED, LED_ON);
                    }
                } else if (pkt.command == CMD_STATE_ALL) {
                    TallyState ts = TallyProtocol::stateForCamera(pkt, SLAVE_CAM_ID);
                    if (ts != currentTallyState) {
                        currentTallyState = ts;
                        Serial.printf("Tally Update: %d (RSSI: %d)\n", ts, radio.getRSSI());
                    }
                }
            }
        }

        // ⚡ Bolt: Fast RX re-arm to avoid standby/SPI overhead and prevent dropped packets
        radio.clearRxIrq();
    }

    // Signal-lost detection (hub broadcasts at TALLY_REFRESH_MS)
    if (!signalLost && millis() - lastRxTime > TALLY_SIGNAL_LOST_MS) {
        signalLost = true;
        Serial.println("[LINK] Signal lost!");
    }

    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }

    delay(2); // End full-speed busy-poll; +2ms worst-case tally latency
}
