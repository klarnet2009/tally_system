#include <Arduino.h>
#include <E28_SX1280.h>
#include <TallyProtocol.h>

// ===== CONFIGURATION =====
// Slave ID (Hardcoded for now, TODO: Dip Switch or WiFi config)
#define SLAVE_CAM_ID    1

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

    // 2. Normal Tally States (matching original exact logic)
    if (currentTallyState == STATE_PROGRAM || currentTallyState == STATE_PREVIEW) {
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

    // Set to RX mode
    radio.startReceive();
}

void loop() {
    if (!connected) return;

    // ⚡ Bolt: Execute non-blocking LED state machine
    updateLed();

    if (radio.available()) {
        // ⚡ Bolt: Use TALLY_PACKET_SIZE to restrict RX length, guaranteeing the
        // E28Radio::receive fast-path (<= 8 bytes) activates and avoids 260-byte block transfer on malformed packets.
        uint8_t buf[TALLY_PACKET_SIZE];
        uint8_t len = radio.receive(buf, TALLY_PACKET_SIZE);
        
        if (len > 0) {
            // Parse packet
            TallyPacket pkt;
            if (TallyProtocol::deserialize(buf, len, pkt)) {
                // Check if it's for us
                if (pkt.cameraId == SLAVE_CAM_ID) {
                    Serial.printf("Tally Update: %d (CMD: %d, RSSI: %d)\n", pkt.state, pkt.command, radio.getRSSI());
                    
                    if (pkt.command == CMD_PING) {
                        // ⚡ Bolt: Start non-blocking locator sequence
                        locatorStartTime = millis();
                        lastLedToggle = millis();
                        ledIsOn = true;
                        digitalWrite(PIN_LED, LED_ON);
                    }
                    else {
                        // ⚡ Bolt: Update state variable, let updateLed() handle GPIO safely
                        currentTallyState = static_cast<TallyState>(pkt.state);
                    }
                }
            } else {
                Serial.println("Invalid Packet");
            }
        }
        
        // ⚡ Bolt: Fast RX re-arm to avoid standby/SPI overhead and prevent dropped packets
        radio.clearRxIrq();
    }
    
    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }
}
