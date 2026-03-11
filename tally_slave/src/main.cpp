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

// ⚡ Bolt: State variables for non-blocking LED sequences
TallyState currentTallyState = STATE_OFF;
uint32_t locatorStartTime = 0;
bool isLocatorActive = false;
int lastLedState = -1;

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

    if (radio.available()) {
        uint8_t buf[E28_MAX_PACKET_SIZE];
        uint8_t len = radio.receive(buf, E28_MAX_PACKET_SIZE);
        
        if (len > 0) {
            // Parse packet
            TallyPacket pkt;
            if (TallyProtocol::deserialize(buf, len, pkt)) {
                // Check if it's for us
                if (pkt.cameraId == SLAVE_CAM_ID) {
                    Serial.printf("Tally Update: %d (CMD: %d, RSSI: %d)\n", pkt.state, pkt.command, radio.getRSSI());
                    
                    if (pkt.command == CMD_PING) {
                        // ⚡ Bolt: Trigger non-blocking Locator sequence
                        isLocatorActive = true;
                        locatorStartTime = millis();
                    } 
                    else {
                        // ⚡ Bolt: Store current state, LED updated by state machine
                        currentTallyState = static_cast<TallyState>(pkt.state);
                    }
                }
            } else {
                Serial.println("Invalid Packet");
            }
        }
        
        // Return to RX mode strictly? 
        radio.startReceive();
    }
    
    // ⚡ Bolt: Non-blocking LED state machine
    int targetLedState = LED_OFF;

    if (isLocatorActive) {
        uint32_t elapsed = millis() - locatorStartTime;
        if (elapsed > 500) {
            // Sequence done (5 blinks x 100ms)
            isLocatorActive = false;
        } else {
            // Blink every 50ms on, 50ms off
            if ((elapsed % 100) < 50) {
                targetLedState = LED_ON;
            } else {
                targetLedState = LED_OFF;
            }
        }
    }

    // Restore normal state if not in locator sequence
    if (!isLocatorActive) {
        if (currentTallyState == STATE_PROGRAM) {
            targetLedState = LED_ON;
        } else if (currentTallyState == STATE_PREVIEW) {
            // Slow blink for preview
            if ((millis() % 1000) < 500) {
                targetLedState = LED_ON;
            } else {
                targetLedState = LED_OFF;
            }
        } else {
            targetLedState = LED_OFF;
        }
    }

    // Only perform hardware I/O if the state actually changed
    if (targetLedState != lastLedState) {
        digitalWrite(PIN_LED, targetLedState);
        lastLedState = targetLedState;
    }

    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }
}
