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

// Non-blocking LED state
uint8_t currentTallyState = STATE_OFF;
uint32_t locatorStartTime = 0;
bool isLocatorActive = false;

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
                        // ⚡ Bolt: Use non-blocking state machine for locator
                        isLocatorActive = true;
                        locatorStartTime = millis();
                    } 
                    else {
                        currentTallyState = pkt.state;
                    }
                }
            } else {
                Serial.println("Invalid Packet");
            }
        }
        
        // Return to RX mode strictly? 
        radio.startReceive();
    }
    
    // Heartbeat debug every 5s
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.printf("Still alive. RSSI: %d\n", radio.getRSSI());
    }

    // ⚡ Bolt: Non-blocking LED state machine
    uint32_t now = millis();

    if (isLocatorActive) {
        // Locator runs for 500ms
        if (now - locatorStartTime < 500) {
            // Fast blink (50ms ON / 50ms OFF)
            if (((now - locatorStartTime) / 50) % 2 == 0) {
                digitalWrite(PIN_LED, LED_ON);
            } else {
                digitalWrite(PIN_LED, LED_OFF);
            }
        } else {
            // Locator finished
            isLocatorActive = false;
        }
    }

    if (!isLocatorActive) {
        // Normal Tally State processing
        if (currentTallyState == STATE_PROGRAM) {
            // Solid ON
            digitalWrite(PIN_LED, LED_ON);
        } else if (currentTallyState == STATE_PREVIEW) {
            // Slow blink (500ms ON / 500ms OFF)
            if ((now / 500) % 2 == 0) {
                digitalWrite(PIN_LED, LED_ON);
            } else {
                digitalWrite(PIN_LED, LED_OFF);
            }
        } else {
            digitalWrite(PIN_LED, LED_OFF);
        }
    }
}
