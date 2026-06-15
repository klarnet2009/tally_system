#ifndef SLAVE_V2_PINS_H
#define SLAVE_V2_PINS_H

// Single source of truth for the v2 wiring (see HARDWARE_GUIDE.md) —
// shared by the production firmware (main.cpp) and the rainbow bring-up
// test (radio_test.cpp), so a rewire can't desync the two.

// LoRa E28-2G4M12SX
#define PIN_LORA_SCK 3
#define PIN_LORA_MOSI 4
#define PIN_LORA_MISO 5
#define PIN_LORA_NRESET 6
#define PIN_LORA_NSS 20
#define PIN_LORA_BUSY 10
#define PIN_LORA_DIO1 1
#define PIN_LORA_RXEN -1 // Not present on 12dBm module
#define PIN_LORA_TXEN -1

// WS2812B RGB LED
#define PIN_LED 7
#define NUM_LEDS 1

// Passive Buzzer
#define PIN_BUZZER 8

#endif // SLAVE_V2_PINS_H
