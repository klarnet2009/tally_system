#pragma once

// ------ Wi-Fi / ATEM (креды) ------
// Реальные значения живут в secrets.h (gitignored).
// Скопируй secrets.h.example -> secrets.h и заполни.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#warning "tally_hub/src/secrets.h not found - using placeholder credentials"
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_WIFI_PASS"
#endif
#ifndef ATEM_IP_STR
// IP ATEM строкой, либо пусто ""
#define ATEM_IP_STR ""
#endif

// E28-2G4M27S (SX1280) LoRa - High Power (27dBm)
// NOTE: this is an ESP32-S3 — GPIO 1/3 are ordinary GPIOs here (UART0 is
// GPIO 43/44, native USB is GPIO 19/20), so serial logging works fine
// alongside the E28. The old "NO USB LOGS" note was classic-ESP32 lore.
#define E28_PIN_SCK     14  // SD CLK
#define E28_PIN_MISO    13  // SD D3
#define E28_PIN_MOSI    11  // SD CMD
#define E28_PIN_NSS     10  // SD D2
#define E28_PIN_BUSY    4   // Input
#define E28_PIN_DIO1    2   // SD D0
#define E28_PIN_RESET   -1  // No reset line: driver soft-wakes the chip
#define E28_PIN_RXEN    1   // LNA Control
#define E28_PIN_TXEN    3   // PA Control

// SSD1306 128x64 I2C
#define OLED_I2C_SDA 17
#define OLED_I2C_SCL 18
#define OLED_ADDR    0x3C

// Какие входы показываем (ровно 8 штук)
static uint8_t TALLY_INPUTS[8] = {1,2,3,4,5,6,7,8};

// Периоды
#define WIFI_RETRY_MS  8000
#define ATEM_RETRY_MS  5000          // Wait between connection attempts
#define ATEM_CONNECT_TIMEOUT_MS 5000 // Give up a single attempt after this
#define POLL_MS         100
