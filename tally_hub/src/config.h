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
// Using SD Card pins + Serial pins (NO USB LOGS!)
#define E28_PIN_SCK     14  // SD CLK
#define E28_PIN_MISO    13  // SD D3
#define E28_PIN_MOSI    11  // SD CMD
#define E28_PIN_NSS     10  // SD D2
#define E28_PIN_BUSY    4   // GPIO 4 (Flash LED?) - Input
#define E28_PIN_DIO1    2   // SD D0
#define E28_PIN_RESET   -1  // Software Reset Only
#define E28_PIN_RXEN    1   // UART TX0 (LNA Control)
#define E28_PIN_TXEN    3   // UART RX0 (PA Control)

// SSD1306 128x64 I2C
#define OLED_I2C_SDA 17
#define OLED_I2C_SCL 18
#define OLED_ADDR    0x3C

// Какие входы показываем (ровно 8 штук)
static uint8_t TALLY_INPUTS[8] = {1,2,3,4,5,6,7,8};

// Периоды
#define WIFI_RETRY_MS  8000
#define ATEM_RETRY_MS  5000
#define POLL_MS         100
