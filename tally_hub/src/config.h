#pragma once

// ------ Wi-Fi ------
#define WIFI_SSID   "SF_tech_network"
#define WIFI_PASS   "f!lmn0td3ad"

// Если знаешь IP ATEM — укажи строкой, иначе оставь пусто "" для авто-поиска
#define ATEM_IP_STR "192.168.1.146"

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
