#include <Arduino.h>
#include <NimBLEDevice.h>
#include <SkaarhojPgmspace.h>
#include <ATEMbase.h>
#include <ATEMmin.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "AtemClientAdapter.h"
#include "E28_SX1280.h"
#include "TallyProtocol.h"



#include <esp_now.h>
#include <esp_wifi.h>

// ===== NimBLE Tally кадр =====
#define TALLY_BLE_COMPANY_ID   0xFFFF
#define TALLY_ADV_INTERVAL_MS  100

struct __attribute__((packed)) TallyBLE {
  uint8_t  ver;
  uint16_t seq;
  uint16_t progBits;
  uint16_t prevBits;
  uint8_t  flags;
  uint8_t  crc;
};

static NimBLEAdvertising* g_bleAdv = nullptr;
static TallyBLE g_bleFrame{};
static uint32_t g_lastBleAdv = 0;
static uint16_t g_seq = 0;

static E28Radio radio;
static uint8_t g_lastTallies[8] = {0};
static uint32_t g_loraTxCount = 0;  // TX packet counter

static uint8_t crc8(const uint8_t* p, size_t n){ uint8_t c=0; for(size_t i=0;i<n;i++) c^=p[i]; return c; }

static void bleSetAdvPayload(uint16_t progMask, uint16_t prevMask){
  g_bleFrame.ver = 0x01;
  g_bleFrame.seq = ++g_seq;
  g_bleFrame.progBits = progMask;
  g_bleFrame.prevBits = prevMask;
  g_bleFrame.flags = 0;
  g_bleFrame.crc = 0;
  g_bleFrame.crc = crc8(reinterpret_cast<uint8_t*>(&g_bleFrame), sizeof(g_bleFrame)-1);

  std::string mfg;
  mfg.reserve(2 + sizeof(g_bleFrame));
  mfg.push_back((uint8_t)(TALLY_BLE_COMPANY_ID & 0xFF));
  mfg.push_back((uint8_t)((TALLY_BLE_COMPANY_ID >> 8) & 0xFF));
  mfg.append(reinterpret_cast<const char*>(&g_bleFrame), sizeof(g_bleFrame));

  NimBLEAdvertisementData ad;
  ad.setFlags(0x06);
  ad.setManufacturerData(mfg);
  g_bleAdv->setAdvertisementData(ad);
}

// ===== (опционально) ESP-NOW дубль =====
static uint8_t ESPNOW_BROADCAST[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
struct TallyFrame { uint8_t ver; uint16_t seq; uint16_t progBits; uint16_t prevBits; uint8_t reserved; uint8_t crc; } __attribute__((packed));
static uint8_t simple_crc8(const uint8_t* d, size_t n){ uint8_t c=0; for(size_t i=0;i<n;i++) c^=d[i]; return c; }
static void sendTallyFrame(uint16_t progBits, uint16_t prevBits){ TallyFrame f{}; f.ver=0x01; f.seq=g_bleFrame.seq; f.progBits=progBits; f.prevBits=prevBits; f.reserved=0; f.crc=0; f.crc=simple_crc8((uint8_t*)&f, sizeof(f)-1); esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&f, sizeof(f)); }

// ===== OLED / ATEM =====
TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(128, 64, &I2Cbus, -1);

static IAtemClient* atem = nullptr;
static bool atemConnected = false;
static IPAddress atemAddr; static uint32_t lastPoll = 0; static uint32_t lastAtemAttempt = 0;

String ipToStr(IPAddress ip){ char b[20]; snprintf(b,sizeof(b), "%u.%u.%u.%u", ip[0],ip[1],ip[2],ip[3]); return String(b); }

// ===== Improved OLED UI =====
// Dual-color display: Yellow top 16px, Blue bottom 48px
#define HDR_H 16   // Yellow zone height

// Status bar icons (drawn in yellow zone)
static void drawWifiIcon(int x, int y, bool connected) {
  if (connected) {
    // WiFi arc
    display.drawPixel(x+2, y, WHITE);
    display.drawLine(x+1, y+1, x+3, y+1, WHITE);
    display.drawLine(x, y+2, x+4, y+2, WHITE);
    display.drawPixel(x+2, y+4, WHITE);
    display.drawPixel(x+2, y+5, WHITE);
  } else {
    display.drawLine(x, y, x+4, y+5, WHITE);
    display.drawLine(x+4, y, x, y+5, WHITE);
  }
}

static void drawLoRaIcon(int x, int y, bool connected) {
  if (connected) {
    // Antenna icon
    display.drawLine(x+2, y, x+2, y+5, WHITE);
    display.drawPixel(x, y+1, WHITE);
    display.drawPixel(x+4, y+1, WHITE);
    display.drawPixel(x+1, y, WHITE);
    display.drawPixel(x+3, y, WHITE);
  } else {
    // X mark
    display.drawLine(x, y, x+4, y+5, WHITE);
    display.drawLine(x+4, y, x, y+5, WHITE);
  }
}

void drawStatusBar(const String& ip, bool wifiOk, bool loraOk, bool atemOk) {
  display.fillRect(0, 0, 128, HDR_H, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  
  // Row 1 (y=0): Icons + label
  drawWifiIcon(1, 1, wifiOk);
  drawLoRaIcon(9, 1, loraOk);
  
  // ATEM dot
  if (atemOk) display.fillCircle(19, 3, 2, WHITE);
  else display.drawCircle(19, 3, 2, WHITE);
  
  display.setCursor(25, 0);
  display.print(atemOk ? "ATEM OK" : "NO ATEM");
  
  // Row 2 (y=8): IP + LoRa status
  display.setCursor(1, 8);
  display.print(ip);
  // LoRa debug status (right side)
  display.setCursor(86, 8);
  display.print(loraOk ? "LoRa OK" : "LoRa X");
  
  // Separator line at zone boundary
  display.drawFastHLine(0, 15, 128, WHITE);
}

void drawCenteredMsg(const char* l1, const char* l2=nullptr){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  int y1 = l2 ? 22 : 28;
  int x1 = (128 - strlen(l1) * 6) / 2;
  if (x1 < 0) x1 = 0;
  display.setCursor(x1, y1);
  display.print(l1);
  
  if (l2) {
    int x2 = (128 - strlen(l2) * 6) / 2;
    if (x2 < 0) x2 = 0;
    display.setCursor(x2, y1 + 12);
    display.print(l2);
  }
  display.display();
}

// Animated spinner: 8 dots around a circle, one filled at a time
static void drawSpinner(int cx, int cy, int r, uint8_t frame) {
  const float step = 3.14159265f * 2.0f / 8.0f;
  for (int i = 0; i < 8; i++) {
    int dx = cx + (int)(cos(step * i) * r);
    int dy = cy + (int)(sin(step * i) * r);
    if (i == (frame % 8)) {
      display.fillCircle(dx, dy, 2, WHITE);  // Active dot
    } else {
      display.drawPixel(dx, dy, WHITE);      // Dim dot
    }
  }
}

// Draw loading screen with spinner
static void drawLoadingScreen(const char* title, const char* detail, uint8_t frame) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Title - centered in upper area
  int tx = (128 - strlen(title) * 6) / 2;
  if (tx < 0) tx = 0;
  display.setCursor(tx, 16);
  display.print(title);
  
  // Spinner in center
  drawSpinner(64, 38, 8, frame);
  
  // Detail text below spinner
  if (detail) {
    int dx = (128 - strlen(detail) * 6) / 2;
    if (dx < 0) dx = 0;
    display.setCursor(dx, 54);
    display.print(detail);
  }
  display.display();
}

static void drawCell(int x, int y, int w, int h, uint8_t camNum, uint8_t status) {
  // Calculate centered position for camera number
  char numStr[4];
  snprintf(numStr, sizeof(numStr), "%d", camNum);
  
  if (status == 2) {
    // ====== PROGRAM (ON AIR) ======
    // Fully filled white cell — maximum visibility
    display.fillRoundRect(x+1, y+1, w-2, h-2, 3, WHITE);
    
    // Large black number centered
    display.setTextSize(2);
    display.setTextColor(BLACK, WHITE);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);
    
  } else if (status == 1) {
    // ====== PREVIEW ======
    // Thick border (double outline), white number
    display.drawRoundRect(x+1, y+1, w-2, h-2, 3, WHITE);
    display.drawRoundRect(x+2, y+2, w-4, h-4, 2, WHITE);
    
    // Large white number centered
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 12) / 2;
    int ty = y + (h - 14) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);
    
  } else {
    // ====== OFF ======
    // Thin outline only
    display.drawRoundRect(x+1, y+1, w-2, h-2, 2, WHITE);
    
    // Small dim number centered
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    int tx = x + (w - strlen(numStr) * 6) / 2;
    int ty = y + (h - 7) / 2;
    display.setCursor(tx, ty);
    display.print(numStr);
  }
}

void drawTallyGrid(const uint8_t tallies[8]) {
  const int cols = 4, rows = 2;
  const int cellW = 128 / cols;          // 32px
  const int cellH = (64 - HDR_H) / rows; // 27px
  const int yOff = HDR_H;
  
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      int idx = r * cols + c;
      int x = c * cellW;
      int y = yOff + r * cellH;
      drawCell(x, y, cellW, cellH, TALLY_INPUTS[idx], tallies[idx]);
    }
  }
}

// ===== LoRa Debug Screen =====
void drawLoRaDebug() {
  radio.checkConnection();
  uint8_t st = radio.getChipStatus();
  bool connected = radio.isConnected();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Yellow zone (top 16px)
  display.setCursor(1, 0);
  display.print(connected ? "LoRa: OK" : "LoRa: NO MODULE");
  display.setCursor(1, 8);
  display.print("SPI:0x");
  if (st < 0x10) display.print("0");
  display.print(st, HEX);
  
  // Decode mode
  uint8_t mode = (st >> 2) & 0x07;
  const char* m = "??";
  if(mode==0) m="SLP";
  else if(mode==1) m="STBY";
  else if(mode==2) m="XOSC";
  else if(mode==3) m="FS";
  else if(mode==4) m="RX";
  else if(mode==5) m="TX";
  display.print(" "); display.print(m);
  
  // BUSY status on right
  display.setCursor(86, 8);
  display.print("BUSY:");
  display.print(digitalRead(E28_PIN_BUSY) ? "H" : "L");
  
  display.drawFastHLine(0, 15, 128, WHITE);
  
  // Blue zone (bottom 48px) - 4 rows
  // Row 1: RSSI + SNR
  display.setCursor(1, 18);
  display.print("RSSI: ");
  display.print(radio.getRSSI());
  display.print("dBm");
  display.setCursor(80, 18);
  display.print("SNR:");
  display.print(radio.getSNR());
  
  // Row 2: TX count
  display.setCursor(1, 28);
  display.print("TX pkts: ");
  display.print(g_loraTxCount);
  
  // Row 3: Power + Freq
  display.setCursor(1, 38);
  display.print("Pwr:1dBm BW:400k SF7");
  
  // Row 4: Uptime
  display.setCursor(1, 48);
  display.print("Up: ");
  uint32_t sec = millis() / 1000;
  display.print(sec / 60); display.print("m ");
  display.print(sec % 60); display.print("s");
  
  display.display();
}

IPAddress findAtemInSubnet(){ IPAddress local=WiFi.localIP(); IPAddress test; for(int i=1;i<=254;i++){ test=IPAddress(local[0],local[1],local[2],i); if(test==local) continue; WiFiClient probe; probe.setTimeout(120); if(probe.connect(test,9910)){ probe.stop(); return test; } delay(5);} return IPAddress(0,0,0,0); }

void tryConnectAtem(){
  lastAtemAttempt = millis();
  uint8_t atemFrame = 0;
  IPAddress target;
  
  String cfgIP = String(ATEM_IP_STR);
  if(!cfgIP.length()) {
    drawCenteredMsg("ATEM: no IP set", "Set ATEM_IP_STR");
    return;
  }
  target.fromString(cfgIP);
  
  // ⚡ Bolt: Hoist IP string conversion to avoid redundant heap allocations in UI loop
  String targetStr = ipToStr(target);
  const char* targetCStr = targetStr.c_str();

  drawLoadingScreen("ATEM", targetCStr, atemFrame++);
  
  if(!atem) atem = CreateAtemClient();
  atem->begin(target); atem->connect();
  
  uint32_t t0=millis(); 
  while(millis()-t0 < 3000) {
    atem->loop(); 
    if(atem->connected()) break;
    drawLoadingScreen("ATEM", targetCStr, atemFrame++);
    delay(50);
  }
  atemConnected = atem->connected();
  if(atemConnected) atemAddr = target;
  
  if(atemConnected) drawCenteredMsg("ATEM: connected", targetCStr);
  else drawCenteredMsg("ATEM: failed", targetCStr);
}

void setup(){
  // Serial.begin(115200); // Disabled: GPIO 1/3 used by E28
  I2Cbus.begin(OLED_I2C_SDA, OLED_I2C_SCL, 400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setTextWrap(false); display.clearDisplay(); display.display();
  
  pinMode(0, INPUT_PULLUP); // Boot button for locator
  pinMode(33, OUTPUT);      // Onboard blue LED
  digitalWrite(33, LOW);

  drawCenteredMsg("Wi-Fi: connecting...", WIFI_SSID);
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);

  // ==== NimBLE init ====
  NimBLEDevice::init("TallyHUB");
  g_bleAdv = NimBLEDevice::getAdvertising();
  bleSetAdvPayload(0,0);
  g_bleAdv->start();
  g_lastBleAdv = millis();

  // ждём Wi‑Fi с анимацией
  uint8_t wifiFrame = 0;
  uint32_t t0=millis(); 
  while(WiFi.status()!=WL_CONNECTED && millis()-t0 < WIFI_RETRY_MS) {
    drawLoadingScreen("Wi-Fi", WIFI_SSID, wifiFrame++);
    delay(150);
  }
  if(WiFi.status()!=WL_CONNECTED) drawCenteredMsg("Wi-Fi: FAILED","Retrying forever..."); 
  else drawCenteredMsg("Wi-Fi: connected", ipToStr(WiFi.localIP()).c_str());

  // ESP-NOW (опционально)
  // esp_wifi_set_ps(WIFI_PS_NONE); // Disabled to prevent crash with BT
  if(esp_now_init()==ESP_OK){ esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, ESPNOW_BROADCAST, 6); peer.channel=0; peer.encrypt=false; esp_now_add_peer(&peer);} 

  // ==== E28 LoRa ====
  radio.begin(E28_PIN_SCK, E28_PIN_MISO, E28_PIN_MOSI,
              E28_PIN_NSS, E28_PIN_BUSY, E28_PIN_DIO1, 
              E28_PIN_RESET, E28_PIN_RXEN, E28_PIN_TXEN);
  // Show LoRa debug screen for 3 seconds
  drawLoRaDebug();
  delay(3000);
}

void loop(){
  // ⚡ Bolt: Non-blocking WiFi reconnect screen to keep background tasks responsive
  if(WiFi.status()!=WL_CONNECTED){
    static uint32_t lastRetry=0;
    static uint32_t lastAnim=0;
    static uint8_t lostFrame=0;
    if(millis()-lastRetry>WIFI_RETRY_MS){
      lastRetry=millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    if(millis()-lastAnim > 150){
      lastAnim=millis();
      drawLoadingScreen("Wi-Fi lost", "Reconnecting...", lostFrame++);
    }
    return;
  }

  // if(!atemConnected && millis()-lastAtemAttempt > 500) tryConnectAtem(); // ATEM disabled for LoRa debug

  if(atemConnected){
    atem->loop();
    if(!atem->connected()){ atemConnected=false; lastAtemAttempt=millis()+ATEM_RETRY_MS; return; }

    if(millis()-lastPoll > POLL_MS){
      lastPoll = millis();
      uint8_t tallies[8]; uint16_t progMask=0, prevMask=0;
      for(int i=0;i<8;i++){
        uint8_t idx0 = TALLY_INPUTS[i]-1;
        bool onAir   = atem->isOnAir(idx0);
        bool preview = atem->isPreview(idx0);
        tallies[i] = onAir ? 2 : (preview ? 1 : 0);
        uint8_t human = TALLY_INPUTS[i]; if(onAir) progMask |= (1U << (human-1)); if(preview) prevMask |= (1U << (human-1));
      }

      // ==== Проверка изменений ====
      static uint16_t lastProgMask = 0xFFFF;
      static uint16_t lastPrevMask = 0xFFFF;
      static uint32_t lastDisplayUpdate = 0;
      
      bool tallyChanged = (progMask != lastProgMask) || (prevMask != lastPrevMask);
      bool timeToRefresh = (millis() - lastDisplayUpdate > 2000); // Обновляем инфо раз в 2 сек (RSSI, статус)

      if (tallyChanged || timeToRefresh) {
        lastProgMask = progMask;
        lastPrevMask = prevMask;
        lastDisplayUpdate = millis();

        // ==== NimBLE реклама ====
        // Обновляем payload только если изменились данные (или редко для надежности)
        if (tallyChanged || (millis()-g_lastBleAdv >= TALLY_ADV_INTERVAL_MS * 5)) {
           g_bleAdv->stop(); 
           bleSetAdvPayload(progMask, prevMask); 
           g_bleAdv->start(); 
           g_lastBleAdv = millis(); 
        }

        // ==== LoRa E28 Send ====
        // Iterate cameras and send update if changed OR if it's a periodic refresh
        for(int i=0; i<8; i++) {
            bool stateChanged = (tallies[i] != g_lastTallies[i]);
            if(stateChanged || timeToRefresh) {
                // Determine state
                TallyState ts = STATE_OFF;
                if(tallies[i] == 2) ts = STATE_PROGRAM;
                else if(tallies[i] == 1) ts = STATE_PREVIEW;
                
                uint8_t camId = TALLY_INPUTS[i];
                TallyPacket pkt = TallyProtocol::createSetStatePacket(camId, ts);
                
                uint8_t buf[TALLY_PACKET_SIZE];
                TallyProtocol::serialize(pkt, buf);
                
                radio.send(buf, TALLY_PACKET_SIZE);
                g_loraTxCount++;
                // Serial.printf("LoRa TX: Cam %d -> %d\n", camId, ts);
                delay(2); // Small delay to avoid packet collision if consecutive
            }
            g_lastTallies[i] = tallies[i];
        }
      }
    }
    
    // Locator / Ping (Button 0)
    static uint32_t lastPing = 0;
    if (digitalRead(0) == LOW && millis() - lastPing > 500) {
        lastPing = millis();
        drawCenteredMsg("LOCATOR", "Ping sent -> Cam 1");
        
        // Send 3 times for reliability + blink LED
        for(int k=0; k<3; k++) {
          digitalWrite(33, HIGH); // Blue LED ON
          TallyPacket pkt = TallyProtocol::createPingPacket(1); 
          uint8_t buf[TALLY_PACKET_SIZE];
          TallyProtocol::serialize(pkt, buf);
          radio.send(buf, TALLY_PACKET_SIZE);
          g_loraTxCount++;
          delay(100);
          digitalWrite(33, LOW);  // Blue LED OFF
          delay(50);
        }
        delay(2000); // Wait so user can see message
    }
  }

  // ==== OLED (LoRa debug mode - always active) ====
  drawLoRaDebug();

  delay(10);
}
