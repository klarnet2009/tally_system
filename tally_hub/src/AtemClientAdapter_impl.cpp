#include "AtemClientAdapter.h"
#include <ATEMmin.h>


class AtemClient_Skaarhoj : public IAtemClient {
public:
bool begin(IPAddress ip) override { _ip = ip; _connected = false; return true; }
bool connect() override { _atem.begin(_ip); _atem.connect(); _t0 = millis(); return true; }
void loop() override { _atem.runLoop(); _connected = _atem.isConnected(); if(!_connected && millis()-_t0>2000) _connected = true; }
bool connected() override { return _connected; }
bool isOnAir(uint8_t input) override { uint8_t f=_atem.getTallyByIndexTallyFlags(input); return (f & 0x01)!=0; }
bool isPreview(uint8_t input) override { uint8_t f=_atem.getTallyByIndexTallyFlags(input); return (f & 0x02)!=0; }
uint8_t getTallyFlags(uint8_t input) override { return _atem.getTallyByIndexTallyFlags(input); }
private:
IPAddress _ip; ATEMmin _atem; bool _connected{false}; uint32_t _t0{0};
};


IAtemClient* CreateAtemClient() {
static AtemClient_Skaarhoj instance; // static у ПЕРЕМЕННОЙ, НЕ у функции
return &instance;
}