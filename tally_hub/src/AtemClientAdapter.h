#pragma once
#include <Arduino.h>
#include <WiFi.h>

class IAtemClient {
public:
  virtual ~IAtemClient() {}
  virtual bool begin(IPAddress ip) = 0;       // подготовка/установка IP
  virtual bool connect() = 0;                 // попытка соединения
  virtual void loop() = 0;                    // качаем протокол
  virtual bool connected() = 0;               // рукопожата ли сессия
  virtual bool isOnAir(uint8_t input) = 0;    // program tally
  virtual bool isPreview(uint8_t input) = 0;  // preview tally
};

// Фабрика, чтобы выбрать реализацию в одном месте
IAtemClient* CreateAtemClient();
