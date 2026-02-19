# SUFIDE Tally — Hardware Guide

> Полная документация по аппаратной части SUFIDE Tally.
> Дата: 2026-02-10 | Статус: Hardware Verified ✅

---

## 1. Компоненты

| Компонент | Модель | Описание |
|-----------|--------|----------|
| MCU | **ESP32-C3 Super Mini** | RISC-V, 160MHz, 4MB Flash, WiFi+BLE |
| LoRa | **E28-2G4M12SX** (EBYTE) | SX1281, 2.4GHz, 12dBm, SPI |
| LED | **WS2812B** | Addressable RGB, 1шт |
| Buzzer | Пассивный | PWM tone(), 3.3V |
| Батарея | **14500 Li-Ion** | 3.7V, ~800mAh, формфактор AA |
| Зарядка | **TP4056** | Li-Ion charger + защита |

---

## 2. Распиновка

### ESP32-C3 Super Mini — доступные GPIO

| GPIO | Назначение | Примечание |
|------|------------|------------|
| **0** | 🟢 Свободен | ⚠️ Strapping pin |
| **1** | LoRa DIO1 | Прерывание от модуля |
| **2** | 🟢 Свободен | ⚠️ Strapping pin |
| **3** | LoRa SCK | SPI Clock |
| **4** | LoRa MOSI | SPI Master Out |
| **5** | LoRa MISO | SPI Master In |
| **6** | LoRa NRESET | Аппаратный сброс модуля |
| **7** | WS2812B LED | Data In |
| **8** | Buzzer | PWM (tone) |
| **9** | 🟢 Свободен | ⚠️ Boot кнопка |
| **10** | LoRa BUSY | Готовность модуля |
| **20** | LoRa NSS | SPI Chip Select |
| **21** | 🟢 Свободен | ✅ Лучший выбор для нового |

**Свободно: GPIO 0, 2, 9, 21** (4 пина)

### LoRa E28-2G4M12SX — подключение к ESP32-C3

```
E28-2G4M12SX          ESP32-C3 Super Mini
─────────────         ─────────────────────
VCC          ──────── 3.3V
GND          ──────── GND
NSS (CTS)    ──────── GPIO 20
SCK (RTS)    ──────── GPIO 3
MOSI (RX)    ──────── GPIO 4
MISO (TX)    ──────── GPIO 5
NRESET       ──────── GPIO 6
BUSY         ──────── GPIO 10
DIO1         ──────── GPIO 1
```

> ⚠️ **Внимание:** при пайке модуля с обратной стороны пины зеркальны!
> Всегда сверяйтесь с маркировкой на самом модуле.

### Другие компоненты

```
WS2812B LED         ESP32-C3
───────────         ─────────
VCC        ──────── 3.3V
GND        ──────── GND
DIN        ──────── GPIO 7

Buzzer              ESP32-C3
──────              ─────────
+          ──────── GPIO 8
-          ──────── GND
```

---

## 3. Схема питания

### От USB (разработка)

```
USB-C ──── ESP32-C3 (встроенный LDO) ──── 3.3V для всей схемы
```

### От батареи 14500 (автономная работа)

```
USB (зарядка)
    │
    ▼
┌────────┐     ┌─────────┐     ┌──────────┐
│ TP4056 │─B+──│ 14500   │     │ ESP32-C3 │
│        │─B-──│ батарея  │     │          │
│        │─OUT+─────────────────│ 5V       │
│        │─OUT-─────────────────│ GND      │
└────────┘     └─────────┘     └──────────┘
```

- **TP4056 OUT+** → ESP32-C3 пин **5V**
- **TP4056 OUT-** → ESP32-C3 пин **GND**
- Встроенный LDO ESP32-C3 понижает 3.7V → 3.3V

### Потребление тока

| Режим | Ток | Время от 800mAh |
|-------|-----|-----------------|
| Всё активно (TX + LED + Buzzer) | ~200 мА | ~4 часа |
| Обычная работа (RX) | ~100 мА | ~8 часов |
| Deep Sleep | ~5 мкА | ~18 лет |

---

## 4. Прошивка

### PlatformIO конфигурация

**platformio.ini:**
```ini
[env:esp32c3]
platform = espressif32@6.9.0
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
upload_port = COM3
monitor_port = COM3

lib_deps =
    adafruit/Adafruit NeoPixel@^1.12.0

build_flags =
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D ARDUINO_USB_MODE=1
```

### Команды

```bash
# Сборка
python -m platformio run

# Прошивка
python -m platformio run -t upload

# Мониторинг Serial
python -m platformio device monitor -p COM3 -b 115200
```

### Serial команды

| Команда | Действие |
|---------|----------|
| `test` | Запустить все 4 теста заново |
| `help` | Показать доступные команды |

### Структура тестов

Прошивка запускает 4 теста при старте и по команде `test`:

1. **[1/4] ESP32-C3 Info** — чип, ревизия, память, SDK
2. **[2/4] WS2812B LED** — цикл R→G→B→W
3. **[3/4] Buzzer** — тоны 400/1000/2400 Hz
4. **[4/4] SX1281 LoRa** — 3 этапа:
   - **[A] SPI Status** — GetStatus, ожидается 0x40
   - **[B] Buffer Write/Read** — записывает `0xDEADBEEF`, читает обратно
   - **[C] Radio Config** — Standby, LoRa mode, 2.402GHz

---

## 5. SPI протокол SX1281

### Ключевые особенности

```
GetStatus:    TX: C0 00 → RX: (echo) (status)
                              ^^^^^ ─ НЕ читать! 
                                      ^^^^^^ ─ настоящий статус

ReadRegister: TX: 1D addr_h addr_l 00 00 → RX: ... ... ... status DATA

WriteBuffer:  TX: 1A offset data0 data1 ...
ReadBuffer:   TX: 1B offset 00 data0 data1 ...
```

### Статус-коды

| Статус | Значение |
|--------|----------|
| 0x40 | STDBY_RC (нормально после сброса) |
| 0x00 | Не отвечает / нет подключения |
| 0xFF | MISO не подключён / floating |

### Радио конфигурация (2.4GHz LoRa)

```
SetStandby     → STDBY_RC (0x00)
SetPacketType  → LoRa (0x01)
SetRfFrequency → 2.402 GHz (0xB8, 0xE3, 0x4C)
SetModParams   → SF7, BW1600, CR 4/5
```

---

## 6. Результаты тестирования

### Тест от 2026-02-10

```
--- [1/4] ESP32-C3 Info ---
  Chip: ESP32-C3, Rev: 4, CPU: 160 MHz, Flash: 4 MB
  >> ESP32-C3 OK ✅

--- [2/4] WS2812B LED Test ---
  >> WS2812B OK ✅

--- [3/4] Buzzer Test ---
  >> Buzzer OK ✅

--- [4/4] SX1281 Full Test ---
  [A] SPI Status: 0x40       >> PASS ✅
  [B] Buffer: DEADBEEF → DEADBEEF  >> PASS ✅
  [C] Radio Config 2.4GHz    >> PASS ✅
  >> SX1281 FULLY VERIFIED! ✅
```

**Оба LoRa модуля прошли все тесты.**

---

## 7. Решённые проблемы

| Проблема | Причина | Решение |
|----------|---------|---------|
| ESP не загружается с LoRa | Strapping пины GPIO 0/2 | Перенос NSS→GPIO20, BUSY→GPIO10 |
| Brownout при подключении LoRa | Бросок тока модуля | Отключён brownout detector |
| Status 0xC0 (ложный) | getStatus читал эхо команды | Исправлено: читаем NOP-ответ |
| Register R/W fail | Регистры SX1281 не R/W | Заменено на WriteBuffer/ReadBuffer |
| Нестабильный SPI | MISO был floating перед init | Добавлены delay(2) вместо waitBusy |

---

## 8. Следующие шаги

- [ ] **Power**: Собрать схему с TP4056 + 14500 батареей
- [ ] **On/Off**: Кнопка включения + Deep Sleep
- [ ] **TX/RX тест**: Передача между двумя LoRa модулями
- [ ] **Протокол**: Разработать протокол Tally (камера ID, статус, команды)
- [ ] **Корпус**: 3D-печать корпуса

---

## 9. Файловая структура проекта

```
sufide.tally/
├── platformio.ini          # Конфигурация PlatformIO
├── HARDWARE_GUIDE.md       # Этот файл
├── src/
│   └── main.cpp            # Основная прошивка (тесты + serial CLI)
└── .pio/                   # Build артефакты (автогенерация)
```

---

*SUFIDE Tally — Wireless Camera Tally System*
*ESP32-C3 + SX1281 LoRa 2.4GHz*
