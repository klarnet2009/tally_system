# ESP32 Tally Project

This project implements a wireless Tally Light system using ESP32 microcontrollers and E28 (SX1280) LoRa modules. it consists of a Hub (Master) and multiple Slaves.

## Project Structure

- **tally_hub/**: Firmware for the Master Controller (ESP32-S3 + E28-2G4M27S + OLED).
- **tally_slave_v2/**: **Primary** receiver firmware (ESP32-C3 SuperMini + E28-2G4M12SX, WS2812B + buzzer). See `tally_slave_v2/src/pins.h` and `tally_slave_v2/HARDWARE_GUIDE.md`.
- **tally_slave/**: Legacy mono-LED receiver firmware (different, older pinout — do not wire a v2 board to it).
- **lib/**: Shared drivers (E28_SX1280, TallyProtocol, TallyLink) used by all projects via `lib_extra_dirs`.
- **documentation/**: Wiring diagrams and pinouts.
- **DEBUGGING.md**: Debug surfaces (serial consoles, log lines, commands, LED/beep codes) and init-error triage.

## Hardware Setup

### Hub (Master)
- **Controller**: ESP32-S3-CAM (or DevKit)
- **Radio**: E28-2G4M27S (High Power)
- **Display**: SSD1306 OLED (I2C)
- **Pinout**: See `tally_hub/src/config.h` or documentation.

### Slave (Receiver) — v2, primary
- **Controller**: ESP32-C3 SuperMini
- **Radio**: E28-2G4M12SX (Low Power)
- **Feedback**: WS2812B RGB LED (GPIO 7) + buzzer (GPIO 8).
- **Pinout**: See `tally_slave_v2/src/pins.h` (single source of truth) and `tally_slave_v2/HARDWARE_GUIDE.md`.
  The legacy `tally_slave/` firmware uses a different older pinout — don't cross them.

## Building & Flashing

Each folder (`tally_hub`, `tally_slave`) is a standalone PlatformIO project.

1. Copy `tally_hub/src/secrets.h.example` to `tally_hub/src/secrets.h` and fill in
   your Wi-Fi credentials and ATEM IP. The file is gitignored.
   (Note: credentials committed before this scheme was introduced remain in git
   history — rotate the Wi-Fi password if the repo is shared.)
2. Open the desired folder in VS Code / PlatformIO.
3. Build and Upload.
   - **Hub**: Needs Boot/Reset for upload on some boards.
   - **Slave (C3)**: Needs Boot button held during Reset to enter download mode.

## Troubleshooting

- **Slave Fast Blue Blink**: LoRa Init Failed. Check wiring (MISO/MOSI).
- **Slave 3 Slow Blinks**: LoRa Init OK. Waiting for signal.
- **Hub Locator**: Press BOOT button on Hub to send PING signal to Cam 1.
