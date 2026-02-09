# ESP32 Tally Project

This project implements a wireless Tally Light system using ESP32 microcontrollers and E28 (SX1280) LoRa modules. it consists of a Hub (Master) and multiple Slaves.

## Project Structure

- **tally_hub/**: Firmware for the Master Controller (ESP32-S3 + E28-2G4M27S + OLED).
  - Includes local `lib/` folder with drivers.
- **tally_slave/**: Firmware for Receiver Units (ESP32-C3 SuperMini + E28-2G4M12SX).
  - Includes local `lib/` folder with drivers.
- **documentation/**: Wiring diagrams and pinouts.

## Hardware Setup

### Hub (Master)
- **Controller**: ESP32-S3-CAM (or DevKit)
- **Radio**: E28-2G4M27S (High Power)
- **Display**: SSD1306 OLED (I2C)
- **Pinout**: See `tally_hub/src/config.h` or documentation.

### Slave (Receiver)
- **Controller**: ESP32-C3 SuperMini
- **Radio**: E28-2G4M12SX (Low Power)
- **Feedback**: Built-in LED (GPIO 8) + External Tally LED.
- **Pinout**: See `tally_slave/src/main.cpp` or `documentation/wiring_slave.svg`.

## Building & Flashing

Each folder (`tally_hub`, `tally_slave`) is a standalone PlatformIO project.

1. Open the desired folder in VS Code / PlatformIO.
2. Build and Upload.
   - **Hub**: Needs Boot/Reset for upload on some boards.
   - **Slave (C3)**: Needs Boot button held during Reset to enter download mode.

## Troubleshooting

- **Slave Fast Blue Blink**: LoRa Init Failed. Check wiring (MISO/MOSI).
- **Slave 3 Slow Blinks**: LoRa Init OK. Waiting for signal.
- **Hub Locator**: Press BOOT button on Hub to send PING signal to Cam 1.
