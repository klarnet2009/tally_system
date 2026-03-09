## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2024-05-15 - Delay loops block radio reception in Tally Slave
**Learning:** Using `delay()` loops in the ESP32 Tally Slave's `loop()` to control LED blink patterns (e.g., Locator sequence for 500ms) pauses the main thread and prevents `radio.available()` from being checked. This creates a large window where critical LoRa updates (like `CMD_SET_STATE`) are ignored and lost.
**Action:** When implementing LED or Buzzer sequences on the Tally Slave, always use non-blocking `millis()`-based state machines evaluated at the end of the `loop()`, ensuring radio processing remains uninterrupted.
