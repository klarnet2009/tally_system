## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2024-05-15 - Hardware GPIO Readiness vs SPI Polling
**Learning:** Removing hardware GPIO readiness checks (e.g., `digitalRead(_pinDIO1)`) in SPI receive loops to save a microsecond of CPU time is a massive performance regression. It forces the main loop to execute full, multi-byte SPI transactions over the bus on every iteration just to check if a packet arrived, wasting CPU cycles and creating SPI bus congestion.
**Action:** Always prefer ~1us hardware GPIO pin checks over continuous SPI polling when determining if a module has data available.
