## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2026-03-10 - Avoiding `millis()` Overhead in Hot Paths
**Learning:** Calling `millis()` within frequent, tight loops (such as hardware busy-waits or `loop()` iterations on an ESP32) adds unnecessary CPU overhead when the condition isn't active. Additionally, allocating large fixed-size arrays inside generic utility functions (e.g., `readCommand` making a 260-byte buffer for a 2-byte read) causes high stack utilization and `memset` overhead.
**Action:** When waiting on hardware pins, implement early return fast-paths (e.g., `if (digitalRead(pin) == LOW) return;`) before calling `millis()` to begin a timeout loop. When reading very small pieces of data (e.g., a 16-bit status register) in a hot path, inline the direct SPI block transfer with the precise buffer size needed rather than calling a generic, heavily-buffered read wrapper.
