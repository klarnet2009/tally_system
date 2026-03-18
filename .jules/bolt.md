## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2024-03-18 - High-Frequency SPI Polling Optimization
**Learning:** Functions like `getIrqStatus()` and `clearIrqStatus()` that wrap generic SPI read/write methods suffer high overhead when those generic methods defensively allocate large stack buffers (e.g., 260 bytes for max packet sizes) and invoke `memset`/`memcpy` for payloads of just 2-3 bytes. In a high-frequency polling loop (like waiting for TX completion or checking for RX availability), this repeated large allocation and memory copying creates measurable CPU latency. Bypassing the wrapper breaks the Hardware Abstraction Layer (HAL) and risks synchronization bugs.
**Action:** When creating high-frequency polling abstractions around hardware interfaces, optimize the underlying generic `readCommand`/`writeCommand` wrapper methods by adding a "fast-path" conditional for small payloads (e.g., `len <= 8`). This fast-path should use small, fixed-size stack buffers to eliminate `memcpy` overhead and redundant stack pressure without breaking encapsulation.
