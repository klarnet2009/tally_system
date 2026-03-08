## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2026-03-08 - Incomplete Optimization Pattern Identified
**Learning:** Partially optimized SPI code (e.g., sending commands byte-by-byte and payload as a block) still suffers from significant locking overhead. Full coalescing of command, offset, and payload into a single stack buffer before a block transfer yields the true performance win.
**Action:** When optimizing SPI transfers, ensure ALL bytes for a transaction are coalesced into a single buffer and sent with one block transfer.
