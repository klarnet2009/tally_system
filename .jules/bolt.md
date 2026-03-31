## 2024-03-05 - SPI Performance Optimization on ESP32
**Learning:** ESP32 hardware SPI FIFO is not efficiently utilized when using byte-by-byte `SPI.transfer()` loops. Each call incurs locking overhead and interrupts. Block transfer functions like `SPI.writeBytes()` and `SPI.transferBytes()` are significantly more performant for arrays/buffers.
**Action:** Always prefer `SPI.writeBytes(data, len)` or `SPI.transferBytes(tx_buf, rx_buf, len)` over manual byte-iteration `for` loops when handling buffer transfers on ESP32-based hardware.

## 2026-03-07 - Dynamic Memory in SPI Overheads
**Learning:** Using heap allocation (`new`/`delete`) to dynamically size temporary buffers in hot paths (like SPI packet sending/receiving) is an anti-pattern on ESP32 due to heap fragmentation and lack of exception handling (OOM causes null pointer crashes). The overhead of the allocator defeats the purpose of coalescing SPI transfers.
**Action:** When coalescing data for SPI block transfers, always use fixed-size stack buffers sized up to the maximum expected hardware packet size (e.g. 260 bytes for LoRa).

## 2024-03-18 - High-Frequency SPI Polling Optimization
**Learning:** Functions like `getIrqStatus()` and `clearIrqStatus()` that wrap generic SPI read/write methods suffer high overhead when those generic methods defensively allocate large stack buffers (e.g., 260 bytes for max packet sizes) and invoke `memset`/`memcpy` for payloads of just 2-3 bytes. In a high-frequency polling loop (like waiting for TX completion or checking for RX availability), this repeated large allocation and memory copying creates measurable CPU latency. Bypassing the wrapper breaks the Hardware Abstraction Layer (HAL) and risks synchronization bugs.
**Action:** When creating high-frequency polling abstractions around hardware interfaces, optimize the underlying generic `readCommand`/`writeCommand` wrapper methods by adding a "fast-path" conditional for small payloads (e.g., `len <= 8`). This fast-path should use small, fixed-size stack buffers to eliminate `memcpy` overhead and redundant stack pressure without breaking encapsulation.

## 2026-03-24 - Redundant String Allocations in Polling/UI Loops
**Learning:** Repeatedly constructing `String` objects (and converting them via `.c_str()`) within non-blocking polling loops or UI rendering sequences (e.g., connection attempts checking `millis()`) creates unnecessary heap allocations, memory fragmentation, and CPU overhead on ESP32 devices.
**Action:** Always hoist invariant string constructions out of repetitive loops. Cache the resulting `String` object and its `.c_str()` pointer *before* entering the loop to ensure minimal footprint during high-frequency execution sequences.

## 2026-03-29 - Unconditional Synchronous I2C UI Updates
**Learning:** Unconditional synchronous I2C peripheral updates (like OLED screens via `drawLoRaDebug()`) within embedded `loop()` functions block the main loop significantly. This starvation prevents high-frequency background tasks (like LoRa radio polling) from executing in a timely manner, which can lead to dropped packets or increased latency.
**Action:** Always rate-limit synchronous UI rendering and slow peripheral communication loops using non-blocking `millis()` timers (e.g., updating at 2-5Hz) to ensure critical system tasks remain responsive.

## 2026-03-31 - Blocking Delays in Polling/UI Loops
**Learning:** Sequential `delay()` calls in UI sequences (like the `LOCATOR` sequence taking ~2.5s) in embedded `loop()` functions cause massive starvation, completely halting critical background operations like `atem->loop()` (TCP connection), WiFi management, and NimBLE advertising. This results in connection drops and missed events.
**Action:** Always replace blocking `delay()` sequences inside the main loop with non-blocking `millis()` state machines that track `elapsed` time and update system state incrementally, allowing high-frequency background tasks to run concurrently.

## 2026-04-01 - Fast-Path Opts in Protocol-Specific Wrappers
**Learning:** Even when generic SPI `writeCommand` and `readCommand` abstractions have been optimized with fast-paths, higher-level protocol-specific wrappers like `send()`, `sendAsync()`, and `receive()` might still naively allocate maximum-size buffers (e.g., 260 bytes) and perform expensive memory copies for very small payloads (like 8-byte Tally packets). This introduces unnecessary CPU latency in tight event loops.
**Action:** When working with embedded drivers that handle variable-length hardware payloads, always ensure that both the generic low-level abstractions AND the protocol-level interface functions (like `send`/`receive`) implement stack-allocation fast-paths for small payloads.
