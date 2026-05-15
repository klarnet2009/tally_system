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

## 2024-04-03 - [TallyProtocol: Eliminate memcpy overhead for tiny payloads]
**Learning:** In C/C++ embedded environments, calling `memcpy` or `memset` for tiny, fixed-size payloads (like the 8-byte `TallyPacket`) incurs function call overhead that can be slower than executing a simple direct loop, as stack allocation and basic assignment are constant-time operations. This is a crucial fast-path optimization for tight, high-frequency protocol parsing.
**Action:** When implementing protocol wrappers or packet serialization/deserialization for small, constant-size payloads (<= 8 bytes), prioritize direct array assignments or explicit loops over generic `memcpy` to eliminate unnecessary function overhead without compromising readability.

## 2024-04-03 - [TallyProtocol: Fast-path early return on noisy packets]
**Learning:** In radio communications like LoRa, the receiver can pick up noise or packets from other systems. The `deserialize` function was unconditionally copying the buffer into the packet struct and then validating the CRC, which is computationally expensive (a loop over 7 bytes). If the first byte isn't the `TALLY_START_BYTE`, the packet is invalid anyway.
**Action:** Always add an early return fast-path to check for the start byte *before* performing expensive memory copies and CRC validation on incoming packets. This saves CPU cycles on the critical RX path, especially in noisy RF environments.

## 2026-04-09 - Algorithmic Fast-Path Early Returns in Broadcast Polling Loops
**Learning:** In broadcast-heavy polling loops where a device continuously receives state packets for multiple clients (e.g., a hub broadcasting states for 8 cameras sequentially), it's highly inefficient for a slave to unconditionally perform memory copying (`memcpy`) and CRC validation on every single packet just to determine it's not the intended recipient.
**Action:** Always implement a fast-path algorithmic early return before computationally expensive operations like deserialization. Inspect the relevant identifier bytes (e.g., `buf[2]` for camera ID) directly from the raw incoming buffer to immediately bypass processing for packets not destined for the current device, saving significant CPU cycles in high-frequency event loops.

## 2024-04-15 - Non-blocking state machines over delay()
**Learning:** In ESP32 applications like this Tally system, connection loops (like ATEM client connection) often default to `delay()` inside `while` loops, severely blocking critical background tasks like `WiFi.status()` and NimBLE updates. Moving to a non-blocking `millis()` based state machine within the main `loop()` is crucial for maintaining real-time system responsiveness.
**Action:** When finding blocking `delay()` calls inside hot loops or initialization sequences, extract them into explicit state machine variables integrated into the main non-blocking execution cycle.

## 2026-05-10 - Avoiding `memcpy` in serialization fast-paths
**Learning:** `memcpy` can introduce function call overhead and prevents some simple loop optimizations by the compiler on specific targets when used for tiny arrays.
**Action:** Replace `memcpy` in serialization and deserialization functions with direct pointer assignment loops for the exact known `TALLY_PACKET_SIZE` to bypass the function call overhead on tight loops.

## 2026-05-01 - Consolidated ATEM Tally Flag Retrieval
**Learning:** Calling separate virtual functions `isOnAir()` and `isPreview()` in a tight polling loop to retrieve individual bits of the same underlying state (tally flags) creates redundant overhead. The underlying implementation was already making two identical `getTallyByIndexTallyFlags()` library calls.
**Action:** When a library or hardware provides a combined bitmask of states, always expose a single method (like `getTallyFlags()`) through abstraction layers to retrieve the entire bitmask at once, rather than forcing callers to make multiple redundant virtual function calls for individual flags.

## 2026-05-11 - Fast RX re-arming
**Learning:** When re-arming continuous RX mode on SX1280 modules, avoid full re-initialization like `startReceive()` which triggers a slow `standby()` and configuration delay. Instead, use a lightweight fast re-arm (e.g., `clearRxIrq()`) to clear the IRQ and re-issue the continuous RX command, halving SPI transaction overhead and minimizing dropped packets.
**Action:** Replace `startReceive()` with `clearRxIrq()` for RX re-arming in loops checking for available radio packets.

## 2026-05-15 - Fast-path TxDone polling to prevent SPI bus starvation
**Learning:** In SX1280 radio driver loops waiting for transmission completion (like `isTxDone`), unconditionally checking the interrupt status via SPI (`getIrqStatus()`) can hammer the SPI bus and waste CPU cycles, especially since a single SPI transfer takes microseconds compared to nanoseconds for a GPIO read.
**Action:** When a hardware interrupt pin like `DIO1` is available and mapped to the relevant interrupt (like TxDone), always implement a fast-path that checks the pin using `digitalRead()` first. Only fall back to querying the full status over SPI if the pin indicates an event has occurred.

## 2026-05-04 - Algorithmic string prefix validation fast-path
**Learning:** Using `strlen()` to validate string length when only a small, fixed-size prefix is required creates unnecessary O(N) overhead, especially for long, untrusted inputs (e.g., from serial communication).
**Action:** Replace `strlen(str) < prefix_len` checks with O(1) direct null-terminator checks (e.g., `str[0] == '\0' || str[1] == '\0'`) to prevent unnecessary full string traversals.

## 2026-05-04 - Manual loops vs memcpy compiler intrinsics
**Learning:** Replacing `memcpy` with manual byte-by-byte `for` loops to "optimize" tiny fixed-size data transfers (e.g., 8-byte packets) is a performance anti-pattern. Modern compilers correctly identify fixed-size `memcpy` calls and optimize them into highly efficient intrinsic instructions (e.g., 64-bit register moves) which are significantly faster than naive loops.
**Action:** Never replace `memcpy` with manual loops for fixed-size memory operations; trust the compiler intrinsics.

## 2024-05-18 - Loop unrolling for tiny CRC calculations
**Learning:** In embedded systems using compilers optimized for size (`-Os`), small fixed-size arithmetic loops (like XORing 7 bytes for a CRC) often are not automatically unrolled. This leaves unnecessary branch and loop counter overhead in critical serialization/deserialization fast-paths.
**Action:** Manually unroll small, fixed-size loops where performance is critical. Guard the unrolled implementation with a `static_assert` on the expected array/packet size to prevent regressions if data structures change in the future.
## 2026-05-19 - Compiler Intrinsics for memcpy
**Learning:** Replacing `memcpy` with manual byte-by-byte `for` loops to 'optimize' small payloads actually degrades performance by preventing the compiler from generating highly optimized intrinsic instructions (like 32-bit or 64-bit register moves) for fixed-size memory transfers.
**Action:** Always rely on standard `memcpy` for fixed-size buffer transfers instead of writing manual loops, especially when the buffer size is known or small.

## 2026-05-19 - Removed trailing waitBusy() blocking calls
**Learning:** In SPI drivers for modules with hardware busy pins (like SX1280), appending a synchronous `waitBusy()` call at the *end* of an SPI transaction unnecessarily starves the CPU. Since every SPI transaction correctly calls `waitBusy()` at its *beginning* to ensure hardware readiness, a trailing wait prevents the CPU from doing productive concurrent work (like toggling GPIOs or looping) while the radio is busy.
**Action:** Always place hardware readiness checks at the beginning of the transaction method and remove trailing synchronous waits to allow CPU execution to overlap with peripheral processing.
