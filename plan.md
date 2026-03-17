1. *Apply fast-paths for high-frequency SPI polling in the `E28_SX1280` library*
   - Modifying `E28_SX1280.cpp` in `lib/`, `tally_hub/lib/`, and `tally_slave/lib/`.
   - Update `getIrqStatus()` to use an inline 4-byte fixed-size stack buffer with `SPI.transferBytes` to bypass the generic `readCommand`'s 520-byte allocation and `memcpy` overhead.
   - Update `waitBusy()` to include an early-return check (`if (digitalRead(_pinBUSY) == LOW) return;`) to bypass `millis()` overhead on the fast path.
2. *Verify code syntax*
   - Run `./test_compile.sh` and `./test_compile_hub.sh` to ensure there are no compilation errors.
3. *Complete pre commit steps*
   - Follow required testing, verifications, reviews and reflections before submitting.
4. *Submit PR*
   - Use the Bolt format for the PR: `⚡ Bolt: [performance improvement]` in the title and include What, Why, Impact, and Measurement in the description.
