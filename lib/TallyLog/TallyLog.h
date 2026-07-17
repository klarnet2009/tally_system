#ifndef TALLY_LOG_H
#define TALLY_LOG_H

#include <Arduino.h>

// Flash-backed field log (LittleFS, two-file ring). Always on: every event
// line the firmware prints also lands here with an uptime stamp, so a day of
// field testing can be pulled off the device afterwards over USB ("log"
// serial command) and pasted into analysis — no laptop tethered during the
// test. Two-file rotation bounds both size and flash wear; at our logging
// rate (~50-80 KB/h) the ring holds the better part of an event day and the
// wear-leveled partition outlives the hardware.
class TallyLogClass {
public:
  // Mount LittleFS (formats the partition on first use), open the log,
  // stamp a "BOOT #N reset_reason=X" line. maxBytes = per-file rotation
  // threshold (at most two files exist: /log.txt and /log.old).
  bool begin(size_t maxBytes);

  // Append one line (should end with '\n'); prefixed with [HH:MM:SS] uptime.
  void write(const char *line);
  void logf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));

  void tick();            // periodic flush — call every loop pass
  void dump(Stream &out); // stream /log.old then /log.txt with cut markers
  void clear();           // wipe both files, start fresh

  size_t size() const { return _size; }
  bool ok() const { return _ok; }

private:
  void rotateIfNeeded();

  bool _ok = false;
  size_t _size = 0;
  size_t _max = 262144;
  uint32_t _lastFlush = 0;
  bool _dirty = false;
};

extern TallyLogClass TallyLog;

#endif // TALLY_LOG_H
