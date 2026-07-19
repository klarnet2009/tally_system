#include "TallyLog.h"

#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>

TallyLogClass TallyLog;

static File s_file;
static const char *kLog = "/log.txt";
static const char *kOld = "/log.old";

bool TallyLogClass::begin(size_t maxBytes) {
  _max = maxBytes;
  if (!LittleFS.begin(true)) { // true = format the partition on first mount
    _ok = false;
    return false;
  }
  s_file = LittleFS.open(kLog, FILE_APPEND);
  _ok = (bool)s_file;
  if (!_ok)
    return false;
  _size = s_file.size();

  // Boot counter (NVS) + reset reason make sessions separable in analysis:
  // "BOOT #7 reset_reason=1" starts every session's slice of the ring.
  Preferences p;
  p.begin("tallylog", false);
  uint32_t bootn = p.getULong("bootn", 0) + 1;
  p.putULong("bootn", bootn);
  p.end();
  logf("===== BOOT #%lu reset_reason=%d =====\n", (unsigned long)bootn,
       (int)esp_reset_reason());
  return true;
}

void TallyLogClass::write(const char *line) {
  if (!_ok)
    return;
  char ts[16];
  uint32_t s = millis() / 1000;
  snprintf(ts, sizeof(ts), "[%02lu:%02lu:%02lu] ", (unsigned long)(s / 3600),
           (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
  _size += s_file.print(ts);
  _size += s_file.print(line);
  _dirty = true;
  rotateIfNeeded();
}

void TallyLogClass::logf(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  write(buf);
}

void TallyLogClass::tick() {
  // Flush every few seconds, not every line: bounds data loss on a crash to
  // seconds while keeping flash traffic (and the occasional multi-ms block
  // erase stall) off the per-line hot path.
  if (_ok && _dirty && millis() - _lastFlush > 3000) {
    _lastFlush = millis();
    _dirty = false;
    s_file.flush();
  }
}

void TallyLogClass::rotateIfNeeded() {
  if (!_ok || _size < _max)
    return;
  s_file.close();
  LittleFS.remove(kOld);
  LittleFS.rename(kLog, kOld);
  s_file = LittleFS.open(kLog, FILE_APPEND);
  _ok = (bool)s_file;
  _size = 0;
  if (_ok)
    logf("===== ROTATED =====\n");
  else
    Serial.println("[LOG] flash reopen failed after rotation — logging disabled");
}

void TallyLogClass::dump(Stream &out) {
  if (!_ok) {
    out.println("[LOG] flash log unavailable");
    return;
  }
  // Close the append handle for the read pass; reopen after. Keeps us off
  // any concurrent-handle edge cases in the FS layer.
  s_file.flush();
  s_file.close();
  const char *names[2] = {kOld, kLog};
  for (int i = 0; i < 2; i++) {
    File f = LittleFS.open(names[i], FILE_READ);
    if (!f)
      continue;
    out.printf("----8<---- LOG %s (%u bytes)\n", names[i], (unsigned)f.size());
    uint8_t buf[256];
    while (true) {
      size_t n = f.read(buf, sizeof(buf));
      if (!n)
        break;
      out.write(buf, n);
    }
    f.close();
    out.printf("---->8---- END %s\n", names[i]);
  }
  s_file = LittleFS.open(kLog, FILE_APPEND);
  _ok = (bool)s_file;
  _size = _ok ? s_file.size() : 0;
  if (!_ok)
    Serial.println("[LOG] flash reopen failed after dump — logging disabled");
}

void TallyLogClass::clear() {
  if (!_ok)
    return;
  s_file.close();
  LittleFS.remove(kOld);
  LittleFS.remove(kLog);
  s_file = LittleFS.open(kLog, FILE_APPEND);
  _ok = (bool)s_file;
  _size = 0;
  if (_ok)
    logf("===== LOG CLEARED =====\n");
  else
    Serial.println("[LOG] flash reopen failed after clear — logging disabled");
}
