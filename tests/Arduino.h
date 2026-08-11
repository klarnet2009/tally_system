// Host test shim: just enough Arduino surface to compile the pure-logic units
// (TallyProtocol, TallyLink, TallyBurst) natively, with a CONTROLLABLE clock.
// The controllable clock is the whole point — every timing rule in the link
// supervisor and the burst scheduler is a function of millis(), and on real
// hardware those rules can only be exercised by waiting in real time.
#ifndef TEST_ARDUINO_SHIM_H
#define TEST_ARDUINO_SHIM_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

typedef uint8_t byte;

// ---- test-controlled clock ----
extern uint32_t g_testMillis;
inline uint32_t millis() { return g_testMillis; }
inline void testSetMillis(uint32_t ms) { g_testMillis = ms; }
inline void testAdvance(uint32_t ms) { g_testMillis += ms; }

inline void delay(uint32_t) {}
inline void yield() {}

// ---- minimal Serial ----
struct SerialShim {
  void begin(int) {}
  void print(const char *s) { fputs(s, stdout); }
  void println(const char *s = "") { printf("%s\n", s); }
  void printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
  }
  void setTxTimeoutMs(unsigned long) {}
  void setTimeout(unsigned long) {}
};
extern SerialShim Serial;

struct EspShim {
  unsigned long long getEfuseMac() { return 0x112233445566ULL; }
  void restart() {}
};
extern EspShim ESP;

#endif // TEST_ARDUINO_SHIM_H
