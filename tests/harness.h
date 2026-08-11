#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H
#include <cstdio>
#include <cstdlib>

static int g_checks = 0, g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                 \
    }                                                                          \
  } while (0)

#define CHECK_EQ(a, b)                                                         \
  do {                                                                         \
    g_checks++;                                                                \
    long long va = (long long)(a), vb = (long long)(b);                         \
    if (va != vb) {                                                            \
      g_fails++;                                                               \
      printf("  FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__, __LINE__,   \
             #a, #b, va, vb);                                                   \
    }                                                                          \
  } while (0)

#define CASE(name) printf("- %s\n", name)

static inline int testSummary(const char *suite) {
  printf("%s: %d checks, %d failures\n", suite, g_checks, g_fails);
  return g_fails ? 1 : 0;
}
#endif
