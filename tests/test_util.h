#ifndef TESTS_TEST_UTIL_H
#define TESTS_TEST_UTIL_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Dead-simple headless test harness -- no framework, no dependencies.
 *
 * Each test_*.c defines a set of void(void) test functions, lists them in a
 * TestCase table, and ends with RUN_TESTS(table). CHECK / CHECK_NEAR record
 * failures (with file:line) without aborting, so one run reports every
 * broken expectation. The process exits non-zero if anything failed, which
 * is all CTest needs. */

static int g_test_failures;
static int g_test_checks;

#define CHECK(cond)                                                             \
  do {                                                                         \
    g_test_checks++;                                                           \
    if (!(cond)) {                                                             \
      g_test_failures++;                                                       \
      fprintf(stderr, "  FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                       \
  do {                                                                         \
    g_test_checks++;                                                           \
    double _a = (actual), _e = (expected), _t = (tol);                        \
    if (!(fabs(_a - _e) <= _t)) {                                             \
      g_test_failures++;                                                       \
      fprintf(stderr,                                                          \
              "  FAIL %s:%d  CHECK_NEAR(%s, %s, %s): |%.9g - %.9g| = %.3g > "  \
              "%.3g\n",                                                        \
              __FILE__, __LINE__, #actual, #expected, #tol, _a, _e,           \
              fabs(_a - _e), _t);                                              \
    }                                                                          \
  } while (0)

typedef struct {
  const char *name;
  void (*fn)(void);
} TestCase;

#define RUN_TESTS(cases)                                                        \
  int main(void) {                                                             \
    size_t _n = sizeof(cases) / sizeof((cases)[0]);                           \
    for (size_t _i = 0; _i < _n; _i++) {                                      \
      int _before = g_test_failures;                                          \
      (cases)[_i].fn();                                                        \
      printf("  %-48s %s\n", (cases)[_i].name,                                \
             g_test_failures == _before ? "ok" : "FAILED");                   \
    }                                                                          \
    printf("%d checks, %d failure(s)\n", g_test_checks, g_test_failures);     \
    return g_test_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;                \
  }

#endif /* TESTS_TEST_UTIL_H */
