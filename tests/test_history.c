#include "util/history.h"

#include "test_util.h"

static void test_empty(void) {
  double buf[4];
  History h;
  history_init(&h, buf, 4);

  CHECK(history_count(&h) == 0);
  CHECK_NEAR(history_last(&h), 0.0, 1e-9);
  CHECK_NEAR(history_min(&h), 0.0, 1e-9);
  CHECK_NEAR(history_max(&h), 0.0, 1e-9);
  CHECK_NEAR(history_at(&h, 0), 0.0, 1e-9);
  CHECK_NEAR(history_at(&h, -1), 0.0, 1e-9);
}

static void test_fill_then_wrap(void) {
  double buf[3];
  History h;
  history_init(&h, buf, 3);

  history_push(&h, 1.0);
  history_push(&h, 2.0);
  CHECK(history_count(&h) == 2);
  CHECK_NEAR(history_at(&h, 0), 1.0, 1e-9); /* oldest */
  CHECK_NEAR(history_at(&h, 1), 2.0, 1e-9); /* newest */
  CHECK_NEAR(history_last(&h), 2.0, 1e-9);

  history_push(&h, 3.0); /* now full: [1,2,3] */
  CHECK(history_count(&h) == 3);

  history_push(&h, 4.0); /* evict oldest: [2,3,4] */
  CHECK(history_count(&h) == 3);
  CHECK_NEAR(history_at(&h, 0), 2.0, 1e-9);
  CHECK_NEAR(history_at(&h, 1), 3.0, 1e-9);
  CHECK_NEAR(history_at(&h, 2), 4.0, 1e-9);
  CHECK_NEAR(history_last(&h), 4.0, 1e-9);

  /* out-of-range age index */
  CHECK_NEAR(history_at(&h, 3), 0.0, 1e-9);
}

static void test_min_max(void) {
  double buf[5];
  History h;
  history_init(&h, buf, 5);

  history_push(&h, 3.0);
  history_push(&h, -1.0);
  history_push(&h, 7.5);
  history_push(&h, 2.0);

  CHECK_NEAR(history_min(&h), -1.0, 1e-9);
  CHECK_NEAR(history_max(&h), 7.5, 1e-9);

  /* after wrap the min/max only see retained samples */
  history_push(&h, 0.0);
  history_push(&h, 0.0); /* evicts the 3.0 */
  history_push(&h, 0.0); /* evicts the -1.0 */
  CHECK_NEAR(history_max(&h), 7.5, 1e-9);
  CHECK_NEAR(history_min(&h), 0.0, 1e-9);
}

static void test_clear(void) {
  double buf[4];
  History h;
  history_init(&h, buf, 4);
  history_push(&h, 9.0);
  history_push(&h, 8.0);
  history_clear(&h);
  CHECK(history_count(&h) == 0);
  CHECK_NEAR(history_last(&h), 0.0, 1e-9);

  history_push(&h, 5.0);
  CHECK(history_count(&h) == 1);
  CHECK_NEAR(history_at(&h, 0), 5.0, 1e-9);
}

static void test_zero_capacity_is_safe(void) {
  History h;
  history_init(&h, NULL, 0);
  history_push(&h, 1.0); /* must not crash / write */
  CHECK(history_count(&h) == 0);
}

static const TestCase kCases[] = {
    {"empty", test_empty},
    {"fill_then_wrap", test_fill_then_wrap},
    {"min_max", test_min_max},
    {"clear", test_clear},
    {"zero_capacity_is_safe", test_zero_capacity_is_safe},
};

RUN_TESTS(kCases)
