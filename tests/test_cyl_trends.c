#include "telemetry/cyl_trends.h"

#include <string.h>

#include "test_util.h"

static CylTrends T;
static CylTrends U;

static void fill_state(ModelState *s, double base) {
  memset(s, 0, sizeof *s);
  for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
    s->cyl[c].cht_c = base + c;
    s->cyl[c].egt_c = 500.0 + base + c;
    s->cyl[c].lambda = 1.0 + 0.01 * c;
    s->cyl[c].misfire_rate = 0.1 * c; /* 0 .. 0.5 */
  }
}

static void test_init_is_empty(void) {
  cyl_trends_init(&T);
  for (int m = 0; m < CYLM_COUNT; m++) {
    for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
      CHECK(history_count(&T.hist[m][c]) == 0);
    }
  }
}

static void test_sample_maps_each_cylinder_and_metric(void) {
  cyl_trends_init(&T);
  ModelState s;
  fill_state(&s, 100.0);
  cyl_trends_sample(&T, &s, 4);

  CHECK_NEAR(history_last(&T.hist[CYLM_CHT][0]), 100.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[CYLM_CHT][3]), 103.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[CYLM_EGT][2]), 602.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[CYLM_LAMBDA][3]), 1.03, 1e-9);
  CHECK_NEAR(history_last(&T.hist[CYLM_MISFIRE][2]), 20.0, 1e-9); /* fraction -> % */
}

static void test_unused_cylinders_stay_empty(void) {
  cyl_trends_init(&T);
  ModelState s;
  fill_state(&s, 100.0);
  cyl_trends_sample(&T, &s, 4);

  CHECK(history_count(&T.hist[CYLM_CHT][3]) == 1);
  CHECK(history_count(&T.hist[CYLM_CHT][4]) == 0);
  CHECK(history_count(&T.hist[CYLM_MISFIRE][5]) == 0);
}

static void test_cylinder_count_is_clamped(void) {
  cyl_trends_init(&T);
  ModelState s;
  fill_state(&s, 100.0);
  cyl_trends_sample(&T, &s, 99);
  CHECK(history_count(&T.hist[CYLM_CHT][ENGINE_MAX_CYLINDERS - 1]) == 1);
  cyl_trends_sample(&T, &s, -5); /* nothing sampled, must not crash */
  CHECK(history_count(&T.hist[CYLM_CHT][0]) == 1);
}

static void test_series_wrap_at_capacity(void) {
  cyl_trends_init(&T);
  ModelState s;
  for (int i = 0; i < CYL_TRENDS_CAP + 5; i++) {
    fill_state(&s, (double)i);
    cyl_trends_sample(&T, &s, 2);
  }
  CHECK(history_count(&T.hist[CYLM_CHT][0]) == CYL_TRENDS_CAP);
  CHECK_NEAR(history_at(&T.hist[CYLM_CHT][0], 0), 5.0, 1e-9);
}

static void test_snapshot_is_independent(void) {
  cyl_trends_init(&T);
  ModelState s;
  fill_state(&s, 100.0);
  cyl_trends_sample(&T, &s, 3);
  fill_state(&s, 200.0);
  cyl_trends_sample(&T, &s, 3);

  cyl_trends_snapshot(&U, &T);
  CHECK(history_count(&U.hist[CYLM_EGT][1]) == 2);
  CHECK_NEAR(history_at(&U.hist[CYLM_EGT][1], 0), 601.0, 1e-9);
  CHECK_NEAR(history_at(&U.hist[CYLM_EGT][1], 1), 701.0, 1e-9);

  /* later samples to the source don't touch the snapshot */
  fill_state(&s, 300.0);
  cyl_trends_sample(&T, &s, 3);
  CHECK(history_count(&T.hist[CYLM_CHT][0]) == 3);
  CHECK(history_count(&U.hist[CYLM_CHT][0]) == 2);
}

static const TestCase cases[] = {
    {"init is empty", test_init_is_empty},
    {"sample maps cylinder and metric", test_sample_maps_each_cylinder_and_metric},
    {"unused cylinders stay empty", test_unused_cylinders_stay_empty},
    {"cylinder count is clamped", test_cylinder_count_is_clamped},
    {"series wrap at capacity", test_series_wrap_at_capacity},
    {"snapshot is independent", test_snapshot_is_independent},
};

RUN_TESTS(cases)
