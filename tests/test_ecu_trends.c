#include "telemetry/ecu_trends.h"

#include <string.h>

#include "test_util.h"

static EcuTrends T;
static EcuTrends U;

static void fill_state(ModelState *s, double rpm, double pilot) {
  memset(s, 0, sizeof *s);
  ecu_init(&s->ecu);
  s->rpm = rpm;
  EcuState *e = &s->ecu;
  e->idle_target_rpm = 800.0;
  e->pilot_throttle = pilot;
  e->idle_throttle = 0.08;
  e->throttle_cmd = pilot > 0.08 ? pilot : 0.08;
  e->idle_p_term = 0.03;
  e->idle_i_term = 0.05;
}

static void test_init_is_empty(void) {
  ecu_trends_init(&T);
  for (int m = 0; m < ECUM_COUNT; m++) {
    CHECK(history_count(&T.hist[m]) == 0);
  }
}

static void test_sample_maps_each_channel_and_scales_to_percent(void) {
  ecu_trends_init(&T);
  ModelState s;
  fill_state(&s, 765.0, 0.02);
  ecu_trends_sample(&T, &s);

  CHECK_NEAR(history_last(&T.hist[ECUM_RPM]), 765.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_TARGET]), 800.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_PILOT]), 2.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_GOVERNOR]), 8.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_COMMAND]), 8.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_P_TERM]), 3.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUM_I_TERM]), 5.0, 1e-9);
}

static void test_series_advance_together_and_wrap_at_capacity(void) {
  ecu_trends_init(&T);
  ModelState s;
  for (int i = 0; i < ECU_TRENDS_CAP + 25; i++) {
    fill_state(&s, 700.0 + i, 0.0);
    ecu_trends_sample(&T, &s);
  }
  for (int m = 0; m < ECUM_COUNT; m++) {
    CHECK(history_count(&T.hist[m]) == ECU_TRENDS_CAP);
  }
  CHECK_NEAR(history_last(&T.hist[ECUM_RPM]), 700.0 + ECU_TRENDS_CAP + 24, 1e-9);
  CHECK_NEAR(history_at(&T.hist[ECUM_RPM], 0), 700.0 + 25, 1e-9); /* oldest kept */
}

static void test_snapshot_is_independent(void) {
  ecu_trends_init(&T);
  ModelState s;
  for (int i = 0; i < 5; i++) {
    fill_state(&s, 800.0 + i, 0.0);
    ecu_trends_sample(&T, &s);
  }
  ecu_trends_snapshot(&U, &T);
  for (int m = 0; m < ECUM_COUNT; m++) {
    CHECK(history_count(&U.hist[m]) == history_count(&T.hist[m]));
  }
  CHECK_NEAR(history_last(&U.hist[ECUM_RPM]), 804.0, 1e-9);

  fill_state(&s, 999.0, 0.0);
  ecu_trends_sample(&T, &s); /* the original moves on... */
  CHECK_NEAR(history_last(&T.hist[ECUM_RPM]), 999.0, 1e-9);
  CHECK(history_count(&U.hist[ECUM_RPM]) == 5); /* ...the copy does not */
  CHECK_NEAR(history_last(&U.hist[ECUM_RPM]), 804.0, 1e-9);
}

static const TestCase CASES[] = {
    {"ecu_trends.init_is_empty", test_init_is_empty},
    {"ecu_trends.sample_maps_each_channel_and_scales_to_percent",
     test_sample_maps_each_channel_and_scales_to_percent},
    {"ecu_trends.series_advance_together_and_wrap_at_capacity",
     test_series_advance_together_and_wrap_at_capacity},
    {"ecu_trends.snapshot_is_independent", test_snapshot_is_independent},
};

RUN_TESTS(CASES)
