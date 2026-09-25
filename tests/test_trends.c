#include "telemetry/trends.h"

#include <string.h>

#include "test_util.h"

static void test_init_is_empty(void) {
  Trends t;
  trends_init(&t);
  CHECK(history_count(&t.rpm) == 0);
  CHECK(history_count(&t.cht) == 0);
  CHECK(history_count(&t.egt) == 0);
  CHECK(history_count(&t.oil_press) == 0);
  CHECK(history_count(&t.batt_pct) == 0);
}

static void test_sample_maps_each_channel(void) {
  Trends t;
  trends_init(&t);

  ModelState s;
  memset(&s, 0, sizeof s);
  s.rpm = 2200.0;
  s.thermal.cht_c = 150.0;
  s.thermal.egt_c = 610.0;
  s.lube.oil_press_kpa = 420.0;
  s.elec.batt_soc = 0.85;
  trends_sample(&t, &s);

  CHECK_NEAR(history_last(&t.rpm), 2200.0, 1e-9);
  CHECK_NEAR(history_last(&t.cht), 150.0, 1e-9);
  CHECK_NEAR(history_last(&t.egt), 610.0, 1e-9);
  CHECK_NEAR(history_last(&t.oil_press), 420.0, 1e-9);
  CHECK_NEAR(history_last(&t.batt_pct), 85.0, 1e-9); /* fraction -> percent */
}

static void test_series_wrap_at_capacity(void) {
  Trends t;
  trends_init(&t);

  ModelState s;
  memset(&s, 0, sizeof s);
  for (int i = 0; i < TRENDS_CAP + 5; i++) {
    s.rpm = (double)i;
    trends_sample(&t, &s);
  }
  CHECK(history_count(&t.rpm) == TRENDS_CAP);
  CHECK_NEAR(history_at(&t.rpm, 0), 5.0, 1e-9); /* oldest 5 evicted */
  CHECK_NEAR(history_last(&t.rpm), (double)(TRENDS_CAP + 4), 1e-9);
}

static const TestCase cases[] = {
    {"init is empty", test_init_is_empty},
    {"sample maps each channel", test_sample_maps_each_channel},
    {"series wrap at capacity", test_series_wrap_at_capacity},
};

RUN_TESTS(cases)
