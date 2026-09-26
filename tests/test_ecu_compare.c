#include "telemetry/ecu_compare.h"

#include <string.h>

#include "model/sync.h"
#include "test_util.h"

static EcuCompareTrends T;
static EcuCompareTrends U;

static void fill(ModelState *s, double rpm, double pilot, double cmd) {
  memset(s, 0, sizeof *s);
  ecu_init(&s->ecu);
  s->rpm = rpm;
  s->ecu.pilot_throttle = pilot;
  s->ecu.throttle_cmd = cmd;
  s->engine.run_state = ENGINE_RUNNING;
}

static void test_init_is_empty(void) {
  ecu_compare_init(&T);
  for (int m = 0; m < ECUC_COUNT; m++) {
    CHECK(history_count(&T.hist[m]) == 0);
  }
}

static void test_sample_maps_each_engine_and_scales_to_percent(void) {
  ecu_compare_init(&T);
  ModelState with_ecu, bare;
  fill(&with_ecu, 800.0, 0.0, 0.08);
  fill(&bare, 520.0, 0.0, 0.0);
  ecu_compare_sample(&T, &with_ecu, &bare);
  CHECK_NEAR(history_last(&T.hist[ECUC_RPM_ECU]), 800.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUC_RPM_BARE]), 520.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUC_THR_ECU]), 8.0, 1e-9);
  CHECK_NEAR(history_last(&T.hist[ECUC_THR_BARE]), 0.0, 1e-9);
}

static void test_wraps_at_capacity_and_snapshot_is_independent(void) {
  ecu_compare_init(&T);
  ModelState a, b;
  for (int i = 0; i < ECU_COMPARE_CAP + 10; i++) {
    fill(&a, 700.0 + i, 0.0, 0.0);
    fill(&b, 500.0 + i, 0.0, 0.0);
    ecu_compare_sample(&T, &a, &b);
  }
  for (int m = 0; m < ECUC_COUNT; m++) {
    CHECK(history_count(&T.hist[m]) == ECU_COMPARE_CAP);
  }
  CHECK_NEAR(history_at(&T.hist[ECUC_RPM_ECU], 0), 710.0, 1e-9);

  ecu_compare_snapshot(&U, &T);
  fill(&a, 1.0, 0.0, 0.0);
  fill(&b, 2.0, 0.0, 0.0);
  ecu_compare_sample(&T, &a, &b);
  CHECK_NEAR(history_last(&T.hist[ECUC_RPM_ECU]), 1.0, 1e-9);
  CHECK(history_last(&U.hist[ECUC_RPM_ECU]) > 100.0); /* the copy did not move */
}

static void test_summary_reports_what_the_ecu_is_doing(void) {
  ModelState with_ecu, bare;

  fill(&with_ecu, 800.0, 0.0, 0.076);
  fill(&bare, 506.0, 0.0, 0.0);
  EcuCompareSummary c = ecu_compare_summarize(&with_ecu, &bare);
  CHECK(c.compensating);
  CHECK_NEAR(c.extra_throttle_pct, 7.6, 1e-9);
  CHECK_NEAR(c.rpm_gain, 294.0, 1e-9);
  CHECK(!c.bare_stalled);

  /* the pilot in control: the ECU adds nothing */
  fill(&with_ecu, 2600.0, 0.8, 0.8);
  fill(&bare, 2600.0, 0.8, 0.8);
  c = ecu_compare_summarize(&with_ecu, &bare);
  CHECK(!c.compensating);
  CHECK_NEAR(c.extra_throttle_pct, 0.0, 1e-9);

  /* the bare engine has stalled while the ECU one carries on */
  fill(&with_ecu, 800.0, 0.0, 0.1);
  fill(&bare, 0.0, 0.0, 0.0);
  bare.engine.run_state = ENGINE_STOPPED;
  c = ecu_compare_summarize(&with_ecu, &bare);
  CHECK(c.bare_stalled);
  CHECK(c.compensating);

  /* both stopped is not "the ECU saved it" */
  with_ecu.engine.run_state = ENGINE_STOPPED;
  c = ecu_compare_summarize(&with_ecu, &bare);
  CHECK(!c.bare_stalled);
}

/* Two runs, same inputs, same fault, one with an ECU (as the app does it): the
 * ECU keeps a weak cylinder's idle up, and the comparison shows it. */
static void test_comparison_shows_the_ecu_hiding_a_weak_cylinder(void) {
  ModelSync a, b;
  model_sync_init(&a);
  a.engine_config.prop.cq_static = 0.0;
  b = a;
  b.engine_config.ecu_fitted = 0;
  a.cyl_config[1].compression_trim = 0.5; /* the same fault in both */
  b.cyl_config[1].compression_trim = 0.5;
  ModelState sa, sb;
  model_state_init(&sa, &a.engine_config, 15.0);
  model_state_init(&sb, &b.engine_config, 15.0);

  const EngineInput in = {0.0, 2.0, 101.325};
  const EnvInput env = {0.0, 0.0, 0.0};
  for (int i = 0; i < 2000; i++) { /* 40 s */
    model_sync_step(&a, &sa, &in, &env, 0.02);
    model_sync_step(&b, &sb, &in, &env, 0.02);
  }
  const EcuCompareSummary c = ecu_compare_summarize(&sa, &sb);
  CHECK(sa.engine.run_state == ENGINE_RUNNING);
  CHECK(sa.rpm > 600.0);
  CHECK(c.compensating);
  CHECK(c.rpm_gain > 150.0 || c.bare_stalled);
}

static const TestCase CASES[] = {
    {"ecu_compare.init_is_empty", test_init_is_empty},
    {"ecu_compare.sample_maps_each_engine_and_scales_to_percent",
     test_sample_maps_each_engine_and_scales_to_percent},
    {"ecu_compare.wraps_at_capacity_and_snapshot_is_independent",
     test_wraps_at_capacity_and_snapshot_is_independent},
    {"ecu_compare.summary_reports_what_the_ecu_is_doing",
     test_summary_reports_what_the_ecu_is_doing},
    {"ecu_compare.comparison_shows_the_ecu_hiding_a_weak_cylinder",
     test_comparison_shows_the_ecu_hiding_a_weak_cylinder},
};

RUN_TESTS(CASES)
