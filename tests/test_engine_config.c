#include <stdio.h>
#include <string.h>

#include "model/sync.h"
#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"
#include "test_util.h"

#define TMP_PATH "test_engine_config_tmp.cfg"

static void test_default_config_has_no_issues(void) {
  EngineConfig cfg = engine_config_default();
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  CHECK(engine_config_check(&cfg, msgs, ENGINE_CONFIG_MAX_ISSUES) == 0);
  CHECK(engine_config_check(&cfg, NULL, 0) == 0); /* NULL msgs just counts */
  CHECK(engine_config_validate(&cfg, NULL) == 0);
}

static void test_check_reports_each_problem_with_a_message(void) {
  EngineConfig cfg = engine_config_default();
  cfg.inertia_kg_m2 = -1.0;
  cfg.geom.compression_ratio = 0.5;
  cfg.starter_torque_nm = 0.0;

  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  const int n = engine_config_check(&cfg, msgs, ENGINE_CONFIG_MAX_ISSUES);
  CHECK(n == 3);
  CHECK(strstr(msgs[0], "inertia_kg_m2") != NULL);
  CHECK(strstr(msgs[1], "starter_torque_nm") != NULL);
  CHECK(strstr(msgs[2], "compression_ratio") != NULL);
}

static void test_check_count_can_exceed_message_capacity(void) {
  EngineConfig cfg = engine_config_default();
  cfg.inertia_kg_m2 = 0.0;
  cfg.map_tau_s = 0.0;
  cfg.friction_coeff_nm_per_rad_s = 0.0;

  char msgs[1][ENGINE_CONFIG_ISSUE_LEN];
  CHECK(engine_config_check(&cfg, msgs, 1) == 3); /* full count, 1 stored */
  CHECK(strstr(msgs[0], "inertia_kg_m2") != NULL);
}

static void test_bad_firing_order_is_reported(void) {
  EngineConfig cfg = engine_config_default(); /* 4 cyl: 1,3,4,2 */
  cfg.firing_order[1] = 1;                    /* repeats cylinder 1 */
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  CHECK(engine_config_check(&cfg, msgs, ENGINE_CONFIG_MAX_ISSUES) >= 1);
  CHECK(strstr(msgs[0], "firing_order[1]") != NULL);

  cfg = engine_config_default();
  cfg.num_cylinders = 9;
  CHECK(engine_config_check(&cfg, msgs, ENGINE_CONFIG_MAX_ISSUES) >= 1);
  CHECK(strstr(msgs[0], "num_cylinders") != NULL);
}

static void test_validate_prints_the_same_messages(void) {
  EngineConfig cfg = engine_config_default();
  cfg.map_tau_s = -2.0;

  FILE *f = fopen(TMP_PATH, "w");
  CHECK(f != NULL);
  if (!f) {
    return;
  }
  CHECK(engine_config_validate(&cfg, f) == 1);
  fclose(f);

  char line[256] = "";
  f = fopen(TMP_PATH, "r");
  CHECK(f != NULL);
  if (f) {
    CHECK(fgets(line, sizeof line, f) != NULL);
    fclose(f);
  }
  CHECK(strstr(line, "map_tau_s = -2: must be positive") != NULL);
  CHECK(line[strlen(line) - 1] == '\n');
  remove(TMP_PATH);
}

static void test_default_firing_orders_are_valid_for_every_count(void) {
  for (int n = 1; n <= ENGINE_MAX_CYLINDERS; n++) {
    EngineConfig cfg = engine_config_default();
    cfg.num_cylinders = n;
    engine_default_firing_order(n, cfg.firing_order);
    CHECK(engine_config_check(&cfg, NULL, 0) == 0);
  }
  int order[ENGINE_MAX_CYLINDERS];
  engine_default_firing_order(4, order);
  CHECK(order[0] == 1 && order[1] == 3 && order[2] == 4 && order[3] == 2);
  engine_default_firing_order(6, order);
  CHECK(order[0] == 1 && order[1] == 5 && order[2] == 3 && order[3] == 6 &&
        order[4] == 2 && order[5] == 4);
}

static void test_default_firing_order_clamps_bad_counts(void) {
  int order[ENGINE_MAX_CYLINDERS];
  engine_default_firing_order(0, order);
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    CHECK(order[i] == 0);
  }
  engine_default_firing_order(99, order); /* clamps to the largest supported */
  CHECK(order[5] == 4);
}

static void test_derived_values(void) {
  EngineConfig cfg = engine_config_default();
  const EngineDerived d = engine_config_derived(&cfg);
  /* 84 mm bore x 90 mm stroke -> 0.4987 L/cyl, 1.995 L for four */
  CHECK_NEAR(d.displacement_per_cyl_l, 0.49875, 1e-3);
  CHECK_NEAR(d.total_displacement_l, 1.995, 1e-3);
  CHECK_NEAR(d.firing_interval_deg, 180.0, 1e-9);
  CHECK_NEAR(d.bore_stroke_ratio, 0.084 / 0.090, 1e-9);
  CHECK_NEAR(d.rod_ratio, 0.150 / 0.090, 1e-9);
  CHECK_NEAR(d.piston_speed_3000rpm_ms, 2.0 * 0.090 * 50.0, 1e-9); /* 9 m/s */
  CHECK(d.clearance_cc > 0.0);

  cfg.num_cylinders = 6;
  CHECK_NEAR(engine_config_derived(&cfg).total_displacement_l, 6.0 * 0.49875,
             1e-3);
}

static void test_derived_is_safe_on_a_broken_config(void) {
  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = 0;
  cfg.geom.stroke_m = 0.0;
  cfg.geom.compression_ratio = 1.0;
  const EngineDerived d = engine_config_derived(&cfg); /* no NaN / crash */
  CHECK_NEAR(d.firing_interval_deg, 0.0, 1e-12);
  CHECK_NEAR(d.rod_ratio, 0.0, 1e-12);
  CHECK_NEAR(d.clearance_cc, 0.0, 1e-12);
  CHECK_NEAR(d.total_displacement_l, 0.0, 1e-12);
}

static void test_starter_fields_round_trip_through_a_spec_file(void) {
  EngineConfig cfg = engine_config_default();
  cfg.starter_torque_nm = 33.5;
  cfg.starter_catch_rpm = 640.0;
  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 0);
  CHECK_NEAR(loaded.starter_torque_nm, 33.5, 1e-9);
  CHECK_NEAR(loaded.starter_catch_rpm, 640.0, 1e-9);
  remove(TMP_PATH);
}

static void test_apply_engine_config_follows_geometry(void) {
  ModelSync sync;
  model_sync_init(&sync);
  CHECK_NEAR(sync.fuel_config.displacement_l, 2.0, 1e-12); /* the placeholder */

  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = 6;
  engine_default_firing_order(6, cfg.firing_order);
  model_sync_apply_engine_config(&sync, &cfg);

  CHECK(sync.engine_config.num_cylinders == 6);
  CHECK_NEAR(sync.fuel_config.displacement_l,
             engine_config_derived(&cfg).total_displacement_l, 1e-9);
  CHECK(sync.fuel_config.displacement_l > 2.5); /* six cylinders, not four */
}

static void test_apply_engine_config_allows_aliasing(void) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.engine_config.num_cylinders = 2;
  engine_default_firing_order(2, sync.engine_config.firing_order);
  model_sync_apply_engine_config(&sync, &sync.engine_config);
  CHECK(sync.engine_config.num_cylinders == 2);
  CHECK_NEAR(sync.fuel_config.displacement_l, 2.0 * 0.49875, 1e-3);
}

static const TestCase cases[] = {
    {"default config has no issues", test_default_config_has_no_issues},
    {"check reports each problem", test_check_reports_each_problem_with_a_message},
    {"issue count can exceed capacity",
     test_check_count_can_exceed_message_capacity},
    {"bad firing order is reported", test_bad_firing_order_is_reported},
    {"validate prints the same messages", test_validate_prints_the_same_messages},
    {"default firing orders are valid",
     test_default_firing_orders_are_valid_for_every_count},
    {"default firing order clamps counts",
     test_default_firing_order_clamps_bad_counts},
    {"derived values", test_derived_values},
    {"derived is safe on broken config", test_derived_is_safe_on_a_broken_config},
    {"starter fields round trip", test_starter_fields_round_trip_through_a_spec_file},
    {"apply config follows geometry", test_apply_engine_config_follows_geometry},
    {"apply config allows aliasing", test_apply_engine_config_allows_aliasing},
};

RUN_TESTS(cases)
