#include "test_util.h"

#include <stdio.h>

#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"

#define TMP_PATH "test_engine_spec_io_tmp.cfg"

static void write_raw(const char *content) {
  FILE *f = fopen(TMP_PATH, "w");
  fputs(content, f);
  fclose(f);
}

static void test_save_then_load_round_trips(void) {
  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = 6;
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg.firing_order[i] = 0;
  }
  int order[] = {1, 5, 3, 6, 2, 4};
  for (int i = 0; i < 6; i++) {
    cfg.firing_order[i] = order[i];
  }
  cfg.inertia_kg_m2 = 0.9;
  cfg.map_tau_s = 0.31;
  cfg.friction_coeff_nm_per_rad_s = 0.17;

  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 0);

  CHECK(loaded.num_cylinders == cfg.num_cylinders);
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    CHECK(loaded.firing_order[i] == cfg.firing_order[i]);
  }
  CHECK_NEAR(loaded.inertia_kg_m2, cfg.inertia_kg_m2, 1e-9);
  CHECK_NEAR(loaded.map_tau_s, cfg.map_tau_s, 1e-9);
  CHECK_NEAR(loaded.friction_coeff_nm_per_rad_s,
             cfg.friction_coeff_nm_per_rad_s, 1e-9);

  remove(TMP_PATH);
}

static void test_missing_keys_fall_back_to_default(void) {
  write_raw("# only overriding one field\ninertia_kg_m2 = 0.95\n");

  EngineConfig defaults = engine_config_default();
  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);

  CHECK_NEAR(loaded.inertia_kg_m2, 0.95, 1e-9);
  CHECK(loaded.num_cylinders == defaults.num_cylinders);
  CHECK_NEAR(loaded.map_tau_s, defaults.map_tau_s, 1e-9);
  CHECK_NEAR(loaded.friction_coeff_nm_per_rad_s,
             defaults.friction_coeff_nm_per_rad_s, 1e-9);

  remove(TMP_PATH);
}

static void test_open_failure_falls_back_to_default(void) {
  EngineConfig defaults = engine_config_default();
  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load("no_such_file_xyz.cfg", &loaded);

  CHECK(r.status == ENGINE_SPEC_ERR_OPEN);
  CHECK_NEAR(loaded.inertia_kg_m2, defaults.inertia_kg_m2, 1e-9);
  CHECK(loaded.num_cylinders == defaults.num_cylinders);
}

static void test_parse_error_on_malformed_line(void) {
  write_raw("inertia_kg_m2 = 0.5\nthis line has no equals sign\n");

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_ERR_PARSE);
  CHECK(r.error_line == 2);

  remove(TMP_PATH);
}

static void test_parse_error_on_unparsable_value(void) {
  write_raw("num_cylinders = four\n");

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_ERR_PARSE);
  CHECK(r.error_line == 1);

  remove(TMP_PATH);
}

static void test_unknown_key_is_soft_warning_not_fatal(void) {
  write_raw("totally_bogus_key = 5\ninertia_kg_m2 = 0.7\n");

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 1);
  CHECK_NEAR(loaded.inertia_kg_m2, 0.7, 1e-9);

  remove(TMP_PATH);
}

static void test_firing_order_list_parses(void) {
  write_raw("num_cylinders = 4\nfiring_order = 1,3,4,2\n");

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(loaded.firing_order[0] == 1);
  CHECK(loaded.firing_order[1] == 3);
  CHECK(loaded.firing_order[2] == 4);
  CHECK(loaded.firing_order[3] == 2);

  remove(TMP_PATH);
}

static void test_validate_accepts_default_config(void) {
  EngineConfig cfg = engine_config_default();
  CHECK(engine_config_validate(&cfg, NULL) == 0);
}

static void test_validate_flags_bad_firing_order(void) {
  EngineConfig cfg = engine_config_default();
  cfg.firing_order[1] = cfg.firing_order[0]; /* duplicate */
  CHECK(engine_config_validate(&cfg, NULL) > 0);
}

static void test_validate_flags_non_positive_fields(void) {
  EngineConfig cfg = engine_config_default();
  cfg.inertia_kg_m2 = 0.0;
  cfg.map_tau_s = -1.0;
  CHECK(engine_config_validate(&cfg, NULL) >= 2);
}

static const TestCase CASES[] = {
    {"engine_spec_io.save_then_load_round_trips",
     test_save_then_load_round_trips},
    {"engine_spec_io.missing_keys_fall_back_to_default",
     test_missing_keys_fall_back_to_default},
    {"engine_spec_io.open_failure_falls_back_to_default",
     test_open_failure_falls_back_to_default},
    {"engine_spec_io.parse_error_on_malformed_line",
     test_parse_error_on_malformed_line},
    {"engine_spec_io.parse_error_on_unparsable_value",
     test_parse_error_on_unparsable_value},
    {"engine_spec_io.unknown_key_is_soft_warning_not_fatal",
     test_unknown_key_is_soft_warning_not_fatal},
    {"engine_spec_io.firing_order_list_parses",
     test_firing_order_list_parses},
    {"engine_spec_io.validate_accepts_default_config",
     test_validate_accepts_default_config},
    {"engine_spec_io.validate_flags_bad_firing_order",
     test_validate_flags_bad_firing_order},
    {"engine_spec_io.validate_flags_non_positive_fields",
     test_validate_flags_non_positive_fields},
};

RUN_TESTS(CASES)
