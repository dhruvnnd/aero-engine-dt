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

/* Phase 1's EngineGeometry fields must round-trip through save/load too --
 * this is the gap flagged after Phase 1 landed (plan's HOW step 5 called for
 * extending the schema in the same pass, which didn't happen until now). */
static void test_geometry_fields_round_trip(void) {
  EngineConfig cfg = engine_config_default();
  cfg.geom.bore_m = 0.081;
  cfg.geom.stroke_m = 0.086;
  cfg.geom.conrod_len_m = 0.145;
  cfg.geom.compression_ratio = 10.2;
  cfg.geom.evo_deg = 125.0;
  cfg.geom.ivc_deg = 585.0;
  cfg.geom.m_recip_kg = 0.42;
  cfg.geom.wiebe_a = 4.5;
  cfg.geom.wiebe_m = 2.3;
  cfg.geom.delta_theta_burn_deg = 45.0;
  cfg.geom.spark_base_btdc_deg = 12.0;
  cfg.geom.spark_rpm_gain_deg_per_1000rpm = 5.5;
  cfg.geom.spark_map_retard_deg_per_kpa = 0.12;
  cfg.geom.combustion_efficiency = 0.28;

  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 0);

  CHECK_NEAR(loaded.geom.bore_m, cfg.geom.bore_m, 1e-9);
  CHECK_NEAR(loaded.geom.stroke_m, cfg.geom.stroke_m, 1e-9);
  CHECK_NEAR(loaded.geom.conrod_len_m, cfg.geom.conrod_len_m, 1e-9);
  CHECK_NEAR(loaded.geom.compression_ratio, cfg.geom.compression_ratio, 1e-9);
  CHECK_NEAR(loaded.geom.evo_deg, cfg.geom.evo_deg, 1e-9);
  CHECK_NEAR(loaded.geom.ivc_deg, cfg.geom.ivc_deg, 1e-9);
  CHECK_NEAR(loaded.geom.m_recip_kg, cfg.geom.m_recip_kg, 1e-9);
  CHECK_NEAR(loaded.geom.wiebe_a, cfg.geom.wiebe_a, 1e-9);
  CHECK_NEAR(loaded.geom.wiebe_m, cfg.geom.wiebe_m, 1e-9);
  CHECK_NEAR(loaded.geom.delta_theta_burn_deg, cfg.geom.delta_theta_burn_deg,
             1e-9);
  CHECK_NEAR(loaded.geom.spark_base_btdc_deg, cfg.geom.spark_base_btdc_deg,
             1e-9);
  CHECK_NEAR(loaded.geom.spark_rpm_gain_deg_per_1000rpm,
             cfg.geom.spark_rpm_gain_deg_per_1000rpm, 1e-9);
  CHECK_NEAR(loaded.geom.spark_map_retard_deg_per_kpa,
             cfg.geom.spark_map_retard_deg_per_kpa, 1e-9);
  CHECK_NEAR(loaded.geom.combustion_efficiency, cfg.geom.combustion_efficiency,
             1e-9);

  remove(TMP_PATH);
}

/* Phase 4: the propeller lives in the same spec file. */
static void test_prop_fields_round_trip(void) {
  EngineConfig cfg = engine_config_default();
  cfg.prop.diameter_m = 1.55;
  cfg.prop.j_zero_thrust = 0.95;
  cfg.prop.ct_static = 0.115;
  cfg.prop.cq_static = 0.0082;
  cfg.prop.cq_unload = 0.35;

  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 0);
  CHECK_NEAR(loaded.prop.diameter_m, 1.55, 1e-9);
  CHECK_NEAR(loaded.prop.j_zero_thrust, 0.95, 1e-9);
  CHECK_NEAR(loaded.prop.ct_static, 0.115, 1e-9);
  CHECK_NEAR(loaded.prop.cq_static, 0.0082, 1e-9);
  CHECK_NEAR(loaded.prop.cq_unload, 0.35, 1e-9);

  remove(TMP_PATH);
}

/* A spec written before the propeller existed must still load, with the
 * default propeller. */
static void test_spec_without_prop_keys_gets_the_default_prop(void) {
  write_raw("num_cylinders = 4\nfiring_order = 1,3,4,2\ninertia_kg_m2 = 0.7\n");

  EngineConfig defaults = engine_config_default();
  EngineConfig loaded;
  EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK_NEAR(loaded.prop.diameter_m, defaults.prop.diameter_m, 1e-12);
  CHECK_NEAR(loaded.prop.cq_static, defaults.prop.cq_static, 1e-12);
  CHECK_NEAR(loaded.inertia_kg_m2, 0.7, 1e-9);

  remove(TMP_PATH);
}

static void test_validate_flags_bad_prop_values(void) {
  EngineConfig cfg = engine_config_default();
  cfg.prop.diameter_m = 0.0;
  cfg.prop.cq_unload = 1.5;
  CHECK(engine_config_validate(&cfg, NULL) >= 2);

  cfg = engine_config_default();
  cfg.prop.cq_static = 0.0; /* "no propeller" is allowed */
  CHECK(engine_config_validate(&cfg, NULL) == 0);
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
    {"engine_spec_io.geometry_fields_round_trip",
     test_geometry_fields_round_trip},
    {"engine_spec_io.prop_fields_round_trip", test_prop_fields_round_trip},
    {"engine_spec_io.spec_without_prop_keys_gets_the_default_prop",
     test_spec_without_prop_keys_gets_the_default_prop},
    {"engine_spec_io.validate_flags_bad_prop_values",
     test_validate_flags_bad_prop_values},
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
