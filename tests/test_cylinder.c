#include "test_util.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/cylinder.h"
#include "physics/engine_model.h"

static void test_config_default_is_nominal(void) {
  CylinderConfig c = cylinder_config_default();
  CHECK_NEAR(c.injector_flow_trim, 1.0, 0.0);
  CHECK_NEAR(c.compression_trim, 1.0, 0.0);
  CHECK_NEAR(c.spark_offset_deg, 0.0, 0.0);
  CHECK_NEAR(c.intake_leak_frac, 0.0, 0.0);
  CHECK_NEAR(c.cooling_trim, 1.0, 0.0);
}

static void test_state_init_cold_start(void) {
  CylinderState s;
  cylinder_state_init(&s, 15.0);
  CHECK_NEAR(s.cht_c, 15.0, 0.0);
  CHECK_NEAR(s.egt_c, 15.0, 0.0);
  CHECK_NEAR(s.lambda, 1.0, 0.0);
  CHECK_NEAR(s.imep_bar, 0.0, 0.0);
  CHECK_NEAR(s.ca50_deg, 0.0, 0.0);
  CHECK_NEAR(s.fuel_pw_ms, 0.0, 0.0);
  CHECK_NEAR(s.misfire_rate, 0.0, 0.0);
}

static void test_engine_config_geometry(void) {
  EngineConfig cfg = engine_config_default();
  CHECK(cfg.num_cylinders == 4);
  CHECK(cfg.num_cylinders <= ENGINE_MAX_CYLINDERS);

  /* firing_order[0..num_cylinders) is a permutation of 1..num_cylinders */
  int seen[ENGINE_MAX_CYLINDERS + 1] = {0};
  for (int i = 0; i < cfg.num_cylinders; i++) {
    int n = cfg.firing_order[i];
    CHECK(n >= 1 && n <= cfg.num_cylinders);
    if (n >= 1 && n <= ENGINE_MAX_CYLINDERS) {
      seen[n]++;
    }
  }
  for (int n = 1; n <= cfg.num_cylinders; n++) {
    CHECK(seen[n] == 1);
  }
  /* trailing slots left zero */
  for (int i = cfg.num_cylinders; i < ENGINE_MAX_CYLINDERS; i++) {
    CHECK(cfg.firing_order[i] == 0);
  }
}

static void test_model_wiring_populates_cylinders(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 20.0);

  for (int i = 0; i < sync.engine_config.num_cylinders; i++) {
    CHECK_NEAR(st.cyl[i].cht_c, 20.0, 0.0);
    CHECK_NEAR(st.cyl[i].egt_c, 20.0, 0.0);
    CHECK_NEAR(st.cyl[i].lambda, 1.0, 0.0);
  }
  for (int i = sync.engine_config.num_cylinders; i < ENGINE_MAX_CYLINDERS; i++) {
    CHECK_NEAR(st.cyl[i].cht_c, 0.0, 0.0);
    CHECK_NEAR(st.cyl[i].egt_c, 0.0, 0.0);
  }
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    CHECK_NEAR(sync.cyl_config[i].injector_flow_trim, 1.0, 0.0);
    CHECK_NEAR(sync.cyl_config[i].cooling_trim, 1.0, 0.0);
  }
}

static const TestCase CASES[] = {
    {"cylinder.config_default_is_nominal", test_config_default_is_nominal},
    {"cylinder.state_init_cold_start", test_state_init_cold_start},
    {"cylinder.engine_config_geometry", test_engine_config_geometry},
    {"cylinder.model_wiring_populates_cylinders",
     test_model_wiring_populates_cylinders},
};

RUN_TESTS(CASES)
