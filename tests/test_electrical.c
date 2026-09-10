#include "test_util.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/electrical.h"

/* sea level, still air, ISA 15 degC -> oat_c == 15.0 */
static const EnvInput ENV_SL = {0.0, 0.0, 0.0};

static void run(ModelSync *sync, ModelState *st, const EngineInput *in,
                double seconds) {
  int steps = (int)(seconds / 0.01 + 0.5);
  for (int i = 0; i < steps; i++) {
    model_sync_step(sync, st, in, &ENV_SL, 0.01);
  }
}

static void test_config_and_init(void) {
  ElecConfig c = elec_config_default();
  CHECK(c.bus_nominal_v > 12.0 && c.bus_nominal_v < 15.0);
  CHECK(c.alt_rated_a > 0.0);
  CHECK(c.load_base_a > 0.0 && c.load_base_a < c.alt_rated_a);
  CHECK(c.batt_capacity_ah > 0.0);
  CHECK(c.alt_cutin_rpm < c.alt_full_output_rpm);
  CHECK_NEAR(c.alt_health, 1.0, 0.0);

  ElecState s;
  elec_state_init(&s);
  CHECK_NEAR(s.batt_soc, 1.0, 0.0);
  CHECK_NEAR(s.alt_current_a, 0.0, 0.0);
}

static void test_alt_output_rises_with_rpm(void) {
  ElecConfig c = elec_config_default();
  ElecState below, above;
  elec_state_init(&below);
  elec_state_init(&above);
  elec_step(&below, &c, 700.0, 0.01);  /* under cut-in */
  elec_step(&above, &c, 2000.0, 0.01); /* over full output */
  CHECK(below.alt_current_a < above.alt_current_a);
  CHECK(above.alt_current_a >= c.load_base_a); /* at least carrying the load */
}

static void test_bus_regulated_at_cruise(void) {
  ElecConfig c = elec_config_default();
  ElecState s;
  elec_state_init(&s);
  for (int i = 0; i < 200; i++) {
    elec_step(&s, &c, 2200.0, 0.01);
  }
  CHECK_NEAR(s.bus_v, c.bus_nominal_v, 1e-6);
  CHECK(s.batt_soc >= 1.0 - 1e-9); /* full battery, alternator has surplus */
}

static void test_alternator_failure_drains_battery(void) {
  ElecConfig healthy = elec_config_default();
  ElecConfig dead = elec_config_default();
  dead.alt_health = 0.0;

  ElecState hs, ds;
  elec_state_init(&hs);
  elec_state_init(&ds);
  for (int i = 0; i < 60000; i++) { /* 10 min at cruise rpm */
    elec_step(&hs, &healthy, 2200.0, 0.01);
    elec_step(&ds, &dead, 2200.0, 0.01);
  }
  CHECK_NEAR(hs.bus_v, healthy.bus_nominal_v, 1e-6); /* healthy: regulated */
  CHECK(ds.bus_v < 13.0);                            /* dead: on the battery */
  CHECK(ds.alt_current_a < 1e-6);                    /* no output */
  CHECK(ds.batt_soc < hs.batt_soc);                  /* draining */
  CHECK(ds.batt_soc < 0.90); /* ~15 A from 15 Ah for 10 min -> soc ~0.83 */
}

static void test_soc_clamps_and_recharges(void) {
  ElecConfig dead = elec_config_default();
  dead.alt_health = 0.0;
  ElecState drained;
  elec_state_init(&drained);
  elec_step(&drained, &dead, 2000.0, 1.0e9); /* absurd dt: would overshoot */
  CHECK_NEAR(drained.batt_soc, 0.0, 1e-9);

  ElecConfig healthy = elec_config_default();
  ElecState charging;
  elec_state_init(&charging);
  charging.batt_soc = 0.40;
  for (int i = 0; i < 120000; i++) { /* 20 min cruise */
    elec_step(&charging, &healthy, 2200.0, 0.01);
  }
  CHECK(charging.batt_soc > 0.40);
  CHECK(charging.batt_soc <= 1.0);
  CHECK_NEAR(charging.bus_v, healthy.bus_nominal_v, 1e-6);
}

static void test_model_wires_electrical(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = {
      .throttle = 0.9, .load_torque_nm = 40.0, .ambient_pressure_kpa = 101.325};
  run(&sync, &st, &in, 60.0);
  CHECK(st.elec.bus_v > 12.0);
  CHECK(st.elec.alt_current_a > 0.0);
  CHECK(st.elec.batt_soc > 0.0 && st.elec.batt_soc <= 1.0);
}

static const TestCase CASES[] = {
    {"elec.config_and_init", test_config_and_init},
    {"elec.alt_output_rises_with_rpm", test_alt_output_rises_with_rpm},
    {"elec.bus_regulated_at_cruise", test_bus_regulated_at_cruise},
    {"elec.alternator_failure_drains_battery",
     test_alternator_failure_drains_battery},
    {"elec.soc_clamps_and_recharges", test_soc_clamps_and_recharges},
    {"elec.model_wires_electrical", test_model_wires_electrical},
};

RUN_TESTS(CASES)
