#include "test_util.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/lubrication.h"

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
  LubeConfig c = lube_config_default();
  CHECK(c.relief_valve_kpa > 0.0);
  CHECK(c.k_pump_kpa_per_rpm > 0.0);
  CHECK(c.visc_falloff_per_c > 0.0);
  CHECK_NEAR(c.bearing_wear, 0.0, 0.0);

  LubeState s;
  lube_state_init(&s);
  CHECK_NEAR(s.oil_press_kpa, 0.0, 0.0);
}

static void test_pressure_rises_with_rpm(void) {
  LubeConfig c = lube_config_default();
  LubeState lo, hi;
  lube_state_init(&lo);
  lube_state_init(&hi);
  lube_step(&lo, &c, 800.0, 90.0);
  lube_step(&hi, &c, 2000.0, 90.0);
  CHECK(lo.oil_press_kpa > 0.0);
  CHECK(hi.oil_press_kpa > lo.oil_press_kpa);
}

static void test_hot_oil_lowers_pressure(void) {
  LubeConfig c = lube_config_default();
  LubeState cold, hot;
  lube_state_init(&cold);
  lube_state_init(&hot);
  lube_step(&cold, &c, 1500.0, 20.0);
  lube_step(&hot, &c, 1500.0, 120.0);
  CHECK(hot.oil_press_kpa < cold.oil_press_kpa);
}

static void test_relief_valve_soft_ceiling(void) {
  LubeConfig c = lube_config_default();

  /* Just onto the relief: the valve bleeds, so pressure sits strictly
   * between the setpoint and the top of the band -- a smooth knee, not a
   * hard corner. */
  LubeState knee;
  lube_state_init(&knee);
  lube_step(&knee, &c, 1250.0, 15.0);
  CHECK(knee.oil_press_kpa > c.relief_valve_kpa);
  CHECK(knee.oil_press_kpa < c.relief_valve_kpa + c.relief_band_kpa);

  /* Far past it: pegged at the band ceiling, never above. */
  LubeState pegged;
  lube_state_init(&pegged);
  lube_step(&pegged, &c, 6000.0, 15.0);
  CHECK(pegged.oil_press_kpa <= c.relief_valve_kpa + c.relief_band_kpa);
  CHECK(pegged.oil_press_kpa > knee.oil_press_kpa);
}

static void test_bearing_wear_lowers_pressure(void) {
  LubeConfig healthy = lube_config_default();
  LubeConfig worn = lube_config_default();
  worn.bearing_wear = 0.6;
  LubeState hs, ws;
  lube_state_init(&hs);
  lube_state_init(&ws);
  lube_step(&hs, &healthy, 1200.0, 100.0);
  lube_step(&ws, &worn, 1200.0, 100.0);
  CHECK(ws.oil_press_kpa < hs.oil_press_kpa);
  CHECK_NEAR(ws.oil_press_kpa / hs.oil_press_kpa, 1.0 / 1.6, 0.02); /* /(1+wear) */
}

static void test_model_oil_pressure_is_live(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  CHECK_NEAR(st.lube.oil_press_kpa, 0.0, 0.0); /* engine not stepped yet */

  EngineInput in = {
      .throttle = 0.8, .load_torque_nm = 40.0, .ambient_pressure_kpa = 101.325};
  run(&sync, &st, &in, 30.0);
  CHECK(st.lube.oil_press_kpa > 50.0);
  CHECK(st.lube.oil_press_kpa <=
        sync.lube_config.relief_valve_kpa + sync.lube_config.relief_band_kpa);
}

static void test_model_bearing_wear_drops_pressure(void) {
  ModelSync healthy, worn;
  model_sync_init(&healthy);
  model_sync_init(&worn);
  worn.lube_config.bearing_wear = 0.6;

  ModelState hs, ws;
  model_state_init(&hs, &healthy.engine_config, 15.0);
  model_state_init(&ws, &worn.engine_config, 15.0);
  EngineInput in = {
      .throttle = 0.7, .load_torque_nm = 30.0, .ambient_pressure_kpa = 101.325};
  run(&healthy, &hs, &in, 120.0);
  run(&worn, &ws, &in, 120.0);
  CHECK(ws.lube.oil_press_kpa < hs.lube.oil_press_kpa);
}

static const TestCase CASES[] = {
    {"lube.config_and_init", test_config_and_init},
    {"lube.pressure_rises_with_rpm", test_pressure_rises_with_rpm},
    {"lube.hot_oil_lowers_pressure", test_hot_oil_lowers_pressure},
    {"lube.relief_valve_soft_ceiling", test_relief_valve_soft_ceiling},
    {"lube.bearing_wear_lowers_pressure", test_bearing_wear_lowers_pressure},
    {"lube.model_oil_pressure_is_live", test_model_oil_pressure_is_live},
    {"lube.model_bearing_wear_drops_pressure",
     test_model_bearing_wear_drops_pressure},
};

RUN_TESTS(CASES)
