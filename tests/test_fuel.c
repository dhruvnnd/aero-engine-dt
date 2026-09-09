#include "test_util.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/fuel.h"

static void run(ModelSync *sync, ModelState *st, const EngineInput *in,
                double seconds) {
  int steps = (int)(seconds / 0.01 + 0.5);
  for (int i = 0; i < steps; i++) {
    model_sync_step(sync, st, in, 15.0, 0.01);
  }
}

static void test_config_and_init(void) {
  FuelConfig c = fuel_config_default();
  CHECK(c.afr_stoich > 10.0 && c.afr_stoich < 20.0);
  CHECK(c.lambda_target > 0.5 && c.lambda_target < 1.5);
  CHECK(c.vol_eff > 0.0 && c.vol_eff <= 1.2);
  CHECK(c.displacement_l > 0.0);
  CHECK(c.pump_press_kpa > 0.0);

  FuelState s;
  fuel_state_init(&s);
  CHECK_NEAR(s.air_flow_gps, 0.0, 0.0);
  CHECK_NEAR(s.fuel_flow_kgph, 0.0, 0.0);
  CHECK_NEAR(s.fuel_press_kpa, 0.0, 0.0);
}

static void test_flow_rises_with_load(void) {
  ModelSync lo, hi;
  model_sync_init(&lo);
  model_sync_init(&hi);
  ModelState ls, hs;
  model_state_init(&ls, &lo.engine_config, 15.0);
  model_state_init(&hs, &hi.engine_config, 15.0);
  EngineInput in_lo = {
      .throttle = 0.3, .load_torque_nm = 20.0, .ambient_pressure_kpa = 101.325};
  EngineInput in_hi = {
      .throttle = 0.9, .load_torque_nm = 20.0, .ambient_pressure_kpa = 101.325};
  run(&lo, &ls, &in_lo, 60.0);
  run(&hi, &hs, &in_hi, 60.0);

  CHECK(ls.fuel.fuel_flow_kgph > 0.0);
  CHECK(hs.fuel.air_flow_gps > ls.fuel.air_flow_gps);
  CHECK(hs.fuel.fuel_flow_kgph > ls.fuel.fuel_flow_kgph);
  CHECK(hs.fuel.fuel_press_kpa < hi.fuel_config.pump_press_kpa); /* demand droop */
}

static void test_meters_to_stoich_at_nominal(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = {
      .throttle = 0.85, .load_torque_nm = 40.0, .ambient_pressure_kpa = 101.325};
  run(&sync, &st, &in, 60.0);

  double expected_kgph =
      st.fuel.air_flow_gps / sync.fuel_config.afr_stoich * 3.6;
  CHECK_NEAR(st.fuel.fuel_flow_kgph, expected_kgph, 1e-6);
  for (int i = 0; i < sync.engine_config.num_cylinders; i++) {
    CHECK_NEAR(st.cyl[i].lambda, 1.0, 1e-9);
  }
}

static void test_clogged_injector_cuts_fuel_and_leans_cylinder(void) {
  ModelSync healthy, clogged;
  model_sync_init(&healthy);
  model_sync_init(&clogged);
  clogged.cyl_config[1].injector_flow_trim = 0.75; /* lean but still firing */

  ModelState hs, cs;
  model_state_init(&hs, &healthy.engine_config, 15.0);
  model_state_init(&cs, &clogged.engine_config, 15.0);
  EngineInput in = {
      .throttle = 0.85, .load_torque_nm = 40.0, .ambient_pressure_kpa = 101.325};
  run(&healthy, &hs, &in, 60.0);
  run(&clogged, &cs, &in, 60.0);

  double ratio = cs.fuel.fuel_flow_kgph / hs.fuel.fuel_flow_kgph;
  CHECK(ratio > 0.83 && ratio < 0.98); /* ~3.75/4, minus a little from lower rpm */
  CHECK(cs.cyl[1].lambda > 1.25);      /* that cylinder runs lean */
  CHECK_NEAR(cs.cyl[1].misfire_rate, 0.0, 0.0); /* ...but not misfiring */
  CHECK_NEAR(cs.cyl[0].lambda, 1.0, 1e-9);      /* the others unaffected */
  CHECK(cs.cyl[1].egt_c > cs.cyl[0].egt_c + 5.0); /* lean -> hotter exhaust */
}

static const TestCase CASES[] = {
    {"fuel.config_and_init", test_config_and_init},
    {"fuel.flow_rises_with_load", test_flow_rises_with_load},
    {"fuel.meters_to_stoich_at_nominal", test_meters_to_stoich_at_nominal},
    {"fuel.clogged_injector_cuts_fuel_and_leans_cylinder",
     test_clogged_injector_cuts_fuel_and_leans_cylinder},
};

RUN_TESTS(CASES)
