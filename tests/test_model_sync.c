#include "test_util.h"

#include "physics/combustion.h"
#include "physics/engine_model.h"
#include "model/sync.h"
#include "model/state.h"

static EngineInput make_input(double throttle, double load, double amb_kpa) {
  EngineInput in;
  in.throttle = throttle;
  in.load_torque_nm = load;
  in.ambient_pressure_kpa = amb_kpa;
  return in;
}

static void test_init_bundles_cold_start(void) {
  ModelSync sync;
  model_sync_init(&sync);
  CHECK(sync.sim_time_s == 0.0);

  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  CHECK_NEAR(st.rpm, 700.0, 1e-6);
  CHECK_NEAR(st.thermal.cht_c, 15.0, 1e-9);
  CHECK_NEAR(st.thermal.egt_c, 15.0, 1e-9);
  CHECK_NEAR(st.thermal.oil_temp_c, 15.0, 1e-9);
  CHECK_NEAR(st.rpm, engine_model_rpm(&st.engine), 1e-9);
  CHECK_NEAR(st.torque_nm, engine_model_torque_nm(&st.engine), 1e-9);
}

static void test_sim_clock_advances_by_dt(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = make_input(0.5, 20.0, 101.325);
  double dt = 1.0 / 120.0;
  for (int i = 0; i < 1200; i++) {
    model_sync_step(&sync, &st, &in, 15.0, dt);
  }
  CHECK_NEAR(sync.sim_time_s, 1200.0 * dt, 1e-9);
}

static void test_derived_readouts_stay_in_sync(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = make_input(0.9, 30.0, 101.325);
  for (int i = 0; i < 500; i++) {
    model_sync_step(&sync, &st, &in, 15.0, 0.01);
    CHECK_NEAR(st.rpm, engine_model_rpm(&st.engine), 1e-9);
    CHECK_NEAR(st.torque_nm, engine_model_torque_nm(&st.engine), 1e-9);
  }
}

static void test_throttle_up_spins_and_heats(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  double amb = 15.0;
  model_state_init(&st, &sync.engine_config, amb);
  EngineInput in = make_input(0.85, 40.0, 101.325);
  for (int i = 0; i < 6000; i++) { /* 60 s at dt = 0.01 */
    model_sync_step(&sync, &st, &in, amb, 0.01);
  }
  CHECK(st.rpm > 900.0);                            /* spun up past cold idle */
  CHECK(st.thermal.egt_c > st.thermal.cht_c);       /* exhaust hotter than head */
  CHECK(st.thermal.cht_c > amb + 20.0);             /* head clearly warmed */
  CHECK(st.thermal.oil_temp_c < st.thermal.cht_c);  /* slow oil lags the head */
  CHECK(st.thermal.oil_temp_c >= amb - 1e-6);       /* never below ambient */
}

static void test_cht_settles_near_load_scaled_target(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  double amb = 15.0;
  model_state_init(&st, &sync.engine_config, amb);
  EngineInput in = make_input(0.7, 35.0, 101.325);
  for (int i = 0; i < 120000; i++) { /* 1200 s -> well past CHT tau */
    model_sync_step(&sync, &st, &in, amb, 0.01);
  }
  double lf =
      combustion_load_fraction(st.engine.map_kpa, st.engine.omega_rad_s);
  double target =
      amb + thermal_rise_c(sync.thermal_config.cht_rise_rated_c, lf);
  CHECK_NEAR(st.thermal.cht_c, target, 2.0);
}

static const TestCase CASES[] = {
    {"model_sync.init_bundles_cold_start", test_init_bundles_cold_start},
    {"model_sync.sim_clock_advances_by_dt", test_sim_clock_advances_by_dt},
    {"model_sync.derived_readouts_stay_in_sync",
     test_derived_readouts_stay_in_sync},
    {"model_sync.throttle_up_spins_and_heats", test_throttle_up_spins_and_heats},
    {"model_sync.cht_settles_near_load_scaled_target",
     test_cht_settles_near_load_scaled_target},
};

RUN_TESTS(CASES)
