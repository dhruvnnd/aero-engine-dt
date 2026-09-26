#include "test_util.h"

#include "physics/engine_model.h"
#include "physics/environment.h"
#include "physics/propeller.h"
#include "model/sync.h"
#include "model/state.h"

static EngineInput make_input(double throttle, double load, double amb_kpa) {
  EngineInput in;
  in.throttle = throttle;
  in.load_torque_nm = load;
  in.ambient_pressure_kpa = amb_kpa;
  return in;
}

/* Sea level, still air; oat_offset carries the desired ambient off ISA 15 C. */
static EnvInput env_at(double ambient_c) {
  EnvInput e = {0.0, 0.0, ambient_c - 15.0};
  return e;
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
  EnvInput env = env_at(15.0);
  double dt = 1.0 / 120.0;
  for (int i = 0; i < 1200; i++) {
    model_sync_step(&sync, &st, &in, &env, dt);
  }
  CHECK_NEAR(sync.sim_time_s, 1200.0 * dt, 1e-9);
}

static void test_derived_readouts_stay_in_sync(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = make_input(0.9, 30.0, 101.325);
  EnvInput env = env_at(15.0);
  for (int i = 0; i < 500; i++) {
    model_sync_step(&sync, &st, &in, &env, 0.01);
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
  EnvInput env = env_at(amb);
  for (int i = 0; i < 6000; i++) { /* 60 s at dt = 0.01 */
    model_sync_step(&sync, &st, &in, &env, 0.01);
  }
  CHECK(st.rpm > 900.0);                            /* spun up past cold idle */
  CHECK(st.thermal.egt_c > st.thermal.cht_c);       /* exhaust hotter than head */
  CHECK(st.thermal.cht_c > amb + 20.0);             /* head clearly warmed */
  CHECK(st.thermal.oil_temp_c < st.thermal.cht_c);  /* slow oil lags the head */
  CHECK(st.thermal.oil_temp_c >= amb - 1e-6);       /* never below ambient */
}

/* After the head has had time to settle, each cylinder's CHT sits at the heat
 * balance its own numbers imply: ambient + head heat * K / cooling. */
static void test_cht_settles_at_the_heat_balance(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  double amb = 15.0;
  model_state_init(&st, &sync.engine_config, amb);
  EngineInput in = make_input(0.7, 35.0, 101.325);
  EnvInput env = env_at(amb); /* sea level, still air */
  for (int i = 0; i < 20000; i++) { /* 400 s -> several CHT tau */
    model_sync_step(&sync, &st, &in, &env, 0.02);
  }
  /* The heat inputs and cooling flow ripple a little with the firing pulses
   * and crank speed; the slow head filter averages that out, so compare it
   * with the balance built from window averages. */
  double heat_sum = 0.0, fric_sum = 0.0, cool_sum = 0.0;
  const int trailing_steps = 3000; /* last 60 s */
  for (int i = 0; i < trailing_steps; i++) {
    model_sync_step(&sync, &st, &in, &env, 0.02);
    heat_sum += st.engine.cyl_thermal[0].heat_w;
    fric_sum += st.engine.cyl_thermal[0].friction_w;
    cool_sum += environment_cool_index(st.env.density_kg_m3,
                                       st.env.airspeed_ms, st.rpm);
  }
  const ThermalConfig *tc = &sync.thermal_config;
  const double head_kw = tc->head_base_kw +
                         (tc->head_heat_share * heat_sum / trailing_steps +
                          tc->friction_head_share * fric_sum / trailing_steps) /
                             1000.0;
  const double target =
      amb + head_kw * tc->cht_k_per_kw /
                thermal_cool_divisor(cool_sum / trailing_steps);
  CHECK_NEAR(st.cyl[0].cht_c, target, 4.0);
}

/* ---- propeller coupling (Phase 4) ---- */

/* Runs a fresh model at a fixed throttle / extra load / altitude / airspeed
 * until it settles, and returns the final state. `cq_static` overrides the
 * propeller's static torque coefficient (< 0 keeps the default; 0 removes the
 * prop). */
static ModelState settle(int ncyl, double throttle, double extra_load_nm,
                         double alt_m, double airspeed_ms, double cq_static) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.engine_config.num_cylinders = ncyl;
  engine_default_firing_order(ncyl, sync.engine_config.firing_order);
  if (cq_static >= 0.0) {
    sync.engine_config.prop.cq_static = cq_static;
  }
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EngineInput in = make_input(throttle, extra_load_nm,
                              environment_isa(alt_m).pressure_kpa);
  EnvInput env = {alt_m, airspeed_ms, 0.0};
  for (int i = 0; i < 1500; i++) { /* 30 s */
    model_sync_step(&sync, &st, &in, &env, 0.02);
  }
  return st;
}

static void test_prop_load_reaches_the_crank(void) {
  ModelState with_prop = settle(4, 0.6, 0.0, 0.0, 0.0, -1.0);
  ModelState no_prop = settle(4, 0.6, 0.0, 0.0, 0.0, 0.0);
  CHECK(with_prop.prop.torque_nm > 5.0);
  CHECK(with_prop.prop.thrust_n > 50.0);
  CHECK_NEAR(no_prop.prop.torque_nm, 0.0, 1e-12);
  CHECK(with_prop.rpm < no_prop.rpm - 300.0); /* the prop drags RPM down */

  /* the reported torque is the prop's torque at (about) the settled speed */
  const EngineConfig defaults = engine_config_default();
  PropState expect;
  prop_step(&expect, &defaults.prop, with_prop.rpm, 0.0,
            with_prop.env.density_kg_m3);
  CHECK_NEAR(with_prop.prop.torque_nm, expect.torque_nm,
             0.03 * expect.torque_nm);
}

/* The standing sanity check: throttle 0 -> 1 gives a monotonic, bounded
 * equilibrium RPM (the flat placeholder load had no such restoring torque). */
static void test_throttle_sweep_gives_monotonic_bounded_rpm(void) {
  double prev = 0.0;
  for (int k = 0; k <= 5; k++) {
    ModelState s = settle(4, 0.2 * k, 0.0, 0.0, 0.0, -1.0);
    CHECK(s.engine.run_state == ENGINE_RUNNING);
    CHECK(s.rpm > prev + 100.0);
    CHECK(s.rpm < 4000.0);
    prev = s.rpm;
  }
}

/* A propeller keeps every cylinder count bounded, and more cylinders (more
 * torque) still settle at higher RPM. */
static void test_every_cylinder_count_settles_bounded_at_wot(void) {
  double prev = 0.0;
  for (int n = 2; n <= 6; n++) {
    ModelState s = settle(n, 1.0, 0.0, 0.0, 0.0, -1.0);
    CHECK(s.engine.run_state == ENGINE_RUNNING);
    CHECK(s.rpm < 5000.0);
    CHECK(s.rpm > prev);
    prev = s.rpm;
  }
}

/* Airspeed raises the advance ratio: the prop unloads, so RPM rises at fixed
 * throttle while thrust falls. */
static void test_airspeed_raises_rpm_and_cuts_thrust(void) {
  ModelState still = settle(4, 0.8, 0.0, 0.0, 0.0, -1.0);
  ModelState fast = settle(4, 0.8, 0.0, 0.0, 40.0, -1.0);
  CHECK(fast.rpm > still.rpm + 100.0);
  CHECK(fast.prop.thrust_n < still.prop.thrust_n);
  CHECK(fast.prop.advance_ratio > still.prop.advance_ratio);
}

/* Thin air loads the prop less (torque ~ density) and the engine makes less
 * power, so both fall with altitude. */
static void test_altitude_lowers_prop_load(void) {
  ModelState sea = settle(4, 1.0, 0.0, 0.0, 0.0, -1.0);
  ModelState high = settle(4, 1.0, 0.0, 3000.0, 0.0, -1.0);
  CHECK(high.prop.torque_nm < sea.prop.torque_nm);
  CHECK(high.prop.thrust_n < sea.prop.thrust_n);
  CHECK(high.rpm < sea.rpm);
}

/* EngineInput.load_torque_nm is accessory load on top of the prop, not a
 * replacement for it. */
static void test_extra_load_adds_to_the_prop(void) {
  ModelState base = settle(4, 0.8, 0.0, 0.0, 0.0, -1.0);
  ModelState loaded = settle(4, 0.8, 15.0, 0.0, 0.0, -1.0);
  CHECK(loaded.rpm < base.rpm - 200.0);
  CHECK(loaded.prop.torque_nm < base.prop.torque_nm); /* slower prop, less load */
}

static void test_stopped_engine_has_no_prop_load(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  st.engine.run_state = ENGINE_STOPPED;
  st.engine.omega_rad_s = 0.0;
  EngineInput in = make_input(0.5, 0.0, 101.325);
  EnvInput env = {0.0, 30.0, 0.0}; /* wind over a stationary prop */
  for (int i = 0; i < 50; i++) {
    model_sync_step(&sync, &st, &in, &env, 0.02);
  }
  CHECK_NEAR(st.prop.torque_nm, 0.0, 1e-12);
  CHECK_NEAR(st.prop.thrust_n, 0.0, 1e-12);
  CHECK_NEAR(st.rpm, 0.0, 1e-9);
}

/* ---- idle behaviour with the real propeller ---- */

/* Closed throttle, still air, with the given governor target (0 = off);
 * optionally cold-started from stopped. */
static ModelState idle_state(int ncyl, double target_rpm, double alt_m,
                             int from_stopped) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.engine_config.num_cylinders = ncyl;
  engine_default_firing_order(ncyl, sync.engine_config.firing_order);
  sync.engine_config.ecu.idle_target_rpm = target_rpm;
  model_sync_apply_engine_config(&sync, &sync.engine_config);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  if (from_stopped) {
    st.engine.run_state = ENGINE_STOPPED;
    st.engine.omega_rad_s = 0.0;
    st.engine.map_kpa = environment_isa(alt_m).pressure_kpa; /* not pumping */
    engine_model_start(&st.engine, &sync.engine_config, st.cyl);
  }
  EngineInput in = make_input(0.0, 0.0, environment_isa(alt_m).pressure_kpa);
  EnvInput env = {alt_m, 0.0, 0.0};
  for (int i = 0; i < 1500; i++) { /* 30 s */
    model_sync_step(&sync, &st, &in, &env, 0.02);
  }
  return st;
}

/* The high-idle bug: with no pumping loss and friction that ignored engine
 * size, every added cylinder pushed the closed-throttle idle up ~500 rpm
 * (6 cyl ~2150 rpm). Now the ungoverned idle is low and nearly independent of
 * cylinder count, and the governor lifts it to the target. */
static void test_idle_no_longer_runs_away_with_cylinder_count(void) {
  for (int n = 3; n <= 6; n++) {
    ModelState natural = idle_state(n, 0.0, 0.0, 0);
    CHECK(natural.engine.run_state == ENGINE_RUNNING);
    CHECK(natural.rpm > 350.0);
    CHECK(natural.rpm < 750.0);

    ModelState governed = idle_state(n, 800.0, 0.0, 0);
    CHECK(governed.engine.run_state == ENGINE_RUNNING);
    CHECK_NEAR(governed.rpm, 800.0, 25.0);
    CHECK(governed.ecu.idle_throttle < 0.15);
  }
}

static void test_idle_governor_holds_from_a_cold_start(void) {
  for (int n = 4; n <= 6; n += 2) {
    ModelState st = idle_state(n, 800.0, 0.0, 1);
    CHECK(st.engine.run_state == ENGINE_RUNNING);
    CHECK_NEAR(st.rpm, 800.0, 25.0);
  }
}

/* Stall it for real (an absurd load), let it sit stopped, then restart with a
 * closed throttle: the manifold has equalised with ambient while stopped, so
 * the restart catches and settles at the governed idle. */
static void test_restart_after_a_stall_at_closed_throttle(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  EnvInput env = {0.0, 0.0, 0.0};
  EngineInput crushing = make_input(0.0, 500.0, 101.325);
  for (int i = 0; i < 500; i++) {
    model_sync_step(&sync, &st, &crushing, &env, 0.02);
  }
  CHECK(st.engine.run_state == ENGINE_STOPPED);
  CHECK_NEAR(st.engine.map_kpa, 101.325, 1e-9);

  engine_model_start(&st.engine, &sync.engine_config, st.cyl);
  EngineInput normal = make_input(0.0, 0.0, 101.325);
  for (int i = 0; i < 1500; i++) {
    model_sync_step(&sync, &st, &normal, &env, 0.02);
  }
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
}

/* A closed throttle at altitude (a UAV descending from cruise) must not
 * stall the engine: closed-throttle MAP scales with ambient pressure. */
static void test_idle_holds_at_altitude(void) {
  ModelState st = idle_state(4, 800.0, 3000.0, 0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK_NEAR(st.rpm, 800.0, 30.0);
  CHECK(st.engine.map_kpa > 15.0);
}

static const TestCase CASES[] = {
    {"model_sync.init_bundles_cold_start", test_init_bundles_cold_start},
    {"model_sync.sim_clock_advances_by_dt", test_sim_clock_advances_by_dt},
    {"model_sync.derived_readouts_stay_in_sync",
     test_derived_readouts_stay_in_sync},
    {"model_sync.throttle_up_spins_and_heats", test_throttle_up_spins_and_heats},
    {"model_sync.cht_settles_at_the_heat_balance",
     test_cht_settles_at_the_heat_balance},
    {"model_sync.prop_load_reaches_the_crank", test_prop_load_reaches_the_crank},
    {"model_sync.throttle_sweep_gives_monotonic_bounded_rpm",
     test_throttle_sweep_gives_monotonic_bounded_rpm},
    {"model_sync.every_cylinder_count_settles_bounded_at_wot",
     test_every_cylinder_count_settles_bounded_at_wot},
    {"model_sync.airspeed_raises_rpm_and_cuts_thrust",
     test_airspeed_raises_rpm_and_cuts_thrust},
    {"model_sync.altitude_lowers_prop_load", test_altitude_lowers_prop_load},
    {"model_sync.extra_load_adds_to_the_prop", test_extra_load_adds_to_the_prop},
    {"model_sync.stopped_engine_has_no_prop_load",
     test_stopped_engine_has_no_prop_load},
    {"model_sync.idle_no_longer_runs_away_with_cylinder_count",
     test_idle_no_longer_runs_away_with_cylinder_count},
    {"model_sync.idle_governor_holds_from_a_cold_start",
     test_idle_governor_holds_from_a_cold_start},
    {"model_sync.restart_after_a_stall_at_closed_throttle",
     test_restart_after_a_stall_at_closed_throttle},
    {"model_sync.idle_holds_at_altitude", test_idle_holds_at_altitude},
};

RUN_TESTS(CASES)
