#include "test_util.h"

#include "math/units.h"
#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "physics/environment.h"

static EngineInput make_input(double throttle, double load, double amb_kpa) {
  EngineInput in;
  in.throttle = throttle;
  in.load_torque_nm = load;
  in.ambient_pressure_kpa = amb_kpa;
  return in;
}

static void run(EngineState *st, const EngineConfig *cfg, const EngineInput *in,
                double duration_s, double dt) {
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    engine_model_step(st, cfg, in, cyl, i * dt, dt);
  }
}

static void test_init_is_cold_idle(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CHECK_NEAR(engine_model_rpm(&st), 700.0, 1e-6);
  CHECK_NEAR(st.map_kpa, 30.0, 1e-6);
}

static void test_config_defaults_are_positive(void) {
  EngineConfig cfg = engine_config_default();
  CHECK(cfg.inertia_kg_m2 > 0.0);
  CHECK(cfg.map_tau_s > 0.0);
  CHECK(cfg.friction_coeff_nm_per_rad_s > 0.0);
}

static void test_wot_reaches_analytic_steady_state(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  EngineInput in = make_input(1.0, 0.0, 101.325);
  run(&st, &cfg, &in, 60.0, 0.005);

  /* WOT MAP target is ambient. */
  CHECK_NEAR(st.map_kpa, 101.325, 0.5);

  /* Above ~255 rad/s the torque falloff is pinned at 0.15, so the steady
   * speed solves  2.2 * MAP * 0.15 = friction_coeff * omega. */
  double omega_star = (2.2 * 101.325 * 0.15) / cfg.friction_coeff_nm_per_rad_s;
  CHECK_NEAR(st.omega_rad_s, omega_star, 3.0);

  double before = st.omega_rad_s;
  run(&st, &cfg, &in, 3.0, 0.005);
  CHECK_NEAR(st.omega_rad_s, before, 0.5); /* genuinely settled */
}

static void test_higher_throttle_gives_higher_steady_rpm(void) {
  EngineConfig cfg = engine_config_default();
  EngineState lo, hi;
  engine_model_init(&lo, &cfg);
  engine_model_init(&hi, &cfg);
  EngineInput in_lo = make_input(0.3, 20.0, 101.325);
  EngineInput in_hi = make_input(0.8, 20.0, 101.325);
  run(&lo, &cfg, &in_lo, 60.0, 0.005);
  run(&hi, &cfg, &in_hi, 60.0, 0.005);
  CHECK(engine_model_rpm(&hi) > engine_model_rpm(&lo) + 100.0);
}

static void test_more_load_gives_lower_steady_rpm(void) {
  EngineConfig cfg = engine_config_default();
  EngineState light, heavy;
  engine_model_init(&light, &cfg);
  engine_model_init(&heavy, &cfg);
  EngineInput in_light = make_input(0.7, 10.0, 101.325);
  EngineInput in_heavy = make_input(0.7, 60.0, 101.325);
  run(&light, &cfg, &in_light, 60.0, 0.005);
  run(&heavy, &cfg, &in_heavy, 60.0, 0.005);
  CHECK(engine_model_rpm(&heavy) < engine_model_rpm(&light));
}

static void test_altitude_lowers_map_ceiling_and_rpm(void) {
  EngineConfig cfg = engine_config_default();
  double p_sea = environment_isa(0.0).pressure_kpa;
  double p_alt = environment_isa(3000.0).pressure_kpa;

  EngineState sea, alt;
  engine_model_init(&sea, &cfg);
  engine_model_init(&alt, &cfg);
  EngineInput in_sea = make_input(1.0, 20.0, p_sea);
  EngineInput in_alt = make_input(1.0, 20.0, p_alt);
  run(&sea, &cfg, &in_sea, 40.0, 0.005);
  run(&alt, &cfg, &in_alt, 40.0, 0.005);

  CHECK(alt.map_kpa < sea.map_kpa - 20.0);
  CHECK(engine_model_rpm(&alt) < engine_model_rpm(&sea));
}

/* Phase 0a regression guard: engine_model_step() sub-steps internally now,
 * so the mean trajectory should be (almost) independent of the caller's
 * frame dt. Compare a coarse frame dt (0.05s, the actual clamp used in
 * main.c) against a fine one (0.0005s) over the cold-idle-to-WOT transient
 * (not just the settled equilibrium, which is an attracting fixed point and
 * would mask integration error) and require them to stay close throughout.
 * Re-run this after every later phase -- if it stops passing, sub-stepping
 * has been broken or ENGINE_SUB_STEP_S needs retuning. */
static void test_frame_dt_does_not_affect_trajectory(void) {
  EngineConfig cfg = engine_config_default();
  EngineInput in = make_input(1.0, 20.0, 101.325);

  EngineState coarse, fine;
  engine_model_init(&coarse, &cfg);
  engine_model_init(&fine, &cfg);

  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }

  double checkpoints_s[] = {0.5, 1.0, 2.0, 5.0};
  double t_coarse = 0.0, t_fine = 0.0;
  for (size_t c = 0; c < sizeof(checkpoints_s) / sizeof(checkpoints_s[0]);
       c++) {
    double target_s = checkpoints_s[c];

    const double dt_coarse = 0.05;
    while (target_s - t_coarse > 1e-9) {
      double h = (target_s - t_coarse) < dt_coarse ? (target_s - t_coarse)
                                                    : dt_coarse;
      engine_model_step(&coarse, &cfg, &in, cyl, t_coarse, h);
      t_coarse += h;
    }

    const double dt_fine = 0.0005;
    while (target_s - t_fine > 1e-9) {
      double h =
          (target_s - t_fine) < dt_fine ? (target_s - t_fine) : dt_fine;
      engine_model_step(&fine, &cfg, &in, cyl, t_fine, h);
      t_fine += h;
    }

    CHECK_NEAR(coarse.omega_rad_s, fine.omega_rad_s, 0.5);
    CHECK_NEAR(coarse.map_kpa, fine.map_kpa, 0.1);
  }
}

static void test_rpm_helper_matches_unit_conversion(void) {
  EngineState st;
  st.omega_rad_s = 261.8;
  st.map_kpa = 80.0;
  CHECK_NEAR(engine_model_rpm(&st), rad_s_to_rpm(261.8), 1e-9);
}

static const TestCase CASES[] = {
    {"engine_model.init_is_cold_idle", test_init_is_cold_idle},
    {"engine_model.config_defaults_are_positive",
     test_config_defaults_are_positive},
    {"engine_model.wot_reaches_analytic_steady_state",
     test_wot_reaches_analytic_steady_state},
    {"engine_model.higher_throttle_gives_higher_steady_rpm",
     test_higher_throttle_gives_higher_steady_rpm},
    {"engine_model.more_load_gives_lower_steady_rpm",
     test_more_load_gives_lower_steady_rpm},
    {"engine_model.altitude_lowers_map_ceiling_and_rpm",
     test_altitude_lowers_map_ceiling_and_rpm},
    {"engine_model.frame_dt_does_not_affect_trajectory",
     test_frame_dt_does_not_affect_trajectory},
    {"engine_model.rpm_helper_matches_unit_conversion",
     test_rpm_helper_matches_unit_conversion},
};

RUN_TESTS(CASES)
