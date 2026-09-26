#include "test_util.h"

#include "math/units.h"
#include "physics/combustion.h"
#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "physics/engine_trace.h"
#include "physics/environment.h"
#include "physics/fuel.h"

#define TEST_INTAKE_TEMP_C 15.0

static EngineInput make_input(double throttle, double load, double amb_kpa) {
  EngineInput in;
  in.throttle = throttle;
  in.load_torque_nm = load;
  in.ambient_pressure_kpa = amb_kpa;
  return in;
}

static void init_cyl_states(CylinderState *cyl_states) {
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cylinder_state_init(&cyl_states[i], TEST_INTAKE_TEMP_C);
  }
}

/* `cyl_states` must persist across calls (a caller-owned array, matching how
 * ModelState.cyl works in the real app) -- cyl_pressure_kpa depends on where
 * the previous call left the burn phased, not just the current instant. */
static void run(EngineState *st, CylinderState *cyl_states,
                const EngineConfig *cfg, const EngineInput *in,
                double duration_s, double dt) {
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  FuelConfig fuel_cfg = fuel_config_default();
  int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    engine_model_step(st, cfg, in, &fuel_cfg, cyl, cyl_states,
                      TEST_INTAKE_TEMP_C, i * dt, dt);
  }
}

static void test_init_is_cold_idle(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CHECK_NEAR(engine_model_rpm(&st), 700.0, 1e-6);
  CHECK_NEAR(st.map_kpa, 30.0, 1e-6);
  CHECK_NEAR(st.theta_deg, 0.0, 1e-9);
}

static void test_config_defaults_are_positive(void) {
  EngineConfig cfg = engine_config_default();
  CHECK(cfg.inertia_kg_m2 > 0.0);
  CHECK(cfg.map_tau_s > 0.0);
  CHECK(cfg.friction_coeff_nm_per_rad_s > 0.0);
  CHECK(engine_config_validate(&cfg, NULL) == 0);
}

/* WOT settling: the exact analytic steady state was tied to the old
 * combustion_indicated_torque_nm() curve's specific constants and no longer
 * applies now that real geometry drives torque (Phase 1). Replaced with a
 * boundedness/convergence-DIRECTION check rather than a tight "is it at
 * equilibrium yet" tolerance: this nonlinear system settles more slowly than
 * the old linear-ish curve did, and chasing a tight bound here just trades
 * test runtime for a precision this check doesn't actually need -- what
 * matters is that RPM stays in a plausible band throughout and that the
 * *rate* of change is shrinking (converging), not diverging or oscillating
 * without bound. MAP reaching ambient at WOT is still an exact, cheap
 * invariant worth checking directly. */
static void test_wot_settles_to_plausible_steady_state(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);
  EngineInput in = make_input(1.0, 20.0, 101.325);

  double samples[3];
  for (int i = 0; i < 3; i++) {
    run(&st, cyl_states, &cfg, &in, 4.0, 0.02);
    samples[i] = st.omega_rad_s;
    CHECK(engine_model_rpm(&st) > 500.0);
    CHECK(engine_model_rpm(&st) < 8000.0);
  }

  CHECK_NEAR(st.map_kpa, 101.325, 0.5);

  /* Converging, not diverging: the change over the last interval should be
   * no larger than over the one before it (allowing some slack for
   * numerical noise rather than requiring strict monotonic decay). */
  double delta1 = fabs(samples[1] - samples[0]);
  double delta2 = fabs(samples[2] - samples[1]);
  CHECK(delta2 < delta1 * 1.5 + 1.0);
}

static void test_higher_throttle_gives_higher_steady_rpm(void) {
  EngineConfig cfg = engine_config_default();
  EngineState lo, hi;
  engine_model_init(&lo, &cfg);
  engine_model_init(&hi, &cfg);
  CylinderState cyl_lo[ENGINE_MAX_CYLINDERS], cyl_hi[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_lo);
  init_cyl_states(cyl_hi);
  EngineInput in_lo = make_input(0.3, 20.0, 101.325);
  EngineInput in_hi = make_input(0.8, 20.0, 101.325);
  run(&lo, cyl_lo, &cfg, &in_lo, 4.0, 0.02);
  run(&hi, cyl_hi, &cfg, &in_hi, 4.0, 0.02);
  CHECK(engine_model_rpm(&hi) > engine_model_rpm(&lo) + 100.0);
}

static void test_more_load_gives_lower_steady_rpm(void) {
  EngineConfig cfg = engine_config_default();
  EngineState light, heavy;
  engine_model_init(&light, &cfg);
  engine_model_init(&heavy, &cfg);
  CylinderState cyl_light[ENGINE_MAX_CYLINDERS], cyl_heavy[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_light);
  init_cyl_states(cyl_heavy);
  EngineInput in_light = make_input(0.7, 10.0, 101.325);
  EngineInput in_heavy = make_input(0.7, 60.0, 101.325);
  run(&light, cyl_light, &cfg, &in_light, 4.0, 0.02);
  run(&heavy, cyl_heavy, &cfg, &in_heavy, 4.0, 0.02);
  CHECK(engine_model_rpm(&heavy) < engine_model_rpm(&light));
}

static void test_altitude_lowers_map_ceiling_and_rpm(void) {
  EngineConfig cfg = engine_config_default();
  double p_sea = environment_isa(0.0).pressure_kpa;
  double p_alt = environment_isa(3000.0).pressure_kpa;

  EngineState sea, alt;
  engine_model_init(&sea, &cfg);
  engine_model_init(&alt, &cfg);
  CylinderState cyl_sea[ENGINE_MAX_CYLINDERS], cyl_alt[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_sea);
  init_cyl_states(cyl_alt);
  EngineInput in_sea = make_input(1.0, 20.0, p_sea);
  EngineInput in_alt = make_input(1.0, 20.0, p_alt);
  run(&sea, cyl_sea, &cfg, &in_sea, 4.0, 0.02);
  run(&alt, cyl_alt, &cfg, &in_alt, 4.0, 0.02);

  CHECK(alt.map_kpa < sea.map_kpa - 20.0);
  CHECK(engine_model_rpm(&alt) < engine_model_rpm(&sea));
}

/* Phase 0a regression guard: engine_model_step() sub-steps internally
 * regardless of the caller's frame dt, so the mean trajectory should be
 * (almost) independent of it. Compare a coarse frame dt (0.05s, the actual
 * clamp used in main.c) against a fine one (0.0005s) over the cold-idle-to-
 * WOT transient. Re-run this after every later phase -- if it stops
 * passing, sub-stepping has been broken or ENGINE_SUB_STEP_S needs
 * retuning. Tolerance loosened from Phase 0a's since Phase 1's per-cylinder
 * pressure state adds real (if small) sensitivity to exactly where a call
 * boundary falls relative to the combustion event. */
static void test_frame_dt_does_not_affect_trajectory(void) {
  EngineConfig cfg = engine_config_default();
  EngineInput in = make_input(1.0, 20.0, 101.325);

  EngineState coarse, fine;
  engine_model_init(&coarse, &cfg);
  engine_model_init(&fine, &cfg);
  CylinderState cyl_coarse[ENGINE_MAX_CYLINDERS], cyl_fine[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_coarse);
  init_cyl_states(cyl_fine);

  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  FuelConfig fuel_cfg = fuel_config_default();

  double checkpoints_s[] = {0.5, 1.5, 3.0};
  double t_coarse = 0.0, t_fine = 0.0;
  for (size_t c = 0; c < sizeof(checkpoints_s) / sizeof(checkpoints_s[0]);
       c++) {
    double target_s = checkpoints_s[c];

    const double dt_coarse = 0.05;
    while (target_s - t_coarse > 1e-9) {
      double h = (target_s - t_coarse) < dt_coarse ? (target_s - t_coarse)
                                                    : dt_coarse;
      engine_model_step(&coarse, &cfg, &in, &fuel_cfg, cyl, cyl_coarse,
                        TEST_INTAKE_TEMP_C, t_coarse, h);
      t_coarse += h;
    }

    const double dt_fine = 0.0005;
    while (target_s - t_fine > 1e-9) {
      double h =
          (target_s - t_fine) < dt_fine ? (target_s - t_fine) : dt_fine;
      engine_model_step(&fine, &cfg, &in, &fuel_cfg, cyl, cyl_fine,
                        TEST_INTAKE_TEMP_C, t_fine, h);
      t_fine += h;
    }

    CHECK_NEAR(coarse.omega_rad_s, fine.omega_rad_s, 3.0);
    CHECK_NEAR(coarse.map_kpa, fine.map_kpa, 0.5);
  }
}

static void test_rpm_helper_matches_unit_conversion(void) {
  EngineState st;
  st.omega_rad_s = 261.8;
  st.map_kpa = 80.0;
  st.theta_deg = 0.0;
  st.torque_nm = 0.0;
  st.run_state = ENGINE_RUNNING;
  CHECK_NEAR(engine_model_rpm(&st), rad_s_to_rpm(261.8), 1e-9);
}

/* Crank angle actually advances and wraps into [0,720) as the engine runs. */
static void test_theta_advances_and_wraps(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);
  EngineInput in = make_input(0.5, 20.0, 101.325);

  run(&st, cyl_states, &cfg, &in, 0.05, 0.005);
  CHECK(st.theta_deg > 0.0);

  run(&st, cyl_states, &cfg, &in, 1.0, 0.005); /* several full cycles */
  CHECK(st.theta_deg >= 0.0);
  CHECK(st.theta_deg < 720.0);
}

/* In-cylinder pressure should settle into a physically sane range: well
 * above vacuum, and (since it includes compression + combustion peaks) well
 * above manifold pressure at its peak over a cycle. */
static void test_cylinder_pressure_stays_sane_and_peaks_above_map(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);
  EngineInput in = make_input(0.8, 20.0, 101.325);
  run(&st, cyl_states, &cfg, &in, 2.0, 0.005); /* settle */

  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  FuelConfig fuel_cfg = fuel_config_default();

  double peak_kpa = 0.0;
  double t = 0.0;
  const double dt = 0.0002; /* fine enough to actually sample the peak */
  for (int i = 0; i < 2000; i++) {
    engine_model_step(&st, &cfg, &in, &fuel_cfg, cyl, cyl_states,
                      TEST_INTAKE_TEMP_C, t, dt);
    t += dt;
    for (int c = 0; c < cfg.num_cylinders; c++) {
      CHECK(cyl_states[c].cyl_pressure_kpa > 0.0);
      if (cyl_states[c].cyl_pressure_kpa > peak_kpa) {
        peak_kpa = cyl_states[c].cyl_pressure_kpa;
      }
    }
  }
  CHECK(peak_kpa > st.map_kpa * 3.0);
}

/* Phase 1's required validation gate: the new crank-angle model's mean
 * torque, at a swept range of throttle/RPM operating points, should be the
 * same order of magnitude as the old combustion_indicated_torque_nm() curve
 * it replaces -- they won't match exactly (the old curve was a guess), but
 * a factor-of-5+ difference would mean the new model's geometry/combustion
 * constants are producing an implausible engine, not just "a different
 * curve". See docs/physical_modeling_plan.md Phase 1, step 6. */
static void test_mean_torque_same_order_of_magnitude_as_old_curve(void) {
  EngineConfig cfg = engine_config_default();
  double throttles[] = {0.3, 0.6, 1.0};

  for (size_t i = 0; i < sizeof(throttles) / sizeof(throttles[0]); i++) {
    EngineState st;
    engine_model_init(&st, &cfg);
    CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
    init_cyl_states(cyl_states);
    EngineInput in = make_input(throttles[i], 20.0, 101.325);
    run(&st, cyl_states, &cfg, &in, 4.0, 0.02); /* settle */

    /* torque_nm is a mean over just the last engine_model_step() call's
     * sub-steps -- at lower RPM a fixed frame dt covers a smaller slice of
     * the 720 deg combustion cycle, so a single frame's sample can land in a
     * net-negative portion of the cycle (inertia torque legitimately goes
     * negative part of the time, even though it integrates to ~0 over a
     * full cycle) purely by chance of phase. Average over a couple more
     * seconds -- many frames spanning many full cycles -- to get the actual
     * full-cycle mean this validation gate is about, rather than comparing
     * one arbitrarily-phased instant. */
    CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
    for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
      cyl[c] = cylinder_config_default();
    }
    FuelConfig fuel_cfg = fuel_config_default();
    double torque_sum = 0.0;
    const int trailing_steps = 100; /* 2 s at dt=0.02 */
    double t = 4.0;
    for (int s = 0; s < trailing_steps; s++) {
      engine_model_step(&st, &cfg, &in, &fuel_cfg, cyl, cyl_states,
                        TEST_INTAKE_TEMP_C, t, 0.02);
      t += 0.02;
      torque_sum += st.torque_nm;
    }
    double new_model_nm = torque_sum / trailing_steps;

    double old_curve_nm =
        combustion_indicated_torque_nm(st.map_kpa, st.omega_rad_s);

    CHECK(new_model_nm > old_curve_nm / 5.0);
    CHECK(new_model_nm < old_curve_nm * 5.0);
  }
}

/* Robustness fix: a fixed load exceeding available torque at low throttle
 * previously drove RPM through zero into reverse rotation -- the
 * crank-angle combustion model assumes forward rotation only, and the
 * angle-based sub-step size (inversely proportional to |omega|) could then
 * shrink toward zero as reverse speed grew, making engine_model_step() take
 * pathologically many iterations and freeze the interactive app. The engine
 * now stalls cleanly (RPM pinned at 0, run_state -> ENGINE_STOPPED) instead,
 * and stays stopped until engine_model_start() is called. If this test
 * hangs, the stall safeguard is broken, not just failing an assertion. */
static void test_engine_stalls_instead_of_reversing(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);

  /* Zero throttle (idle MAP only) plus a load far beyond anything idle
   * combustion can sustain -- guaranteed to stall quickly. */
  EngineInput stalling_in = make_input(0.0, 500.0, 101.325);
  run(&st, cyl_states, &cfg, &stalling_in, 5.0, 0.02);

  CHECK(st.run_state == ENGINE_STOPPED);
  CHECK_NEAR(st.omega_rad_s, 0.0, 1e-9);
  CHECK_NEAR(engine_model_rpm(&st), 0.0, 1e-9);

  /* Stepping a stopped engine is a safe no-op -- state doesn't drift. */
  double omega_before = st.omega_rad_s;
  run(&st, cyl_states, &cfg, &stalling_in, 1.0, 0.02);
  CHECK(st.run_state == ENGINE_STOPPED);
  CHECK_NEAR(st.omega_rad_s, omega_before, 1e-12);

  /* engine_model_start() begins cranking -- not an instant jump to
   * running -- so speed should build up gradually rather than snap to a
   * cold-idle guess. */
  engine_model_start(&st, &cfg, cyl_states);
  CHECK(st.run_state == ENGINE_CRANKING);
  CHECK_NEAR(st.omega_rad_s, omega_before, 1e-12); /* still ~0 right away */

  /* Under a sane operating point (not the absurd stalling load), cranking
   * should climb speed and catch on its own within a few seconds. The flat
   * load is kept small: unlike a propeller it doesn't vanish at rest, and the
   * starter must beat it plus breakaway friction to turn the crank at all. */
  EngineInput normal_in = make_input(0.7, 10.0, 101.325);
  run(&st, cyl_states, &cfg, &normal_in, 3.0, 0.02);
  CHECK(st.run_state == ENGINE_RUNNING);
  /* the idle governor holds the target (= catch speed here), so "above the
   * catch speed" is really "at it" */
  CHECK(engine_model_rpm(&st) > 0.9 * cfg.starter_catch_rpm);

  /* And a second engine_model_start() call while already running is a
   * documented no-op (matches a real ignition switch). */
  double rpm_before_noop = engine_model_rpm(&st);
  engine_model_start(&st, &cfg, cyl_states);
  CHECK(st.run_state == ENGINE_RUNNING);
  CHECK_NEAR(engine_model_rpm(&st), rpm_before_noop, 1e-9);
}

/* The starter motor itself: applying starter_torque_nm while cranking (no
 * throttle, so combustion alone can't do much) should still be enough to
 * spin the engine up from rest and past the catch threshold, the same way
 * turning a car key cranks a cold, unfueled-yet engine. */
static void test_starter_alone_can_crank_engine_up(void) {
  EngineConfig cfg = engine_config_default();
  EngineState st;
  engine_model_init(&st, &cfg);
  st.run_state = ENGINE_STOPPED;
  st.omega_rad_s = 0.0;
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);

  engine_model_start(&st, &cfg, cyl_states);
  CHECK(st.run_state == ENGINE_CRANKING);

  /* Zero throttle, light load -- close to what cranking a real engine
   * before it catches looks like. */
  EngineInput in = make_input(0.0, 2.0, 101.325);
  run(&st, cyl_states, &cfg, &in, 3.0, 0.02);

  CHECK(st.run_state == ENGINE_RUNNING);
  /* the idle governor's target equals the catch speed by default, so it holds
   * the engine there rather than above it */
  CHECK(engine_model_rpm(&st) > 0.9 * cfg.starter_catch_rpm);
}

/* ---- Phase 2: firing order, phase offsets, torque ripple ---- */

static void test_phase_offsets_follow_firing_order(void) {
  EngineConfig cfg = engine_config_default(); /* 4 cyl, 1-3-4-2 */
  double off[ENGINE_MAX_CYLINDERS];
  engine_cylinder_phase_offsets(&cfg, off);
  CHECK_NEAR(off[0], 0.0, 1e-12);   /* cylinder 1 fires first */
  CHECK_NEAR(off[2], 180.0, 1e-12); /* then 3 */
  CHECK_NEAR(off[3], 360.0, 1e-12); /* then 4 */
  CHECK_NEAR(off[1], 540.0, 1e-12); /* then 2 */
  CHECK_NEAR(off[4], 0.0, 1e-12);   /* unused slots stay zero */

  cfg.num_cylinders = 6;
  engine_default_firing_order(6, cfg.firing_order); /* 1-5-3-6-2-4 */
  engine_cylinder_phase_offsets(&cfg, off);
  CHECK_NEAR(off[0], 0.0, 1e-12);
  CHECK_NEAR(off[4], 120.0, 1e-12);
  CHECK_NEAR(off[2], 240.0, 1e-12);
  CHECK_NEAR(off[5], 360.0, 1e-12);
  CHECK_NEAR(off[1], 480.0, 1e-12);
  CHECK_NEAR(off[3], 600.0, 1e-12);

  cfg.num_cylinders = 1;
  engine_default_firing_order(1, cfg.firing_order);
  engine_cylinder_phase_offsets(&cfg, off);
  CHECK_NEAR(off[0], 0.0, 1e-12);
}

/* A firing_order that isn't a permutation must not index out of bounds or
 * stack cylinders on top of each other: it falls back to the default order. */
static void test_phase_offsets_fall_back_on_invalid_firing_order(void) {
  EngineConfig cfg = engine_config_default();
  cfg.firing_order[0] = 2; /* 2,3,4,2: repeated entry */
  cfg.firing_order[1] = 3;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 2;
  double off[ENGINE_MAX_CYLINDERS];
  engine_cylinder_phase_offsets(&cfg, off);
  CHECK_NEAR(off[0], 0.0, 1e-12); /* default 1-3-4-2 */
  CHECK_NEAR(off[2], 180.0, 1e-12);
  CHECK_NEAR(off[3], 360.0, 1e-12);
  CHECK_NEAR(off[1], 540.0, 1e-12);

  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg.firing_order[i] = 0; /* nothing set at all */
  }
  engine_cylinder_phase_offsets(&cfg, off);
  CHECK_NEAR(off[2], 180.0, 1e-12);
}

/* Steady, healthy engine with a trace attached; returns after `settle_s`
 * seconds plus a couple more so the ring holds several settled cycles. */
static void run_traced(EngineState *st, EngineTrace *tr, const EngineConfig *cfg,
                       CylinderConfig *cyl, double throttle, double load) {
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cyl_states);
  FuelConfig fuel_cfg = fuel_config_default();
  engine_model_init(st, cfg);
  st->trace = tr;
  EngineInput in = make_input(throttle, load, 101.325);
  double t = 0.0;
  for (int i = 0; i < 200; i++) { /* 4 s */
    engine_model_step(st, cfg, &in, &fuel_cfg, cyl, cyl_states,
                      TEST_INTAKE_TEMP_C, t, 0.02);
    t += 0.02;
  }
}

static void default_cyl_configs(CylinderConfig *cyl) {
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
}

/* Window of the newest full cycle inside `tr`: fills *first and returns n. */
static int newest_cycle(const EngineTrace *tr, int *first) {
  int n = engine_trace_last_cycle_count(tr);
  *first = engine_trace_count(tr) - n;
  return n;
}

/* Crank angle where cylinder `cyl`'s gas torque peaks in the newest cycle. */
static double peak_gas_theta_deg(const EngineTrace *tr, int cyl) {
  int first;
  int n = newest_cycle(tr, &first);
  double best = -1e30;
  double at = 0.0;
  for (int i = 0; i < n; i++) {
    const EngineTraceSample *s = engine_trace_at(tr, first + i);
    if (s->cyl_gas_nm[cyl] > best) {
      best = s->cyl_gas_nm[cyl];
      at = s->theta_deg;
    }
  }
  return at;
}

/* Forward crank angle from a to b, deg in [0,720). */
static double gap_deg(double a, double b) { return fmod(b - a + 1440.0, 720.0); }

/* The point of the phase offsets: with 1-3-4-2, the cylinders' gas-torque
 * pulses follow each other 180 deg apart instead of coinciding. */
static void check_firing_spacing(const EngineConfig *cfg, const EngineTrace *tr) {
  const double spacing = 720.0 / cfg->num_cylinders;
  for (int k = 0; k < cfg->num_cylinders; k++) {
    int a = cfg->firing_order[k] - 1;
    int b = cfg->firing_order[(k + 1) % cfg->num_cylinders] - 1;
    double gap = gap_deg(peak_gas_theta_deg(tr, a), peak_gas_theta_deg(tr, b));
    CHECK_NEAR(gap, spacing, 10.0);
  }
}

static void test_cylinders_fire_in_order_evenly_spaced(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.8, 10.0);
  CHECK(st.run_state == ENGINE_RUNNING);
  check_firing_spacing(&cfg, &tr);
}

/* firing_order must actually be read: 1-2-4-3 puts cylinder 2 second. */
static void test_alternate_firing_order_changes_the_sequence(void) {
  EngineConfig cfg = engine_config_default();
  cfg.firing_order[0] = 1;
  cfg.firing_order[1] = 2;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 3;
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.8, 10.0);
  check_firing_spacing(&cfg, &tr);
  CHECK_NEAR(gap_deg(peak_gas_theta_deg(&tr, 0), peak_gas_theta_deg(&tr, 1)),
             180.0, 10.0);
}

static void test_six_cylinder_spacing(void) {
  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = 6;
  engine_default_firing_order(6, cfg.firing_order);
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.8, 10.0);
  CHECK(st.run_state == ENGINE_RUNNING);
  check_firing_spacing(&cfg, &tr);
}

/* Staggering spreads the same work over the cycle: the summed torque swings
 * far less than N cylinders firing in unison would (approximated as N x one
 * cylinder's own gas+inertia torque), while the cycle mean is unchanged. */
static void test_staggering_reduces_ripple_and_conserves_mean(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.8, 10.0);

  int first;
  int n = newest_cycle(&tr, &first);
  CHECK(n > 300);
  double n_cyl = (double)cfg.num_cylinders;

  double tot_min = 1e30, tot_max = -1e30, tot_sum = 0.0;
  double uni_min = 1e30, uni_max = -1e30, uni_sum = 0.0;
  for (int i = 0; i < n; i++) {
    const EngineTraceSample *s = engine_trace_at(&tr, first + i);
    double total = s->torque_nm;
    double unison = n_cyl * (s->cyl_gas_nm[0] + s->cyl_inertia_nm[0]);
    tot_min = total < tot_min ? total : tot_min;
    tot_max = total > tot_max ? total : tot_max;
    tot_sum += total;
    uni_min = unison < uni_min ? unison : uni_min;
    uni_max = unison > uni_max ? unison : uni_max;
    uni_sum += unison;
  }
  double tot_mean = tot_sum / n;
  double uni_mean = uni_sum / n;

  CHECK((tot_max - tot_min) < 0.6 * (uni_max - uni_min));
  CHECK(tot_mean > 0.0);
  CHECK_NEAR(tot_mean, uni_mean, 0.10 * fabs(uni_mean) + 1.0);
}

/* Reciprocating-mass inertia torque does no net work over a full cycle, per
 * cylinder, at (nearly) constant speed. */
static void test_inertia_torque_averages_to_zero_over_a_cycle(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.8, 10.0);

  int first;
  int n = newest_cycle(&tr, &first);
  for (int c = 0; c < cfg.num_cylinders; c++) {
    double sum = 0.0, peak = 0.0;
    for (int i = 0; i < n; i++) {
      double v = engine_trace_at(&tr, first + i)->cyl_inertia_nm[c];
      sum += v;
      peak = fabs(v) > peak ? fabs(v) : peak;
    }
    CHECK(peak > 1.0); /* it is really there... */
    CHECK(fabs(sum / n) < 0.1 * peak); /* ...and cancels */
  }
}

/* A dead cylinder leaves a hole in the pulse train at its own firing slot,
 * not everywhere. */
static void test_dead_cylinder_drops_its_own_pulse(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  cyl[2].injector_flow_trim = 0.4; /* lambda 2.5: outside flammability -> misfire */
  static EngineTrace tr;
  EngineState st;
  run_traced(&st, &tr, &cfg, cyl, 0.9, 5.0);
  CHECK(st.run_state == ENGINE_RUNNING);

  int first;
  int n = newest_cycle(&tr, &first);
  double peak[ENGINE_MAX_CYLINDERS] = {0};
  for (int i = 0; i < n; i++) {
    const EngineTraceSample *s = engine_trace_at(&tr, first + i);
    for (int c = 0; c < cfg.num_cylinders; c++) {
      peak[c] = s->cyl_gas_nm[c] > peak[c] ? s->cyl_gas_nm[c] : peak[c];
    }
  }
  for (int c = 0; c < cfg.num_cylinders; c++) {
    if (c != 2) {
      CHECK(peak[2] < 0.5 * peak[c]);
    }
  }
}

/* Attaching a trace is observation only. */
static void test_trace_does_not_change_the_simulation(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  FuelConfig fuel_cfg = fuel_config_default();
  EngineInput in = make_input(0.7, 12.0, 101.325);

  static EngineTrace tr;
  EngineState with, without;
  engine_model_init(&with, &cfg);
  engine_model_init(&without, &cfg);
  CHECK(without.trace == NULL);
  with.trace = &tr;
  CylinderState cs_with[ENGINE_MAX_CYLINDERS], cs_without[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs_with);
  init_cyl_states(cs_without);
  for (int i = 0; i < 100; i++) {
    engine_model_step(&with, &cfg, &in, &fuel_cfg, cyl, cs_with,
                      TEST_INTAKE_TEMP_C, i * 0.02, 0.02);
    engine_model_step(&without, &cfg, &in, &fuel_cfg, cyl, cs_without,
                      TEST_INTAKE_TEMP_C, i * 0.02, 0.02);
  }
  CHECK(engine_trace_count(&tr) > 0);
  CHECK(with.omega_rad_s == without.omega_rad_s);
  CHECK(with.map_kpa == without.map_kpa);
  CHECK(with.theta_deg == without.theta_deg);
  CHECK(with.torque_nm == without.torque_nm);
}

/* Trace samples are the integrator's own numbers: the frame-mean torque is
 * the mean of that frame's samples. */
static void test_trace_total_matches_reported_mean_torque(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  default_cyl_configs(cyl);
  FuelConfig fuel_cfg = fuel_config_default();
  EngineInput in = make_input(0.7, 12.0, 101.325);
  static EngineTrace tr;
  EngineState st;
  engine_model_init(&st, &cfg);
  st.trace = &tr;
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  for (int i = 0; i < 50; i++) {
    engine_model_step(&st, &cfg, &in, &fuel_cfg, cyl, cs, TEST_INTAKE_TEMP_C,
                      i * 0.02, 0.02);
  }
  engine_trace_clear(&tr);
  engine_model_step(&st, &cfg, &in, &fuel_cfg, cyl, cs, TEST_INTAKE_TEMP_C, 1.0,
                    0.02);
  int n = engine_trace_count(&tr);
  CHECK(n > 10);
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += engine_trace_at(&tr, i)->torque_nm;
  }
  CHECK_NEAR(sum / n, st.torque_nm, 0.01 * fabs(st.torque_nm) + 0.01);
}

/* engine_model_start() resets crank angle to 0, so it drops the old trace. */
static void test_start_clears_the_trace(void) {
  EngineConfig cfg = engine_config_default();
  static EngineTrace tr;
  EngineState st;
  engine_model_init(&st, &cfg);
  st.trace = &tr;
  st.run_state = ENGINE_STOPPED;
  st.omega_rad_s = 0.0;
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineTraceSample s = {0};
  engine_trace_push(&tr, &s);
  CHECK(engine_trace_count(&tr) == 1);
  engine_model_start(&st, &cfg, cs);
  CHECK(engine_trace_count(&tr) == 0);
}

/* ---- gas exchange, friction and the idle governor ---- */

/* Mean total crank torque over the newest cycle of a motored engine (ignition
 * off, no load): what is left is compression/expansion asymmetry plus the
 * pumping loop. */
static double motored_mean_torque_nm(double throttle, double map0_kpa) {
  EngineConfig cfg = engine_config_default();
  cfg.idle_target_rpm = 0.0;
  static EngineTrace tr;
  EngineState st;
  engine_model_init(&st, &cfg);
  st.trace = &tr;
  st.ignition_on = 0;
  st.omega_rad_s = rpm_to_rad_s(1800.0);
  st.map_kpa = map0_kpa;
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput in = make_input(throttle, 0.0, 101.325);
  run(&st, cs, &cfg, &in, 0.3, 0.02);
  int first;
  int n = newest_cycle(&tr, &first);
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += engine_trace_at(&tr, first + i)->torque_nm;
  }
  return sum / n;
}

/* The pumping loop: with the throttle closed the piston pushes against
 * ambient-pressure exhaust and pulls against ~30 kPa intake, so a motored
 * engine is a net brake of about (ambient - MAP) * displacement / 4pi; wide
 * open there is (almost) none. */
static void test_closed_throttle_motoring_has_pumping_loss(void) {
  EngineConfig cfg = engine_config_default();
  EngineDerived d = engine_config_derived(&cfg);
  double expected_nm = (101.325 - 30.0) * 1000.0 * d.total_displacement_l *
                       1e-3 / (4.0 * UNITS_PI);

  double closed = motored_mean_torque_nm(0.0, 30.0);
  double wot = motored_mean_torque_nm(1.0, 101.325);
  CHECK(closed < -0.6 * expected_nm);
  CHECK(closed > -1.4 * expected_nm);
  CHECK(fabs(wot) < 0.25 * expected_nm);
}

static EngineConfig friction_only_cfg(int cylinders, double fmep_const,
                                      double fmep_speed, double viscous) {
  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = cylinders;
  engine_default_firing_order(cylinders, cfg.firing_order);
  cfg.friction_fmep_const_kpa = fmep_const;
  cfg.friction_fmep_per_ms_kpa = fmep_speed;
  cfg.friction_coeff_nm_per_rad_s = viscous;
  return cfg;
}

static void test_friction_scales_with_displacement(void) {
  const double w = rpm_to_rad_s(2000.0);
  EngineConfig four = friction_only_cfg(4, 40.0, 14.0, 0.02);
  EngineConfig six = friction_only_cfg(6, 40.0, 14.0, 0.02);
  double viscous = 0.02 * w;
  double f4 = engine_friction_torque_nm(&four, w) - viscous;
  double f6 = engine_friction_torque_nm(&six, w) - viscous;
  CHECK_NEAR(f6 / f4, 1.5, 1e-9); /* the FMEP part follows displacement */

  /* and equals displacement / 4pi * FMEP, FMEP = const + speed * piston speed */
  double vd_m3 = cylinder_displacement_m3(&four.geom) * 4.0;
  double c_m = 2.0 * four.geom.stroke_m * 2000.0 / 60.0;
  double fmep_pa = (40.0 + 14.0 * c_m) * 1000.0;
  CHECK_NEAR(f4, vd_m3 * fmep_pa / (4.0 * UNITS_PI), 1e-9);
}

static void test_friction_grows_with_speed_and_vanishes_at_rest(void) {
  EngineConfig cfg = engine_config_default();
  CHECK_NEAR(engine_friction_torque_nm(&cfg, 0.0), 0.0, 1e-12);
  CHECK_NEAR(engine_friction_torque_nm(&cfg, -50.0), 0.0, 1e-12);
  double lo = engine_friction_torque_nm(&cfg, rpm_to_rad_s(500.0));
  double mid = engine_friction_torque_nm(&cfg, rpm_to_rad_s(1500.0));
  double hi = engine_friction_torque_nm(&cfg, rpm_to_rad_s(3000.0));
  CHECK(lo > 0.0 && mid > lo && hi > mid);
  /* a constant part: friction at very low speed is still well above zero */
  CHECK(engine_friction_torque_nm(&cfg, rpm_to_rad_s(50.0)) > 3.0);
}

static EngineConfig idle_cfg(double target_rpm) {
  EngineConfig cfg = engine_config_default();
  cfg.idle_target_rpm = target_rpm;
  return cfg;
}

/* Settled closed-throttle engine (no prop at this level, so a small flat load
 * stands in for it). Returns the state after `seconds`. */
static EngineState idle_run(const EngineConfig *cfg, double load_nm,
                            double seconds) {
  EngineState st;
  engine_model_init(&st, cfg);
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput in = make_input(0.0, load_nm, 101.325);
  run(&st, cs, cfg, &in, seconds, 0.02);
  return st;
}

static void test_idle_governor_holds_the_target(void) {
  for (double target = 700.0; target <= 1100.0; target += 200.0) {
    EngineConfig cfg = idle_cfg(target);
    EngineState st = idle_run(&cfg, 4.0, 40.0);
    CHECK(st.run_state == ENGINE_RUNNING);
    CHECK_NEAR(engine_model_rpm(&st), target, 20.0);
    CHECK(st.ecu.idle_throttle > 0.0);
    CHECK(st.ecu.idle_throttle < cfg.idle_max_throttle);
  }
}

static void test_idle_governor_off_or_pointless_adds_nothing(void) {
  EngineConfig off = idle_cfg(0.0);
  EngineState a = idle_run(&off, 1.0, 40.0);
  CHECK_NEAR(a.ecu.idle_throttle, 0.0, 1e-12);
  CHECK(a.run_state == ENGINE_RUNNING);

  /* a target below where the engine idles by itself: the governor can only
   * add throttle, so it stays out of the way and the engine idles naturally */
  EngineConfig low = idle_cfg(200.0);
  EngineState b = idle_run(&low, 1.0, 40.0);
  CHECK_NEAR(b.ecu.idle_throttle, 0.0, 1e-9);
  CHECK_NEAR(engine_model_rpm(&b), engine_model_rpm(&a), 5.0);
  CHECK(engine_model_rpm(&b) > 400.0);
}

/* With the governor a load step at idle is absorbed: the RPM returns to the
 * target and the governor opens up to carry the extra load. */
static void test_idle_governor_recovers_from_a_load_step(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput light = make_input(0.0, 4.0, 101.325);
  EngineInput heavy = make_input(0.0, 7.0, 101.325);
  run(&st, cs, &cfg, &light, 30.0, 0.02);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 20.0);
  double gov_light = st.ecu.idle_throttle;

  run(&st, cs, &cfg, &heavy, 30.0, 0.02);
  CHECK(st.run_state == ENGINE_RUNNING);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 25.0);
  CHECK(st.ecu.idle_throttle > gov_light);
}

/* Authority is finite: an overload the governor can't carry leaves RPM below
 * the target with the governor pinned at its limit -- and when the overload
 * goes away the integrator hasn't wound up, so there is no big overshoot. */
static void test_idle_governor_authority_limit_and_no_windup(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput overload = make_input(0.0, 12.0, 101.325);
  run(&st, cs, &cfg, &overload, 30.0, 0.02);
  CHECK(st.run_state == ENGINE_RUNNING);
  CHECK(engine_model_rpm(&st) < 750.0);
  CHECK_NEAR(st.ecu.idle_throttle, cfg.idle_max_throttle, 1e-9);
  CHECK(st.ecu.idle_i_term <= cfg.idle_max_throttle + 1e-12);

  EngineInput normal = make_input(0.0, 4.0, 101.325);
  double peak = 0.0;
  for (int i = 0; i < 1000; i++) { /* 20 s */
    run(&st, cs, &cfg, &normal, 0.02, 0.02);
    peak = fmax(peak, engine_model_rpm(&st));
  }
  CHECK(peak < 800.0 + 200.0);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 25.0);
}

/* The live operator switch: turning the governor off mid-run lets idle sag to
 * the engine's natural speed (without stalling it); turning it back on brings
 * it home again. */
static void test_idle_governor_can_be_toggled_during_a_run(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput in = make_input(0.0, 1.0, 101.325);

  run(&st, cs, &cfg, &in, 30.0, 0.02);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 20.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
  CHECK(st.ecu.idle_throttle > 0.0);

  ecu_set_idle_enabled(&st.ecu, 0);
  run(&st, cs, &cfg, &in, 30.0, 0.02);
  CHECK(st.run_state == ENGINE_RUNNING); /* sags, doesn't stall */
  CHECK(st.ecu.idle_mode == ECU_IDLE_OFF);
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(st.ecu.throttle_cmd, 0.0, 0.0);
  CHECK(engine_model_rpm(&st) < 700.0);
  CHECK(engine_model_rpm(&st) > 400.0);

  ecu_set_idle_enabled(&st.ecu, 1);
  run(&st, cs, &cfg, &in, 40.0, 0.02);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 20.0);
}

/* The command the engine receives is the ECU's, and it is what drives MAP. */
static void test_engine_receives_the_ecu_throttle_command(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st = idle_run(&cfg, 4.0, 30.0);
  CHECK_NEAR(st.ecu.pilot_throttle, 0.0, 0.0);
  CHECK(st.ecu.throttle_cmd > 0.05);
  CHECK_NEAR(st.ecu.throttle_cmd, st.ecu.idle_throttle, 1e-12);
  /* MAP sits above the closed-throttle value by the governor's opening */
  double closed_map = 101.325 * 0.2963;
  CHECK(st.map_kpa > closed_map + 3.0);
}

/* A stopped engine reports the governor standing by, not a stale loop. */
static void test_stopped_engine_reports_standby(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st = idle_run(&cfg, 4.0, 20.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
  st.run_state = ENGINE_STOPPED;
  st.omega_rad_s = 0.0;
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput in = make_input(0.0, 4.0, 101.325);
  run(&st, cs, &cfg, &in, 0.1, 0.02);
  CHECK(st.ecu.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(st.ecu.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(st.ecu.throttle_cmd, 0.0, 0.0);
}

/* Above its authority the governor steps aside: the pilot's throttle is all
 * that counts, so the engine behaves exactly as with no governor. */
static void test_idle_governor_steps_aside_for_the_pilot(void) {
  EngineConfig on = idle_cfg(800.0);
  EngineConfig off = idle_cfg(0.0);
  EngineState a, b;
  engine_model_init(&a, &on);
  engine_model_init(&b, &off);
  CylinderState ca[ENGINE_MAX_CYLINDERS], cb[ENGINE_MAX_CYLINDERS];
  init_cyl_states(ca);
  init_cyl_states(cb);
  EngineInput in = make_input(0.6, 8.0, 101.325);
  run(&a, ca, &on, &in, 20.0, 0.02);
  run(&b, cb, &off, &in, 20.0, 0.02);
  CHECK_NEAR(a.omega_rad_s, b.omega_rad_s, 1e-6);
  CHECK_NEAR(a.map_kpa, b.map_kpa, 1e-6);
}

/* The chop from cruise: closing the throttle from 0.8 must not stall the
 * engine, and it settles at the governed idle. */
static void test_idle_governor_survives_a_throttle_chop(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st;
  engine_model_init(&st, &cfg);
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput cruise = make_input(0.8, 6.0, 101.325);
  EngineInput chop = make_input(0.0, 6.0, 101.325);
  run(&st, cs, &cfg, &cruise, 20.0, 0.02);
  CHECK(engine_model_rpm(&st) > 1500.0);
  double lowest = 1e9;
  for (int i = 0; i < 1500; i++) { /* 30 s at closed throttle */
    run(&st, cs, &cfg, &chop, 0.02, 0.02);
    lowest = fmin(lowest, engine_model_rpm(&st));
    CHECK(st.run_state == ENGINE_RUNNING);
  }
  CHECK(lowest > 400.0);
  CHECK_NEAR(engine_model_rpm(&st), 800.0, 25.0);
}

/* The governor belongs to a running engine: nothing with the ignition off or
 * while cranking. */
static void test_idle_governor_only_acts_on_a_running_engine(void) {
  EngineConfig cfg = idle_cfg(800.0);
  EngineState st = idle_run(&cfg, 4.0, 20.0);
  CHECK(st.ecu.idle_throttle > 0.0);

  engine_model_stop(&st); /* ignition off, still spinning down */
  CylinderState cs[ENGINE_MAX_CYLINDERS];
  init_cyl_states(cs);
  EngineInput in = make_input(0.0, 4.0, 101.325);
  run(&st, cs, &cfg, &in, 0.1, 0.02);
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 1e-12);
  CHECK_NEAR(st.ecu.idle_i_term, 0.0, 1e-12);

  st.run_state = ENGINE_STOPPED;
  st.omega_rad_s = 0.0;
  engine_model_start(&st, &cfg, cs);
  CHECK(st.run_state == ENGINE_CRANKING);
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 1e-12);
  CHECK_NEAR(st.ecu.idle_i_term, 0.0, 1e-12);
}

/* Closed-throttle MAP follows ambient: a fixed vacuum drop went to zero at
 * altitude and starved the engine. */
static void test_closed_throttle_map_scales_with_ambient(void) {
  EngineConfig cfg = idle_cfg(0.0);
  double ratio_sl = 0.0, ratio_alt = 0.0;
  for (int k = 0; k < 2; k++) {
    double amb = k == 0 ? 101.325 : 70.1; /* ~3000 m */
    EngineState st;
    engine_model_init(&st, &cfg);
    CylinderState cs[ENGINE_MAX_CYLINDERS];
    init_cyl_states(cs);
    EngineInput in = make_input(0.0, 2.0, amb);
    st.map_kpa = amb;
    run(&st, cs, &cfg, &in, 3.0, 0.02);
    if (k == 0) {
      ratio_sl = st.map_kpa / amb;
    } else {
      ratio_alt = st.map_kpa / amb;
    }
  }
  CHECK_NEAR(ratio_sl, 0.2963, 0.01);
  CHECK_NEAR(ratio_alt, ratio_sl, 0.005);
}

static const TestCase CASES[] = {
    {"engine_model.init_is_cold_idle", test_init_is_cold_idle},
    {"engine_model.config_defaults_are_positive",
     test_config_defaults_are_positive},
    {"engine_model.wot_settles_to_plausible_steady_state",
     test_wot_settles_to_plausible_steady_state},
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
    {"engine_model.theta_advances_and_wraps", test_theta_advances_and_wraps},
    {"engine_model.cylinder_pressure_stays_sane_and_peaks_above_map",
     test_cylinder_pressure_stays_sane_and_peaks_above_map},
    {"engine_model.mean_torque_same_order_of_magnitude_as_old_curve",
     test_mean_torque_same_order_of_magnitude_as_old_curve},
    {"engine_model.engine_stalls_instead_of_reversing",
     test_engine_stalls_instead_of_reversing},
    {"engine_model.starter_alone_can_crank_engine_up",
     test_starter_alone_can_crank_engine_up},
    {"engine_model.phase_offsets_follow_firing_order",
     test_phase_offsets_follow_firing_order},
    {"engine_model.phase_offsets_fall_back_on_invalid_firing_order",
     test_phase_offsets_fall_back_on_invalid_firing_order},
    {"engine_model.cylinders_fire_in_order_evenly_spaced",
     test_cylinders_fire_in_order_evenly_spaced},
    {"engine_model.alternate_firing_order_changes_the_sequence",
     test_alternate_firing_order_changes_the_sequence},
    {"engine_model.six_cylinder_spacing", test_six_cylinder_spacing},
    {"engine_model.staggering_reduces_ripple_and_conserves_mean",
     test_staggering_reduces_ripple_and_conserves_mean},
    {"engine_model.inertia_torque_averages_to_zero_over_a_cycle",
     test_inertia_torque_averages_to_zero_over_a_cycle},
    {"engine_model.dead_cylinder_drops_its_own_pulse",
     test_dead_cylinder_drops_its_own_pulse},
    {"engine_model.trace_does_not_change_the_simulation",
     test_trace_does_not_change_the_simulation},
    {"engine_model.trace_total_matches_reported_mean_torque",
     test_trace_total_matches_reported_mean_torque},
    {"engine_model.start_clears_the_trace", test_start_clears_the_trace},
    {"engine_model.closed_throttle_motoring_has_pumping_loss",
     test_closed_throttle_motoring_has_pumping_loss},
    {"engine_model.friction_scales_with_displacement",
     test_friction_scales_with_displacement},
    {"engine_model.friction_grows_with_speed_and_vanishes_at_rest",
     test_friction_grows_with_speed_and_vanishes_at_rest},
    {"engine_model.idle_governor_holds_the_target",
     test_idle_governor_holds_the_target},
    {"engine_model.idle_governor_off_or_pointless_adds_nothing",
     test_idle_governor_off_or_pointless_adds_nothing},
    {"engine_model.idle_governor_recovers_from_a_load_step",
     test_idle_governor_recovers_from_a_load_step},
    {"engine_model.idle_governor_authority_limit_and_no_windup",
     test_idle_governor_authority_limit_and_no_windup},
    {"engine_model.idle_governor_can_be_toggled_during_a_run",
     test_idle_governor_can_be_toggled_during_a_run},
    {"engine_model.engine_receives_the_ecu_throttle_command",
     test_engine_receives_the_ecu_throttle_command},
    {"engine_model.stopped_engine_reports_standby",
     test_stopped_engine_reports_standby},
    {"engine_model.idle_governor_steps_aside_for_the_pilot",
     test_idle_governor_steps_aside_for_the_pilot},
    {"engine_model.idle_governor_survives_a_throttle_chop",
     test_idle_governor_survives_a_throttle_chop},
    {"engine_model.idle_governor_only_acts_on_a_running_engine",
     test_idle_governor_only_acts_on_a_running_engine},
    {"engine_model.closed_throttle_map_scales_with_ambient",
     test_closed_throttle_map_scales_with_ambient},
};

RUN_TESTS(CASES)
