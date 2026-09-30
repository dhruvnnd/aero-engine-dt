#include "test_util.h"

#include "physics/crank_thermo.h"
#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "physics/engine_trace.h"
#include "physics/fuel.h"
#include "telemetry/pv_diagram.h"

#define TEST_INTAKE_TEMP_C 15.0

/* ~700 KB and ~100 KB respectively: keep them off the stack */
static EngineTrace g_trace;
static PvLoop g_loop;

/* Pushes `cycles` whole crank cycles at `step` deg per sample, the pressure of
 * cylinder 0 given by fn(local angle). Cylinder 0 has phase 0 in a one-cylinder
 * engine, so crank angle and local angle coincide. */
typedef double (*PressureFn)(double local_deg, int cycle);

static void push_cycles(int cycles, double step, PressureFn fn) {
  engine_trace_clear(&g_trace);
  for (int c = 0; c < cycles; c++) {
    for (double a = 0.0; a < 720.0 - 1e-9; a += step) {
      EngineTraceSample s = {0};
      s.theta_deg = (float)a;
      s.omega_rad_s = 200.0f;
      s.cyl_pressure_kpa[0] = (float)fn(a, c);
      engine_trace_push(&g_trace, &s);
    }
  }
}

static EngineConfig one_cyl_config(void) {
  EngineConfig cfg = engine_config_default();
  cfg.num_cylinders = 1;
  engine_default_firing_order(1, cfg.firing_order);
  return cfg;
}

/* Ideal closed-cycle compression/expansion curve through (Vc, 100 kPa * CR^n)
 * -- no combustion, no valves: a polytrope all the way round. */
static double polytrope_p(double local_deg, int cycle) {
  (void)cycle;
  EngineConfig cfg = one_cyl_config();
  const double v = pv_volume_cc(&cfg, NULL, local_deg);
  const double v_bdc = pv_volume_cc(&cfg, NULL, 180.0);
  return 100.0 * pow(v_bdc / v, PV_MODEL_POLYTROPIC_N);
}

/* 200 kPa during the expansion stroke, 100 kPa the rest of the cycle. The
 * closed loop then encloses (200 - 100) * displacement of net work. */
static double stepped_p(double local_deg, int cycle) {
  (void)cycle;
  return local_deg < 180.0 ? 200.0 : 100.0;
}

static double cycle_tagged_p(double local_deg, int cycle) {
  (void)local_deg;
  return 100.0 + 50.0 * cycle;
}

static void test_volume_matches_geometry(void) {
  const EngineConfig cfg = one_cyl_config();
  const double v_tdc = pv_volume_cc(&cfg, NULL, 0.0);
  const double v_bdc = pv_volume_cc(&cfg, NULL, 180.0);
  const double disp_cc = cylinder_displacement_m3(&cfg.geom) * 1.0e6;
  CHECK_NEAR(v_bdc - v_tdc, disp_cc, 1e-6);
  CHECK_NEAR(v_bdc / v_tdc, cfg.geom.compression_ratio, 1e-6);

  /* compression trim lowers the effective ratio, and so raises the clearance */
  CylinderConfig weak = cylinder_config_default();
  weak.compression_trim = 0.8;
  CHECK(pv_volume_cc(&cfg, &weak, 0.0) > v_tdc);
  CHECK_NEAR(pv_volume_cc(&cfg, &weak, 180.0) / pv_volume_cc(&cfg, &weak, 0.0),
             0.8 * cfg.geom.compression_ratio, 1e-6);
}

static void test_empty_or_bad_arguments_give_nothing(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  engine_trace_clear(&g_trace);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st) == 0);
  CHECK(st.valid == 0);
  CHECK(g_loop.n == 0);

  push_cycles(2, 1.0, stepped_p);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 1, 0, &g_loop, &st) == 0);  /* no cyl 1 */
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, -1, 0, &g_loop, &st) == 0);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, -1, &g_loop, &st) == 0);
  CHECK(pv_loop_build(NULL, &cfg, NULL, 0, 0, &g_loop, &st) == 0);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, NULL) > 0); /* stats optional */
}

static void test_loop_is_one_cycle_in_increasing_angle(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  push_cycles(3, 1.0, polytrope_p);
  const int n = pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st);
  CHECK(st.valid == 1);
  CHECK(n >= 719 && n <= 721);
  CHECK(g_loop.n == n);
  for (int k = 1; k < n; k++) {
    CHECK(g_loop.angle_deg[k] > g_loop.angle_deg[k - 1]);
  }
  CHECK(g_loop.angle_deg[0] < 2.0f);
  CHECK(g_loop.angle_deg[n - 1] > 717.0f);
}

static void test_a_partial_cycle_is_not_a_loop(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  engine_trace_clear(&g_trace);
  for (double a = 0.0; a < 500.0; a += 1.0) {
    EngineTraceSample s = {0};
    s.theta_deg = (float)a;
    engine_trace_push(&g_trace, &s);
  }
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st) == 0);
  CHECK(st.valid == 0);
}

static void test_closed_polytrope_does_no_net_work_and_fits_its_index(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  push_cycles(2, 0.5, polytrope_p);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st) > 0);
  CHECK(st.valid == 1);

  /* the same curve out and back encloses no area */
  CHECK_NEAR(st.work_net_j, 0.0, 0.5);
  CHECK_NEAR(st.imep_net_kpa, 0.0, 10.0);

  CHECK_NEAR(st.compression_n, PV_MODEL_POLYTROPIC_N, 0.01);

  /* highest at minimum volume, which is firing TDC */
  CHECK_NEAR(st.peak_angle_deg, 0.0, 2.0);
  CHECK_NEAR(st.peak_kpa, 100.0 * pow(cfg.geom.compression_ratio,
                                      PV_MODEL_POLYTROPIC_N),
             st.peak_kpa * 0.01);
  CHECK_NEAR(st.comp_ratio, cfg.geom.compression_ratio, 1e-6);
  CHECK_NEAR(st.rpm, 200.0 * 60.0 / (2.0 * 3.14159265358979), 0.5);
}

static void test_net_work_is_the_enclosed_area(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  push_cycles(2, 0.5, stepped_p);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st) > 0);

  const double disp_cc = st.v_max_cc - st.v_min_cc;
  /* (200 - 100) kPa over the displaced volume: IMEP is the 100 kPa step */
  CHECK_NEAR(st.imep_net_kpa, 100.0, 3.0);
  CHECK_NEAR(st.work_net_j, 100.0 * disp_cc * 1e-3, 0.05 * 100.0 * disp_cc * 1e-3);
  /* the exhaust/intake strokes here are at one constant pressure: no pumping */
  CHECK_NEAR(st.pmep_kpa, 0.0, 3.0);
  CHECK_NEAR(st.imep_gross_kpa, st.imep_net_kpa, 3.0);
  CHECK_NEAR(st.power_kw, st.work_net_j * st.rpm / 120.0 / 1000.0, 1e-9);
}

static void test_cycles_back_selects_older_cycles(void) {
  const EngineConfig cfg = one_cyl_config();
  PvStats st;
  /* cycles carry 100, 150, 200, 250 kPa; the oldest is cut short by where the
   * trace starts, so only the newer three are whole cycles */
  push_cycles(4, 1.0, cycle_tagged_p);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 0, &g_loop, &st) > 0);
  CHECK_NEAR(g_loop.p_kpa[0], 250.0, 1e-3);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 1, &g_loop, &st) > 0);
  CHECK_NEAR(g_loop.p_kpa[0], 200.0, 1e-3);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 2, &g_loop, &st) > 0);
  CHECK_NEAR(g_loop.p_kpa[0], 150.0, 1e-3);
  CHECK(pv_loop_build(&g_trace, &cfg, NULL, 0, 10, &g_loop, &st) == 0);
  CHECK(st.valid == 0);
}

/* ---- against the real engine model ---------------------------------- */

static void run_engine(const EngineConfig *cfg, CylinderConfig *cyl,
                       double throttle, double load) {
  CylinderState cyl_states[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cylinder_state_init(&cyl_states[i], TEST_INTAKE_TEMP_C);
  }
  FuelConfig fuel_cfg = fuel_config_default();
  static EngineState st;
  engine_model_init(&st, cfg);
  engine_trace_clear(&g_trace);
  st.trace = &g_trace;
  EngineInput in;
  in.throttle = throttle;
  in.load_torque_nm = load;
  in.ambient_pressure_kpa = 101.325;
  double t = 0.0;
  for (int i = 0; i < 200; i++) { /* 4 s */
    engine_model_step(&st, cfg, &in, &fuel_cfg, cyl, cyl_states,
                      TEST_INTAKE_TEMP_C, t, 0.02);
    t += 0.02;
  }
}

static void test_real_engine_compression_follows_the_model_index(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  run_engine(&cfg, cyl, 0.8, 10.0);

  for (int c = 0; c < cfg.num_cylinders; c++) {
    PvStats st;
    CHECK(pv_loop_build(&g_trace, &cfg, &cyl[c], c, 0, &g_loop, &st) > 0);
    CHECK(st.valid == 1);
    /* before the burn starts the gas is just the model's polytrope */
    CHECK_NEAR(st.compression_n, PV_MODEL_POLYTROPIC_N, 0.03);
    /* a firing cylinder does positive work, peaking shortly after firing TDC
     * -- the same angle for every cylinder once phase is accounted for */
    CHECK(st.imep_net_kpa > 100.0);
    CHECK(st.peak_angle_deg > 0.0 && st.peak_angle_deg < 40.0);
    CHECK(st.peak_kpa > st.ivc_p_kpa * 5.0);
    CHECK(st.power_kw > 0.0);
  }
}

static void test_real_engine_part_throttle_pays_a_pumping_loss(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  run_engine(&cfg, cyl, 0.3, 5.0);
  PvStats part;
  CHECK(pv_loop_build(&g_trace, &cfg, &cyl[0], 0, 0, &g_loop, &part) > 0);

  run_engine(&cfg, cyl, 1.0, 5.0);
  PvStats wide;
  CHECK(pv_loop_build(&g_trace, &cfg, &cyl[0], 0, 0, &g_loop, &wide) > 0);

  CHECK(part.pmep_kpa > wide.pmep_kpa);
  CHECK(part.imep_gross_kpa > part.imep_net_kpa); /* net = gross - pumping */
  CHECK(wide.peak_kpa > part.peak_kpa);
}

static void test_real_engine_low_compression_shows_up_in_the_loop(void) {
  EngineConfig cfg = engine_config_default();
  CylinderConfig cyl[ENGINE_MAX_CYLINDERS];
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cyl[i] = cylinder_config_default();
  }
  cyl[2].compression_trim = 0.7;
  run_engine(&cfg, cyl, 0.8, 10.0);

  PvStats good, bad;
  CHECK(pv_loop_build(&g_trace, &cfg, &cyl[0], 0, 0, &g_loop, &good) > 0);
  CHECK(pv_loop_build(&g_trace, &cfg, &cyl[2], 2, 0, &g_loop, &bad) > 0);
  CHECK_NEAR(bad.comp_ratio, 0.7 * cfg.geom.compression_ratio, 1e-6);
  CHECK(bad.peak_kpa < good.peak_kpa);
  CHECK(bad.imep_net_kpa < good.imep_net_kpa);
}

static const TestCase tests[] = {
    {"volume_matches_geometry", test_volume_matches_geometry},
    {"empty_or_bad_arguments_give_nothing",
     test_empty_or_bad_arguments_give_nothing},
    {"loop_is_one_cycle_in_increasing_angle",
     test_loop_is_one_cycle_in_increasing_angle},
    {"a_partial_cycle_is_not_a_loop", test_a_partial_cycle_is_not_a_loop},
    {"closed_polytrope_does_no_net_work_and_fits_its_index",
     test_closed_polytrope_does_no_net_work_and_fits_its_index},
    {"net_work_is_the_enclosed_area", test_net_work_is_the_enclosed_area},
    {"cycles_back_selects_older_cycles",
     test_cycles_back_selects_older_cycles},
    {"real_engine_compression_follows_the_model_index",
     test_real_engine_compression_follows_the_model_index},
    {"real_engine_part_throttle_pays_a_pumping_loss",
     test_real_engine_part_throttle_pays_a_pumping_loss},
    {"real_engine_low_compression_shows_up_in_the_loop",
     test_real_engine_low_compression_shows_up_in_the_loop},
};

RUN_TESTS(tests)
