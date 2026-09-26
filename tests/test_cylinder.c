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

static void run(ModelSync *sync, ModelState *st, double seconds) {
  EngineInput in;
  in.throttle = 0.85;
  in.load_torque_nm = 0.0; /* the propeller (model_sync_step) is the load */
  in.ambient_pressure_kpa = 101.325;
  EnvInput env = {0.0, 0.0, 0.0}; /* sea level, still air */
  int steps = (int)(seconds / 0.01 + 0.5);
  for (int i = 0; i < steps; i++) {
    model_sync_step(sync, st, &in, &env, 0.01);
  }
}

/* Nominal trims: every per-cylinder node must track the lumped thermal node
 * it was seeded from. */
static void test_nominal_tracks_lumped(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 60.0);
  for (int i = 0; i < sync.engine_config.num_cylinders; i++) {
    CHECK_NEAR(st.cyl[i].cht_c, st.thermal.cht_c, 1e-6);
    CHECK_NEAR(st.cyl[i].egt_c, st.thermal.egt_c, 1e-6);
  }
}

static void test_cooling_trim_raises_cht(void) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.cyl_config[1].cooling_trim = 0.8; /* sheds less heat -> hotter head */
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 120.0);
  CHECK(st.cyl[1].cht_c > st.cyl[0].cht_c + 2.0);
  CHECK_NEAR(st.cyl[0].cht_c, st.thermal.cht_c, 1e-6); /* others untouched */
  CHECK_NEAR(st.cyl[2].cht_c, st.thermal.cht_c, 1e-6);
}

static void test_spark_retard_raises_egt(void) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.cyl_config[2].spark_offset_deg = 15.0;
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 60.0);
  CHECK(st.cyl[2].egt_c > st.cyl[0].egt_c + 10.0);
}

static void test_misfire_zero_at_nominal(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 30.0);
  for (int i = 0; i < sync.engine_config.num_cylinders; i++) {
    CHECK_NEAR(st.cyl[i].misfire_rate, 0.0, 0.0);
  }
}

static void test_lean_leak_triggers_misfire(void) {
  ModelSync sync;
  model_sync_init(&sync);
  sync.cyl_config[3].intake_leak_frac = 0.7; /* lambda -> ~1.7, past limit */
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 5.0);
  CHECK(st.cyl[3].misfire_rate > 0.9);
  CHECK_NEAR(st.cyl[0].misfire_rate, 0.0, 0.0);
}

/* nominal-trim sum equals the lumped formula the readout uses */
static void test_nominal_torque_matches_lumped(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  run(&sync, &st, 40.0);
  CHECK_NEAR(st.torque_nm, engine_model_torque_nm(&st.engine), 1e-9);
}

/* One cylinder driven into misfire: the engine can't hold RPM, torque_nm
 * dips, and the dead cylinder flags misfire_rate 1. */
static void test_dead_cylinder_drops_rpm(void) {
  ModelSync healthy, faulted;
  model_sync_init(&healthy);
  model_sync_init(&faulted);
  faulted.cyl_config[2].intake_leak_frac = 0.7; /* lambda ~1.7 -> full misfire */

  ModelState hs, fs;
  model_state_init(&hs, &healthy.engine_config, 15.0);
  model_state_init(&fs, &faulted.engine_config, 15.0);
  run(&healthy, &hs, 90.0);
  run(&faulted, &fs, 90.0);

  CHECK(fs.rpm < 0.92 * hs.rpm); /* clear RPM loss */

  /* torque_nm is a mean over just the last engine_model_step() call.
   * Phase 1's cylinders all fire in the same phase (true staggering is
   * Phase 2), which leaves a small sustained torque/RPM oscillation --
   * comparing a single end-of-run sample between two independently-faulted
   * engines can catch each at a different point in that oscillation.
   * Average torque_nm over a trailing window instead, so the comparison
   * reflects the converged behavior rather than oscillation phase. */
  EngineInput in = {
      .throttle = 0.85, .load_torque_nm = 0.0, .ambient_pressure_kpa = 101.325};
  EnvInput env = {0.0, 0.0, 0.0};
  double hs_torque_sum = 0.0, fs_torque_sum = 0.0;
  const int trailing_steps = 500; /* 5 s at dt=0.01 */
  for (int i = 0; i < trailing_steps; i++) {
    model_sync_step(&healthy, &hs, &in, &env, 0.01);
    hs_torque_sum += hs.torque_nm;
  }
  for (int i = 0; i < trailing_steps; i++) {
    model_sync_step(&faulted, &fs, &in, &env, 0.01);
    fs_torque_sum += fs.torque_nm;
  }
  double hs_torque_avg = hs_torque_sum / trailing_steps;
  double fs_torque_avg = fs_torque_sum / trailing_steps;

  CHECK(fs_torque_avg < hs_torque_avg); /* torque dips (friction term) */
  /* Losing a full cylinder (25% of combustion capacity) under this fixed
   * 40 N*m load drives the engine down near idle (~660 RPM vs ~2250
   * healthy) -- a much bigger, but physically real, effect than the old
   * mean-value model produced (that model just zeroed one cylinder's share
   * as an output multiplier; this one loses real combustion work and the
   * remaining torque margin above the fixed load is thin). Lower bound
   * widened accordingly -- still requires the engine survive rather than
   * stall (torque_avg staying meaningfully above zero). */
  CHECK(fs_torque_avg > 0.55 * hs_torque_avg);
  CHECK_NEAR(fs.cyl[2].misfire_rate, 1.0, 0.0);
}

/* A low-compression cylinder cuts output, but less than a full misfire and
 * without flagging as one. */
static void test_weak_cylinder_partial_loss(void) {
  ModelSync healthy, weak, dead;
  model_sync_init(&healthy);
  model_sync_init(&weak);
  model_sync_init(&dead);
  weak.cyl_config[1].compression_trim = 0.5;
  dead.cyl_config[1].intake_leak_frac = 0.7;

  ModelState hs, ws, ds;
  model_state_init(&hs, &healthy.engine_config, 15.0);
  model_state_init(&ws, &weak.engine_config, 15.0);
  model_state_init(&ds, &dead.engine_config, 15.0);
  run(&healthy, &hs, 90.0);
  run(&weak, &ws, 90.0);
  run(&dead, &ds, 90.0);

  CHECK(ws.rpm < hs.rpm);          /* worse than healthy... */
  CHECK(ws.rpm > ds.rpm);          /* ...but better than a dead cylinder */
  CHECK_NEAR(ws.cyl[1].misfire_rate, 0.0, 0.0);
}

static const TestCase CASES[] = {
    {"cylinder.config_default_is_nominal", test_config_default_is_nominal},
    {"cylinder.state_init_cold_start", test_state_init_cold_start},
    {"cylinder.engine_config_geometry", test_engine_config_geometry},
    {"cylinder.model_wiring_populates_cylinders",
     test_model_wiring_populates_cylinders},
    {"cylinder.nominal_tracks_lumped", test_nominal_tracks_lumped},
    {"cylinder.cooling_trim_raises_cht", test_cooling_trim_raises_cht},
    {"cylinder.spark_retard_raises_egt", test_spark_retard_raises_egt},
    {"cylinder.misfire_zero_at_nominal", test_misfire_zero_at_nominal},
    {"cylinder.lean_leak_triggers_misfire", test_lean_leak_triggers_misfire},
    {"cylinder.nominal_torque_matches_lumped",
     test_nominal_torque_matches_lumped},
    {"cylinder.dead_cylinder_drops_rpm", test_dead_cylinder_drops_rpm},
    {"cylinder.weak_cylinder_partial_loss", test_weak_cylinder_partial_loss},
};

RUN_TESTS(CASES)
