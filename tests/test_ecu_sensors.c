#include "test_util.h"

#include <string.h>

#include "model/state.h"
#include "model/sync.h"
#include "physics/ecu.h"
#include "physics/engine_model.h"

/* The ECU's two speed sensors and its diagnostics against the real engine: what
 * a lying sensor does to a blind ECU, and what the checks and fallback do about
 * it. Flat loads stand in for the propeller (cq_static = 0). */

static ModelSync make_sync(int fitted, double target_rpm) {
  ModelSync s;
  model_sync_init(&s);
  s.engine_config.ecu_fitted = fitted;
  s.engine_config.ecu.idle_target_rpm = target_rpm;
  s.engine_config.prop.cq_static = 0.0;
  return s;
}

static ModelState make_state(const ModelSync *s) {
  ModelState st;
  model_state_init(&st, &s->engine_config, 15.0);
  return st;
}

static void run(ModelSync *s, ModelState *st, double throttle, double load_nm,
                double seconds) {
  const EngineInput in = {throttle, load_nm, 101.325};
  const EnvInput env = {0.0, 0.0, 0.0}; /* sea level, still air */
  const int n = (int)(seconds / 0.02 + 0.5);
  for (int i = 0; i < n; i++) {
    model_sync_step(s, st, &in, &env, 0.02);
  }
}

/* ---- the ECU with a lying crank-speed sensor ---- */

/* A governed idle, settled, ready to have a fault injected. */
static ModelState settled_idle(ModelSync *s) {
  ModelState st = make_state(s);
  run(s, &st, 0.0, 4.0, 40.0);
  return st;
}

/* The next four show what a lying sensor does to an ECU that trusts it blindly,
 * so they run with the ECU's diagnostics switched off. */

/* Reads 0: the ECU sees a huge error and opens up to its authority, so the real
 * engine runs well above idle. It doesn't stall, and it recovers when cleared. */
static void test_dropout_makes_the_governor_flare(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  CHECK_NEAR(st.rpm, 800.0, 20.0);
  ecu_set_diag_enabled(&st.ecu, 0);

  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 4.0, 30.0);
  CHECK_NEAR(st.ecu.rpm_seen, 0.0, 0.0);
  CHECK(st.rpm > 950.0); /* true speed is well above the target */
  CHECK_NEAR(st.ecu.idle_throttle, 0.15, 1e-9);
  CHECK(st.ecu.idle_mode == ECU_IDLE_LIMITED); /* the ECU thinks it's failing */
  CHECK(st.engine.run_state == ENGINE_RUNNING);

  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_NONE, 0.0);
  run(&s, &st, 0.0, 4.0, 40.0);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
}

/* Reads high: the ECU holds the wrong speed, and the engine settles that far
 * below the target. */
static void test_offset_makes_the_governor_hold_the_wrong_speed(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_set_diag_enabled(&st.ecu, 0);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_OFFSET, 100.0);
  run(&s, &st, 0.0, 4.0, 40.0);
  CHECK_NEAR(st.ecu.rpm_seen, 800.0, 25.0); /* what it thinks it holds */
  CHECK_NEAR(st.rpm, 700.0, 30.0);          /* what the engine really does */
  CHECK(st.rpm < st.ecu.rpm_seen);
}

static void test_scale_error_shifts_the_held_speed(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_set_diag_enabled(&st.ecu, 0);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_SCALE, 0.9);
  run(&s, &st, 0.0, 4.0, 40.0);
  CHECK_NEAR(st.ecu.rpm_seen, 800.0, 25.0);
  CHECK_NEAR(st.rpm, 800.0 / 0.9, 30.0);
}

/* A frozen reading: the ECU can't see a load step, so it doesn't respond to it
 * and the real speed falls. */
static void test_stuck_sensor_blinds_the_governor_to_a_load_step(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_set_diag_enabled(&st.ecu, 0);
  const double throttle_before = st.ecu.idle_throttle;

  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_STUCK, 0.0);
  run(&s, &st, 0.0, 4.0, 5.0); /* latches the settled reading */
  const double latched = st.ecu.rpm_seen;
  run(&s, &st, 0.0, 7.0, 30.0); /* now a load step the ECU cannot see */
  CHECK_NEAR(st.ecu.rpm_seen, latched, 0.0);
  CHECK(st.rpm < 750.0); /* the speed fell... */
  /* ...and the ECU never opened up to meet it (a frozen reading a few rpm above
   * the target even trims the throttle down slowly) */
  CHECK(st.ecu.idle_throttle <= throttle_before + 0.005);
}

/* The fault is in the ECU's input, not the engine: without an ECU it does
 * nothing. */
static void test_sensor_fault_does_nothing_without_an_ecu(void) {
  ModelSync a = make_sync(0, 800.0);
  ModelSync b = make_sync(0, 800.0);
  ecu_sensor_fault_set(&b.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  ModelState sa = make_state(&a);
  ModelState sb = make_state(&b);
  run(&a, &sa, 0.0, 1.0, 30.0);
  run(&b, &sb, 0.0, 1.0, 30.0);
  CHECK_NEAR(sa.engine.omega_rad_s, sb.engine.omega_rad_s, 0.0);
}

/* ---- diagnostics and fallback, against the real engine ---- */

/* How far the real engine strays while something happens to a sensor. */
typedef struct {
  double min_rpm, max_rpm;
} Swing;

static Swing run_tracking(ModelSync *s, ModelState *st, double throttle,
                          double load_nm, double seconds) {
  Swing w = {1e9, 0.0};
  const int n = (int)(seconds / 0.02 + 0.5);
  for (int i = 0; i < n; i++) {
    run(s, st, throttle, load_nm, 0.02);
    w.min_rpm = st->rpm < w.min_rpm ? st->rpm : w.min_rpm;
    w.max_rpm = st->rpm > w.max_rpm ? st->rpm : w.max_rpm;
  }
  return w;
}

static int dtc_active(const ModelState *st, EcuDtcId id) {
  return st->ecu.diag.dtc[id].active;
}

/* Ordinary flying raises no fault code: idle, throttle snaps, a chop, a stall
 * and a restart, on engines of every size. */
static void test_healthy_operation_raises_no_codes(void) {
  for (int n = 2; n <= 6; n++) {
    ModelSync s = make_sync(1, 800.0);
    s.engine_config.num_cylinders = n;
    engine_default_firing_order(n, s.engine_config.firing_order);
    ModelState st = make_state(&s);
    run(&s, &st, 0.0, 2.0, 20.0);
    const double throttles[] = {0.6, 1.0, 0.3, 0.0, 0.8, 0.0};
    for (size_t i = 0; i < sizeof throttles / sizeof throttles[0]; i++) {
      run(&s, &st, throttles[i], 2.0, 8.0);
    }
    CHECK(st.engine.run_state == ENGINE_RUNNING);
    CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
    CHECK(st.ecu.speed_source == ECU_SRC_PRIMARY);
  }

  /* a shutdown (ignition off, coasting to a stop) and a restart */
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 2.0, 10.0);
  engine_model_stop(&st.engine);
  run(&s, &st, 0.0, 2.0, 15.0);
  CHECK(st.engine.run_state == ENGINE_STOPPED);
  engine_model_start(&st.engine, &s.engine_config, st.cyl);
  run(&s, &st, 0.0, 2.0, 30.0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
}

/* The crank sensor drops out: caught at once, the ECU switches to the other
 * sensor, and the engine never flares (unlike a blind ECU). */
static void test_dropout_is_caught_and_the_engine_does_not_flare(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 4.0, 0.1);
  CHECK(dtc_active(&st, ECU_DTC_RPM1_LOST));
  CHECK(dtc_active(&st, ECU_DTC_RPM1_JUMP));
  CHECK(st.ecu.speed_source == ECU_SRC_SECONDARY);

  Swing w = run_tracking(&s, &st, 0.0, 4.0, 30.0);
  CHECK(w.max_rpm < 850.0);
  CHECK(w.min_rpm > 750.0);
  CHECK_NEAR(st.rpm, 800.0, 20.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
  CHECK(st.ecu.diag.dtc[ECU_DTC_RPM1_LOST].count == 1);
}

/* A step offset: caught, the ECU uses the good sensor, so the engine idles at
 * the target rather than 100 rpm low. */
static void test_offset_is_caught_and_idle_stays_on_target(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_OFFSET, 100.0);
  run(&s, &st, 0.0, 4.0, 0.1);
  CHECK(dtc_active(&st, ECU_DTC_RPM1_JUMP));
  CHECK(st.ecu.speed_source == ECU_SRC_SECONDARY);
  run(&s, &st, 0.0, 4.0, 30.0);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
  CHECK(st.ecu.rpm1_seen > st.rpm + 50.0);  /* the bad sensor still lies */
  /* the ECU acts on the good sensor's reading from the start of the step */
  CHECK_NEAR(st.ecu.rpm_seen, st.rpm, 20.0);
}

/* A frozen crank sensor: flagged after the frozen time, and then a load step
 * IS answered, because the ECU has moved to the other sensor. */
static void test_frozen_sensor_is_caught_and_a_load_step_is_answered(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_STUCK, 0.0);
  run(&s, &st, 0.0, 4.0, 1.0);
  CHECK(!dtc_active(&st, ECU_DTC_RPM1_FROZEN)); /* not yet */
  run(&s, &st, 0.0, 4.0, 3.0);
  CHECK(dtc_active(&st, ECU_DTC_RPM1_FROZEN));
  CHECK(st.ecu.speed_source == ECU_SRC_SECONDARY);

  run(&s, &st, 0.0, 7.0, 40.0); /* the load step the frozen sensor could not see */
  CHECK_NEAR(st.rpm, 800.0, 25.0);
}

/* Both sensors dead: limp-home. The engine keeps running on the fixed throttle
 * instead of an ECU acting on nothing. */
static void test_both_sensors_dead_means_limp_home(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  ecu_sensor_fault_set(&s.ecu_rpm2_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 4.0, 20.0);
  CHECK(st.ecu.speed_source == ECU_SRC_NONE);
  CHECK(st.ecu.idle_mode == ECU_IDLE_LIMP);
  CHECK_NEAR(st.ecu.throttle_cmd, s.engine_config.ecu.limp_throttle, 1e-12);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK(st.rpm > 550.0 && st.rpm < 1000.0);
}

/* A slow drift the ECU cannot attribute: mismatch, limp-home, engine still runs. */
static void test_a_slow_drift_ends_in_limp_home(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  double offset = 0.0;
  for (int i = 0; i < 1500; i++) { /* the crank sensor drifts +10 rpm/s */
    offset += 10.0 * 0.02;
    ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_OFFSET, offset);
    run(&s, &st, 0.0, 4.0, 0.02);
  }
  CHECK(dtc_active(&st, ECU_DTC_MISMATCH));
  CHECK(!dtc_active(&st, ECU_DTC_RPM1_JUMP));
  CHECK(st.ecu.idle_mode == ECU_IDLE_LIMP);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
}

/* A redundant sensor failing changes nothing for the engine. */
static void test_a_secondary_fault_leaves_idle_alone(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm2_fault, ECU_FAULT_DROPOUT, 0.0);
  Swing w = run_tracking(&s, &st, 0.0, 4.0, 20.0);
  CHECK(dtc_active(&st, ECU_DTC_RPM2_LOST));
  CHECK(st.ecu.speed_source == ECU_SRC_PRIMARY);
  CHECK(w.max_rpm < 840.0 && w.min_rpm > 760.0);
}

/* Diagnostics off is the blind ECU again: the same dropout flares the engine and
 * no code is recorded. */
static void test_diagnostics_off_lets_the_dropout_flare(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_set_diag_enabled(&st.ecu, 0);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 4.0, 30.0);
  CHECK(st.rpm > 950.0);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
}

/* The sensor comes back: after the heal time the ECU returns to it, the code
 * stays latched for maintenance, and clearing it empties the list. */
static void test_recovery_returns_to_the_primary_and_codes_can_be_cleared(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = settled_idle(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 4.0, 5.0);
  CHECK(st.ecu.speed_source == ECU_SRC_SECONDARY);

  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_NONE, 0.0);
  run(&s, &st, 0.0, 4.0, s.engine_config.ecu.diag.heal_s + 3.0);
  CHECK(st.ecu.speed_source == ECU_SRC_PRIMARY);
  CHECK(ecu_diag_active_count(&st.ecu.diag) == 0);
  CHECK(st.ecu.diag.dtc[ECU_DTC_RPM1_LOST].latched == 1);
  CHECK_NEAR(st.rpm, 800.0, 20.0);

  ecu_clear_codes(&st.ecu);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
}

/* Without an ECU there is nothing to diagnose. */
static void test_no_ecu_means_no_diagnostics(void) {
  ModelSync s = make_sync(0, 800.0);
  ModelState st = make_state(&s);
  ecu_sensor_fault_set(&s.ecu_rpm_fault, ECU_FAULT_DROPOUT, 0.0);
  ecu_sensor_fault_set(&s.ecu_rpm2_fault, ECU_FAULT_DROPOUT, 0.0);
  run(&s, &st, 0.0, 1.0, 20.0);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
  CHECK(st.ecu.fitted == 0);
}

static const TestCase CASES[] = {
    {"ecu_sensors.dropout_makes_the_governor_flare",
     test_dropout_makes_the_governor_flare},
    {"ecu_sensors.offset_makes_the_governor_hold_the_wrong_speed",
     test_offset_makes_the_governor_hold_the_wrong_speed},
    {"ecu_sensors.scale_error_shifts_the_held_speed",
     test_scale_error_shifts_the_held_speed},
    {"ecu_sensors.stuck_sensor_blinds_the_governor_to_a_load_step",
     test_stuck_sensor_blinds_the_governor_to_a_load_step},
    {"ecu_sensors.sensor_fault_does_nothing_without_an_ecu",
     test_sensor_fault_does_nothing_without_an_ecu},
    {"ecu_sensors.healthy_operation_raises_no_codes",
     test_healthy_operation_raises_no_codes},
    {"ecu_sensors.dropout_is_caught_and_the_engine_does_not_flare",
     test_dropout_is_caught_and_the_engine_does_not_flare},
    {"ecu_sensors.offset_is_caught_and_idle_stays_on_target",
     test_offset_is_caught_and_idle_stays_on_target},
    {"ecu_sensors.frozen_sensor_is_caught_and_a_load_step_is_answered",
     test_frozen_sensor_is_caught_and_a_load_step_is_answered},
    {"ecu_sensors.both_sensors_dead_means_limp_home",
     test_both_sensors_dead_means_limp_home},
    {"ecu_sensors.a_slow_drift_ends_in_limp_home",
     test_a_slow_drift_ends_in_limp_home},
    {"ecu_sensors.a_secondary_fault_leaves_idle_alone",
     test_a_secondary_fault_leaves_idle_alone},
    {"ecu_sensors.diagnostics_off_lets_the_dropout_flare",
     test_diagnostics_off_lets_the_dropout_flare},
    {"ecu_sensors.recovery_returns_to_the_primary_and_codes_can_be_cleared",
     test_recovery_returns_to_the_primary_and_codes_can_be_cleared},
    {"ecu_sensors.no_ecu_means_no_diagnostics",
     test_no_ecu_means_no_diagnostics},
};

RUN_TESTS(CASES)
