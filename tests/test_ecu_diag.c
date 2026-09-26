#include "test_util.h"

#include <math.h>
#include <string.h>

#include "physics/ecu.h"

/* The ECU's self-diagnosis, driven by hand-made speed signals (no engine). */

#define DT 0.02

typedef struct {
  EcuState e;
  EcuConfig c;
  double t;
  double pilot;
  int running;
  double cht, egt, oil; /* temperature readings, healthy unless a test heats them */
} Rig;

static void rig_init(Rig *r) {
  ecu_init(&r->e);
  r->c = ecu_config_default();
  r->t = 0.0;
  r->pilot = 0.0;
  r->running = 1;
  r->cht = 150.0;
  r->egt = 600.0;
  r->oil = 80.0;
}

/* One control step with the given readings. */
static void rig_step(Rig *r, double r1, double r2) {
  const EcuSensors s = {r1, r2, r->running, r->running, r->cht, r->egt, r->oil};
  const EcuPilotCmd p = {r->pilot};
  EcuActuators out;
  ecu_step(&r->e, &r->c, &s, &p, &out, DT);
  r->t += DT;
}

/* A live speed signal: a mean plus the crank-speed ripple. Both sensors see the
 * same shaft, so the same signal. */
static double live(double mean, double t) {
  return mean + 10.0 * sin(2.0 * 3.14159265358979 * 18.0 * t) +
         2.0 * sin(2.0 * 3.14159265358979 * 3.1 * t);
}

static void run_healthy(Rig *r, double mean, double seconds) {
  const int n = (int)(seconds / DT + 0.5);
  for (int i = 0; i < n; i++) {
    const double v = live(mean, r->t);
    rig_step(r, v, v);
  }
}

static int active(const Rig *r, EcuDtcId id) { return r->e.diag.dtc[id].active; }

static void test_a_healthy_run_raises_nothing(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 30.0);

  /* a hard throttle snap: 800 -> 2600 rpm at ~1100 rpm/s, then back down */
  for (int i = 0; i < 82; i++) {
    const double v = live(800.0 + 22.0 * i, r.t);
    rig_step(&r, v, v);
  }
  run_healthy(&r, 2600.0, 10.0);
  for (int i = 0; i < 82; i++) {
    const double v = live(2600.0 - 22.0 * i, r.t);
    rig_step(&r, v, v);
  }
  run_healthy(&r, 800.0, 30.0);

  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK(ecu_diag_latched_count(&r.e.diag) == 0);
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
}

static void test_a_jump_flags_the_channel_and_switches_source(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  const double v = live(800.0, r.t);
  rig_step(&r, v + 100.0, v); /* the primary steps by +100 */

  CHECK(active(&r, ECU_DTC_RPM1_JUMP));
  CHECK(!active(&r, ECU_DTC_RPM2_JUMP));
  CHECK(r.e.speed_source == ECU_SRC_SECONDARY);
  CHECK_NEAR(r.e.rpm_seen, v, 1e-9); /* it now acts on the good sensor */
  CHECK_NEAR(r.e.rpm1_seen, v + 100.0, 1e-9);
  CHECK(r.e.idle_mode != ECU_IDLE_LIMP);
}

/* Small errors are not this check's business: they are the twin's. */
static void test_a_small_step_is_not_a_jump(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  for (int i = 0; i < 1000; i++) { /* 20 s with the primary reading 40 high */
    const double v = live(800.0, r.t);
    rig_step(&r, v + 40.0, v);
  }
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
}

static void test_a_dropout_is_lost_and_a_jump(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  const double v = live(800.0, r.t);
  rig_step(&r, 0.0, v);
  CHECK(active(&r, ECU_DTC_RPM1_LOST));
  CHECK(active(&r, ECU_DTC_RPM1_JUMP));
  CHECK(r.e.speed_source == ECU_SRC_SECONDARY);
}

static void test_a_frozen_reading_is_flagged_after_the_frozen_time(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  const double frozen = live(800.0, r.t);
  int steps_to_flag = 0;
  for (int i = 0; i < 400; i++) {
    const double v = live(800.0, r.t);
    rig_step(&r, frozen, v);
    if (active(&r, ECU_DTC_RPM1_FROZEN)) {
      steps_to_flag = i + 1;
      break;
    }
  }
  CHECK(steps_to_flag > 0);
  CHECK_NEAR(steps_to_flag * DT, r.c.diag.frozen_s, 0.2); /* not before it */
  CHECK(r.e.speed_source == ECU_SRC_SECONDARY);
  CHECK(!active(&r, ECU_DTC_RPM1_JUMP)); /* it froze, it didn't jump */
}

static void test_the_secondary_failing_leaves_the_primary_in_charge(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  const double v = live(800.0, r.t);
  rig_step(&r, v, 0.0);
  CHECK(active(&r, ECU_DTC_RPM2_LOST));
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
  CHECK(r.e.idle_mode != ECU_IDLE_LIMP);
  CHECK(ecu_dtc_info(ECU_DTC_RPM2_LOST)->severity == 0); /* a caution */
}

/* Both sensors bad: no trusted speed, so the governor is dropped for the fixed
 * limp throttle. */
static void test_no_trusted_sensor_means_limp_home(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  rig_step(&r, 0.0, 0.0);
  CHECK(r.e.speed_source == ECU_SRC_NONE);
  CHECK(r.e.idle_mode == ECU_IDLE_LIMP);
  CHECK_NEAR(r.e.idle_throttle, r.c.limp_throttle, 1e-12);
  CHECK_NEAR(r.e.throttle_cmd, r.c.limp_throttle, 1e-12);
  CHECK_NEAR(r.e.rpm_seen, 0.0, 0.0);
  CHECK_NEAR(r.e.idle_i_term, 0.0, 0.0); /* the loop is cleared */

  /* the pilot still wins above the limp throttle */
  r.pilot = 0.3;
  rig_step(&r, 0.0, 0.0);
  CHECK_NEAR(r.e.throttle_cmd, 0.3, 1e-12);

  /* with the governor switched off there is nothing to be in limp about */
  r.pilot = 0.0;
  ecu_set_idle_enabled(&r.e, 0);
  rig_step(&r, 0.0, 0.0);
  CHECK(r.e.idle_mode == ECU_IDLE_OFF);
  CHECK_NEAR(r.e.throttle_cmd, 0.0, 0.0);
}

/* Two sensors that disagree slowly, with neither misbehaving on its own: the
 * ECU cannot tell which is right, so it is a mismatch and limp-home. */
static void test_slow_drift_is_an_unresolved_mismatch(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  double drift = 0.0;
  double time_to_flag = -1.0;
  for (int i = 0; i < 1500; i++) { /* the primary drifts +10 rpm/s */
    drift += 10.0 * DT;
    const double v = live(800.0, r.t);
    rig_step(&r, v + drift, v);
    if (active(&r, ECU_DTC_MISMATCH) && time_to_flag < 0.0) {
      time_to_flag = (i + 1) * DT;
    }
  }
  CHECK(time_to_flag > 0.0);
  /* it needs >60 rpm of disagreement for a second: about 7 s of drift */
  CHECK(time_to_flag > 5.0 && time_to_flag < 9.0);
  CHECK(!active(&r, ECU_DTC_RPM1_JUMP)); /* no channel fault of its own */
  CHECK(r.e.speed_source == ECU_SRC_NONE);
  CHECK(r.e.idle_mode == ECU_IDLE_LIMP);
}

/* A fault heals once the channel has been clean and agreeing for the heal time;
 * the code stays latched, and a repeat counts again. */
static void test_a_fault_heals_but_its_code_stays_latched(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  double v = live(800.0, r.t);
  rig_step(&r, v + 100.0, v);
  CHECK(active(&r, ECU_DTC_RPM1_JUMP));
  CHECK(r.e.speed_source == ECU_SRC_SECONDARY);

  run_healthy(&r, 800.0, r.c.diag.heal_s + 1.0); /* back to agreeing */
  /* (returning to normal is itself a step, which restarts the clean timer) */
  CHECK(!active(&r, ECU_DTC_RPM1_JUMP));
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
  CHECK(r.e.diag.dtc[ECU_DTC_RPM1_JUMP].latched == 1);
  CHECK(r.e.diag.dtc[ECU_DTC_RPM1_JUMP].count == 1);

  v = live(800.0, r.t);
  rig_step(&r, v + 100.0, v); /* again */
  CHECK(active(&r, ECU_DTC_RPM1_JUMP));
  CHECK(r.e.diag.dtc[ECU_DTC_RPM1_JUMP].count == 2);
}

/* The freeze frame is the operating point at the (latest) occurrence, kept
 * after the fault heals. */
static void test_the_freeze_frame_records_the_operating_point(void) {
  Rig r;
  rig_init(&r);
  r.pilot = 0.02;
  run_healthy(&r, 812.0, 5.0);
  const double v = live(812.0, r.t);
  const double t_fault = r.t;
  rig_step(&r, v + 100.0, v);

  const EcuDtc *d = &r.e.diag.dtc[ECU_DTC_RPM1_JUMP];
  CHECK_NEAR(d->freeze.rpm1, v + 100.0, 1e-9);
  CHECK_NEAR(d->freeze.rpm2, v, 1e-9);
  CHECK_NEAR(d->freeze.pilot_throttle, 0.02, 1e-12);
  CHECK(d->freeze.source == ECU_SRC_SECONDARY);
  CHECK_NEAR(d->first_s, t_fault + DT, 1e-6);
  CHECK_NEAR(d->last_s, d->first_s, 1e-12);

  run_healthy(&r, 812.0, 20.0); /* heals... */
  CHECK(!active(&r, ECU_DTC_RPM1_JUMP));
  CHECK_NEAR(d->freeze.rpm1, v + 100.0, 1e-9); /* ...the snapshot remains */
}

static void test_clearing_codes_forgets_history_but_not_live_faults(void) {
  Rig r;
  rig_init(&r);
  run_healthy(&r, 800.0, 5.0);
  double v = live(800.0, r.t);
  rig_step(&r, 0.0, v);
  CHECK(ecu_diag_latched_count(&r.e.diag) >= 2); /* jump + lost */

  ecu_clear_codes(&r.e);
  CHECK(ecu_diag_latched_count(&r.e.diag) == 0);
  CHECK(r.e.diag.dtc[ECU_DTC_RPM1_LOST].count == 0);

  /* the sensor is still dead: the fault registers again as a new occurrence */
  v = live(800.0, r.t);
  rig_step(&r, 0.0, v);
  CHECK(active(&r, ECU_DTC_RPM1_LOST));
  CHECK(r.e.diag.dtc[ECU_DTC_RPM1_LOST].count == 1);
}

/* Nothing is checked while the engine is not running, and starting up is not a
 * jump. */
static void test_nothing_is_checked_while_the_engine_is_not_running(void) {
  Rig r;
  rig_init(&r);
  r.running = 0;
  for (int i = 0; i < 1000; i++) { /* 20 s at rest: dead-still zeros */
    rig_step(&r, 0.0, 0.0);
  }
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);

  r.running = 1; /* it catches and runs */
  run_healthy(&r, 800.0, 10.0);
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK(ecu_diag_latched_count(&r.e.diag) == 0);

  /* faults are held (not cleared) while it is stopped */
  const double v = live(800.0, r.t);
  rig_step(&r, 0.0, v);
  CHECK(active(&r, ECU_DTC_RPM1_LOST));
  r.running = 0;
  rig_step(&r, 0.0, 0.0);
  CHECK(active(&r, ECU_DTC_RPM1_LOST));
}

/* With diagnostics off the ECU trusts the primary blindly, as before, and
 * records nothing. */
static void test_diagnostics_off_trusts_the_primary_blindly(void) {
  Rig r;
  rig_init(&r);
  ecu_set_diag_enabled(&r.e, 0);
  run_healthy(&r, 800.0, 5.0);
  const double v = live(800.0, r.t);
  rig_step(&r, 0.0, v); /* the primary drops out */
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
  CHECK_NEAR(r.e.rpm_seen, 0.0, 0.0);
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK(r.e.idle_mode == ECU_IDLE_LIMITED); /* fooled: it thinks it is failing */

  ecu_set_diag_enabled(&r.e, 1);
  run_healthy(&r, 800.0, 2.0);
  CHECK(r.e.diag_enabled == 1);
}

/* ---- temperature limit codes ---- */

/* Runs `seconds` of healthy speed with the rig's current temperatures. */
static void hold_temps(Rig *r, double seconds) { run_healthy(r, 800.0, seconds); }

static void test_normal_temperatures_raise_nothing(void) {
  Rig r;
  rig_init(&r);
  hold_temps(&r, 60.0);
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);
  CHECK_NEAR(r.e.cht_seen, 150.0, 0.0);
  CHECK_NEAR(r.e.egt_seen, 600.0, 0.0);
  CHECK_NEAR(r.e.oil_seen, 80.0, 0.0);
}

/* Over the limit for the hold time sets the code; a brief spike does not. */
static void test_a_temperature_code_needs_the_hold_time(void) {
  Rig r;
  rig_init(&r);
  hold_temps(&r, 5.0);

  r.cht = 215.0; /* over the high limit (210), under critical (240) */
  hold_temps(&r, r.c.diag.temp_hold_s - 0.5);
  CHECK(!active(&r, ECU_DTC_CHT_HIGH));
  hold_temps(&r, 1.0);
  CHECK(active(&r, ECU_DTC_CHT_HIGH));
  CHECK(!active(&r, ECU_DTC_CHT_CRIT));

  /* a spike that comes and goes never adds up */
  Rig q;
  rig_init(&q);
  for (int i = 0; i < 5; i++) {
    q.cht = 230.0;
    hold_temps(&q, 2.0);
    q.cht = 150.0;
    hold_temps(&q, 1.0);
  }
  CHECK(!active(&q, ECU_DTC_CHT_HIGH));
  CHECK(ecu_diag_latched_count(&q.e.diag) == 0);
}

static void test_high_and_critical_are_separate_limits_with_their_severity(void) {
  Rig r;
  rig_init(&r);
  r.cht = 250.0; /* past both */
  hold_temps(&r, 4.0);
  CHECK(active(&r, ECU_DTC_CHT_HIGH));
  CHECK(active(&r, ECU_DTC_CHT_CRIT));
  CHECK(ecu_dtc_info(ECU_DTC_CHT_HIGH)->severity == 0);
  CHECK(ecu_dtc_info(ECU_DTC_CHT_CRIT)->severity == 1);

  Rig e;
  rig_init(&e);
  e.egt = 900.0;
  e.oil = 130.0;
  hold_temps(&e, 4.0);
  CHECK(active(&e, ECU_DTC_EGT_HIGH) && active(&e, ECU_DTC_EGT_CRIT));
  CHECK(active(&e, ECU_DTC_OIL_HIGH) && active(&e, ECU_DTC_OIL_CRIT));
  CHECK(!active(&e, ECU_DTC_CHT_HIGH));
}

/* Hysteresis: just under the limit is not clear; clear needs 5 degC under it
 * for the heal time. The code stays latched. */
static void test_a_temperature_code_clears_with_hysteresis(void) {
  Rig r;
  rig_init(&r);
  r.oil = 118.0; /* over 110 */
  hold_temps(&r, 4.0);
  CHECK(active(&r, ECU_DTC_OIL_HIGH));

  r.oil = 107.0; /* under the limit, but within the 5 degC band */
  hold_temps(&r, r.c.diag.heal_s + 3.0);
  CHECK(active(&r, ECU_DTC_OIL_HIGH));

  r.oil = 100.0; /* properly clear */
  hold_temps(&r, r.c.diag.heal_s - 1.0);
  CHECK(active(&r, ECU_DTC_OIL_HIGH)); /* not for long enough yet */
  hold_temps(&r, 2.0);
  CHECK(!active(&r, ECU_DTC_OIL_HIGH));
  CHECK(r.e.diag.dtc[ECU_DTC_OIL_HIGH].latched == 1);
  CHECK(r.e.diag.dtc[ECU_DTC_OIL_HIGH].count == 1);
}

/* Temperature codes are about the engine, not the speed sensors: they never
 * change which speed the ECU uses or push it into limp-home. */
static void test_temperature_codes_leave_the_speed_source_alone(void) {
  Rig r;
  rig_init(&r);
  r.cht = 260.0;
  r.egt = 900.0;
  r.oil = 130.0;
  hold_temps(&r, 10.0);
  CHECK(ecu_diag_active_count(&r.e.diag) == 6);
  CHECK(r.e.speed_source == ECU_SRC_PRIMARY);
  CHECK(r.e.idle_mode != ECU_IDLE_LIMP);
}

static void test_temperatures_are_only_checked_while_running(void) {
  Rig r;
  rig_init(&r);
  r.running = 0;
  r.cht = 300.0; /* a hot-soaked engine at rest */
  for (int i = 0; i < 1000; i++) {
    rig_step(&r, 0.0, 0.0);
  }
  CHECK(ecu_diag_active_count(&r.e.diag) == 0);

  r.running = 1;
  hold_temps(&r, 4.0);
  CHECK(active(&r, ECU_DTC_CHT_HIGH));
  r.running = 0; /* shut down: the fault is held, not cleared */
  r.cht = 100.0;
  for (int i = 0; i < 1000; i++) {
    rig_step(&r, 0.0, 0.0);
  }
  CHECK(active(&r, ECU_DTC_CHT_HIGH));
}

static void test_the_freeze_frame_of_a_temperature_code_has_the_temperatures(void) {
  Rig r;
  rig_init(&r);
  r.egt = 870.0;
  r.oil = 90.0;
  hold_temps(&r, 4.0);
  const EcuDtc *d = &r.e.diag.dtc[ECU_DTC_EGT_CRIT];
  CHECK(d->active);
  CHECK_NEAR(d->freeze.egt_c, 870.0, 0.0);
  CHECK_NEAR(d->freeze.cht_c, 150.0, 0.0);
  CHECK_NEAR(d->freeze.oil_c, 90.0, 0.0);
  CHECK_NEAR(d->first_s, r.c.diag.temp_hold_s, 0.1); /* when the hold ran out */
}

static void test_dtc_table_is_complete_and_sensible(void) {
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    const EcuDtcInfo *a = ecu_dtc_info((EcuDtcId)i);
    CHECK(a != NULL);
    if (!a) {
      continue;
    }
    CHECK(a->code != NULL && a->code[0] == 'E');
    CHECK(a->description != NULL && a->description[0] != '\0');
    CHECK(a->severity == 0 || a->severity == 1);
    for (int j = i + 1; j < (int)ECU_DTC_COUNT; j++) {
      CHECK(strcmp(a->code, ecu_dtc_info((EcuDtcId)j)->code) != 0);
    }
  }
  CHECK(ecu_dtc_info(ECU_DTC_COUNT) == NULL);
  CHECK(ecu_dtc_info(ECU_DTC_RPM1_LOST)->severity == 1); /* the crank: a warning */
  CHECK(strcmp(ecu_speed_source_name(ECU_SRC_SECONDARY), "alternator") == 0);
}

static const TestCase CASES[] = {
    {"ecu_diag.a_healthy_run_raises_nothing", test_a_healthy_run_raises_nothing},
    {"ecu_diag.a_jump_flags_the_channel_and_switches_source",
     test_a_jump_flags_the_channel_and_switches_source},
    {"ecu_diag.a_small_step_is_not_a_jump", test_a_small_step_is_not_a_jump},
    {"ecu_diag.a_dropout_is_lost_and_a_jump", test_a_dropout_is_lost_and_a_jump},
    {"ecu_diag.a_frozen_reading_is_flagged_after_the_frozen_time",
     test_a_frozen_reading_is_flagged_after_the_frozen_time},
    {"ecu_diag.the_secondary_failing_leaves_the_primary_in_charge",
     test_the_secondary_failing_leaves_the_primary_in_charge},
    {"ecu_diag.no_trusted_sensor_means_limp_home",
     test_no_trusted_sensor_means_limp_home},
    {"ecu_diag.slow_drift_is_an_unresolved_mismatch",
     test_slow_drift_is_an_unresolved_mismatch},
    {"ecu_diag.a_fault_heals_but_its_code_stays_latched",
     test_a_fault_heals_but_its_code_stays_latched},
    {"ecu_diag.the_freeze_frame_records_the_operating_point",
     test_the_freeze_frame_records_the_operating_point},
    {"ecu_diag.clearing_codes_forgets_history_but_not_live_faults",
     test_clearing_codes_forgets_history_but_not_live_faults},
    {"ecu_diag.nothing_is_checked_while_the_engine_is_not_running",
     test_nothing_is_checked_while_the_engine_is_not_running},
    {"ecu_diag.diagnostics_off_trusts_the_primary_blindly",
     test_diagnostics_off_trusts_the_primary_blindly},
    {"ecu_diag.normal_temperatures_raise_nothing",
     test_normal_temperatures_raise_nothing},
    {"ecu_diag.a_temperature_code_needs_the_hold_time",
     test_a_temperature_code_needs_the_hold_time},
    {"ecu_diag.high_and_critical_are_separate_limits_with_their_severity",
     test_high_and_critical_are_separate_limits_with_their_severity},
    {"ecu_diag.a_temperature_code_clears_with_hysteresis",
     test_a_temperature_code_clears_with_hysteresis},
    {"ecu_diag.temperature_codes_leave_the_speed_source_alone",
     test_temperature_codes_leave_the_speed_source_alone},
    {"ecu_diag.temperatures_are_only_checked_while_running",
     test_temperatures_are_only_checked_while_running},
    {"ecu_diag.the_freeze_frame_of_a_temperature_code_has_the_temperatures",
     test_the_freeze_frame_of_a_temperature_code_has_the_temperatures},
    {"ecu_diag.dtc_table_is_complete_and_sensible",
     test_dtc_table_is_complete_and_sensible},
};

RUN_TESTS(CASES)
