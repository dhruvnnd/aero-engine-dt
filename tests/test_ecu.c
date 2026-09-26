#include "test_util.h"

#include "physics/ecu.h"

/* The ECU is tested on its own, against hand-made sensor readings: it needs no
 * engine. */

static EcuConfig cfg(void) {
  EcuConfig c = ecu_config_default();
  c.idle_target_rpm = 800.0;
  c.idle_kp = 0.0003;
  c.idle_ki = 0.0003;
  c.idle_max_throttle = 0.15;
  return c;
}

/* A governor-only ECU: these tests feed hand-made constant readings, which the
 * diagnostics would (rightly) flag as frozen, so they run without them. */
static void init(EcuState *e) {
  ecu_init(e);
  ecu_set_diag_enabled(e, 0);
}

/* One step; returns the throttle the ECU would drive. */
static double step(EcuState *e, const EcuConfig *c, double rpm, double pilot,
                   double dt, int running, int ignition) {
  const EcuSensors s = {rpm, rpm, running, ignition};
  const EcuPilotCmd p = {pilot};
  EcuActuators out;
  ecu_step(e, c, &s, &p, &out, dt);
  CHECK_NEAR(out.throttle, e->throttle_cmd, 0.0);
  return out.throttle;
}

static void test_default_config_is_sane(void) {
  EcuConfig c = ecu_config_default();
  CHECK(c.idle_target_rpm > 0.0);
  CHECK(c.idle_kp >= 0.0 && c.idle_ki >= 0.0);
  CHECK(c.idle_max_throttle > 0.0 && c.idle_max_throttle <= 1.0);
  CHECK(c.limp_throttle > 0.0 && c.limp_throttle <= 1.0);
  CHECK(c.diag.jump_rpm > 0.0 && c.diag.mismatch_rpm > 0.0);
  CHECK(c.diag.mismatch_s > 0.0 && c.diag.frozen_s > 0.0 && c.diag.heal_s > 0.0);
}

static void test_init_is_fitted_enabled_and_zeroed(void) {
  EcuState e;
  ecu_init(&e);
  CHECK(e.fitted == 1);
  CHECK(e.idle_enabled == 1);
  CHECK(e.diag_enabled == 1);
  CHECK(e.speed_source == ECU_SRC_PRIMARY);
  CHECK(ecu_diag_latched_count(&e.diag) == 0);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(e.throttle_cmd, 0.0, 0.0);
}

/* The loop terms are exactly what the description says. */
static void test_terms_follow_the_pi_law(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  double cmd = step(&e, &c, 700.0, 0.0, 0.1, 1, 1); /* 100 rpm low */
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);
  CHECK_NEAR(e.idle_error_rpm, 100.0, 1e-12);
  CHECK_NEAR(e.idle_p_term, 0.0003 * 100.0, 1e-12);
  CHECK_NEAR(e.idle_i_term, 0.0003 * 100.0 * 0.1, 1e-12); /* ki * e * dt */
  CHECK_NEAR(e.idle_raw, e.idle_p_term + e.idle_i_term, 1e-12);
  CHECK_NEAR(e.idle_throttle, e.idle_raw, 1e-12);
  CHECK_NEAR(cmd, e.idle_throttle, 1e-12);
  CHECK_NEAR(e.idle_target_rpm, 800.0, 0.0);
  CHECK(e.fitted == 1);
}

static void test_integrator_accumulates(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  double prev = 0.0;
  for (int i = 0; i < 10; i++) {
    step(&e, &c, 750.0, 0.0, 0.1, 1, 1);
    CHECK(e.idle_i_term > prev);
    prev = e.idle_i_term;
  }
  CHECK_NEAR(e.idle_i_term, 10 * 0.0003 * 50.0 * 0.1, 1e-12);
}

/* Above the target the governor adds nothing and the integrator unwinds to 0
 * without going negative: it can only add throttle. */
static void test_overspeed_adds_nothing_and_never_goes_negative(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  for (int i = 0; i < 20; i++) {
    step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  }
  CHECK(e.idle_i_term > 0.0);
  for (int i = 0; i < 400; i++) {
    step(&e, &c, 1000.0, 0.0, 0.1, 1, 1);
    CHECK(e.idle_i_term >= 0.0);
    CHECK(e.idle_throttle >= 0.0);
  }
  CHECK_NEAR(e.idle_i_term, 0.0, 1e-12);
  CHECK_NEAR(e.idle_throttle, 0.0, 1e-12);
  CHECK(e.idle_p_term < 0.0); /* the raw terms still show the error's sign */
}

static void test_authority_clamp_and_limited_mode(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  for (int i = 0; i < 2000; i++) {
    step(&e, &c, 400.0, 0.0, 0.1, 1, 1);
    CHECK(e.idle_throttle <= c.idle_max_throttle + 1e-12);
    CHECK(e.idle_i_term <= c.idle_max_throttle + 1e-12);
  }
  CHECK(e.idle_mode == ECU_IDLE_LIMITED);
  CHECK_NEAR(e.idle_throttle, c.idle_max_throttle, 1e-12);
  CHECK(e.idle_raw >= c.idle_max_throttle);
}

/* max(pilot, governor): the pilot's throttle is never reduced. */
static void test_command_is_max_of_pilot_and_governor(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  double cmd = step(&e, &c, 700.0, 0.02, 0.1, 1, 1); /* governor > 0.02 */
  CHECK(cmd > 0.02);
  CHECK_NEAR(cmd, e.idle_throttle, 1e-12);

  init(&e);
  cmd = step(&e, &c, 790.0, 0.10, 0.1, 1, 1); /* pilot > governor */
  CHECK_NEAR(cmd, 0.10, 1e-12);
  CHECK_NEAR(e.pilot_throttle, 0.10, 1e-12);
}

/* At/above the authority the pilot is in control and the integrator is held. */
static void test_pilot_in_control_holds_the_integrator(void) {
  EcuState e;
  init(&e);
  EcuConfig c = cfg();
  for (int i = 0; i < 50; i++) {
    step(&e, &c, 740.0, 0.0, 0.1, 1, 1);
  }
  double held = e.idle_i_term;
  CHECK(held > 0.0);

  for (int i = 0; i < 50; i++) {
    double cmd = step(&e, &c, 300.0, 0.6, 0.1, 1, 1);
    CHECK(e.idle_mode == ECU_IDLE_PILOT);
    CHECK_NEAR(cmd, 0.6, 1e-12);
    CHECK_NEAR(e.idle_i_term, held, 1e-12); /* held, not wound up or reset */
  }
  /* a chop finds the idle throttle it had already found */
  step(&e, &c, 740.0, 0.0, 0.1, 1, 1);
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);
  CHECK(e.idle_i_term >= held);
}

static void test_operator_off_disabled_and_standby(void) {
  EcuConfig c = cfg();
  EcuState e;
  init(&e);
  for (int i = 0; i < 20; i++) {
    step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  }
  CHECK(e.idle_i_term > 0.0);

  /* operator switch: pilot only, loop cleared, error still reported */
  ecu_set_idle_enabled(&e, 0);
  double cmd = step(&e, &c, 700.0, 0.03, 0.1, 1, 1);
  CHECK(e.idle_mode == ECU_IDLE_OFF);
  CHECK_NEAR(cmd, 0.03, 1e-12);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(e.idle_error_rpm, 100.0, 1e-12);

  /* back on: starts from a clean integrator */
  ecu_set_idle_enabled(&e, 1);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);

  /* not running / ignition off: standby, loop cleared */
  step(&e, &c, 700.0, 0.0, 0.1, 0, 1);
  CHECK(e.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  step(&e, &c, 700.0, 0.0, 0.1, 1, 0);
  CHECK(e.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(e.throttle_cmd, 0.0, 0.0);

  /* no target configured: disabled, command is the pilot's */
  EcuConfig none = c;
  none.idle_target_rpm = 0.0;
  cmd = step(&e, &none, 700.0, 0.2, 0.1, 1, 1);
  CHECK(e.idle_mode == ECU_IDLE_DISABLED);
  CHECK_NEAR(cmd, 0.2, 1e-12);
  CHECK_NEAR(e.idle_error_rpm, 0.0, 0.0);
  EcuConfig no_authority = c;
  no_authority.idle_max_throttle = 0.0;
  step(&e, &no_authority, 700.0, 0.0, 0.1, 1, 1);
  CHECK(e.idle_mode == ECU_IDLE_DISABLED);
}

static void test_reset_idle_keeps_the_operator_switch(void) {
  EcuConfig c = cfg();
  EcuState e;
  init(&e);
  ecu_set_idle_enabled(&e, 0);
  ecu_reset_idle(&e);
  CHECK(e.idle_enabled == 0);

  ecu_set_idle_enabled(&e, 1);
  for (int i = 0; i < 10; i++) {
    step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  }
  ecu_reset_idle(&e);
  CHECK(e.idle_enabled == 1);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_p_term, 0.0, 0.0);
}

/* No ECU in the loop: the pilot's throttle passes through untouched, whatever
 * the loop had been doing, and the state says there is no ECU. */
static void test_bypass_passes_the_pilot_straight_through(void) {
  EcuConfig c = cfg();
  EcuState e;
  init(&e);
  for (int i = 0; i < 20; i++) {
    step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  }
  CHECK(e.idle_throttle > 0.0);

  const EcuPilotCmd pilot = {0.37};
  EcuActuators out;
  ecu_bypass(&e, &pilot, &out);
  CHECK_NEAR(out.throttle, 0.37, 0.0);
  CHECK(e.fitted == 0);
  CHECK(e.idle_mode == ECU_IDLE_DISABLED);
  CHECK_NEAR(e.throttle_cmd, 0.37, 0.0);
  CHECK_NEAR(e.pilot_throttle, 0.37, 0.0);
  CHECK_NEAR(e.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_target_rpm, 0.0, 0.0);

  /* and a step afterwards puts it back in the loop */
  step(&e, &c, 700.0, 0.0, 0.1, 1, 1);
  CHECK(e.fitted == 1);
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);
}

/* ---- sensor faults on the ECU's crank-speed input ---- */

static void test_sensor_fault_kinds_change_what_is_read(void) {
  EcuSensorFault f = {0};
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 800.0), 800.0, 0.0); /* none */

  ecu_sensor_fault_set(&f, ECU_FAULT_OFFSET, 120.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 800.0), 920.0, 1e-12);
  ecu_sensor_fault_set(&f, ECU_FAULT_OFFSET, -120.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 800.0), 680.0, 1e-12);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 50.0), 0.0, 0.0); /* never negative */

  ecu_sensor_fault_set(&f, ECU_FAULT_SCALE, 0.9);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1000.0), 900.0, 1e-12);

  ecu_sensor_fault_set(&f, ECU_FAULT_DROPOUT, 0.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1000.0), 0.0, 0.0);

  ecu_sensor_fault_set(&f, ECU_FAULT_NONE, 0.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1000.0), 1000.0, 0.0);
}

/* A stuck sensor latches the first value it sees; the latch restarts when the
 * kind is set again but not when only the value moves. */
static void test_stuck_sensor_holds_its_first_reading(void) {
  EcuSensorFault f = {0};
  ecu_sensor_fault_set(&f, ECU_FAULT_STUCK, 0.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 800.0), 800.0, 0.0); /* latches */
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1500.0), 800.0, 0.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 200.0), 800.0, 0.0);

  ecu_sensor_fault_set(&f, ECU_FAULT_STUCK, 5.0); /* value moved: same latch */
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1200.0), 800.0, 0.0);

  ecu_sensor_fault_set(&f, ECU_FAULT_NONE, 0.0);
  ecu_sensor_fault_set(&f, ECU_FAULT_STUCK, 0.0); /* a new fault: new latch */
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 1200.0), 1200.0, 0.0);
  CHECK_NEAR(ecu_sensor_fault_apply(&f, 900.0), 1200.0, 0.0);
}

static void test_fault_kind_names_exist_and_differ(void) {
  for (int i = 0; i < (int)ECU_FAULT_KIND_COUNT; i++) {
    const char *a = ecu_fault_kind_name((EcuFaultKind)i);
    CHECK(a != NULL && a[0] != '\0' && a[0] != '?');
    for (int j = i + 1; j < (int)ECU_FAULT_KIND_COUNT; j++) {
      CHECK(a != ecu_fault_kind_name((EcuFaultKind)j));
    }
  }
}

/* The state records what the ECU read, not the truth. */
static void test_state_records_the_reading_the_ecu_used(void) {
  EcuState e;
  init(&e);
  CHECK_NEAR(e.rpm_seen, 0.0, 0.0);
  EcuConfig c = cfg();
  step(&e, &c, 733.0, 0.0, 0.1, 1, 1);
  CHECK_NEAR(e.rpm_seen, 733.0, 0.0);
}

static void test_mode_names_exist_and_differ(void) {
  const EcuIdleMode modes[] = {ECU_IDLE_DISABLED, ECU_IDLE_OFF,
                               ECU_IDLE_STANDBY,  ECU_IDLE_PILOT,
                               ECU_IDLE_ACTIVE,   ECU_IDLE_LIMITED,
                               ECU_IDLE_LIMP};
  const int n = (int)(sizeof modes / sizeof modes[0]);
  for (int i = 0; i < n; i++) {
    const char *a = ecu_idle_mode_name(modes[i]);
    CHECK(a != NULL && a[0] != '\0' && a[0] != '?');
    for (int j = i + 1; j < n; j++) {
      CHECK(a != ecu_idle_mode_name(modes[j]));
    }
  }
}

static const TestCase CASES[] = {
    {"ecu.default_config_is_sane", test_default_config_is_sane},
    {"ecu.init_is_fitted_enabled_and_zeroed",
     test_init_is_fitted_enabled_and_zeroed},
    {"ecu.terms_follow_the_pi_law", test_terms_follow_the_pi_law},
    {"ecu.integrator_accumulates", test_integrator_accumulates},
    {"ecu.overspeed_adds_nothing_and_never_goes_negative",
     test_overspeed_adds_nothing_and_never_goes_negative},
    {"ecu.authority_clamp_and_limited_mode",
     test_authority_clamp_and_limited_mode},
    {"ecu.command_is_max_of_pilot_and_governor",
     test_command_is_max_of_pilot_and_governor},
    {"ecu.pilot_in_control_holds_the_integrator",
     test_pilot_in_control_holds_the_integrator},
    {"ecu.operator_off_disabled_and_standby",
     test_operator_off_disabled_and_standby},
    {"ecu.reset_idle_keeps_the_operator_switch",
     test_reset_idle_keeps_the_operator_switch},
    {"ecu.bypass_passes_the_pilot_straight_through",
     test_bypass_passes_the_pilot_straight_through},
    {"ecu.sensor_fault_kinds_change_what_is_read",
     test_sensor_fault_kinds_change_what_is_read},
    {"ecu.stuck_sensor_holds_its_first_reading",
     test_stuck_sensor_holds_its_first_reading},
    {"ecu.fault_kind_names_exist_and_differ",
     test_fault_kind_names_exist_and_differ},
    {"ecu.state_records_the_reading_the_ecu_used",
     test_state_records_the_reading_the_ecu_used},
    {"ecu.mode_names_exist_and_differ", test_mode_names_exist_and_differ},
};

RUN_TESTS(CASES)
