#include "test_util.h"

#include "physics/ecu.h"

static EcuIdleParams params(void) {
  EcuIdleParams p;
  p.target_rpm = 800.0;
  p.kp = 0.0003;
  p.ki = 0.0003;
  p.max_throttle = 0.15;
  return p;
}

static void test_init_is_enabled_and_zeroed(void) {
  EcuState e;
  ecu_init(&e);
  CHECK(e.idle_enabled == 1);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(e.throttle_cmd, 0.0, 0.0);
}

/* The loop terms are exactly what the description says. */
static void test_terms_follow_the_pi_law(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  double cmd = ecu_step(&e, &p, 1, 1, 700.0, 0.0, 0.1); /* 100 rpm low */
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);
  CHECK_NEAR(e.idle_error_rpm, 100.0, 1e-12);
  CHECK_NEAR(e.idle_p_term, 0.0003 * 100.0, 1e-12);
  CHECK_NEAR(e.idle_i_term, 0.0003 * 100.0 * 0.1, 1e-12); /* ki * e * dt */
  CHECK_NEAR(e.idle_raw, e.idle_p_term + e.idle_i_term, 1e-12);
  CHECK_NEAR(e.idle_throttle, e.idle_raw, 1e-12);
  CHECK_NEAR(cmd, e.idle_throttle, 1e-12);
  CHECK_NEAR(e.throttle_cmd, cmd, 0.0);
  CHECK_NEAR(e.idle_target_rpm, 800.0, 0.0);
}

/* The integrator keeps accumulating while the error persists. */
static void test_integrator_accumulates(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  double prev = 0.0;
  for (int i = 0; i < 10; i++) {
    ecu_step(&e, &p, 1, 1, 750.0, 0.0, 0.1);
    CHECK(e.idle_i_term > prev);
    prev = e.idle_i_term;
  }
  CHECK_NEAR(e.idle_i_term, 10 * 0.0003 * 50.0 * 0.1, 1e-12);
}

/* Above the target the governor adds nothing and the integrator unwinds to 0
 * without going negative: it can only add throttle. */
static void test_overspeed_adds_nothing_and_never_goes_negative(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  for (int i = 0; i < 20; i++) {
    ecu_step(&e, &p, 1, 1, 700.0, 0.0, 0.1);
  }
  CHECK(e.idle_i_term > 0.0);
  for (int i = 0; i < 400; i++) {
    ecu_step(&e, &p, 1, 1, 1000.0, 0.0, 0.1);
    CHECK(e.idle_i_term >= 0.0);
    CHECK(e.idle_throttle >= 0.0);
  }
  CHECK_NEAR(e.idle_i_term, 0.0, 1e-12);
  CHECK_NEAR(e.idle_throttle, 0.0, 1e-12);
  CHECK(e.idle_p_term < 0.0); /* the raw terms still show the error's sign */
}

/* Output and integrator are held to the authority; a full overload reports
 * LIMITED, not ACTIVE. */
static void test_authority_clamp_and_limited_mode(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  for (int i = 0; i < 2000; i++) {
    ecu_step(&e, &p, 1, 1, 400.0, 0.0, 0.1);
    CHECK(e.idle_throttle <= p.max_throttle + 1e-12);
    CHECK(e.idle_i_term <= p.max_throttle + 1e-12);
  }
  CHECK(e.idle_mode == ECU_IDLE_LIMITED);
  CHECK_NEAR(e.idle_throttle, p.max_throttle, 1e-12);
  CHECK(e.idle_raw >= p.max_throttle);
}

/* max(pilot, governor): the pilot's throttle is never reduced. */
static void test_command_is_max_of_pilot_and_governor(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  double cmd = ecu_step(&e, &p, 1, 1, 700.0, 0.02, 0.1); /* governor > 0.02 */
  CHECK(cmd > 0.02);
  CHECK_NEAR(cmd, e.idle_throttle, 1e-12);

  ecu_init(&e);
  cmd = ecu_step(&e, &p, 1, 1, 790.0, 0.10, 0.1); /* pilot > governor */
  CHECK_NEAR(cmd, 0.10, 1e-12);
  CHECK_NEAR(e.pilot_throttle, 0.10, 1e-12);
}

/* At/above the authority the pilot is in control and the integrator is held. */
static void test_pilot_in_control_holds_the_integrator(void) {
  EcuState e;
  ecu_init(&e);
  EcuIdleParams p = params();
  for (int i = 0; i < 50; i++) {
    ecu_step(&e, &p, 1, 1, 740.0, 0.0, 0.1);
  }
  double held = e.idle_i_term;
  CHECK(held > 0.0);

  for (int i = 0; i < 50; i++) {
    double cmd = ecu_step(&e, &p, 1, 1, 300.0, 0.6, 0.1);
    CHECK(e.idle_mode == ECU_IDLE_PILOT);
    CHECK_NEAR(cmd, 0.6, 1e-12);
    CHECK_NEAR(e.idle_i_term, held, 1e-12); /* held, not wound up or reset */
  }
  /* a chop finds the idle throttle it had already found */
  ecu_step(&e, &p, 1, 1, 740.0, 0.0, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);
  CHECK(e.idle_i_term >= held);
}

static void test_operator_off_disabled_and_standby(void) {
  EcuIdleParams p = params();
  EcuState e;
  ecu_init(&e);
  for (int i = 0; i < 20; i++) {
    ecu_step(&e, &p, 1, 1, 700.0, 0.0, 0.1);
  }
  CHECK(e.idle_i_term > 0.0);

  /* operator switch: pilot only, loop cleared, error still reported */
  ecu_set_idle_enabled(&e, 0);
  double cmd = ecu_step(&e, &p, 1, 1, 700.0, 0.03, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_OFF);
  CHECK_NEAR(cmd, 0.03, 1e-12);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(e.idle_error_rpm, 100.0, 1e-12);

  /* back on: starts from a clean integrator */
  ecu_set_idle_enabled(&e, 1);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  ecu_step(&e, &p, 1, 1, 700.0, 0.0, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_ACTIVE);

  /* not running / ignition off: standby, loop cleared */
  ecu_step(&e, &p, 0, 1, 700.0, 0.0, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  ecu_step(&e, &p, 1, 0, 700.0, 0.0, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(e.throttle_cmd, 0.0, 0.0);

  /* no target configured: disabled, command is the pilot's */
  EcuIdleParams none = p;
  none.target_rpm = 0.0;
  cmd = ecu_step(&e, &none, 1, 1, 700.0, 0.2, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_DISABLED);
  CHECK_NEAR(cmd, 0.2, 1e-12);
  CHECK_NEAR(e.idle_error_rpm, 0.0, 0.0);
  EcuIdleParams no_authority = p;
  no_authority.max_throttle = 0.0;
  ecu_step(&e, &no_authority, 1, 1, 700.0, 0.0, 0.1);
  CHECK(e.idle_mode == ECU_IDLE_DISABLED);
}

/* Clearing the loop for an engine start keeps the operator's switch. */
static void test_reset_idle_keeps_the_operator_switch(void) {
  EcuIdleParams p = params();
  EcuState e;
  ecu_init(&e);
  ecu_set_idle_enabled(&e, 0);
  ecu_reset_idle(&e);
  CHECK(e.idle_enabled == 0);

  ecu_set_idle_enabled(&e, 1);
  for (int i = 0; i < 10; i++) {
    ecu_step(&e, &p, 1, 1, 700.0, 0.0, 0.1);
  }
  ecu_reset_idle(&e);
  CHECK(e.idle_enabled == 1);
  CHECK_NEAR(e.idle_i_term, 0.0, 0.0);
  CHECK_NEAR(e.idle_p_term, 0.0, 0.0);
}

static void test_mode_names_exist_and_differ(void) {
  const EcuIdleMode modes[] = {ECU_IDLE_DISABLED, ECU_IDLE_OFF, ECU_IDLE_STANDBY,
                               ECU_IDLE_PILOT,    ECU_IDLE_ACTIVE, ECU_IDLE_LIMITED};
  const int n = (int)(sizeof modes / sizeof modes[0]);
  for (int i = 0; i < n; i++) {
    const char *a = ecu_idle_mode_name(modes[i]);
    CHECK(a != NULL && a[0] != '\0' && a[0] != '?');
    for (int j = i + 1; j < n; j++) {
      const char *b = ecu_idle_mode_name(modes[j]);
      CHECK(a != b);
    }
  }
}

static const TestCase CASES[] = {
    {"ecu.init_is_enabled_and_zeroed", test_init_is_enabled_and_zeroed},
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
    {"ecu.mode_names_exist_and_differ", test_mode_names_exist_and_differ},
};

RUN_TESTS(CASES)
