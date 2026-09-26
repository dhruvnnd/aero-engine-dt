#include "test_util.h"

#include <string.h>

#include "model/state.h"
#include "model/sync.h"
#include "physics/ecu.h"
#include "physics/engine_model.h"

/* The ECU wired to the engine by model_sync_step(). Flat loads stand in for
 * the propeller here (cq_static = 0) so the numbers are the engine's own. */

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

/* Settled closed-throttle engine with a flat load. */
static ModelState idle_at(double target_rpm, double load_nm, double seconds) {
  ModelSync s = make_sync(1, target_rpm);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, load_nm, seconds);
  return st;
}

/* ---- the governor, through the whole system ---- */

static void test_governor_holds_the_target(void) {
  for (double target = 700.0; target <= 1100.0; target += 200.0) {
    ModelState st = idle_at(target, 4.0, 40.0);
    CHECK(st.engine.run_state == ENGINE_RUNNING);
    CHECK_NEAR(st.rpm, target, 20.0);
    CHECK(st.ecu.idle_throttle > 0.0);
    CHECK(st.ecu.idle_throttle < 0.15);
    CHECK(st.ecu.fitted == 1);
  }
}

static void test_governor_off_or_pointless_adds_nothing(void) {
  ModelState off = idle_at(0.0, 1.0, 40.0);
  CHECK(off.ecu.idle_mode == ECU_IDLE_DISABLED);
  CHECK_NEAR(off.ecu.idle_throttle, 0.0, 0.0);
  CHECK(off.engine.run_state == ENGINE_RUNNING);

  /* a target below where the engine idles by itself: the governor can only add
   * throttle, so it stays out of the way */
  ModelState low = idle_at(200.0, 1.0, 40.0);
  CHECK_NEAR(low.ecu.idle_throttle, 0.0, 1e-9);
  CHECK_NEAR(low.rpm, off.rpm, 5.0);
  CHECK(low.rpm > 400.0);
}

static void test_governor_recovers_from_a_load_step(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 4.0, 30.0);
  CHECK_NEAR(st.rpm, 800.0, 20.0);
  const double before = st.ecu.idle_throttle;

  run(&s, &st, 0.0, 7.0, 30.0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
  CHECK(st.ecu.idle_throttle > before); /* opened up to carry the load */
}

/* Authority is finite: an overload leaves RPM below the target with the
 * governor pinned; when it goes away there is no big overshoot (anti-windup). */
static void test_governor_authority_limit_and_no_windup(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 12.0, 30.0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  CHECK(st.rpm < 750.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_LIMITED);
  CHECK_NEAR(st.ecu.idle_throttle, 0.15, 1e-9);
  CHECK(st.ecu.idle_i_term <= 0.15 + 1e-12);

  double peak = 0.0;
  for (int i = 0; i < 1000; i++) { /* 20 s */
    run(&s, &st, 0.0, 4.0, 0.02);
    peak = st.rpm > peak ? st.rpm : peak;
  }
  CHECK(peak < 1000.0);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
}

/* Closing the throttle from cruise must not stall the engine. */
static void test_governor_survives_a_throttle_chop(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.8, 6.0, 20.0);
  CHECK(st.rpm > 1500.0);
  double lowest = 1e9;
  for (int i = 0; i < 1500; i++) { /* 30 s at closed throttle */
    run(&s, &st, 0.0, 6.0, 0.02);
    lowest = st.rpm < lowest ? st.rpm : lowest;
    CHECK(st.engine.run_state == ENGINE_RUNNING);
  }
  CHECK(lowest > 400.0);
  CHECK_NEAR(st.rpm, 800.0, 25.0);
}

/* Nothing with the ignition off, and a stopped engine reports standby. */
static void test_governor_only_acts_on_a_running_engine(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 4.0, 20.0);
  CHECK(st.ecu.idle_throttle > 0.0);

  engine_model_stop(&st.engine); /* ignition off, still spinning down */
  run(&s, &st, 0.0, 4.0, 0.1);
  CHECK(st.ecu.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(st.ecu.idle_i_term, 0.0, 0.0);

  run(&s, &st, 0.0, 4.0, 10.0); /* runs down and stalls */
  CHECK(st.engine.run_state == ENGINE_STOPPED);
  CHECK(st.ecu.idle_mode == ECU_IDLE_STANDBY);
  CHECK_NEAR(st.ecu.throttle_cmd, 0.0, 0.0);
}

/* Above its authority the ECU steps aside: a fitted ECU and no ECU at all
 * behave exactly the same at a throttle the pilot controls. */
static void test_ecu_steps_aside_above_its_authority(void) {
  ModelSync a = make_sync(1, 800.0);
  ModelSync b = make_sync(0, 800.0);
  ModelState sa = make_state(&a);
  ModelState sb = make_state(&b);
  run(&a, &sa, 0.6, 8.0, 20.0);
  run(&b, &sb, 0.6, 8.0, 20.0);
  CHECK_NEAR(sa.engine.omega_rad_s, sb.engine.omega_rad_s, 1e-9);
  CHECK_NEAR(sa.engine.map_kpa, sb.engine.map_kpa, 1e-9);
}

/* The live operator switch: off lets idle sag to the engine's natural speed
 * (without stalling it); on brings it home again. */
static void test_governor_can_be_toggled_during_a_run(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 1.0, 30.0);
  CHECK_NEAR(st.rpm, 800.0, 20.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);

  ecu_set_idle_enabled(&st.ecu, 0);
  run(&s, &st, 0.0, 1.0, 30.0);
  CHECK(st.engine.run_state == ENGINE_RUNNING); /* sags, doesn't stall */
  CHECK(st.ecu.idle_mode == ECU_IDLE_OFF);
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 0.0);
  CHECK_NEAR(st.ecu.throttle_cmd, 0.0, 0.0);
  CHECK(st.rpm < 700.0);
  CHECK(st.rpm > 400.0);

  ecu_set_idle_enabled(&st.ecu, 1);
  run(&s, &st, 0.0, 1.0, 40.0);
  CHECK(st.ecu.idle_mode == ECU_IDLE_ACTIVE);
  CHECK_NEAR(st.rpm, 800.0, 20.0);
}

/* ---- decoupling: the engine cannot tell an ECU from a pilot ---- */

/* An engine driven by an ECU behaves exactly like one whose pilot simply holds
 * the throttle the ECU settled on. */
static void test_engine_only_sees_the_throttle_it_is_given(void) {
  ModelSync s = make_sync(1, 800.0);
  ModelState st = make_state(&s);
  run(&s, &st, 0.0, 4.0, 60.0);
  const double command = st.ecu.throttle_cmd;
  CHECK(command > 0.05);

  ModelSync p = make_sync(0, 800.0); /* no ECU: the pilot holds `command` */
  ModelState sp = make_state(&p);
  run(&p, &sp, command, 4.0, 60.0);
  CHECK_NEAR(sp.rpm, st.rpm, 15.0);
  CHECK_NEAR(sp.engine.map_kpa, st.engine.map_kpa, 2.0);
}

/* ---- an engine with no ECU ---- */

static void test_no_ecu_means_the_pilot_drives_the_engine(void) {
  ModelSync s = make_sync(0, 800.0);
  ModelState st = make_state(&s);
  CHECK(st.ecu.fitted == 0);
  run(&s, &st, 0.0, 1.0, 40.0);

  CHECK(st.ecu.fitted == 0);
  CHECK_NEAR(st.ecu.throttle_cmd, 0.0, 0.0); /* the pilot's, untouched */
  CHECK_NEAR(st.ecu.idle_throttle, 0.0, 0.0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
  /* nothing holds idle: it sits at the engine's natural speed, below what a
   * governor would hold */
  CHECK(st.rpm < 700.0);
  CHECK(st.rpm > 400.0);
}

/* The operator switch means nothing without an ECU. */
static void test_operator_switch_has_no_effect_without_an_ecu(void) {
  ModelSync a = make_sync(0, 800.0);
  ModelSync b = make_sync(0, 800.0);
  ModelState sa = make_state(&a);
  ModelState sb = make_state(&b);
  run(&a, &sa, 0.0, 1.0, 10.0);
  run(&b, &sb, 0.0, 1.0, 10.0);
  ecu_set_idle_enabled(&sb.ecu, 0);
  ecu_set_idle_enabled(&sb.ecu, 1);
  run(&a, &sa, 0.0, 1.0, 20.0);
  run(&b, &sb, 0.0, 1.0, 20.0);
  CHECK_NEAR(sa.engine.omega_rad_s, sb.engine.omega_rad_s, 0.0);
  CHECK(sb.ecu.fitted == 0);
}

/* No ECU and an ECU with the governor configured off are the same engine to the
 * bit: with nothing to add, the pilot's throttle is all the engine ever gets. */
static void test_no_ecu_equals_an_ecu_that_does_nothing(void) {
  ModelSync a = make_sync(0, 800.0);
  ModelSync b = make_sync(1, 0.0); /* fitted, but idle_target_rpm = 0 */
  ModelState sa = make_state(&a);
  ModelState sb = make_state(&b);
  const double throttles[] = {0.0, 0.3, 0.9, 0.05, 0.0, 0.6};
  for (size_t i = 0; i < sizeof throttles / sizeof throttles[0]; i++) {
    run(&a, &sa, throttles[i], 3.0, 8.0);
    run(&b, &sb, throttles[i], 3.0, 8.0);
    CHECK_NEAR(sa.engine.omega_rad_s, sb.engine.omega_rad_s, 0.0);
    CHECK_NEAR(sa.engine.map_kpa, sb.engine.map_kpa, 0.0);
    CHECK_NEAR(sa.thermal.cht_c, sb.thermal.cht_c, 0.0);
  }
  CHECK(sa.ecu.fitted == 0);
  CHECK(sb.ecu.fitted == 1);
}

/* A restart keeps working with and without an ECU. */
static void test_engine_starts_with_and_without_an_ecu(void) {
  for (int fitted = 0; fitted <= 1; fitted++) {
    ModelSync s = make_sync(fitted, 800.0);
    s.engine_config.prop = prop_config_default(); /* the real propeller */
    ModelState st = make_state(&s);
    st.engine.run_state = ENGINE_STOPPED;
    st.engine.omega_rad_s = 0.0;
    st.engine.map_kpa = 101.325;
    engine_model_start(&st.engine, &s.engine_config, st.cyl);
    run(&s, &st, 0.7, 0.0, 6.0); /* cranks, catches, runs at 0.7 throttle */
    CHECK(st.engine.run_state == ENGINE_RUNNING);
    CHECK(st.rpm > 1000.0);
    CHECK(st.ecu.fitted == fitted);
  }
}

static const TestCase CASES[] = {
    {"ecu_integration.governor_holds_the_target",
     test_governor_holds_the_target},
    {"ecu_integration.governor_off_or_pointless_adds_nothing",
     test_governor_off_or_pointless_adds_nothing},
    {"ecu_integration.governor_recovers_from_a_load_step",
     test_governor_recovers_from_a_load_step},
    {"ecu_integration.governor_authority_limit_and_no_windup",
     test_governor_authority_limit_and_no_windup},
    {"ecu_integration.governor_survives_a_throttle_chop",
     test_governor_survives_a_throttle_chop},
    {"ecu_integration.governor_only_acts_on_a_running_engine",
     test_governor_only_acts_on_a_running_engine},
    {"ecu_integration.ecu_steps_aside_above_its_authority",
     test_ecu_steps_aside_above_its_authority},
    {"ecu_integration.governor_can_be_toggled_during_a_run",
     test_governor_can_be_toggled_during_a_run},
    {"ecu_integration.engine_only_sees_the_throttle_it_is_given",
     test_engine_only_sees_the_throttle_it_is_given},
    {"ecu_integration.no_ecu_means_the_pilot_drives_the_engine",
     test_no_ecu_means_the_pilot_drives_the_engine},
    {"ecu_integration.operator_switch_has_no_effect_without_an_ecu",
     test_operator_switch_has_no_effect_without_an_ecu},
    {"ecu_integration.no_ecu_equals_an_ecu_that_does_nothing",
     test_no_ecu_equals_an_ecu_that_does_nothing},
    {"ecu_integration.engine_starts_with_and_without_an_ecu",
     test_engine_starts_with_and_without_an_ecu},
};

RUN_TESTS(CASES)
