#include "test_util.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/ecu.h"
#include "physics/engine_model.h"

/* The ECU's temperature codes against the real engine. Thermal effects take
 * tens of simulated seconds, so these live in their own executable and run
 * alongside the others. */

static int dtc_active(const ModelState *st, EcuDtcId id) {
  return st->ecu.diag.dtc[id].active;
}

/* Cruise-like flight on the default engine, with its real propeller. */
static void cruise(ModelSync *s, ModelState *st, double seconds) {
  const EngineInput in = {0.75, 0.0, 101.325};
  const EnvInput env = {0.0, 30.0, 0.0};
  const int n = (int)(seconds / 0.05 + 0.5);
  for (int i = 0; i < n; i++) {
    model_sync_step(s, st, &in, &env, 0.05);
  }
}

/* A cooling problem heats one head far past its limit: the ECU sets the high
 * code, then the critical one, and neither touches the speed sensing. */
static void test_a_cooling_loss_sets_the_head_temperature_codes(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  cruise(&s, &st, 5.0);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);

  const double injected_at = s.sim_time_s;
  s.cyl_config[2].cooling_trim = 0.3;
  cruise(&s, &st, 80.0);
  CHECK(dtc_active(&st, ECU_DTC_CHT_HIGH));
  CHECK(dtc_active(&st, ECU_DTC_CHT_CRIT));
  const double delay = st.ecu.diag.dtc[ECU_DTC_CHT_HIGH].first_s - injected_at;
  CHECK(delay > 30.0 && delay < 75.0); /* the head has to heat up first */
  CHECK(st.ecu.cht_seen > 240.0);
  CHECK(!dtc_active(&st, ECU_DTC_EGT_HIGH)); /* only the head is hot */
  CHECK(st.ecu.speed_source == ECU_SRC_PRIMARY);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
}

/* A lean cylinder runs its exhaust port far too hot. */
static void test_a_lean_cylinder_sets_the_exhaust_temperature_codes(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  cruise(&s, &st, 5.0);

  s.cyl_config[3].injector_flow_trim = 0.35;
  cruise(&s, &st, 45.0);
  CHECK(dtc_active(&st, ECU_DTC_EGT_HIGH));
  CHECK(dtc_active(&st, ECU_DTC_EGT_CRIT));
  CHECK(st.ecu.egt_seen > 850.0);
  CHECK(!dtc_active(&st, ECU_DTC_CHT_HIGH));
}

/* Ordinary cruise raises no code at all. */
static void test_healthy_cruise_sets_no_temperature_codes(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  cruise(&s, &st, 60.0);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
  CHECK(st.ecu.cht_seen > 40.0 && st.ecu.cht_seen < 200.0); /* warming, in limits */
}

/* The ECU's blind spot: a compression loss leaves every temperature normal, so
 * it sets no code (the Engine Faults panel shows it only as lost power). */
static void test_a_compression_loss_sets_no_ecu_code(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  cruise(&s, &st, 5.0);
  s.cyl_config[1].compression_trim = 0.5;
  cruise(&s, &st, 40.0);
  CHECK(ecu_diag_latched_count(&st.ecu.diag) == 0);
  CHECK(st.engine.run_state == ENGINE_RUNNING);
}

static const TestCase CASES[] = {
    {"ecu_temps.a_cooling_loss_sets_the_head_temperature_codes",
     test_a_cooling_loss_sets_the_head_temperature_codes},
    {"ecu_temps.a_lean_cylinder_sets_the_exhaust_temperature_codes",
     test_a_lean_cylinder_sets_the_exhaust_temperature_codes},
    {"ecu_temps.healthy_cruise_sets_no_temperature_codes",
     test_healthy_cruise_sets_no_temperature_codes},
    {"ecu_temps.a_compression_loss_sets_no_ecu_code",
     test_a_compression_loss_sets_no_ecu_code},
};

RUN_TESTS(CASES)
