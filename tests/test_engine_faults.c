#include "telemetry/engine_faults.h"

#include "model/sync.h"

#include <string.h>

#include "test_util.h"

static EngineTrace TR; /* ~700 KB: off the stack */

static void healthy_state(ModelState *s) {
  memset(s, 0, sizeof *s);
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    s->cyl[i].cht_c = 150.0;
    s->cyl[i].egt_c = 600.0;
    s->cyl[i].lambda = 1.0;
    s->cyl[i].misfire_rate = 0.0;
  }
}

static void healthy_configs(CylinderConfig *c) {
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    c[i] = cylinder_config_default();
  }
}

static void test_a_healthy_cylinder_has_no_fault(void) {
  CylinderConfig c = cylinder_config_default();
  CHECK(!cylinder_has_fault(&c));
  char text[64];
  cylinder_fault_text(&c, text, sizeof text);
  CHECK(text[0] == '\0');
}

static void test_each_trim_off_nominal_is_a_fault(void) {
  CylinderConfig c = cylinder_config_default();
  c.injector_flow_trim = 0.35;
  CHECK(cylinder_has_fault(&c));
  c = cylinder_config_default();
  c.compression_trim = 0.5;
  CHECK(cylinder_has_fault(&c));
  c = cylinder_config_default();
  c.spark_offset_deg = 20.0;
  CHECK(cylinder_has_fault(&c));
  c = cylinder_config_default();
  c.intake_leak_frac = 0.5;
  CHECK(cylinder_has_fault(&c));
  c = cylinder_config_default();
  c.cooling_trim = 0.3;
  CHECK(cylinder_has_fault(&c));
  c = cylinder_config_default();
  c.cooling_trim = 1.0 + 1e-9; /* float dust is not a fault */
  CHECK(!cylinder_has_fault(&c));
}

static void test_fault_text_lists_only_the_trims_that_are_off(void) {
  CylinderConfig c = cylinder_config_default();
  c.compression_trim = 0.5;
  c.cooling_trim = 0.3;
  char text[96];
  cylinder_fault_text(&c, text, sizeof text);
  CHECK(strcmp(text, "compression 0.50, cooling 0.30") == 0);

  c = cylinder_config_default();
  c.spark_offset_deg = 20.0;
  cylinder_fault_text(&c, text, sizeof text);
  CHECK(strcmp(text, "spark +20 deg") == 0);

  /* a too-small buffer is cut short, not overrun */
  c.injector_flow_trim = 0.35;
  c.intake_leak_frac = 0.5;
  char small[12];
  cylinder_fault_text(&c, small, sizeof small);
  CHECK(strlen(small) < sizeof small);
}

static void test_symptom_status_is_the_worst_of_the_limits(void) {
  CylinderState c = {0};
  c.cht_c = 150.0;
  c.egt_c = 600.0;
  c.lambda = 1.0;
  CHECK(cylinder_symptom_status(&c) == CHANNEL_OK);

  c.cht_c = 215.0;
  CHECK(cylinder_symptom_status(&c) == CHANNEL_WARN);
  c.cht_c = 245.0;
  CHECK(cylinder_symptom_status(&c) == CHANNEL_ALERT);

  c.cht_c = 150.0;
  c.lambda = 1.6; /* lean past the misfire edge */
  CHECK(cylinder_symptom_status(&c) == CHANNEL_ALERT);

  c.lambda = 1.0;
  c.misfire_rate = 0.2;
  CHECK(cylinder_symptom_status(&c) == CHANNEL_WARN);
  c.misfire_rate = 0.9;
  CHECK(cylinder_symptom_status(&c) == CHANNEL_ALERT);
}

/* The tracker times the fault, and what the monitors did about it. */
static void test_tracker_times_the_fault_and_the_first_flag(void) {
  EngineFaultTracker t;
  engine_faults_init(&t);
  CylinderConfig cfg[ENGINE_MAX_CYLINDERS];
  healthy_configs(cfg);
  ModelState s;
  healthy_state(&s);

  engine_faults_update(&t, cfg, &s, 4, 1.0);
  CHECK(!t.cyl[1].faulty);

  cfg[1].compression_trim = 0.5; /* injected at t = 5 */
  engine_faults_update(&t, cfg, &s, 4, 5.0);
  CHECK(t.cyl[1].faulty);
  CHECK_NEAR(t.cyl[1].since_s, 5.0, 0.0);
  CHECK(!t.cyl[1].flagged); /* nothing has noticed */
  CHECK(t.cyl[1].now == CHANNEL_OK);

  engine_faults_update(&t, cfg, &s, 4, 9.0);
  CHECK(!t.cyl[1].flagged); /* still nothing */

  s.cyl[1].misfire_rate = 0.3; /* the monitors see it at t = 12 */
  engine_faults_update(&t, cfg, &s, 4, 12.0);
  CHECK(t.cyl[1].flagged);
  CHECK_NEAR(t.cyl[1].first_flag_s, 12.0, 0.0);
  CHECK(t.cyl[1].now == CHANNEL_WARN);

  s.cyl[1].misfire_rate = 0.9; /* gets worse */
  engine_faults_update(&t, cfg, &s, 4, 14.0);
  CHECK(t.cyl[1].now == CHANNEL_ALERT);
  CHECK(t.cyl[1].worst == CHANNEL_ALERT);
  CHECK_NEAR(t.cyl[1].first_flag_s, 12.0, 0.0); /* the first flag is kept */

  s.cyl[1].misfire_rate = 0.0; /* symptoms clear, the fault is still there */
  engine_faults_update(&t, cfg, &s, 4, 20.0);
  CHECK(t.cyl[1].now == CHANNEL_OK);
  CHECK(t.cyl[1].flagged);
  CHECK(t.cyl[1].worst == CHANNEL_ALERT);
}

static void test_tracker_forgets_a_removed_fault_and_ignores_other_cylinders(void) {
  EngineFaultTracker t;
  engine_faults_init(&t);
  CylinderConfig cfg[ENGINE_MAX_CYLINDERS];
  healthy_configs(cfg);
  ModelState s;
  healthy_state(&s);

  cfg[0].cooling_trim = 0.3;
  cfg[3].injector_flow_trim = 0.35;
  s.cyl[3].misfire_rate = 1.0;
  engine_faults_update(&t, cfg, &s, 4, 3.0);
  CHECK(t.cyl[0].faulty && t.cyl[3].faulty);
  CHECK(!t.cyl[1].faulty && !t.cyl[2].faulty);
  CHECK(t.cyl[3].flagged && !t.cyl[0].flagged); /* independent records */

  cfg[3] = cylinder_config_default(); /* repaired */
  engine_faults_update(&t, cfg, &s, 4, 6.0);
  CHECK(!t.cyl[3].faulty);
  CHECK(!t.cyl[3].flagged);
  CHECK(t.cyl[0].faulty);

  /* a fault on a slot beyond the cylinder count is not tracked */
  cfg[5].compression_trim = 0.5;
  engine_faults_update(&t, cfg, &s, 4, 7.0);
  CHECK(!t.cyl[5].faulty);

  /* re-injecting starts a fresh record */
  cfg[3].injector_flow_trim = 0.35;
  engine_faults_update(&t, cfg, &s, 4, 9.0);
  CHECK_NEAR(t.cyl[3].since_s, 9.0, 0.0);
}

static void test_symptoms_are_measured_against_the_other_cylinders(void) {
  ModelState s;
  healthy_state(&s);
  s.cyl[2].cht_c = 195.0; /* 45 hotter than the others' 150 */
  s.cyl[2].egt_c = 560.0; /* 40 cooler than 600 */
  s.cyl[2].lambda = 1.4;
  s.cyl[2].misfire_rate = 0.25;

  CylSymptoms y = engine_faults_symptoms(&s, 2, 4, NULL);
  CHECK_NEAR(y.d_cht_c, 45.0, 1e-9);
  CHECK_NEAR(y.d_egt_c, -40.0, 1e-9);
  CHECK_NEAR(y.lambda, 1.4, 1e-9);
  CHECK_NEAR(y.misfire_pct, 25.0, 1e-9);
  CHECK(y.power_pct < 0.0); /* no trace: unknown */

  /* nothing to compare against with a single cylinder */
  y = engine_faults_symptoms(&s, 0, 1, NULL);
  CHECK_NEAR(y.d_cht_c, 0.0, 0.0);
  CHECK(y.power_pct < 0.0);
  /* out of range */
  y = engine_faults_symptoms(&s, 7, 4, NULL);
  CHECK_NEAR(y.lambda, 0.0, 0.0);
}

/* Fills a trace with one full cycle where each cylinder's torque is `q[i]`. */
static void fill_trace(const double q[4]) {
  engine_trace_clear(&TR);
  for (int k = 0; k < 720; k++) {
    EngineTraceSample x;
    memset(&x, 0, sizeof x);
    x.theta_deg = (float)k;
    for (int i = 0; i < 4; i++) {
      x.cyl_gas_nm[i] = (float)q[i];
      x.cyl_inertia_nm[i] = 0.0f;
    }
    engine_trace_push(&TR, &x);
  }
}

static void test_power_share_comes_from_the_trace(void) {
  ModelState s;
  healthy_state(&s);
  const double q[4] = {20.0, 20.0, 10.0, 20.0}; /* cylinder 3 makes half */
  fill_trace(q);
  CylSymptoms y = engine_faults_symptoms(&s, 2, 4, &TR);
  CHECK_NEAR(y.power_pct, 50.0, 0.5);
  y = engine_faults_symptoms(&s, 0, 4, &TR);
  /* a healthy cylinder against the mixed others: 20 vs (20 + 10 + 20) / 3 */
  CHECK_NEAR(y.power_pct, 120.0, 0.5);

  /* an engine that is not making torque has no meaningful share */
  const double none[4] = {0.0, 0.0, 0.0, 0.0};
  fill_trace(none);
  y = engine_faults_symptoms(&s, 2, 4, &TR);
  CHECK(y.power_pct < 0.0);
}

/* ---- against the real engine ---- */

/* Cruise-like flight (0.75 throttle, 30 m/s) with the tracker sampled as the
 * app does; returns the state and tracker after `seconds`. */
static void fly(ModelSync *s, ModelState *st, EngineFaultTracker *t,
                double seconds) {
  const EngineInput in = {0.75, 0.0, 101.325};
  const EnvInput env = {0.0, 30.0, 0.0};
  const int n = (int)(seconds / 0.05 + 0.5);
  for (int i = 0; i < n; i++) {
    model_sync_step(s, st, &in, &env, 0.05);
    engine_faults_update(t, s->cyl_config, st, s->engine_config.num_cylinders,
                         s->sim_time_s);
  }
}

/* A lean injector misfires the cylinder and the monitors flag it at once. */
static void test_a_lean_injector_is_flagged_immediately(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  EngineFaultTracker t;
  engine_faults_init(&t);
  fly(&s, &st, &t, 10.0);
  CHECK(!t.cyl[1].faulty);

  s.cyl_config[1].injector_flow_trim = 0.35;
  fly(&s, &st, &t, 10.0);
  CHECK(t.cyl[1].faulty);
  CHECK(t.cyl[1].flagged);
  CHECK(t.cyl[1].first_flag_s - t.cyl[1].since_s < 2.0);
  CHECK(t.cyl[1].now == CHANNEL_ALERT);
  const CylSymptoms y = engine_faults_symptoms(&st, 1, 4, NULL);
  CHECK(y.misfire_pct > 90.0);
  CHECK(y.lambda > 1.55);
  CHECK(!t.cyl[0].faulty && !t.cyl[2].faulty && !t.cyl[3].faulty);
}

/* A cooling problem heats one head; the monitors flag it once it passes the
 * limit, tens of seconds later. */
static void test_a_cooling_loss_is_flagged_once_the_head_is_hot(void) {
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  EngineFaultTracker t;
  engine_faults_init(&t);
  fly(&s, &st, &t, 5.0);

  s.cyl_config[2].cooling_trim = 0.3;
  fly(&s, &st, &t, 30.0);
  CHECK(t.cyl[2].faulty);
  CHECK(!t.cyl[2].flagged); /* the head has not yet crossed the limit */

  fly(&s, &st, &t, 50.0);
  CHECK(t.cyl[2].flagged);
  const double delay = t.cyl[2].first_flag_s - t.cyl[2].since_s;
  CHECK(delay > 20.0 && delay < 70.0);
  const CylSymptoms y = engine_faults_symptoms(&st, 2, 4, NULL);
  CHECK(y.d_cht_c > 100.0); /* far hotter than the other heads */
}

/* The blind spot: a compression loss cuts the cylinder's power but leaves every
 * monitored channel inside its limits, so nothing ever flags it. */
static void test_a_compression_loss_is_never_flagged_but_shows_in_power(void) {
  static EngineTrace trace;
  ModelSync s;
  model_sync_init(&s);
  ModelState st;
  model_state_init(&st, &s.engine_config, 15.0);
  st.engine.trace = &trace;
  EngineFaultTracker t;
  engine_faults_init(&t);
  fly(&s, &st, &t, 5.0);

  s.cyl_config[1].compression_trim = 0.5;
  fly(&s, &st, &t, 60.0);
  CHECK(t.cyl[1].faulty);
  CHECK(!t.cyl[1].flagged);
  CHECK(t.cyl[1].worst == CHANNEL_OK);

  const CylSymptoms y = engine_faults_symptoms(&st, 1, 4, &trace);
  CHECK(y.power_pct > 60.0 && y.power_pct < 95.0); /* down, but not by much */
  CHECK_NEAR(y.misfire_pct, 0.0, 0.0);
}

static const TestCase CASES[] = {
    {"engine_faults.a_lean_injector_is_flagged_immediately",
     test_a_lean_injector_is_flagged_immediately},
    {"engine_faults.a_cooling_loss_is_flagged_once_the_head_is_hot",
     test_a_cooling_loss_is_flagged_once_the_head_is_hot},
    {"engine_faults.a_compression_loss_is_never_flagged_but_shows_in_power",
     test_a_compression_loss_is_never_flagged_but_shows_in_power},
    {"engine_faults.a_healthy_cylinder_has_no_fault",
     test_a_healthy_cylinder_has_no_fault},
    {"engine_faults.each_trim_off_nominal_is_a_fault",
     test_each_trim_off_nominal_is_a_fault},
    {"engine_faults.fault_text_lists_only_the_trims_that_are_off",
     test_fault_text_lists_only_the_trims_that_are_off},
    {"engine_faults.symptom_status_is_the_worst_of_the_limits",
     test_symptom_status_is_the_worst_of_the_limits},
    {"engine_faults.tracker_times_the_fault_and_the_first_flag",
     test_tracker_times_the_fault_and_the_first_flag},
    {"engine_faults.tracker_forgets_a_removed_fault_and_ignores_other_cylinders",
     test_tracker_forgets_a_removed_fault_and_ignores_other_cylinders},
    {"engine_faults.symptoms_are_measured_against_the_other_cylinders",
     test_symptoms_are_measured_against_the_other_cylinders},
    {"engine_faults.power_share_comes_from_the_trace",
     test_power_share_comes_from_the_trace},
};

RUN_TESTS(CASES)
