#include "test_util.h"

#include "physics/thermal.h"

static void run(ThermalState *st, const ThermalConfig *cfg, double q,
                double load_frac, double amb, double duration_s, double dt) {
  int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    thermal_step(st, cfg, q, load_frac, amb, i * dt, dt);
  }
}

static void test_init_puts_everything_at_ambient(void) {
  ThermalState st;
  thermal_init(&st, 15.0);
  CHECK_NEAR(st.cht_c, 15.0, 1e-9);
  CHECK_NEAR(st.egt_c, 15.0, 1e-9);
  CHECK_NEAR(st.oil_temp_c, 15.0, 1e-9);
}

static void test_config_time_constants_are_ordered(void) {
  ThermalConfig cfg = thermal_config_default();
  CHECK(cfg.egt_tau_s < cfg.cht_tau_s); /* exhaust gas responds fastest */
  CHECK(cfg.cht_tau_s < cfg.oil_tau_s); /* oil sump slowest */
}

/* Stopped engine (no load, no waste heat) settles everything at ambient. */
static void test_stopped_engine_holds_at_ambient(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  thermal_init(&st, 20.0);
  run(&st, &cfg, 0.0, 0.0, 20.0, 300.0, 0.05);
  CHECK_NEAR(st.cht_c, 20.0, 1e-6);
  CHECK_NEAR(st.egt_c, 20.0, 1e-6);
  CHECK_NEAR(st.oil_temp_c, 20.0, 1e-6);
}

static void test_rise_is_load_saturating(void) {
  ThermalConfig cfg = thermal_config_default();
  double amb = 15.0;

  /* rated load -> full rated rise; oil still linear in waste heat */
  ThermalState hi;
  thermal_init(&hi, amb);
  run(&hi, &cfg, 9000.0, 1.0, amb, 6000.0, 0.1); /* >> slowest tau */
  CHECK_NEAR(hi.cht_c, amb + cfg.cht_rise_rated_c, 0.5);
  CHECK_NEAR(hi.egt_c, amb + cfg.egt_rise_rated_c, 0.5);
  CHECK_NEAR(hi.oil_temp_c, amb + cfg.oil_gain_c_per_w * 9000.0, 0.5);

  /* quarter load -> sqrt(0.25) = half the rated rise */
  ThermalState q;
  thermal_init(&q, amb);
  run(&q, &cfg, 9000.0, 0.25, amb, 6000.0, 0.1);
  CHECK_NEAR(q.cht_c, amb + 0.5 * cfg.cht_rise_rated_c, 0.5);
  CHECK_NEAR(q.egt_c, amb + 0.5 * cfg.egt_rise_rated_c, 0.5);

  /* load fraction clamps: >1 gives no more than the rated rise */
  ThermalState over;
  thermal_init(&over, amb);
  run(&over, &cfg, 9000.0, 3.0, amb, 6000.0, 0.1);
  CHECK_NEAR(over.egt_c, amb + cfg.egt_rise_rated_c, 0.5);
}

static void test_response_speed_matches_time_constants(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  double amb = 15.0, q = 9000.0;
  thermal_init(&st, amb);
  run(&st, &cfg, q, 1.0, amb, 10.0, 0.05); /* short window, rated load */

  double cht_frac = (st.cht_c - amb) / cfg.cht_rise_rated_c;
  double egt_frac = (st.egt_c - amb) / cfg.egt_rise_rated_c;
  double oil_frac = (st.oil_temp_c - amb) / (cfg.oil_gain_c_per_w * q);

  CHECK(egt_frac > cht_frac);
  CHECK(cht_frac > oil_frac);
  CHECK(egt_frac > 0.8); /* tau 5 s, ~2 tau elapsed -> 1 - e^-2 ~ 0.86 */
  CHECK(oil_frac < 0.1); /* tau 240 s, barely moved */
}

static void test_cht_rises_monotonically_to_above_ambient(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  double amb = 15.0;
  thermal_init(&st, amb);
  double prev = st.cht_c;
  for (int i = 0; i < 3000; i++) {
    thermal_step(&st, &cfg, 6000.0, 0.7, amb, i * 0.1, 0.1);
    CHECK(st.cht_c >= prev - 1e-9);
    prev = st.cht_c;
  }
  CHECK(st.cht_c > amb);
}

static const TestCase CASES[] = {
    {"thermal.init_puts_everything_at_ambient",
     test_init_puts_everything_at_ambient},
    {"thermal.config_time_constants_are_ordered",
     test_config_time_constants_are_ordered},
    {"thermal.stopped_engine_holds_at_ambient",
     test_stopped_engine_holds_at_ambient},
    {"thermal.rise_is_load_saturating", test_rise_is_load_saturating},
    {"thermal.response_speed_matches_time_constants",
     test_response_speed_matches_time_constants},
    {"thermal.cht_rises_monotonically_to_above_ambient",
     test_cht_rises_monotonically_to_above_ambient},
};

RUN_TESTS(CASES)
