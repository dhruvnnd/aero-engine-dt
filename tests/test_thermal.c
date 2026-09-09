#include "test_util.h"

#include "physics/thermal.h"

static void run(ThermalState *st, const ThermalConfig *cfg, double q, double amb,
                double duration_s, double dt) {
  int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    thermal_step(st, cfg, q, amb, i * dt, dt);
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

static void test_no_waste_heat_holds_at_ambient(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  thermal_init(&st, 20.0);
  run(&st, &cfg, 0.0, 20.0, 300.0, 0.05);
  CHECK_NEAR(st.cht_c, 20.0, 1e-6);
  CHECK_NEAR(st.egt_c, 20.0, 1e-6);
  CHECK_NEAR(st.oil_temp_c, 20.0, 1e-6);
}

static void test_converges_to_gain_scaled_targets(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  double amb = 15.0, q = 9000.0;
  thermal_init(&st, amb);
  run(&st, &cfg, q, amb, 6000.0, 0.1); /* >> slowest tau (240 s) */
  CHECK_NEAR(st.cht_c, amb + 0.008 * q, 0.5);
  CHECK_NEAR(st.egt_c, amb + 0.035 * q, 0.5);
  CHECK_NEAR(st.oil_temp_c, amb + 0.004 * q, 0.5);
}

static void test_response_speed_matches_time_constants(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  double amb = 15.0, q = 9000.0;
  thermal_init(&st, amb);
  run(&st, &cfg, q, amb, 10.0, 0.05); /* short window */

  double cht_frac = (st.cht_c - amb) / (0.008 * q);
  double egt_frac = (st.egt_c - amb) / (0.035 * q);
  double oil_frac = (st.oil_temp_c - amb) / (0.004 * q);

  CHECK(egt_frac > cht_frac);
  CHECK(cht_frac > oil_frac);
  CHECK(egt_frac > 0.8);  /* tau 5 s, ~2 tau elapsed -> 1 - e^-2 ~ 0.86 */
  CHECK(oil_frac < 0.1);  /* tau 240 s, barely moved */
}

static void test_cht_rises_monotonically_to_above_ambient(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  double amb = 15.0, q = 6000.0;
  thermal_init(&st, amb);
  double prev = st.cht_c;
  for (int i = 0; i < 3000; i++) {
    thermal_step(&st, &cfg, q, amb, i * 0.1, 0.1);
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
    {"thermal.no_waste_heat_holds_at_ambient",
     test_no_waste_heat_holds_at_ambient},
    {"thermal.converges_to_gain_scaled_targets",
     test_converges_to_gain_scaled_targets},
    {"thermal.response_speed_matches_time_constants",
     test_response_speed_matches_time_constants},
    {"thermal.cht_rises_monotonically_to_above_ambient",
     test_cht_rises_monotonically_to_above_ambient},
};

RUN_TESTS(CASES)
