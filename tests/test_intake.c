#include "test_util.h"

#include "physics/intake.h"

#define AMB_KPA 101.325
#define AMB_TEMP_C 15.0

static void test_default_config_is_plausible(void) {
  IntakeConfig c = intake_config_default();
  CHECK(c.plenum_vol_m3 > 0.0);
  CHECK(c.throttle_bore_m > 0.0);
  CHECK(c.throttle_leak_frac >= 0.0 && c.throttle_leak_frac < 1.0);
  CHECK(c.throttle_cd > 0.0 && c.throttle_cd <= 1.0);
}

static void test_no_flow_when_pressures_equal_or_reversed(void) {
  IntakeConfig c = intake_config_default();
  CHECK_NEAR(throttle_flow_kg_s(1.0, AMB_KPA, AMB_KPA, AMB_TEMP_C, &c), 0.0,
             1e-12);
  CHECK_NEAR(throttle_flow_kg_s(1.0, AMB_KPA, AMB_KPA + 5.0, AMB_TEMP_C, &c),
             0.0, 1e-12);
}

static void test_flow_grows_with_throttle(void) {
  IntakeConfig c = intake_config_default();
  double closed = throttle_flow_kg_s(0.0, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  double half = throttle_flow_kg_s(0.5, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  double open = throttle_flow_kg_s(1.0, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  CHECK(closed > 0.0); /* leak fraction: an idling engine can still breathe */
  CHECK(half > closed);
  CHECK(open > half);
}

/* Below the critical pressure ratio (~0.528 for air) flow is choked: it no
 * longer depends on how low downstream pressure is. */
static void test_flow_is_choked_below_critical_ratio(void) {
  IntakeConfig c = intake_config_default();
  double a = throttle_flow_kg_s(1.0, AMB_KPA, 40.0, AMB_TEMP_C, &c);
  double b = throttle_flow_kg_s(1.0, AMB_KPA, 20.0, AMB_TEMP_C, &c);
  double d = throttle_flow_kg_s(1.0, AMB_KPA, 1.0, AMB_TEMP_C, &c);
  CHECK_NEAR(a, b, 1e-9);
  CHECK_NEAR(b, d, 1e-9);
}

/* Above the critical ratio, flow should fall as the pressure difference
 * shrinks -- unchoked, subsonic behavior. */
static void test_flow_falls_with_pressure_ratio_when_unchoked(void) {
  IntakeConfig c = intake_config_default();
  double near_amb = throttle_flow_kg_s(1.0, AMB_KPA, 95.0, AMB_TEMP_C, &c);
  double closer = throttle_flow_kg_s(1.0, AMB_KPA, 99.0, AMB_TEMP_C, &c);
  CHECK(near_amb > 0.0);
  CHECK(closer < near_amb);
}

static void test_flow_scales_with_bore_area(void) {
  IntakeConfig small = intake_config_default();
  IntakeConfig big = small;
  big.throttle_bore_m = small.throttle_bore_m * 2.0;
  double f_small = throttle_flow_kg_s(1.0, AMB_KPA, 30.0, AMB_TEMP_C, &small);
  double f_big = throttle_flow_kg_s(1.0, AMB_KPA, 30.0, AMB_TEMP_C, &big);
  CHECK_NEAR(f_big / f_small, 4.0, 1e-6); /* area ~ bore^2 */
}

static void test_out_of_range_throttle_is_clamped(void) {
  IntakeConfig c = intake_config_default();
  double below = throttle_flow_kg_s(-0.5, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  double at_zero = throttle_flow_kg_s(0.0, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  double above = throttle_flow_kg_s(1.5, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  double at_one = throttle_flow_kg_s(1.0, AMB_KPA, 30.0, AMB_TEMP_C, &c);
  CHECK_NEAR(below, at_zero, 1e-12);
  CHECK_NEAR(above, at_one, 1e-12);
}

static const TestCase CASES[] = {
    {"intake.default_config_is_plausible", test_default_config_is_plausible},
    {"intake.no_flow_when_pressures_equal_or_reversed",
     test_no_flow_when_pressures_equal_or_reversed},
    {"intake.flow_grows_with_throttle", test_flow_grows_with_throttle},
    {"intake.flow_is_choked_below_critical_ratio",
     test_flow_is_choked_below_critical_ratio},
    {"intake.flow_falls_with_pressure_ratio_when_unchoked",
     test_flow_falls_with_pressure_ratio_when_unchoked},
    {"intake.flow_scales_with_bore_area", test_flow_scales_with_bore_area},
    {"intake.out_of_range_throttle_is_clamped",
     test_out_of_range_throttle_is_clamped},
};

RUN_TESTS(CASES)
