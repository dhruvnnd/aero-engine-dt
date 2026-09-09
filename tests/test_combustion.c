#include "test_util.h"

#include "physics/combustion.h"

static void test_torque_at_zero_speed_matches_formula(void) {
  /* falloff = 1.0 at omega = 0, so torque = 2.2 * map */
  CHECK_NEAR(combustion_indicated_torque_nm(50.0, 0.0), 110.0, 1e-9);
  CHECK_NEAR(combustion_indicated_torque_nm(0.0, 0.0), 0.0, 1e-9);
}

static void test_torque_is_linear_in_map(void) {
  double omega = 150.0;
  double lo = combustion_indicated_torque_nm(40.0, omega);
  double hi = combustion_indicated_torque_nm(90.0, omega);
  CHECK(hi > lo);
  CHECK_NEAR(hi / lo, 90.0 / 40.0, 1e-9);
}

static void test_torque_falls_off_then_clamps_with_speed(void) {
  double mid = combustion_indicated_torque_nm(100.0, 150.0);
  double fast = combustion_indicated_torque_nm(100.0, 400.0);
  CHECK(fast < mid); /* torque drops as speed rises... */

  double faster = combustion_indicated_torque_nm(100.0, 600.0);
  CHECK_NEAR(fast, faster, 1e-9);            /* ...until the 0.15 floor */
  CHECK_NEAR(fast, 2.2 * 100.0 * 0.15, 1e-9);
}

static void test_power_is_torque_times_speed(void) {
  double map = 70.0, omega = 200.0;
  CHECK_NEAR(combustion_indicated_power_w(map, omega),
             combustion_indicated_torque_nm(map, omega) * omega, 1e-9);
}

static void test_waste_heat_follows_efficiency_split(void) {
  double map = 80.0, omega = 220.0;
  double p = combustion_indicated_power_w(map, omega);
  double q = combustion_waste_heat_w(map, omega);
  /* indicated efficiency 0.30 -> waste = P * (1/0.30 - 1) */
  CHECK_NEAR(q, p * (1.0 / 0.30 - 1.0), 1e-6);
  CHECK(q > 0.0);
  CHECK_NEAR(combustion_waste_heat_w(50.0, 0.0), 0.0, 1e-9); /* no op point */
}

static const TestCase CASES[] = {
    {"combustion.torque_at_zero_speed_matches_formula",
     test_torque_at_zero_speed_matches_formula},
    {"combustion.torque_is_linear_in_map", test_torque_is_linear_in_map},
    {"combustion.torque_falls_off_then_clamps_with_speed",
     test_torque_falls_off_then_clamps_with_speed},
    {"combustion.power_is_torque_times_speed",
     test_power_is_torque_times_speed},
    {"combustion.waste_heat_follows_efficiency_split",
     test_waste_heat_follows_efficiency_split},
};

RUN_TESTS(CASES)
