#include "test_util.h"

#include "physics/environment.h"

/* --- ISA sanity (already exercised elsewhere, kept minimal here) ---------- */

static void test_isa_sea_level(void) {
  AtmosphereState atm = environment_isa(0.0);
  CHECK_NEAR(atm.temperature_k, 288.15, 1e-6);
  CHECK_NEAR(atm.pressure_kpa, 101.325, 1e-6);
  CHECK_NEAR(atm.density_kg_m3, 1.225, 0.005);
}

/* --- density altitude ---------------------------------------------------- */

static void test_density_altitude_sea_level_is_zero(void) {
  AtmosphereState atm = environment_isa(0.0);
  CHECK_NEAR(environment_density_altitude_m(atm.density_kg_m3), 0.0, 1.0);
}

/* Feeding ISA density back through the inversion recovers the altitude. */
static void test_density_altitude_round_trips_isa(void) {
  double heights[] = {500.0, 2000.0, 5000.0, 9000.0};
  for (int i = 0; i < 4; i++) {
    AtmosphereState atm = environment_isa(heights[i]);
    double da = environment_density_altitude_m(atm.density_kg_m3);
    CHECK_NEAR(da, heights[i], 1.0);
  }
}

static void test_density_altitude_monotonic_in_thinning_air(void) {
  AtmosphereState lo = environment_isa(1000.0);
  AtmosphereState hi = environment_isa(6000.0);
  CHECK(environment_density_altitude_m(hi.density_kg_m3) >
        environment_density_altitude_m(lo.density_kg_m3));
}

/* --- EnvState assembly ------------------------------------------------------ */

static void test_state_sea_level_standard_day(void) {
  EnvState env;
  environment_state(&env, 0.0, 0.0, 50.0);
  CHECK_NEAR(env.altitude_m, 0.0, 1e-9);
  CHECK_NEAR(env.airspeed_ms, 50.0, 1e-9);
  CHECK_NEAR(env.oat_c, 15.0, 1e-6);
  CHECK_NEAR(env.ambient_kpa, 101.325, 1e-6);
  CHECK_NEAR(env.density_kg_m3, 1.225, 0.005);
  CHECK_NEAR(env.density_alt_m, 0.0, 5.0);
  /* q = 0.5 * 1.225 * 50^2 = 1531 Pa */
  CHECK_NEAR(env.dynamic_press_pa, 0.5 * 1.225 * 2500.0, 5.0);
}

/* A hot day at the same altitude is genuinely thinner air: lower density,
 * higher density altitude, OAT above ISA. */
static void test_state_hot_day_is_thinner_air(void) {
  EnvState isa;
  environment_state(&isa, 1500.0, 0.0, 40.0);
  EnvState hot;
  environment_state(&hot, 1500.0, 25.0, 40.0);

  CHECK_NEAR(hot.oat_c, isa.oat_c + 25.0, 1e-6);
  CHECK(hot.density_kg_m3 < isa.density_kg_m3);
  CHECK(hot.density_alt_m > isa.density_alt_m);
  CHECK_NEAR(hot.ambient_kpa, isa.ambient_kpa, 1e-9); /* pressure altitude same */
}

static void test_state_zero_airspeed_zero_q(void) {
  EnvState env;
  environment_state(&env, 3000.0, 0.0, 0.0);
  CHECK_NEAR(env.dynamic_press_pa, 0.0, 1e-9);
}

/* --- cooling-air index --------------------------------------------------- */

/* The calibration point: sea-level density, reference cruise airspeed and
 * reference crank speed give exactly 1.0. This is what sync.c feeds today. */
static void test_cool_index_reference_is_unity(void) {
  CHECK_NEAR(environment_cool_index(1.225, 50.0, 1900.0), 1.0, 1e-9);
}

static void test_cool_index_ground_idle_below_unity(void) {
  /* Parked, prop turning at idle: only the propwash term. */
  double idx = environment_cool_index(1.225, 0.0, 700.0);
  CHECK(idx > 0.05);
  CHECK(idx < 1.0);
}

static void test_cool_index_faster_is_more_cooling(void) {
  double slow = environment_cool_index(1.225, 30.0, 1900.0);
  double fast = environment_cool_index(1.225, 80.0, 1900.0);
  CHECK(fast > slow);
}

/* Same true airspeed and rpm, thinner air -> less mass flow -> less cooling. */
static void test_cool_index_thinner_air_is_less_cooling(void) {
  double dense = environment_cool_index(1.225, 50.0, 1900.0);
  double thin = environment_cool_index(0.90, 50.0, 1900.0);
  CHECK(thin < dense);
}

static void test_cool_index_clamps(void) {
  CHECK_NEAR(environment_cool_index(0.0, 0.0, 0.0), 0.05, 1e-9);
  CHECK_NEAR(environment_cool_index(1.4, 400.0, 6000.0), 3.0, 1e-9);
}

static void test_cool_index_negative_inputs_are_floored_not_signed(void) {
  /* Negative airspeed/rpm are treated as zero, not as a subtraction. */
  double a = environment_cool_index(1.225, -20.0, -100.0);
  double b = environment_cool_index(1.225, 0.0, 0.0);
  CHECK_NEAR(a, b, 1e-12);
}

static const TestCase CASES[] = {
    {"environment.isa_sea_level", test_isa_sea_level},
    {"environment.density_altitude_sea_level_is_zero",
     test_density_altitude_sea_level_is_zero},
    {"environment.density_altitude_round_trips_isa",
     test_density_altitude_round_trips_isa},
    {"environment.density_altitude_monotonic_in_thinning_air",
     test_density_altitude_monotonic_in_thinning_air},
    {"environment.state_sea_level_standard_day",
     test_state_sea_level_standard_day},
    {"environment.state_hot_day_is_thinner_air",
     test_state_hot_day_is_thinner_air},
    {"environment.state_zero_airspeed_zero_q", test_state_zero_airspeed_zero_q},
    {"environment.cool_index_reference_is_unity",
     test_cool_index_reference_is_unity},
    {"environment.cool_index_ground_idle_below_unity",
     test_cool_index_ground_idle_below_unity},
    {"environment.cool_index_faster_is_more_cooling",
     test_cool_index_faster_is_more_cooling},
    {"environment.cool_index_thinner_air_is_less_cooling",
     test_cool_index_thinner_air_is_less_cooling},
    {"environment.cool_index_clamps", test_cool_index_clamps},
    {"environment.cool_index_negative_inputs_are_floored_not_signed",
     test_cool_index_negative_inputs_are_floored_not_signed},
};

RUN_TESTS(CASES)
