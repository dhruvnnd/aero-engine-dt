#include "test_util.h"

#include "physics/engine_model.h"
#include "physics/propeller.h"

#define RHO_SL 1.225

static PropState at(const PropConfig *cfg, double rpm, double v, double rho) {
  PropState s;
  prop_step(&s, cfg, rpm, v, rho);
  return s;
}

static void test_default_config_is_valid_and_plausible(void) {
  PropConfig c = prop_config_default();
  CHECK(c.diameter_m > 0.0);
  CHECK(c.j_zero_thrust > 0.0);
  CHECK(c.ct_static > 0.0);
  CHECK(c.cq_static > 0.0);
  CHECK(c.cq_unload >= 0.0 && c.cq_unload <= 1.0);
  CHECK(engine_config_default().prop.diameter_m == c.diameter_m);
  EngineConfig e = engine_config_default();
  CHECK(engine_config_validate(&e, NULL) == 0);
}

static void test_stopped_prop_makes_no_load_or_thrust(void) {
  PropConfig c = prop_config_default();
  PropState s = at(&c, 0.0, 0.0, RHO_SL);
  CHECK_NEAR(s.torque_nm, 0.0, 1e-12);
  CHECK_NEAR(s.thrust_n, 0.0, 1e-12);
  s = at(&c, 0.0, 30.0, RHO_SL); /* stopped, but the aircraft is moving */
  CHECK_NEAR(s.torque_nm, 0.0, 1e-12);
  CHECK_NEAR(s.thrust_n, 0.0, 1e-12);
  s = at(&c, -500.0, 0.0, RHO_SL); /* reverse rotation is not modelled */
  CHECK_NEAR(s.torque_nm, 0.0, 1e-12);
}

/* Static: torque and thrust follow rpm^2 (Cq, Ct are constants at J = 0). */
static void test_static_load_scales_with_rpm_squared(void) {
  PropConfig c = prop_config_default();
  PropState a = at(&c, 1000.0, 0.0, RHO_SL);
  PropState b = at(&c, 2000.0, 0.0, RHO_SL);
  CHECK(a.torque_nm > 0.0);
  CHECK(a.thrust_n > 0.0);
  CHECK_NEAR(b.torque_nm / a.torque_nm, 4.0, 1e-9);
  CHECK_NEAR(b.thrust_n / a.thrust_n, 4.0, 1e-9);
}

static void test_static_torque_matches_the_formula(void) {
  PropConfig c = prop_config_default();
  const double n = 2400.0 / 60.0;
  PropState s = at(&c, 2400.0, 0.0, RHO_SL);
  double d5 = c.diameter_m * c.diameter_m * c.diameter_m * c.diameter_m *
              c.diameter_m;
  CHECK_NEAR(s.torque_nm, c.cq_static * RHO_SL * n * n * d5, 1e-9);
  CHECK_NEAR(s.thrust_n, c.ct_static * RHO_SL * n * n * d5 / c.diameter_m,
             1e-9);
  CHECK_NEAR(s.advance_ratio, 0.0, 1e-12);
}

static void test_load_scales_with_air_density(void) {
  PropConfig c = prop_config_default();
  PropState sea = at(&c, 2400.0, 10.0, RHO_SL);
  PropState high = at(&c, 2400.0, 10.0, 0.5 * RHO_SL);
  CHECK_NEAR(high.torque_nm / sea.torque_nm, 0.5, 1e-9);
  CHECK_NEAR(high.thrust_n / sea.thrust_n, 0.5, 1e-9);
}

/* Airspeed unloads the blades: torque and thrust fall as J rises. */
static void test_airspeed_unloads_the_prop(void) {
  PropConfig c = prop_config_default();
  double prev_torque = 1e30;
  double prev_thrust = 1e30;
  for (double v = 0.0; v <= 50.0; v += 10.0) {
    PropState s = at(&c, 2400.0, v, RHO_SL);
    CHECK(s.torque_nm < prev_torque);
    CHECK(s.thrust_n < prev_thrust);
    CHECK(s.torque_nm > 0.0);
    prev_torque = s.torque_nm;
    prev_thrust = s.thrust_n;
  }
}

static void test_advance_ratio_and_zero_thrust_point(void) {
  PropConfig c = prop_config_default();
  const double rpm = 2400.0;
  const double n = rpm / 60.0;
  CHECK_NEAR(prop_advance_ratio(&c, rpm, 20.0), 20.0 / (n * c.diameter_m),
             1e-12);

  /* thrust is zero at J = j_zero_thrust, negative (drag) beyond it */
  const double v0 = c.j_zero_thrust * n * c.diameter_m;
  PropState at_zero = at(&c, rpm, v0, RHO_SL);
  CHECK_NEAR(at_zero.thrust_n, 0.0, 1e-6);
  PropState beyond = at(&c, rpm, 1.2 * v0, RHO_SL);
  CHECK(beyond.thrust_n < 0.0);

  /* a stopped prop reports the capped maximum J, not infinity */
  CHECK(prop_advance_ratio(&c, 0.0, 30.0) < 10.0);
}

static void test_extreme_airspeed_stays_bounded(void) {
  PropConfig c = prop_config_default();
  PropState fast = at(&c, 600.0, 400.0, RHO_SL);
  CHECK(fast.torque_nm >= 0.0); /* never drives the crank */
  CHECK(fast.torque_nm < 1e4);
  CHECK(fast.thrust_n > -1e5);
  PropState huge = at(&c, 2400.0, 1e9, RHO_SL);
  CHECK(huge.torque_nm >= 0.0 && huge.torque_nm < 1e4);

  c.cq_unload = 1.0; /* torque would go negative past J0 -- clamped at 0 */
  PropState windmill = at(&c, 2400.0, 1e3, RHO_SL);
  CHECK(windmill.torque_nm >= 0.0);
}

static void test_zero_cq_means_no_propeller(void) {
  PropConfig c = prop_config_default();
  c.cq_static = 0.0;
  PropState s = at(&c, 3000.0, 0.0, RHO_SL);
  CHECK_NEAR(s.torque_nm, 0.0, 1e-12);
}

static void test_bigger_diameter_loads_harder(void) {
  PropConfig small = prop_config_default();
  PropConfig big = small;
  big.diameter_m = 1.1 * small.diameter_m;
  PropState a = at(&small, 2400.0, 0.0, RHO_SL);
  PropState b = at(&big, 2400.0, 0.0, RHO_SL);
  CHECK_NEAR(b.torque_nm / a.torque_nm, 1.61051, 1e-4); /* 1.1^5 */
  CHECK_NEAR(b.thrust_n / a.thrust_n, 1.4641, 1e-4);    /* 1.1^4 */
}

static void test_derived_prop_figures(void) {
  EngineConfig e = engine_config_default();
  EngineDerived d = engine_config_derived(&e);
  PropState s = at(&e.prop, PROP_REF_RPM, 0.0, RHO_SL);
  CHECK_NEAR(d.prop_static_torque_nm, s.torque_nm, 1e-9);
  CHECK_NEAR(d.prop_static_thrust_n, s.thrust_n, 1e-9);
  CHECK(d.prop_tip_mach > 0.2 && d.prop_tip_mach < 0.8); /* subsonic tips */
}

static const TestCase CASES[] = {
    {"propeller.default_config_is_valid_and_plausible",
     test_default_config_is_valid_and_plausible},
    {"propeller.stopped_prop_makes_no_load_or_thrust",
     test_stopped_prop_makes_no_load_or_thrust},
    {"propeller.static_load_scales_with_rpm_squared",
     test_static_load_scales_with_rpm_squared},
    {"propeller.static_torque_matches_the_formula",
     test_static_torque_matches_the_formula},
    {"propeller.load_scales_with_air_density",
     test_load_scales_with_air_density},
    {"propeller.airspeed_unloads_the_prop", test_airspeed_unloads_the_prop},
    {"propeller.advance_ratio_and_zero_thrust_point",
     test_advance_ratio_and_zero_thrust_point},
    {"propeller.extreme_airspeed_stays_bounded",
     test_extreme_airspeed_stays_bounded},
    {"propeller.zero_cq_means_no_propeller", test_zero_cq_means_no_propeller},
    {"propeller.bigger_diameter_loads_harder",
     test_bigger_diameter_loads_harder},
    {"propeller.derived_prop_figures", test_derived_prop_figures},
};

RUN_TESTS(CASES)
