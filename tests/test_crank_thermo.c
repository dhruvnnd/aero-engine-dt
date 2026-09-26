#include "test_util.h"

#include <math.h>

#include "math/units.h"
#include "physics/crank_thermo.h"

static void test_wrap720(void) {
  CHECK_NEAR(crank_wrap720_deg(0.0), 0.0, 1e-9);
  CHECK_NEAR(crank_wrap720_deg(719.9), 719.9, 1e-9);
  CHECK_NEAR(crank_wrap720_deg(720.0), 0.0, 1e-9);
  CHECK_NEAR(crank_wrap720_deg(730.0), 10.0, 1e-9);
  CHECK_NEAR(crank_wrap720_deg(-10.0), 710.0, 1e-9);
  CHECK_NEAR(crank_wrap720_deg(1440.0 + 5.0), 5.0, 1e-9);
}

/* TDC (theta=0) is the smallest volume (just the clearance volume); BDC
 * (theta=180, mid-power-stroke bottom) is the largest -- Vc + full swept
 * volume. */
static void test_volume_extremes(void) {
  EngineGeometry g = engine_geometry_default();
  double vc = cylinder_clearance_m3(&g, g.compression_ratio);
  double vd = cylinder_displacement_m3(&g);

  CHECK_NEAR(cylinder_volume_m3(0.0, &g, g.compression_ratio), vc, 1e-9);
  CHECK_NEAR(cylinder_volume_m3(180.0, &g, g.compression_ratio), vc + vd,
             1e-7);
  CHECK_NEAR(cylinder_volume_m3(720.0, &g, g.compression_ratio), vc, 1e-9);

  /* Volume is monotonically non-decreasing over the power stroke [0,180]. */
  double prev = cylinder_volume_m3(0.0, &g, g.compression_ratio);
  for (double th = 10.0; th <= 180.0; th += 10.0) {
    double v = cylinder_volume_m3(th, &g, g.compression_ratio);
    CHECK(v >= prev - 1e-12);
    prev = v;
  }
}

/* A weaker effective compression ratio (compression_trim < 1) must enlarge
 * clearance volume -- and hence every instantaneous volume -- since less of
 * the swept volume is "compressed away". */
static void test_lower_compression_ratio_raises_clearance(void) {
  EngineGeometry g = engine_geometry_default();
  double vc_nominal = cylinder_clearance_m3(&g, g.compression_ratio);
  double vc_weak = cylinder_clearance_m3(&g, g.compression_ratio * 0.5);
  CHECK(vc_weak > vc_nominal);
}

/* cylinder_dvolume_ddeg must match a central finite difference of
 * cylinder_volume_m3 -- this is the cross-check that also validates the
 * exact-derivative torque formula shares correct kinematics. */
static void test_dvolume_matches_finite_difference(void) {
  EngineGeometry g = engine_geometry_default();
  double eps = 1e-4;
  for (double th = 5.0; th < 720.0; th += 37.0) {
    double v_plus = cylinder_volume_m3(th + eps, &g, g.compression_ratio);
    double v_minus = cylinder_volume_m3(th - eps, &g, g.compression_ratio);
    double numeric = (v_plus - v_minus) / (2.0 * eps);
    double analytic = cylinder_dvolume_ddeg(th, &g);
    CHECK_NEAR(analytic, numeric, fabs(numeric) * 1e-4 + 1e-12);
  }
}

static void test_valve_closed_gating(void) {
  EngineGeometry g = engine_geometry_default(); /* evo=130, ivc=590 */
  CHECK(cylinder_valve_closed(0.0, &g));         /* deep in closed range */
  CHECK(cylinder_valve_closed(719.0, &g));
  CHECK(cylinder_valve_closed(600.0, &g));
  CHECK(!cylinder_valve_closed(200.0, &g)); /* exhaust stroke */
  CHECK(!cylinder_valve_closed(450.0, &g)); /* intake stroke */
  CHECK(!cylinder_valve_closed(g.evo_deg + 0.1, &g));
  CHECK(cylinder_valve_closed(g.ivc_deg + 0.1, &g));
}

static void test_wiebe_fraction_endpoints_and_monotonic(void) {
  double start = 700.0, dur = 50.0, a = 5.0, m = 2.0;
  CHECK_NEAR(wiebe_burn_fraction(start, start, dur, a, m), 0.0, 1e-9);
  CHECK(wiebe_burn_fraction(start + dur, start, dur, a, m) >= 0.999);
  CHECK_NEAR(wiebe_burn_fraction(start + dur + 100.0, start, dur, a, m), 1.0,
             1e-9);

  double prev = -1.0;
  for (double d = 0.0; d <= dur; d += 2.0) {
    double xb = wiebe_burn_fraction(start + d, start, dur, a, m);
    CHECK(xb >= prev - 1e-12);
    prev = xb;
  }

  /* Wraps correctly through the 720/0 seam. */
  double wrapped = wiebe_burn_fraction(710.0, 700.0, dur, a, m);
  double unwrapped = wiebe_burn_fraction(10.0, 0.0, dur, a, m);
  CHECK_NEAR(wrapped, unwrapped, 1e-9);
}

static void test_wiebe_rate_matches_finite_difference(void) {
  double start = 100.0, dur = 50.0, a = 5.0, m = 2.0;
  double eps = 1e-4;
  for (double d = 2.0; d < dur; d += 5.0) {
    double th = start + d;
    double f_plus = wiebe_burn_fraction(th + eps, start, dur, a, m);
    double f_minus = wiebe_burn_fraction(th - eps, start, dur, a, m);
    double numeric = (f_plus - f_minus) / (2.0 * eps);
    double analytic = wiebe_burn_rate_per_deg(th, start, dur, a, m);
    CHECK_NEAR(analytic, numeric, fabs(numeric) * 1e-3 + 1e-9);
  }
  CHECK_NEAR(wiebe_burn_rate_per_deg(start + dur + 5.0, start, dur, a, m), 0.0,
             1e-12);
}

static void test_spark_advance_increases_with_rpm_decreases_with_map(void) {
  EngineGeometry g = engine_geometry_default();
  double low_rpm = spark_advance_curve(800.0, 30.0, &g);
  double high_rpm = spark_advance_curve(5000.0, 30.0, &g);
  CHECK(high_rpm > low_rpm);

  double low_map = spark_advance_curve(2000.0, 30.0, &g);
  double high_map = spark_advance_curve(2000.0, 90.0, &g);
  CHECK(high_map < low_map);
}

/* Gas torque must vanish at TDC/BDC (dV/dtheta = 0 there) regardless of
 * pressure, and be positive during expansion just after TDC firing (theta
 * just above 0) at a plausible post-combustion pressure. */
static void test_gas_torque_zero_at_dead_centers(void) {
  EngineGeometry g = engine_geometry_default();
  CHECK_NEAR(cylinder_gas_torque_nm(0.0, 3000.0, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_gas_torque_nm(180.0, 3000.0, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_gas_torque_nm(360.0, 200.0, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_gas_torque_nm(540.0, 200.0, &g), 0.0, 1e-6);

  CHECK(cylinder_gas_torque_nm(30.0, 3000.0, &g) > 0.0);
}

/* Inertia torque: zero at every dead center (zero lever arm, regardless of
 * peak acceleration there); integrates to ~0 net work over a full cycle at
 * constant omega (it only reshapes ripple, doesn't add/remove mean torque --
 * see docs/physical_modeling_plan.md Phase 1). */
static void test_inertia_torque_zero_at_dead_centers_and_zero_mean(void) {
  EngineGeometry g = engine_geometry_default();
  double omega = rpm_to_rad_s(2500.0);

  CHECK_NEAR(cylinder_inertia_torque_nm(0.0, omega, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_inertia_torque_nm(180.0, omega, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_inertia_torque_nm(360.0, omega, &g), 0.0, 1e-6);
  CHECK_NEAR(cylinder_inertia_torque_nm(540.0, omega, &g), 0.0, 1e-6);

  double sum = 0.0;
  int n = 720;
  for (int i = 0; i < n; i++) {
    sum += cylinder_inertia_torque_nm((double)i, omega, &g);
  }
  double mean = sum / n;
  CHECK_NEAR(mean, 0.0, 5.0); /* N*m -- small vs. peak inertia torque */
}

static void test_charge_energy_scales_with_map_and_zeroed_by_misfire(void) {
  EngineGeometry g = engine_geometry_default();
  double e_low_map = cylinder_charge_energy_j(30.0, 15.0, &g,
                                              g.compression_ratio, 14.7, 1.0,
                                              0.0);
  double e_high_map = cylinder_charge_energy_j(90.0, 15.0, &g,
                                               g.compression_ratio, 14.7, 1.0,
                                               0.0);
  CHECK(e_high_map > e_low_map);
  CHECK(e_low_map > 0.0);

  double e_misfired = cylinder_charge_energy_j(90.0, 15.0, &g,
                                               g.compression_ratio, 14.7, 1.0,
                                               1.0);
  CHECK_NEAR(e_misfired, 0.0, 1e-9);

  /* Leaner mixture (higher lambda) burns less fuel for the same trapped
   * air, so less energy. */
  double e_lean = cylinder_charge_energy_j(90.0, 15.0, &g, g.compression_ratio,
                                           14.7, 1.5, 0.0);
  CHECK(e_lean < e_high_map);
}

/* In the closed window, pressure should rise sharply through the burn (a
 * compression-then-combustion trace should show a higher dP/dtheta right at
 * ignition than well before it); in the open window, the derivative should
 * just relax pressure toward map_kpa. */
static void test_pressure_derivative_open_vs_closed(void) {
  EngineGeometry g = engine_geometry_default();
  double eff_cr = g.compression_ratio;
  double map_kpa = 80.0;

  const double exhaust_kpa = 101.3;

  /* Open period (intake stroke): pressure below MAP should show a positive
   * derivative pulling it up toward MAP. */
  double dp_open = cylinder_pressure_dtheta(450.0, 40.0, &g, eff_cr, 700.0,
                                            0.0, 1.3, map_kpa, exhaust_kpa);
  CHECK(dp_open > 0.0);

  /* Closed period, well before ignition (pure compression, q_total=0):
   * pressure should be rising (piston moving up, volume shrinking). */
  double dp_compress = cylinder_pressure_dtheta(
      600.0, map_kpa, &g, eff_cr, 700.0, 0.0, 1.3, map_kpa, exhaust_kpa);
  CHECK(dp_compress > 0.0);

  /* Adding heat release at the same point should make the pressure rise
   * faster than compression alone. */
  double dp_with_heat = cylinder_pressure_dtheta(
      705.0, map_kpa * 3.0, &g, eff_cr, 700.0, 1500.0, 1.3, map_kpa,
      exhaust_kpa);
  double dp_without_heat = cylinder_pressure_dtheta(
      705.0, map_kpa * 3.0, &g, eff_cr, 700.0, 0.0, 1.3, map_kpa, exhaust_kpa);
  CHECK(dp_with_heat > dp_without_heat);
}

/* Gas exchange: the exhaust stroke vents toward exhaust pressure, the intake
 * stroke toward manifold pressure -- the two halves of the pumping loop. */
static void test_open_valve_target_splits_exhaust_and_intake(void) {
  CHECK_NEAR(cylinder_open_valve_target_kpa(200.0, 30.0, 101.3), 101.3, 1e-12);
  CHECK_NEAR(cylinder_open_valve_target_kpa(359.0, 30.0, 101.3), 101.3, 1e-12);
  CHECK_NEAR(cylinder_open_valve_target_kpa(361.0, 30.0, 101.3), 30.0, 1e-12);
  CHECK_NEAR(cylinder_open_valve_target_kpa(500.0, 30.0, 101.3), 30.0, 1e-12);
  CHECK_NEAR(cylinder_open_valve_target_kpa(-220.0, 30.0, 101.3), 30.0,
             1e-12); /* wraps: -220 = 500 */

  EngineGeometry g = engine_geometry_default();
  /* the same low pressure is pulled UP on the exhaust stroke and stays put on
   * the intake stroke when it already sits at MAP */
  double dp_exh = cylinder_pressure_dtheta(250.0, 30.0, &g, g.compression_ratio,
                                           700.0, 0.0, 1.3, 30.0, 101.3);
  double dp_int = cylinder_pressure_dtheta(450.0, 30.0, &g, g.compression_ratio,
                                           700.0, 0.0, 1.3, 30.0, 101.3);
  CHECK(dp_exh > 0.0);
  CHECK_NEAR(dp_int, 0.0, 1e-12);
}

/* cylinder_volume_and_deriv() / cylinder_kinematics() are a performance fast
 * path for engine_model.c's hot loop (shares one sin/cos evaluation across
 * volume, its derivative, and the inertia-torque kinematics instead of each
 * recomputing it) -- must agree exactly with the individually-tested
 * functions they duplicate the math of. */
static void test_combined_kinematics_match_individual_functions(void) {
  EngineGeometry g = engine_geometry_default();
  for (double th = 3.0; th < 720.0; th += 41.0) {
    CylinderVolumeDeriv vd =
        cylinder_volume_and_deriv(th, &g, g.compression_ratio);
    CHECK_NEAR(vd.v_m3, cylinder_volume_m3(th, &g, g.compression_ratio), 1e-12);
    CHECK_NEAR(vd.dv_ddeg, cylinder_dvolume_ddeg(th, &g), 1e-15);
    CHECK_NEAR(vd.dv_drad, vd.dv_ddeg * (180.0 / UNITS_PI), 1e-9);

    /* dv_drad should also match what cylinder_gas_torque_nm's internals use:
     * gas torque = pressure_pa * dv_drad, so cross-check via that relation
     * at an arbitrary pressure. */
    double pressure_kpa = 2500.0;
    double torque_from_combined = kpa_to_pa(pressure_kpa) * vd.dv_drad;
    CHECK_NEAR(torque_from_combined,
               cylinder_gas_torque_nm(th, pressure_kpa, &g), 1e-6);

    CylinderKinematics k = cylinder_kinematics(th, &g);
    double omega = rpm_to_rad_s(3000.0);
    double f_inertia = -g.m_recip_kg * omega * omega * k.d2x_drad2;
    double inertia_from_combined = f_inertia * k.dx_drad;
    CHECK_NEAR(inertia_from_combined,
               cylinder_inertia_torque_nm(th, omega, &g), 1e-6);
  }
}

/* cylinder_charge_energy_j_with_vivc() is a performance fast path (caches
 * the IVC-volume calc instead of recomputing it every call) -- must match
 * cylinder_charge_energy_j() exactly when given the same v_ivc it would
 * have computed internally. */
static void test_charge_energy_with_vivc_matches_original(void) {
  EngineGeometry g = engine_geometry_default();
  double v_ivc = cylinder_volume_m3(g.ivc_deg, &g, g.compression_ratio);

  double a = cylinder_charge_energy_j(90.0, 15.0, &g, g.compression_ratio,
                                      14.7, 1.1, 0.0);
  double b = cylinder_charge_energy_j_with_vivc(90.0, 15.0, v_ivc, &g, 14.7,
                                                1.1, 0.0);
  CHECK_NEAR(a, b, 1e-9);
}

static const TestCase CASES[] = {
    {"crank_thermo.wrap720", test_wrap720},
    {"crank_thermo.volume_extremes", test_volume_extremes},
    {"crank_thermo.lower_compression_ratio_raises_clearance",
     test_lower_compression_ratio_raises_clearance},
    {"crank_thermo.dvolume_matches_finite_difference",
     test_dvolume_matches_finite_difference},
    {"crank_thermo.valve_closed_gating", test_valve_closed_gating},
    {"crank_thermo.wiebe_fraction_endpoints_and_monotonic",
     test_wiebe_fraction_endpoints_and_monotonic},
    {"crank_thermo.wiebe_rate_matches_finite_difference",
     test_wiebe_rate_matches_finite_difference},
    {"crank_thermo.spark_advance_increases_with_rpm_decreases_with_map",
     test_spark_advance_increases_with_rpm_decreases_with_map},
    {"crank_thermo.gas_torque_zero_at_dead_centers",
     test_gas_torque_zero_at_dead_centers},
    {"crank_thermo.inertia_torque_zero_at_dead_centers_and_zero_mean",
     test_inertia_torque_zero_at_dead_centers_and_zero_mean},
    {"crank_thermo.charge_energy_scales_with_map_and_zeroed_by_misfire",
     test_charge_energy_scales_with_map_and_zeroed_by_misfire},
    {"crank_thermo.pressure_derivative_open_vs_closed",
     test_pressure_derivative_open_vs_closed},
    {"crank_thermo.open_valve_target_splits_exhaust_and_intake",
     test_open_valve_target_splits_exhaust_and_intake},
    {"crank_thermo.combined_kinematics_match_individual_functions",
     test_combined_kinematics_match_individual_functions},
    {"crank_thermo.charge_energy_with_vivc_matches_original",
     test_charge_energy_with_vivc_matches_original},
};

RUN_TESTS(CASES)
