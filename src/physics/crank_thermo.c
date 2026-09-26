#include "physics/crank_thermo.h"

#include <math.h>

#include "math/units.h"

double crank_wrap720_deg(double theta_deg) {
  double w = fmod(theta_deg, 720.0);
  if (w < 0.0) {
    w += 720.0;
  }
  return w;
}

static double slider_crank_u(double theta_rad, double r, double l) {
  double s = sin(theta_rad);
  double u = l * l - r * r * s * s;
  return u < 1e-12 ? 1e-12 : u;
}

static double slider_crank_disp_from_tdc(double theta_rad, double r, double l) {
  double c = cos(theta_rad);
  return r + l - r * c - sqrt(slider_crank_u(theta_rad, r, l));
}

static double slider_crank_dx_drad(double theta_rad, double r, double l) {
  double s = sin(theta_rad);
  double c = cos(theta_rad);
  double u = slider_crank_u(theta_rad, r, l);
  return r * s + (r * r * s * c) / sqrt(u);
}

static double slider_crank_d2x_drad2(double theta_rad, double r, double l) {
  double s = sin(theta_rad);
  double c = cos(theta_rad);
  double u = slider_crank_u(theta_rad, r, l);
  double su = sqrt(u);
  return r * c + r * r * (c * c - s * s) / su +
         (r * r * r * r) * s * s * c * c / (u * su);
}

EngineGeometry engine_geometry_default(void) {
  EngineGeometry g;
  g.bore_m = 0.084;
  g.stroke_m = 0.090;
  g.conrod_len_m = 0.150;
  g.compression_ratio = 9.5;
  g.evo_deg = 130.0;
  g.ivc_deg = 590.0;
  g.m_recip_kg = 0.45;
  g.wiebe_a = 5.0;
  g.wiebe_m = 2.0;
  g.delta_theta_burn_deg = 50.0;
  g.spark_base_btdc_deg = 10.0;
  g.spark_rpm_gain_deg_per_1000rpm = 6.0;
  g.spark_map_retard_deg_per_kpa = 0.15;
  g.combustion_efficiency = 0.30;
  return g;
}

double cylinder_displacement_m3(const EngineGeometry *geom) {
  return (UNITS_PI / 4.0) * geom->bore_m * geom->bore_m * geom->stroke_m;
}

double cylinder_clearance_m3(const EngineGeometry *geom,
                             double effective_compression_ratio) {
  double cr =
      effective_compression_ratio > 1.01 ? effective_compression_ratio : 1.01;
  return cylinder_displacement_m3(geom) / (cr - 1.0);
}

double cylinder_volume_m3(double theta_deg, const EngineGeometry *geom,
                          double effective_compression_ratio) {
  double theta_rad = crank_wrap720_deg(theta_deg) * UNITS_PI / 180.0;
  double r = geom->stroke_m / 2.0;
  double l = geom->conrod_len_m;
  double disp = slider_crank_disp_from_tdc(theta_rad, r, l);
  double area = (UNITS_PI / 4.0) * geom->bore_m * geom->bore_m;
  return cylinder_clearance_m3(geom, effective_compression_ratio) + area * disp;
}

static double cylinder_dvolume_drad(double theta_deg,
                                    const EngineGeometry *geom) {
  double theta_rad = crank_wrap720_deg(theta_deg) * UNITS_PI / 180.0;
  double r = geom->stroke_m / 2.0;
  double l = geom->conrod_len_m;
  double area = (UNITS_PI / 4.0) * geom->bore_m * geom->bore_m;
  return area * slider_crank_dx_drad(theta_rad, r, l);
}

double cylinder_dvolume_ddeg(double theta_deg, const EngineGeometry *geom) {
  return cylinder_dvolume_drad(theta_deg, geom) * (UNITS_PI / 180.0);
}

CylinderVolumeDeriv
cylinder_volume_and_deriv(double theta_deg, const EngineGeometry *geom,
                          double effective_compression_ratio) {
  double theta_rad = crank_wrap720_deg(theta_deg) * UNITS_PI / 180.0;
  double r = geom->stroke_m / 2.0;
  double l = geom->conrod_len_m;
  double area = (UNITS_PI / 4.0) * geom->bore_m * geom->bore_m;

  double s = sin(theta_rad);
  double c = cos(theta_rad);
  double u = l * l - r * r * s * s;
  if (u < 1e-12) {
    u = 1e-12;
  }
  double su = sqrt(u);

  double disp = r + l - r * c - su;
  double dx_drad = r * s + (r * r * s * c) / su;

  CylinderVolumeDeriv out;
  out.v_m3 =
      cylinder_clearance_m3(geom, effective_compression_ratio) + area * disp;
  out.dv_drad = area * dx_drad;
  out.dv_ddeg = out.dv_drad * (UNITS_PI / 180.0);
  return out;
}

CylinderKinematics cylinder_kinematics(double theta_deg,
                                       const EngineGeometry *geom) {
  double theta_rad = crank_wrap720_deg(theta_deg) * UNITS_PI / 180.0;
  double r = geom->stroke_m / 2.0;
  double l = geom->conrod_len_m;

  double s = sin(theta_rad);
  double c = cos(theta_rad);
  double u = l * l - r * r * s * s;
  if (u < 1e-12) {
    u = 1e-12;
  }
  double su = sqrt(u);

  CylinderKinematics out;
  out.dx_drad = r * s + (r * r * s * c) / su;
  out.d2x_drad2 = r * c + r * r * (c * c - s * s) / su +
                  (r * r * r * r) * s * s * c * c / (u * su);
  return out;
}

int cylinder_valve_closed(double theta_deg, const EngineGeometry *geom) {
  double t = crank_wrap720_deg(theta_deg);
  if (geom->evo_deg <= geom->ivc_deg) {
    return (t < geom->evo_deg) || (t >= geom->ivc_deg);
  }
  return 1;
}

double wiebe_burn_fraction(double theta_deg, double theta_start_deg,
                           double delta_theta_burn_deg, double a, double m) {
  double d = fmod(crank_wrap720_deg(theta_deg) -
                      crank_wrap720_deg(theta_start_deg) + 720.0,
                  720.0);
  if (delta_theta_burn_deg <= 0.0) {
    return d <= 0.0 ? 0.0 : 1.0;
  }
  if (d >= delta_theta_burn_deg) {
    return 1.0;
  }
  double x = d / delta_theta_burn_deg;
  return 1.0 - exp(-a * pow(x, m + 1.0));
}

double wiebe_burn_rate_per_deg(double theta_deg, double theta_start_deg,
                               double delta_theta_burn_deg, double a,
                               double m) {
  double d = fmod(crank_wrap720_deg(theta_deg) -
                      crank_wrap720_deg(theta_start_deg) + 720.0,
                  720.0);
  if (delta_theta_burn_deg <= 0.0 || d >= delta_theta_burn_deg) {
    return 0.0;
  }
  double x = d / delta_theta_burn_deg;
  return (a * (m + 1.0) / delta_theta_burn_deg) * pow(x, m) *
         exp(-a * pow(x, m + 1.0));
}

double spark_advance_curve(double rpm, double map_kpa,
                           const EngineGeometry *geom) {
  const double min_btdc = 5.0;
  const double max_btdc = 35.0;

  double adv = geom->spark_base_btdc_deg +
               geom->spark_rpm_gain_deg_per_1000rpm * (rpm / 1000.0) -
               geom->spark_map_retard_deg_per_kpa * (map_kpa - 30.0);
  if (adv < min_btdc) {
    adv = min_btdc;
  }
  if (adv > max_btdc) {
    adv = max_btdc;
  }
  return adv;
}

double cylinder_gas_torque_nm(double theta_deg, double pressure_kpa,
                              const EngineGeometry *geom) {
  return kpa_to_pa(pressure_kpa) * cylinder_dvolume_drad(theta_deg, geom);
}

double cylinder_inertia_torque_nm(double theta_deg, double omega_rad_s,
                                  const EngineGeometry *geom) {
  double theta_rad = crank_wrap720_deg(theta_deg) * UNITS_PI / 180.0;
  double r = geom->stroke_m / 2.0;
  double l = geom->conrod_len_m;
  double dx_drad = slider_crank_dx_drad(theta_rad, r, l);
  double d2x_drad2 = slider_crank_d2x_drad2(theta_rad, r, l);
  double accel = omega_rad_s * omega_rad_s * d2x_drad2;
  double f_inertia = -geom->m_recip_kg * accel;
  return f_inertia * dx_drad;
}

/* The exhaust stroke ends at the TDC halfway round the cycle; the intake
 * stroke runs from there to IVC. */
#define EXHAUST_END_DEG 360.0

double cylinder_open_valve_target_kpa(double theta_deg, double map_kpa,
                                      double exhaust_kpa) {
  return crank_wrap720_deg(theta_deg) < EXHAUST_END_DEG ? exhaust_kpa : map_kpa;
}

double cylinder_pressure_dtheta(double theta_deg, double pressure_kpa,
                                const EngineGeometry *geom,
                                double effective_compression_ratio,
                                double theta_start_deg, double q_total_j,
                                double n, double map_kpa, double exhaust_kpa) {
  if (!cylinder_valve_closed(theta_deg, geom)) {
    const double relax_deg = 5.0;
    return (cylinder_open_valve_target_kpa(theta_deg, map_kpa, exhaust_kpa) -
            pressure_kpa) /
           relax_deg;
  }

  double v = cylinder_volume_m3(theta_deg, geom, effective_compression_ratio);
  double dv_ddeg = cylinder_dvolume_ddeg(theta_deg, geom);
  double dq_ddeg =
      q_total_j * wiebe_burn_rate_per_deg(theta_deg, theta_start_deg,
                                          geom->delta_theta_burn_deg,
                                          geom->wiebe_a, geom->wiebe_m);

  double p_pa = kpa_to_pa(pressure_kpa);
  double dp_ddeg_pa = -n * (p_pa / v) * dv_ddeg + (n - 1.0) / v * dq_ddeg;
  return pa_to_kpa(dp_ddeg_pa);
}

double cylinder_charge_energy_j_with_vivc(double map_kpa, double intake_temp_c,
                                          double v_ivc_m3,
                                          const EngineGeometry *geom,
                                          double afr_stoich, double lambda,
                                          double misfire_frac) {
  const double r_air_j_per_kgk = 287.05;
  const double fuel_lhv_j_per_kg = 44.0e6; /* gasoline, placeholder */

  double p_pa = kpa_to_pa(map_kpa);
  double t_k = celsius_to_kelvin(intake_temp_c);
  double m_air_kg = (p_pa * v_ivc_m3) / (r_air_j_per_kgk * t_k);

  double lam = lambda > 0.1 ? lambda : 0.1;
  double m_fuel_kg = m_air_kg / (afr_stoich * lam);

  double firing = 1.0 - misfire_frac;
  if (firing < 0.0) {
    firing = 0.0;
  }
  return m_fuel_kg * fuel_lhv_j_per_kg * firing * geom->combustion_efficiency;
}

double cylinder_charge_energy_j(double map_kpa, double intake_temp_c,
                                const EngineGeometry *geom,
                                double effective_compression_ratio,
                                double afr_stoich, double lambda,
                                double misfire_frac) {
  double v_ivc =
      cylinder_volume_m3(geom->ivc_deg, geom, effective_compression_ratio);
  return cylinder_charge_energy_j_with_vivc(map_kpa, intake_temp_c, v_ivc, geom,
                                            afr_stoich, lambda, misfire_frac);
}
