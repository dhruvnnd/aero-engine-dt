#ifndef PHYSICS_CRANK_THERMO_H
#define PHYSICS_CRANK_THERMO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double bore_m;
  double stroke_m;
  double conrod_len_m;
  double compression_ratio;    /* Vd/Vc + 1 */
  double ivc_deg;              /* intake valve close */
  double evo_deg;              /* exhaust valve open */
  double m_recip_kg;           /* reciprocating mass per cylinder */
  double wiebe_a;              /* Wiebe efficiency parameter, typically ~5 */
  double wiebe_m;              /* Wiebe shape parameter (~2) */
  double delta_theta_burn_deg; /* total burn duration, crank deg, ~40-60 */
  double spark_base_btdc_deg;  /* spark_advance_curve() base advance */
  double spark_rpm_gain_deg_per_1000rpm; /* advance added per 1000 RPM */
  double spark_map_retard_deg_per_kpa;   /* advance removed per kPa of MAP */
  double combustion_efficiency; /* fraction of the fuel's chemical energy that
                                   manifests as effective in-cylinder heat*/
} EngineGeometry;

EngineGeometry engine_geometry_default(void);

/* Wraps an arbitrary crank angle into [0, 720). */
double crank_wrap720_deg(double theta_deg);

/* Displaced (swept) volume of one cylinder, m^3, from bore/stroke. */
double cylinder_displacement_m3(const EngineGeometry *geom);

/* Clearance volume at TDC, m^3, from displacement and compression ratio */
double cylinder_clearance_m3(const EngineGeometry *geom,
                             double effective_compression_ratio);

/* Instantaneous cylinder volume at crank angle theta_deg, m^3 */
double cylinder_volume_m3(double theta_deg, const EngineGeometry *geom,
                          double effective_compression_ratio);

/* dV/d(theta_deg) at theta_deg, m^3/deg */
double cylinder_dvolume_ddeg(double theta_deg, const EngineGeometry *geom);

/* True while the valves are shut; false during the exhaust/intake
 * strokes, when the cylinder is open */
int cylinder_valve_closed(double theta_deg, const EngineGeometry *geom);

/* Wiebe combustion heat-release fraction, 0..1 */
double wiebe_burn_fraction(double theta_deg, double theta_start_deg,
                           double delta_theta_burn_deg, double a, double m);

/* d(wiebe_burn_fraction)/d(theta_deg) */
double wiebe_burn_rate_per_deg(double theta_deg, double theta_start_deg,
                               double delta_theta_burn_deg, double a, double m);

/* Base spark advance, deg BTDC, from RPM and MAP (advances with RPM,
 * retards with MAP/load) */
double spark_advance_curve(double rpm, double map_kpa,
                           const EngineGeometry *geom);

/* Gas-pressure torque at crank angle theta_deg given the cylinder's current
 * pressure, N*m */
double cylinder_gas_torque_nm(double theta_deg, double pressure_kpa,
                              const EngineGeometry *geom);

/* Reciprocating-mass inertia torque at crank angle theta_deg, N*m, from the
 * exact slider-crank piston acceleration */
double cylinder_inertia_torque_nm(double theta_deg, double omega_rad_s,
                                  const EngineGeometry *geom);

/* In-cylinder pressure derivative dP/d(theta_deg), kPa/deg */
double cylinder_pressure_dtheta(double theta_deg, double pressure_kpa,
                                const EngineGeometry *geom,
                                double effective_compression_ratio,
                                double theta_start_deg, double q_total_j,
                                double n, double map_kpa);

/* this exists purely to cut redundant sin/cos/sqrt calls in the sub-stepped
 * integration loop */
typedef struct {
  double v_m3;    /* cylinder_volume_m3(theta_deg, geom, eff_cr) */
  double dv_drad; /* dV/d(theta_radians), m^3/rad */
  double dv_ddeg; /* cylinder_dvolume_ddeg(theta_deg, geom) */
} CylinderVolumeDeriv;
CylinderVolumeDeriv
cylinder_volume_and_deriv(double theta_deg, const EngineGeometry *geom,
                          double effective_compression_ratio);
typedef struct {
  double dx_drad;
  double d2x_drad2;
} CylinderKinematics;
CylinderKinematics cylinder_kinematics(double theta_deg,
                                       const EngineGeometry *geom);

/* Total effective heat release for one cylinder's charge this cycle */
double cylinder_charge_energy_j(double map_kpa, double intake_temp_c,
                                const EngineGeometry *geom,
                                double effective_compression_ratio,
                                double afr_stoich, double lambda,
                                double misfire_frac);

double cylinder_charge_energy_j_with_vivc(double map_kpa, double intake_temp_c,
                                          double v_ivc_m3,
                                          const EngineGeometry *geom,
                                          double afr_stoich, double lambda,
                                          double misfire_frac);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_CRANK_THERMO_H */
