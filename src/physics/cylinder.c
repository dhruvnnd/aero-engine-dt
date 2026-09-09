#include "physics/cylinder.h"

#include "physics/combustion.h"
#include "physics/integrator.h"

/* Retarded spark and a leaner charge both push exhaust temperature up. */
static double egt_trim_factor(const CylinderConfig *c) {
  const double k_spark_per_deg = 0.004;
  const double k_leak = 0.6;
  double f = 1.0 + k_spark_per_deg * c->spark_offset_deg +
             k_leak * c->intake_leak_frac;
  return f > 0.0 ? f : 0.0;
}

/* cooling_trim < 1 means the head sheds less heat -> hotter. */
static double cht_cool_factor(const CylinderConfig *c) {
  return c->cooling_trim > 0.05 ? c->cooling_trim : 0.05;
}

/* Per-cylinder AFR ratio: extra fuel richens (lambda < 1), an intake leak
 * adds air and leans it (lambda > 1). */
static double cylinder_lambda(const CylinderConfig *c) {
  double flow = c->injector_flow_trim > 0.05 ? c->injector_flow_trim : 0.05;
  return (1.0 / flow) * (1.0 + c->intake_leak_frac);
}

/* Fraction of recent cycles that fail to fire, from the mixture strength. */
static double misfire_fraction(double lambda) {
  if (lambda < 0.55 || lambda > 1.55) {
    return 1.0; /* outside the flammability band */
  }
  if (lambda < 0.65 || lambda > 1.40) {
    return 0.3; /* ragged edge */
  }
  return 0.0;
}

/* Share of nominal indicated work this cylinder still makes (1.0 = full).
 * Less fuel (when lean), lower compression, spark away from MBT, or a lean
 * intake leak each cut it; 1.0 at nominal trims. */
static double torque_trim_factor(const CylinderConfig *c) {
  double inj = c->injector_flow_trim > 0.05 ? c->injector_flow_trim : 0.05;
  double fuel = inj < 1.0 ? inj : 1.0; /* excess fuel past stoich doesn't burn */

  double comp = c->compression_trim > 0.0 ? c->compression_trim : 0.0;

  double spark = 1.0 - 0.0008 * c->spark_offset_deg * c->spark_offset_deg;
  if (spark < 0.0) {
    spark = 0.0;
  }

  double leak = 1.0 - 0.5 * c->intake_leak_frac;
  if (leak < 0.0) {
    leak = 0.0;
  }

  return fuel * comp * spark * leak;
}

double cylinder_torque_nm(const CylinderConfig *config, double map_kpa,
                          double omega_rad_s, int num_cylinders) {
  int n = num_cylinders > 0 ? num_cylinders : 1;
  double share = combustion_indicated_torque_nm(map_kpa, omega_rad_s) / n;
  double firing = 1.0 - misfire_fraction(cylinder_lambda(config));
  return share * torque_trim_factor(config) * firing;
}

double cylinders_total_torque_nm(const CylinderConfig *cyl, int num_cylinders,
                                 double map_kpa, double omega_rad_s) {
  int n = num_cylinders > 0 ? num_cylinders : 1;
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += cylinder_torque_nm(&cyl[i], map_kpa, omega_rad_s, n);
  }
  return sum;
}

CylinderConfig cylinder_config_default(void) {
  CylinderConfig cfg;
  cfg.injector_flow_trim = 1.0;
  cfg.compression_trim = 1.0;
  cfg.spark_offset_deg = 0.0;
  cfg.intake_leak_frac = 0.0;
  cfg.cooling_trim = 1.0;
  return cfg;
}

void cylinder_state_init(CylinderState *state, double ambient_temp_c) {
  state->cht_c = ambient_temp_c;
  state->egt_c = ambient_temp_c;
  state->lambda = 1.0; /* stoichiometric: neutral, and safe against /lambda */
  state->imep_bar = 0.0;
  state->ca50_deg = 0.0;
  state->fuel_pw_ms = 0.0;
  state->misfire_rate = 0.0;
}

enum { CN_CHT = 0, CN_EGT = 1, CN_COUNT };

typedef struct {
  double cht_target_c;
  double egt_target_c;
  double cht_tau_s;
  double egt_tau_s;
} CylDerivParams;

static void cyl_derivative(const double *s, double *ds, double t,
                           void *user_data) {
  (void)t;
  const CylDerivParams *p = (const CylDerivParams *)user_data;
  ds[CN_CHT] = (p->cht_target_c - s[CN_CHT]) / p->cht_tau_s;
  ds[CN_EGT] = (p->egt_target_c - s[CN_EGT]) / p->egt_tau_s;
}

/* Integrates each cylinder's cht_c/egt_c first-order nodes off the shared
 * operating point + its trims
 * */
void cylinder_step(CylinderState *state, const CylinderConfig *config,
                   double map_kpa, double omega_rad_s, double ambient_c,
                   const ThermalConfig *thermal_cfg, int num_cylinders,
                   double t, double dt) {
  double q_w = combustion_waste_heat_w(map_kpa, omega_rad_s);

  CylDerivParams p;
  p.cht_target_c =
      ambient_c + thermal_cfg->cht_gain_c_per_w * q_w / cht_cool_factor(config);
  p.egt_target_c =
      ambient_c + thermal_cfg->egt_gain_c_per_w * q_w * egt_trim_factor(config);
  p.cht_tau_s = thermal_cfg->cht_tau_s;
  p.egt_tau_s = thermal_cfg->egt_tau_s;

  double vec[CN_COUNT] = {state->cht_c, state->egt_c};
  integrator_rk4_step(vec, CN_COUNT, t, dt, cyl_derivative, &p);
  state->cht_c = vec[CN_CHT];
  state->egt_c = vec[CN_EGT];

  /* Algebraic outputs -- not integrated. ca50/fuel_pw are rough placeholders
   * until a fuel path and displacement config land. */
  int n = num_cylinders > 0 ? num_cylinders : 1;
  state->imep_bar =
      0.02 * cylinder_torque_nm(config, map_kpa, omega_rad_s, n);
  state->lambda = cylinder_lambda(config);
  state->misfire_rate = misfire_fraction(state->lambda);
  state->ca50_deg =
      8.0 - config->spark_offset_deg; /* nominal MBT ~8 deg ATDC */
  state->fuel_pw_ms = (1.5 + 0.03 * map_kpa) * config->injector_flow_trim;
}
