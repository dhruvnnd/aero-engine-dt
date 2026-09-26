#include "physics/cylinder.h"

#include "physics/integrator.h"

/* cooling_trim < 1 means the head sheds less heat -> hotter. */
static double cht_cool_factor(const CylinderConfig *c) {
  return c->cooling_trim > 0.05 ? c->cooling_trim : 0.05;
}

/* Per-cylinder AFR ratio: extra fuel richens (lambda < 1), an intake leak
 * or a lean injector leans it (lambda > 1). */
double cylinder_lambda(const CylinderConfig *c) {
  double flow = c->injector_flow_trim > 0.05 ? c->injector_flow_trim : 0.05;
  return (1.0 / flow) * (1.0 + c->intake_leak_frac);
}

/* Fraction of recent cycles that fail to fire, from the mixture strength. */
double misfire_fraction(double lambda) {
  if (lambda < 0.55 || lambda > 1.55) {
    return 1.0; /* outside the flammability band */
  }
  if (lambda < 0.65 || lambda > 1.40) {
    return 0.3; /* ragged edge */
  }
  return 0.0;
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
  state->cyl_pressure_kpa = 101.325;
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

void cylinder_step(CylinderState *state, const CylinderConfig *config,
                   const CylinderThermalInput *in, double map_kpa,
                   double ambient_c, const ThermalConfig *thermal_cfg,
                   double cool_index, double t, double dt) {
  const double gas_cp_j_per_kgk = 1100.0;
  const double head_heat_kw = (in->heat_w > 1.0 ? thermal_cfg->head_base_kw : 0.0) +
                              (thermal_cfg->head_heat_share * in->heat_w +
                               thermal_cfg->friction_head_share * in->friction_w) /
                              1000.0;
  const double blowdown_rise_c = in->blowdown_c - ambient_c;

  CylDerivParams p;
  p.cht_target_c = ambient_c + head_heat_kw * thermal_cfg->cht_k_per_kw /
                                   cht_cool_factor(config) /
                                   thermal_cool_divisor(cool_index);
  double egt_rise_c = thermal_cfg->egt_port_factor *
                      (blowdown_rise_c > 0.0 ? blowdown_rise_c : 0.0);
  if (in->gas_flow_kg_s > 1e-6) {
    egt_rise_c -=
        thermal_cfg->egt_port_loss_w / (in->gas_flow_kg_s * gas_cp_j_per_kgk);
  }
  p.egt_target_c = ambient_c + (egt_rise_c > 0.0 ? egt_rise_c : 0.0);
  p.cht_tau_s = thermal_cfg->cht_tau_s;
  p.egt_tau_s = thermal_cfg->egt_tau_s;

  double vec[CN_COUNT] = {state->cht_c, state->egt_c};
  integrator_rk4_step(vec, CN_COUNT, t, dt, cyl_derivative, &p);
  state->cht_c = vec[CN_CHT];
  state->egt_c = vec[CN_EGT];

  /* Algebraic outputs -- not integrated. ca50/fuel_pw are rough placeholders
   * until a fuel path lands. */
  state->imep_bar = in->imep_kpa / 100.0;
  state->lambda = cylinder_lambda(config);
  state->misfire_rate = misfire_fraction(state->lambda);
  state->ca50_deg =
      8.0 - config->spark_offset_deg; /* nominal MBT ~8 deg ATDC */
  state->fuel_pw_ms = (1.5 + 0.03 * map_kpa) * config->injector_flow_trim;
}
