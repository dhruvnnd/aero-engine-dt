#include "physics/thermal.h"

#include <math.h>

#include "physics/integrator.h"

typedef struct {
  double target_c;
  double tau_s;
} OilDerivParams;

double thermal_cool_divisor(double cool_index) {
  double c = cool_index;
  if (c < 0.35) {
    c = 0.35;
  }
  if (c > 4.0) {
    c = 4.0;
  }
  return sqrt(c);
}

static void oil_derivative(const double *state, double *dstate, double t,
                           void *user_data) {
  (void)t;
  const OilDerivParams *p = (const OilDerivParams *)user_data;
  dstate[0] = (p->target_c - state[0]) / p->tau_s;
}

ThermalConfig thermal_config_default(void) {
  ThermalConfig cfg;
  cfg.cht_tau_s = 90.0;  /* metal head: moderate thermal mass */
  cfg.egt_tau_s = 5.0;   /* exhaust gas: responds almost immediately */
  cfg.oil_tau_s = 240.0; /* oil sump: large thermal mass, slowest to move */
  cfg.head_heat_share = 0.25;
  cfg.head_base_kw = 1.6;
  cfg.friction_head_share = 0.4;
  cfg.cht_k_per_kw = 28.0;
  cfg.egt_port_factor = 1.19;
  cfg.egt_port_loss_w = 150.0;
  cfg.oil_base_kw = 5.0;
  cfg.oil_heat_share = 0.10;
  cfg.oil_k_per_kw = 3.3;
  return cfg;
}

void thermal_init(ThermalState *state, double ambient_temp_c) {
  state->cht_c = ambient_temp_c;
  state->egt_c = ambient_temp_c;
  state->oil_temp_c = ambient_temp_c;
}

void thermal_oil_step(ThermalState *state, const ThermalConfig *config,
                      double friction_w, double combustion_heat_w,
                      double cool_index, double ambient_temp_c, double t,
                      double dt) {
  const double oil_kw =
      (combustion_heat_w > 1.0 ? config->oil_base_kw : 0.0) +
      ((1.0 - config->friction_head_share) * friction_w +
       config->oil_heat_share * combustion_heat_w) /
          1000.0;
  OilDerivParams p;
  p.target_c = ambient_temp_c +
               oil_kw * config->oil_k_per_kw / thermal_cool_divisor(cool_index);
  p.tau_s = config->oil_tau_s;

  double vec[1] = {state->oil_temp_c};
  integrator_rk4_step(vec, 1, t, dt, oil_derivative, &p);
  state->oil_temp_c = vec[0];
}
