#include "physics/thermal.h"
#include "physics/integrator.h"

enum {
  THERMAL_STATE_CHT = 0,
  THERMAL_STATE_EGT = 1,
  THERMAL_STATE_OIL = 2,
  THERMAL_STATE_COUNT
};

typedef struct {
  double waste_heat_w;
  double ambient_temp_c;
  double cht_tau_s;
  double egt_tau_s;
  double oil_tau_s;
  double cht_gain_c_per_w;
  double egt_gain_c_per_w;
  double oil_gain_c_per_w;
} ThermalDerivParams;

/* First-order target: ambient plus a linear rise with waste heat. */
static double node_target_c(double gain_c_per_w, double waste_heat_w,
                            double ambient_temp_c) {
  return ambient_temp_c + gain_c_per_w * waste_heat_w;
}

static void thermal_derivative(const double *state, double *dstate, double t,
                               void *user_data) {
  (void)t;
  const ThermalDerivParams *p = (const ThermalDerivParams *)user_data;

  double cht = state[THERMAL_STATE_CHT];
  double egt = state[THERMAL_STATE_EGT];
  double oil = state[THERMAL_STATE_OIL];

  dstate[THERMAL_STATE_CHT] =
      (node_target_c(p->cht_gain_c_per_w, p->waste_heat_w, p->ambient_temp_c) -
       cht) /
      p->cht_tau_s;
  dstate[THERMAL_STATE_EGT] =
      (node_target_c(p->egt_gain_c_per_w, p->waste_heat_w, p->ambient_temp_c) -
       egt) /
      p->egt_tau_s;
  dstate[THERMAL_STATE_OIL] =
      (node_target_c(p->oil_gain_c_per_w, p->waste_heat_w, p->ambient_temp_c) -
       oil) /
      p->oil_tau_s;
}

ThermalConfig thermal_config_default(void) {
  ThermalConfig cfg;
  cfg.cht_tau_s = 90.0;  /* metal head: moderate thermal mass */
  cfg.egt_tau_s = 5.0;   /* exhaust gas: responds almost immediately */
  cfg.oil_tau_s = 240.0; /* oil sump: large thermal mass, slowest to move */
  cfg.cht_gain_c_per_w = 0.008;
  cfg.egt_gain_c_per_w = 0.035;
  cfg.oil_gain_c_per_w = 0.004;
  return cfg;
}

void thermal_init(ThermalState *state, double ambient_temp_c) {
  state->cht_c = ambient_temp_c;
  state->egt_c = ambient_temp_c;
  state->oil_temp_c = ambient_temp_c;
}

void thermal_step(ThermalState *state, const ThermalConfig *config,
                  double waste_heat_w, double ambient_temp_c, double t,
                  double dt) {
  double vec[THERMAL_STATE_COUNT] = {state->cht_c, state->egt_c,
                                     state->oil_temp_c};

  ThermalDerivParams params;
  params.waste_heat_w = waste_heat_w;
  params.ambient_temp_c = ambient_temp_c;
  params.cht_tau_s = config->cht_tau_s;
  params.egt_tau_s = config->egt_tau_s;
  params.oil_tau_s = config->oil_tau_s;
  params.cht_gain_c_per_w = config->cht_gain_c_per_w;
  params.egt_gain_c_per_w = config->egt_gain_c_per_w;
  params.oil_gain_c_per_w = config->oil_gain_c_per_w;

  integrator_rk4_step(vec, THERMAL_STATE_COUNT, t, dt, thermal_derivative,
                      &params);

  state->cht_c = vec[THERMAL_STATE_CHT];
  state->egt_c = vec[THERMAL_STATE_EGT];
  state->oil_temp_c = vec[THERMAL_STATE_OIL];
}
