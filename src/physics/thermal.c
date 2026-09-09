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
} ThermalDerivParams;

static double cht_target_c(double waste_heat_w, double ambient_temp_c) {
  const double k_cht_c_per_w = 0.008;
  return ambient_temp_c + k_cht_c_per_w * waste_heat_w;
}

static double egt_target_c(double waste_heat_w, double ambient_temp_c) {
  const double k_egt_c_per_w = 0.035;
  return ambient_temp_c + k_egt_c_per_w * waste_heat_w;
}

static double oil_target_c(double waste_heat_w, double ambient_temp_c) {
  const double k_oil_c_per_w = 0.004;
  return ambient_temp_c + k_oil_c_per_w * waste_heat_w;
}

static void thermal_derivative(const double *state, double *dstate, double t,
                               void *user_data) {
  (void)t;
  const ThermalDerivParams *p = (const ThermalDerivParams *)user_data;

  double cht = state[THERMAL_STATE_CHT];
  double egt = state[THERMAL_STATE_EGT];
  double oil = state[THERMAL_STATE_OIL];

  dstate[THERMAL_STATE_CHT] =
      (cht_target_c(p->waste_heat_w, p->ambient_temp_c) - cht) / p->cht_tau_s;
  dstate[THERMAL_STATE_EGT] =
      (egt_target_c(p->waste_heat_w, p->ambient_temp_c) - egt) / p->egt_tau_s;
  dstate[THERMAL_STATE_OIL] =
      (oil_target_c(p->waste_heat_w, p->ambient_temp_c) - oil) / p->oil_tau_s;
}

ThermalConfig thermal_config_default(void) {
  ThermalConfig cfg;
  cfg.cht_tau_s = 90.0;  /* metal head: moderate thermal mass */
  cfg.egt_tau_s = 5.0;   /* exhaust gas: responds almost immediately */
  cfg.oil_tau_s = 240.0; /* oil sump: large thermal mass, slowest to move */
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

  integrator_rk4_step(vec, THERMAL_STATE_COUNT, t, dt, thermal_derivative,
                      &params);

  state->cht_c = vec[THERMAL_STATE_CHT];
  state->egt_c = vec[THERMAL_STATE_EGT];
  state->oil_temp_c = vec[THERMAL_STATE_OIL];
}
