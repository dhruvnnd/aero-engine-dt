#include "physics/thermal.h"

#include <math.h>

#include "physics/integrator.h"

enum {
  THERMAL_STATE_CHT = 0,
  THERMAL_STATE_EGT = 1,
  THERMAL_STATE_OIL = 2,
  THERMAL_STATE_COUNT
};

typedef struct {
  double waste_heat_w;
  double load_frac;
  double cool_div;
  double ambient_temp_c;
  double cht_tau_s;
  double egt_tau_s;
  double oil_tau_s;
  double cht_rise_rated_c;
  double egt_rise_rated_c;
  double oil_gain_c_per_w;
} ThermalDerivParams;

/* CHT/EGT target: ambient plus a rise that saturates with load. sqrt() so it
 * is 0 with the engine stopped, ~half at idle, full at rated power -- gas and
 * head temperatures don't scale without bound with absolute engine power. */
double thermal_rise_c(double rise_rated_c, double load_frac) {
  double lf = load_frac;
  if (lf < 0.0) {
    lf = 0.0;
  }
  if (lf > 1.0) {
    lf = 1.0;
  }
  return rise_rated_c * sqrt(lf);
}

double thermal_cool_divisor(double cool_index) {
  double c = cool_index;
  if (c < 0.05) {
    c = 0.05;
  }
  if (c > 4.0) {
    c = 4.0;
  }
  return sqrt(c);
}

/* Oil target: linear in raw waste heat (slow bulk sink, no saturation),
 * scaled by cooling airflow -- the oil cooler sees ram air too. */
static double oil_target_c(double gain_c_per_w, double waste_heat_w,
                           double cool_div, double ambient_temp_c) {
  return ambient_temp_c + gain_c_per_w * waste_heat_w / cool_div;
}

static void thermal_derivative(const double *state, double *dstate, double t,
                               void *user_data) {
  (void)t;
  const ThermalDerivParams *p = (const ThermalDerivParams *)user_data;

  double cht = state[THERMAL_STATE_CHT];
  double egt = state[THERMAL_STATE_EGT];
  double oil = state[THERMAL_STATE_OIL];

  double cht_target =
      p->ambient_temp_c +
      thermal_rise_c(p->cht_rise_rated_c, p->load_frac) / p->cool_div;
  double egt_target =
      p->ambient_temp_c + thermal_rise_c(p->egt_rise_rated_c, p->load_frac);

  dstate[THERMAL_STATE_CHT] = (cht_target - cht) / p->cht_tau_s;
  dstate[THERMAL_STATE_EGT] = (egt_target - egt) / p->egt_tau_s;
  dstate[THERMAL_STATE_OIL] =
      (oil_target_c(p->oil_gain_c_per_w, p->waste_heat_w, p->cool_div,
                    p->ambient_temp_c) -
       oil) /
      p->oil_tau_s;
}

ThermalConfig thermal_config_default(void) {
  ThermalConfig cfg;
  cfg.cht_tau_s = 90.0;  /* metal head: moderate thermal mass */
  cfg.egt_tau_s = 5.0;   /* exhaust gas: responds almost immediately */
  cfg.oil_tau_s = 240.0; /* oil sump: large thermal mass, slowest to move */
  cfg.cht_rise_rated_c = 210.0;
  cfg.egt_rise_rated_c = 780.0;
  cfg.oil_gain_c_per_w = 0.004;
  return cfg;
}

void thermal_init(ThermalState *state, double ambient_temp_c) {
  state->cht_c = ambient_temp_c;
  state->egt_c = ambient_temp_c;
  state->oil_temp_c = ambient_temp_c;
}

void thermal_step(ThermalState *state, const ThermalConfig *config,
                  double waste_heat_w, double load_frac, double cool_index,
                  double ambient_temp_c, double t, double dt) {
  double vec[THERMAL_STATE_COUNT] = {state->cht_c, state->egt_c,
                                     state->oil_temp_c};

  ThermalDerivParams params;
  params.waste_heat_w = waste_heat_w;
  params.load_frac = load_frac;
  params.cool_div = thermal_cool_divisor(cool_index);
  params.ambient_temp_c = ambient_temp_c;
  params.cht_tau_s = config->cht_tau_s;
  params.egt_tau_s = config->egt_tau_s;
  params.oil_tau_s = config->oil_tau_s;
  params.cht_rise_rated_c = config->cht_rise_rated_c;
  params.egt_rise_rated_c = config->egt_rise_rated_c;
  params.oil_gain_c_per_w = config->oil_gain_c_per_w;

  integrator_rk4_step(vec, THERMAL_STATE_COUNT, t, dt, thermal_derivative,
                      &params);

  state->cht_c = vec[THERMAL_STATE_CHT];
  state->egt_c = vec[THERMAL_STATE_EGT];
  state->oil_temp_c = vec[THERMAL_STATE_OIL];
}
