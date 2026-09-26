#include "physics/propeller.h"

#include <math.h>

/* x = J / j_zero_thrust is capped here, so thrust and torque stay bounded at
 * high airspeed / low RPM instead of growing with x^2. */
#define PROP_X_MAX 1.3

PropConfig prop_config_default(void) {
  PropConfig p;
  p.diameter_m = 1.2;
  p.j_zero_thrust = 0.85;
  p.ct_static = 0.10;
  p.cq_static = 0.0075;
  p.cq_unload = 0.45;
  return p;
}

static double clamp_x(const PropConfig *cfg, double j) {
  double x = cfg->j_zero_thrust > 0.0 ? j / cfg->j_zero_thrust : PROP_X_MAX;
  return x > PROP_X_MAX ? PROP_X_MAX : (x < 0.0 ? 0.0 : x);
}

double prop_advance_ratio(const PropConfig *cfg, double rpm,
                          double airspeed_ms) {
  const double v = airspeed_ms > 0.0 ? airspeed_ms : 0.0;
  const double n = rpm > 0.0 ? rpm / 60.0 : 0.0;
  const double x_max_j = PROP_X_MAX * cfg->j_zero_thrust;
  if (n <= 0.0 || cfg->diameter_m <= 0.0) {
    return x_max_j;
  }
  const double j = v / (n * cfg->diameter_m);
  return j > x_max_j ? x_max_j : j;
}

void prop_step(PropState *state, const PropConfig *cfg, double rpm,
               double airspeed_ms, double density_kg_m3) {
  const double n = rpm > 0.0 ? rpm / 60.0 : 0.0;
  const double j = prop_advance_ratio(cfg, rpm, airspeed_ms);
  const double x = clamp_x(cfg, j);
  const double d = cfg->diameter_m;
  const double rho_n2 = density_kg_m3 * n * n;

  double cq = cfg->cq_static * (1.0 - cfg->cq_unload * x * x);
  if (cq < 0.0) {
    cq = 0.0;
  }
  const double ct = cfg->ct_static * (1.0 - x * x);

  state->advance_ratio = j;
  state->torque_nm = cq * rho_n2 * pow(d, 5.0);
  state->thrust_n = ct * rho_n2 * pow(d, 4.0);
}
