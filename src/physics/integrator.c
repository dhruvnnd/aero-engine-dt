#include "physics/integrator.h"

#include <assert.h>

void integrator_rk4_step(double *state, int n, double t, double dt,
                         IntegratorDerivFn deriv, void *user_data) {
  assert(n > 0 && n <= INTEGRATOR_MAX_STATE);

  double k1[INTEGRATOR_MAX_STATE];
  double k2[INTEGRATOR_MAX_STATE];
  double k3[INTEGRATOR_MAX_STATE];
  double k4[INTEGRATOR_MAX_STATE];
  double temp[INTEGRATOR_MAX_STATE];

  deriv(state, k1, t, user_data);

  for (int i = 0; i < n; ++i) {
    temp[i] = state[i] + 0.5 * dt * k1[i];
  }
  deriv(temp, k2, t + 0.5 * dt, user_data);

  for (int i = 0; i < n; ++i) {
    temp[i] = state[i] + 0.5 * dt * k2[i];
  }
  deriv(temp, k3, t + 0.5 * dt, user_data);

  for (int i = 0; i < n; ++i) {
    temp[i] = state[i] + dt * k3[i];
  }
  deriv(temp, k4, t + dt, user_data);

  for (int i = 0; i < n; ++i) {
    state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
}
