#ifndef PHYSICS_INTEGRATOR_H
#define PHYSICS_INTEGRATOR_H

/* Upper bound on the number of scalars in a single state vector used with
 * the stepper below. Keeps it allocation-free (fixed stack buffers) at the
 * cost of a compile-time cap -- generous for any one submodel (engine,
 * thermal, etc.) we expect to write. */
#define INTEGRATOR_MAX_STATE 16

/* Computes the derivative of `state` (an n-element vector) at time `t`,
 * writing the result into `dstate` (also n elements). `user_data` is
 * whatever the caller passed to integrator_rk4_step -- typically submodel
 * parameters/inputs (e.g. current throttle position) that aren't
 * themselves part of the integrated state. */
typedef void (*IntegratorDerivFn)(const double *state, double *dstate, double t,
                                  void *user_data);

/* Advances `state` (n elements, n <= INTEGRATOR_MAX_STATE) by one step of
 * size `dt` using classical 4th-order Runge-Kutta, calling `deriv` to
 * evaluate the derivative at four points within the step. Overwrites
 * `state` in place with the result at t + dt. */
void integrator_rk4_step(double *state, int n, double t, double dt,
                         IntegratorDerivFn deriv, void *user_data);

#endif /* PHYSICS_INTEGRATOR_H */
