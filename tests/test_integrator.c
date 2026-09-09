#include "test_util.h"

#include "math/units.h" /* UNITS_PI */
#include "physics/integrator.h"

#define E_CONST 2.71828182845904523536

/* dy/dt = y  ->  y(t) = y(0) e^t */
static void deriv_exp(const double *y, double *dy, double t, void *ud) {
  (void)t;
  (void)ud;
  dy[0] = y[0];
}

static void test_exponential_growth(void) {
  double y[1] = {1.0};
  double dt = 0.001;
  for (int i = 0; i < 1000; i++) {
    integrator_rk4_step(y, 1, i * dt, dt, deriv_exp, NULL);
  }
  CHECK_NEAR(y[0], E_CONST, 1e-5);
}

/* dy/dt = -k y  ->  y(t) = y(0) e^{-k t} */
static void deriv_decay(const double *y, double *dy, double t, void *ud) {
  (void)t;
  const double *k = (const double *)ud;
  dy[0] = -(*k) * y[0];
}

static void test_exponential_decay(void) {
  double k = 2.0;
  double y[1] = {5.0};
  double dt = 0.001;
  for (int i = 0; i < 2000; i++) {
    integrator_rk4_step(y, 1, i * dt, dt, deriv_decay, &k);
  }
  /* t = 2 -> 5 * e^{-4} */
  CHECK_NEAR(y[0], 5.0 * exp(-4.0), 1e-5);
}

/* Simple harmonic oscillator: x'' = -x, carried as [x, v]. */
static void deriv_sho(const double *s, double *ds, double t, void *ud) {
  (void)t;
  (void)ud;
  ds[0] = s[1];
  ds[1] = -s[0];
}

static void test_harmonic_oscillator_returns_and_conserves_energy(void) {
  double s[2] = {1.0, 0.0}; /* energy = 0.5 (x^2 + v^2) = 0.5 */
  double dt = 0.0005;
  int steps = (int)((2.0 * UNITS_PI) / dt + 0.5); /* one full period */
  for (int i = 0; i < steps; i++) {
    integrator_rk4_step(s, 2, i * dt, dt, deriv_sho, NULL);
  }
  CHECK_NEAR(s[0], 1.0, 1e-3);
  CHECK_NEAR(s[1], 0.0, 1e-3);
  CHECK_NEAR(0.5 * (s[0] * s[0] + s[1] * s[1]), 0.5, 1e-4);
}

/* Constant derivative -> RK4 is exact (to rounding). */
static void deriv_const(const double *y, double *dy, double t, void *ud) {
  (void)t;
  (void)y;
  const double *a = (const double *)ud;
  dy[0] = *a;
  dy[1] = *a * 2.0;
}

static void test_constant_derivative_is_exact(void) {
  double a = 3.5;
  double y[2] = {0.0, 10.0};
  double dt = 0.1;
  for (int i = 0; i < 100; i++) {
    integrator_rk4_step(y, 2, i * dt, dt, deriv_const, &a);
  }
  CHECK_NEAR(y[0], 35.0, 1e-9);       /* 3.5 * 10 */
  CHECK_NEAR(y[1], 10.0 + 70.0, 1e-9); /* 10 + 7.0 * 10 */
}

static const TestCase CASES[] = {
    {"integrator.exponential_growth", test_exponential_growth},
    {"integrator.exponential_decay", test_exponential_decay},
    {"integrator.harmonic_oscillator_returns_and_conserves_energy",
     test_harmonic_oscillator_returns_and_conserves_energy},
    {"integrator.constant_derivative_is_exact",
     test_constant_derivative_is_exact},
};

RUN_TESTS(CASES)
