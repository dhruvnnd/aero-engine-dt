#ifndef MATH_UNITS_H
#define MATH_UNITS_H

#define UNITS_PI 3.14159265358979323846

/* Angular velocity: revolutions-per-minute <-> radians-per-second. */
static inline double rpm_to_rad_s(double rpm) {
  return rpm * (2.0 * UNITS_PI) / 60.0;
}

static inline double rad_s_to_rpm(double rad_s) {
  return rad_s * 60.0 / (2.0 * UNITS_PI);
}

/* Temperature: Celsius <-> Kelvin. */
static inline double celsius_to_kelvin(double c) { return c + 273.15; }

static inline double kelvin_to_celsius(double k) { return k - 273.15; }

/* Pressure: kilopascals <-> pascals. */
static inline double kpa_to_pa(double kpa) { return kpa * 1000.0; }

static inline double pa_to_kpa(double pa) { return pa / 1000.0; }

#endif /* MATH_UNITS_H */
