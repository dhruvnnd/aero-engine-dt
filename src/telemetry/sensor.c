#include "telemetry/sensor.h"

#include <math.h>

#include "math/units.h" /* UNITS_PI */

static uint32_t sensor_rng_next(Sensor *sensor) {
  uint32_t x = sensor->rng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  sensor->rng_state = x;
  return x;
}

/* Uniform in [0, 1). */
static double sensor_uniform01(Sensor *sensor) {
  return (double)sensor_rng_next(sensor) / 4294967296.0;
}

/* One draw from N(0, sigma^2) via the Box-Muller transform. */
static double sensor_gaussian(Sensor *sensor, double sigma) {
  if (sigma <= 0.0) {
    return 0.0;
  }
  double u1 = sensor_uniform01(sensor);
  double u2 = sensor_uniform01(sensor);
  if (u1 < 1e-12) {
    u1 = 1e-12; /* keep log() finite */
  }
  return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * UNITS_PI * u2);
}

static bool sensor_channel_dropped(Sensor *sensor) {
  return sensor_uniform01(sensor) < sensor->config.dropout_probability;
}

SensorConfig sensor_config_default(void) {
  SensorConfig cfg;
  cfg.noise_rpm = 12.0; /* visible tach wander; also stands in for firing ripple */
  cfg.noise_map_kpa = 0.3;
  cfg.noise_cht_c = 1.5;
  cfg.noise_egt_c = 4.0;
  cfg.noise_oil_temp_c = 1.0;
  cfg.noise_oil_press_kpa = 4.0;
  cfg.noise_fuel_press_kpa = 3.0;
  cfg.noise_bus_v = 0.05;
  cfg.noise_frac = 0.008; /* ~0.8% on torque / flows / lambda */
  cfg.dropout_probability = 0.005;
  return cfg;
}

void sensor_init(Sensor *sensor, const SensorConfig *config, uint32_t seed) {
  sensor->config = *config;
  sensor->rng_state = (seed != 0u) ? seed : 0x9E3779B9u;
}

SensorReading sensor_read(Sensor *sensor, const ModelState *truth) {
  const SensorConfig *c = &sensor->config;
  SensorReading r;

  r.value.rpm = truth->rpm + sensor_gaussian(sensor, c->noise_rpm);
  r.value.map_kpa =
      truth->engine.map_kpa + sensor_gaussian(sensor, c->noise_map_kpa);
  r.value.cht_c =
      truth->thermal.cht_c + sensor_gaussian(sensor, c->noise_cht_c);
  r.value.egt_c =
      truth->thermal.egt_c + sensor_gaussian(sensor, c->noise_egt_c);
  r.value.oil_temp_c =
      truth->thermal.oil_temp_c + sensor_gaussian(sensor, c->noise_oil_temp_c);

  r.ok.rpm = !sensor_channel_dropped(sensor);
  r.ok.map_kpa = !sensor_channel_dropped(sensor);
  r.ok.cht_c = !sensor_channel_dropped(sensor);
  r.ok.egt_c = !sensor_channel_dropped(sensor);
  r.ok.oil_temp_c = !sensor_channel_dropped(sensor);

  if (!r.ok.rpm) {
    r.value.rpm = 0.0;
  }
  if (!r.ok.map_kpa) {
    r.value.map_kpa = 0.0;
  }
  if (!r.ok.cht_c) {
    r.value.cht_c = 0.0;
  }
  if (!r.ok.egt_c) {
    r.value.egt_c = 0.0;
  }
  if (!r.ok.oil_temp_c) {
    r.value.oil_temp_c = 0.0;
  }

  return r;
}

/* Additive noise of `frac` of |v|'s magnitude, in v's own units. */
static double sensor_noise_frac(Sensor *sensor, double v, double frac) {
  return sensor_gaussian(sensor, fabs(v) * frac);
}

void sensor_read_state(Sensor *sensor, const ModelState *truth,
                       ModelState *out) {
  const SensorConfig *c = &sensor->config;

  *out = *truth;

  out->rpm += sensor_gaussian(sensor, c->noise_rpm);
  out->torque_nm += sensor_noise_frac(sensor, truth->torque_nm, c->noise_frac);

  out->engine.map_kpa += sensor_gaussian(sensor, c->noise_map_kpa);

  out->thermal.cht_c += sensor_gaussian(sensor, c->noise_cht_c);
  out->thermal.egt_c += sensor_gaussian(sensor, c->noise_egt_c);
  out->thermal.oil_temp_c += sensor_gaussian(sensor, c->noise_oil_temp_c);

  out->lube.oil_press_kpa += sensor_gaussian(sensor, c->noise_oil_press_kpa);

  out->fuel.fuel_press_kpa +=
      sensor_gaussian(sensor, c->noise_fuel_press_kpa);
  out->fuel.fuel_flow_kgph +=
      sensor_noise_frac(sensor, truth->fuel.fuel_flow_kgph, c->noise_frac);
  out->fuel.air_flow_gps +=
      sensor_noise_frac(sensor, truth->fuel.air_flow_gps, c->noise_frac);

  out->elec.bus_v += sensor_gaussian(sensor, c->noise_bus_v);
  out->elec.alt_current_a +=
      sensor_noise_frac(sensor, truth->elec.alt_current_a, c->noise_frac);
  out->elec.alt_field_a +=
      sensor_noise_frac(sensor, truth->elec.alt_field_a, c->noise_frac);
  /* batt_soc is a coulomb-counted estimate, not a raw sensor -- left clean. */

  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    out->cyl[i].cht_c += sensor_gaussian(sensor, c->noise_cht_c);
    out->cyl[i].egt_c += sensor_gaussian(sensor, c->noise_egt_c);
    out->cyl[i].lambda += sensor_gaussian(sensor, c->noise_frac);
  }
}
