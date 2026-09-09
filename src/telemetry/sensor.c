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
  cfg.noise_rpm = 5.0;
  cfg.noise_map_kpa = 0.3;
  cfg.noise_cht_c = 1.5;
  cfg.noise_egt_c = 4.0;
  cfg.noise_oil_temp_c = 1.0;
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
