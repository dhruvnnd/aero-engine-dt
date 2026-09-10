#ifndef TELEMETRY_SENSOR_H
#define TELEMETRY_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "model/state.h"

typedef struct {
  double rpm;
  double map_kpa;
  double cht_c;
  double egt_c;
  double oil_temp_c;
} SensorSample;

/* Per-channel validity for one reading. A false field means that channel
 * dropped out this sample; the matching SensorSample field is then 0.0 and
 * must not be read as a measurement. */
typedef struct {
  bool rpm;
  bool map_kpa;
  bool cht_c;
  bool egt_c;
  bool oil_temp_c;
} SensorChannelOk;

typedef struct {
  SensorSample value;
  SensorChannelOk ok;
} SensorReading;

/* 1-sigma additive noise per channel, each in that channel's own units,
 * and the independent probability [0,1] that any one channel drops out of
 * a given sample. */
typedef struct {
  double noise_rpm;
  double noise_map_kpa;
  double noise_cht_c;
  double noise_egt_c;
  double noise_oil_temp_c;
  double noise_oil_press_kpa;
  double noise_fuel_press_kpa;
  double noise_bus_v;
  double noise_frac; /* fractional 1-sigma for torque, flows, lambda */
  double dropout_probability;
} SensorConfig;

/* Plausible bench-instrumentation noise levels with ~0.5% per-channel
 * dropout. Placeholder figures, to be replaced with numbers from a real
 * sensor/DAQ spec later. */
SensorConfig sensor_config_default(void);

typedef struct {
  SensorConfig config;
  uint32_t rng_state; /* xorshift32; deterministic for a given seed */
} Sensor;

/* `seed` fixes the noise/dropout sequence for reproducible runs; a seed of
 * 0 is replaced with a nonzero default (xorshift can't run from 0). */
void sensor_init(Sensor *sensor, const SensorConfig *config, uint32_t seed);

/* Samples the model once: each channel is copied from `truth`, has Gaussian
 * noise added, and is then independently dropped with probability
 * config.dropout_probability. */
SensorReading sensor_read(Sensor *sensor, const ModelState *truth);

/* Fills `out` with a copy of `truth` in which every displayed channel
 * (engine, thermal, fuel, lube, per-cylinder) carries additive Gaussian
 * noise -- the "instrument feed" a dashboard shows instead of the exact
 * model state. No dropout; misfire_rate and other derived fields are left
 * as-is. */
void sensor_read_state(Sensor *sensor, const ModelState *truth,
                       ModelState *out);

#endif /* TELEMETRY_SENSOR_H */
