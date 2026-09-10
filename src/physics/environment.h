#ifndef PHYSICS_ENVIRONMENT_H
#define PHYSICS_ENVIRONMENT_H

/*
 * International Standard Atmosphere (troposphere model, valid roughly
 * 0-11,000m / 0-36,000 ft)
 */
typedef struct {
  double temperature_k;
  double pressure_kpa;
  double density_kg_m3;
} AtmosphereState;

/* Computes ambient temperature, pressure, and density at the given
 * geopotential altitude (meters above sea level). */
AtmosphereState environment_isa(double altitude_m);

typedef struct {
  double altitude_m;
  double airspeed_ms;
  double oat_c;
  double ambient_kpa;
  double density_kg_m3;
  double density_alt_m;
  double dynamic_press_pa; /* 0.5 * rho * V^2 */
} EnvState;

void environment_state(EnvState *env, double altitude_m, double oat_offset_c,
                       double airspeed_ms);

/* Mission-driven flight condition, fed to model_sync_step each tick. All
 * zero is sea level, still air, standard ISA temperature. */
typedef struct {
  double altitude_m;   /* pressure altitude (ISA), m */
  double airspeed_ms;  /* true airspeed, m/s */
  double oat_offset_c; /* outside-air temp minus ISA at that altitude */
} EnvInput;

double environment_density_altitude_m(double density_kg_m3);

/* Cooling-air-flow index: ~1.0 at sea level and reference cruise airspeed,
 * lower as air thins or the aircraft slows; a propwash term keeps it
 * positive at a standstill. CHT and oil targets are scaled by
 * 1 / sqrt(this) -- thin/slow cooling air runs the engine hotter. */
double environment_cool_index(double density_kg_m3, double airspeed_ms,
                              double rpm);

#endif /* PHYSICS_ENVIRONMENT_H */
