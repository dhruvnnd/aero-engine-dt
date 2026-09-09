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

#endif /* PHYSICS_ENVIRONMENT_H */
