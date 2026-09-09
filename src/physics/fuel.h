#ifndef PHYSICS_FUEL_H
#define PHYSICS_FUEL_H

#include "physics/cylinder.h" /* CylinderConfig */

/* Engine air / fuel mass balance */

typedef struct {
  double afr_stoich;     /* stoichiometric air/fuel mass ratio (~14.7) */
  double lambda_target;  /* commanded mixture, 1.0 = stoichiometric */
  double vol_eff;        /* volumetric efficiency, fraction */
  double displacement_l; /* total swept volume, litres */
  double pump_press_kpa; /* fuel-rail pressure at zero demand */
} FuelConfig;

typedef struct {
  double air_flow_gps;   /* ingested air mass flow, g/s */
  double fuel_flow_kgph; /* delivered fuel mass flow, kg/h */
  double fuel_press_kpa; /* fuel-rail pressure */
} FuelState;

/* placeholder numbers for a small aero piston engine. */
FuelConfig fuel_config_default(void);

/* Zeroes all flows and pressure (engine not running). */
void fuel_state_init(FuelState *state);

/* Refreshes the fuel state from the current operating point */
void fuel_step(FuelState *state, const FuelConfig *config,
               const CylinderConfig *cyl, int num_cylinders, double map_kpa,
               double rpm, double intake_temp_c);

#endif /* PHYSICS_FUEL_H */
