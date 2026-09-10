#ifndef MODEL_STATE_H
#define MODEL_STATE_H

#include "physics/cylinder.h"
#include "physics/electrical.h"
#include "physics/engine_model.h"
#include "physics/environment.h"
#include "physics/fuel.h"
#include "physics/lubrication.h"
#include "physics/thermal.h"

typedef struct {
  EngineState engine;   /* omega_rad_s, map_kpa */
  ThermalState thermal; /* cht_c, egt_c, oil_temp_c (engine-wide) */
  FuelState fuel;       /* air_flow_gps, fuel_flow_kgph, fuel_press_kpa */
  LubeState lube;       /* oil_press_kpa */
  ElecState elec;       /* bus_v, alt_current_a, alt_field_a, batt_soc */
  EnvState env;         /* flight condition: OAT, ambient press, density alt */

  CylinderState cyl[ENGINE_MAX_CYLINDERS];

  /* Derived from `engine`; refreshed by model_state_refresh_derived() after
   * each physics step. */
  double rpm;
  double torque_nm;
} ModelState;

/* Cold-start: engine at cold idle, all temperatures at ambient, derived
 * readouts made consistent with that */
void model_state_init(ModelState *state, const EngineConfig *engine_config,
                      double ambient_temp_c);

/* Recomputes the cached derived readouts (rpm, torque_nm) from engine. */
void model_state_refresh_derived(ModelState *state);

#endif /* MODEL_STATE_H */
