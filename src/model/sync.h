#ifndef MODEL_SYNC_H
#define MODEL_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "physics/cylinder.h"
#include "physics/ecu.h"
#include "physics/electrical.h"
#include "physics/engine_model.h"
#include "physics/environment.h"
#include "physics/fuel.h"
#include "physics/lubrication.h"
#include "physics/thermal.h"

typedef struct {
  EngineConfig engine_config;
  ThermalConfig thermal_config;
  FuelConfig fuel_config;
  LubeConfig lube_config;
  ElecConfig elec_config;
  CylinderConfig cyl_config[ENGINE_MAX_CYLINDERS]; /* one per cylinder slot */
  EcuSensorFault ecu_rpm_fault;  /* injected faults on the ECU's crank-speed */
  EcuSensorFault ecu_rpm2_fault; /* and redundant (alternator) speed inputs */
  double sim_time_s; /* accumulated simulated time, passed to the integrators */
} ModelSync;

/* Loads the default engine/thermal/fuel/lube/cylinder configs, zeroes clock. */
void model_sync_init(ModelSync *sync);

/* Installs `cfg` as the engine config and keeps the fuel model's displacement in
 * step with the engine geometry (bore, stroke, cylinder count) so fuel and air
 * flow follow the engine that was configured. `cfg` may alias
 * sync->engine_config. */
void model_sync_apply_engine_config(ModelSync *sync, const EngineConfig *cfg);

void model_sync_step(ModelSync *sync, ModelState *state,
                     const EngineInput *input, const EnvInput *env, double dt);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_SYNC_H */
