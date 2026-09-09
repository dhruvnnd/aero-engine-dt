#ifndef MODEL_SYNC_H
#define MODEL_SYNC_H

#include "model/state.h"
#include "physics/engine_model.h"
#include "physics/thermal.h"

typedef struct {
  EngineConfig engine_config;
  ThermalConfig thermal_config;
  double sim_time_s; /* accumulated simulated time, passed to the integrators */
} ModelSync;

/* Loads the default engine/thermal configs and zeroes the sim clock. */
void model_sync_init(ModelSync *sync);

void model_sync_step(ModelSync *sync, ModelState *state,
                     const EngineInput *input, double ambient_temp_c,
                     double dt);

#endif /* MODEL_SYNC_H */
