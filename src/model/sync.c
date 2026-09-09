#include "model/sync.h"

#include "physics/combustion.h"
#include "physics/engine_model.h"
#include "physics/thermal.h"

void model_sync_init(ModelSync *sync) {
  sync->engine_config = engine_config_default();
  sync->thermal_config = thermal_config_default();
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    sync->cyl_config[i] = cylinder_config_default();
  }
  sync->sim_time_s = 0.0;
}

void model_sync_step(ModelSync *sync, ModelState *state, const EngineInput *input,
                     double ambient_temp_c, double dt) {
  engine_model_step(&state->engine, &sync->engine_config, input,
                    sync->sim_time_s, dt);

  /* Heat into the thermal model is whatever the current (post-step)
   * operating point releases; at frame-scale dt the half-step lag versus
   * evaluating it at the pre-step point is negligible. */
  double waste_heat_w = combustion_waste_heat_w(state->engine.map_kpa,
                                                state->engine.omega_rad_s);

  thermal_step(&state->thermal, &sync->thermal_config, waste_heat_w,
               ambient_temp_c, sync->sim_time_s, dt);

  sync->sim_time_s += dt;

  model_state_refresh_derived(state);
}
