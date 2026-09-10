#include "model/state.h"

void model_state_init(ModelState *state, const EngineConfig *engine_config,
                      double ambient_temp_c) {
  engine_model_init(&state->engine, engine_config);
  thermal_init(&state->thermal, ambient_temp_c);
  fuel_state_init(&state->fuel);
  lube_state_init(&state->lube);
  elec_state_init(&state->elec);

  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    if (i < engine_config->num_cylinders) {
      cylinder_state_init(&state->cyl[i], ambient_temp_c);
    } else {
      CylinderState zero = {0}; /* inert slot */
      state->cyl[i] = zero;
    }
  }

  model_state_refresh_derived(state);
}

void model_state_refresh_derived(ModelState *state) {
  state->rpm = engine_model_rpm(&state->engine);
  state->torque_nm = engine_model_torque_nm(&state->engine);
}
