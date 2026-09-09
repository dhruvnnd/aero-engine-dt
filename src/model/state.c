#include "model/state.h"

void model_state_init(ModelState *state, const EngineConfig *engine_config,
                      double ambient_temp_c) {
  engine_model_init(&state->engine, engine_config);
  thermal_init(&state->thermal, ambient_temp_c);
  model_state_refresh_derived(state);
}

void model_state_refresh_derived(ModelState *state) {
  state->rpm = engine_model_rpm(&state->engine);
  state->torque_nm = engine_model_torque_nm(&state->engine);
}
