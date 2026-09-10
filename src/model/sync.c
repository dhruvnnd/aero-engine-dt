#include "model/sync.h"

#include "physics/combustion.h"
#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "physics/fuel.h"
#include "physics/lubrication.h"
#include "physics/thermal.h"

void model_sync_init(ModelSync *sync) {
  sync->engine_config = engine_config_default();
  sync->thermal_config = thermal_config_default();
  sync->fuel_config = fuel_config_default();
  sync->lube_config = lube_config_default();
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    sync->cyl_config[i] = cylinder_config_default();
  }
  sync->sim_time_s = 0.0;
}

void model_sync_step(ModelSync *sync, ModelState *state,
                     const EngineInput *input, double ambient_temp_c,
                     double dt) {
  engine_model_step(&state->engine, &sync->engine_config, input,
                    sync->cyl_config, sync->sim_time_s, dt);

  /* Thermal drivers from the current (post-step) operating point: raw waste
   * heat for the oil node, load fraction for the saturating CHT/EGT nodes. */
  double waste_heat_w =
      combustion_waste_heat_w(state->engine.map_kpa, state->engine.omega_rad_s);
  double load_frac = combustion_load_fraction(state->engine.map_kpa,
                                              state->engine.omega_rad_s);

  thermal_step(&state->thermal, &sync->thermal_config, waste_heat_w, load_frac,
               ambient_temp_c, sync->sim_time_s, dt);

  for (int i = 0; i < sync->engine_config.num_cylinders; i++) {
    cylinder_step(&state->cyl[i], &sync->cyl_config[i], state->engine.map_kpa,
                  state->engine.omega_rad_s, ambient_temp_c,
                  &sync->thermal_config, sync->engine_config.num_cylinders,
                  sync->sim_time_s, dt);
  }

  sync->sim_time_s += dt;

  model_state_refresh_derived(state);

  state->torque_nm = cylinders_total_torque_nm(
      sync->cyl_config, sync->engine_config.num_cylinders,
      state->engine.map_kpa, state->engine.omega_rad_s);

  /* Fuel path reads the settled operating point (rpm just refreshed above);
   * intake temp is the ambient proxy until MAT lands. */
  fuel_step(&state->fuel, &sync->fuel_config, sync->cyl_config,
            sync->engine_config.num_cylinders, state->engine.map_kpa,
            state->rpm, ambient_temp_c);

  /* Oil pressure from the settled crank speed and current oil temperature. */
  lube_step(&state->lube, &sync->lube_config, state->rpm,
            state->thermal.oil_temp_c);
}
