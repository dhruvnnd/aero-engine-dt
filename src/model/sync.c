#include "model/sync.h"

#include <string.h>

#include "physics/cylinder.h"
#include "physics/ecu.h"
#include "physics/electrical.h"
#include "physics/engine_model.h"
#include "physics/environment.h"
#include "physics/fuel.h"
#include "physics/lubrication.h"
#include "physics/propeller.h"
#include "physics/thermal.h"

void model_sync_init(ModelSync *sync) {
  sync->engine_config = engine_config_default();
  sync->thermal_config = thermal_config_default();
  sync->fuel_config = fuel_config_default();
  sync->lube_config = lube_config_default();
  sync->elec_config = elec_config_default();
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    sync->cyl_config[i] = cylinder_config_default();
  }
  memset(&sync->ecu_rpm_fault, 0, sizeof sync->ecu_rpm_fault);
  memset(&sync->ecu_rpm2_fault, 0, sizeof sync->ecu_rpm2_fault);
  sync->sim_time_s = 0.0;
}

void model_sync_apply_engine_config(ModelSync *sync, const EngineConfig *cfg) {
  const EngineConfig c = *cfg;
  sync->engine_config = c;
  const double displacement_l =
      cylinder_displacement_m3(&c.geom) * (double)c.num_cylinders * 1000.0;
  if (displacement_l > 0.0) {
    sync->fuel_config.displacement_l = displacement_l;
  }
}

void model_sync_step(ModelSync *sync, ModelState *state,
                     const EngineInput *input, const EnvInput *env, double dt) {
  /* Computed first (it only depends on `env`, not engine state) so its
   * ambient_temp_c is available to feed engine_model_step()'s combustion-
   * energy calculation this same step, rather than lagging a frame behind. */
  environment_state(&state->env, env->altitude_m, env->oat_offset_c,
                    env->airspeed_ms);
  double ambient_temp_c = state->env.oat_c;

  /* The propeller loads the crank */
  prop_step(&state->prop, &sync->engine_config.prop,
            engine_model_rpm(&state->engine), state->env.airspeed_ms,
            state->env.density_kg_m3);
  EngineInput engine_input = *input;
  engine_input.load_torque_nm += state->prop.torque_nm;

  const EcuPilotCmd pilot = {input->throttle};
  EcuActuators actuators;
  if (sync->engine_config.ecu_fitted) {
    /* two independent readings of the same crank speed, each with its own
     * injectable fault */
    const double true_rpm = engine_model_rpm(&state->engine);
    /* and temperature probes: the hottest head and exhaust port, and the oil */
    double cht_max = 0.0;
    double egt_max = 0.0;
    for (int i = 0; i < sync->engine_config.num_cylinders; i++) {
      cht_max = state->cyl[i].cht_c > cht_max ? state->cyl[i].cht_c : cht_max;
      egt_max = state->cyl[i].egt_c > egt_max ? state->cyl[i].egt_c : egt_max;
    }
    const EcuSensors sensors = {
        ecu_sensor_fault_apply(&sync->ecu_rpm_fault, true_rpm),
        ecu_sensor_fault_apply(&sync->ecu_rpm2_fault, true_rpm),
        state->engine.run_state == ENGINE_RUNNING,
        state->engine.ignition_on,
        cht_max,
        egt_max,
        state->thermal.oil_temp_c};
    ecu_step(&state->ecu, &sync->engine_config.ecu, &sensors, &pilot,
             &actuators, dt);
  } else {
    ecu_bypass(&state->ecu, &pilot, &actuators);
  }
  engine_input.throttle = actuators.throttle;

  engine_model_step(&state->engine, &sync->engine_config, &engine_input,
                    &sync->fuel_config, sync->cyl_config, state->cyl,
                    ambient_temp_c, sync->sim_time_s, dt);

  double cool_index =
      environment_cool_index(state->env.density_kg_m3, state->env.airspeed_ms,
                             engine_model_rpm(&state->engine));

  /* Each cylinder's head and exhaust port follow its own heat release and
   * exhaust gas; the engine-wide readings are the hottest head and the mean
   * port. Oil is heated by friction plus a share of the combustion heat. */
  const int n_cyl = sync->engine_config.num_cylinders;
  double heat_total_w = 0.0;
  double cht_max = ambient_temp_c;
  double egt_sum = 0.0;
  for (int i = 0; i < n_cyl; i++) {
    cylinder_step(&state->cyl[i], &sync->cyl_config[i],
                  &state->engine.cyl_thermal[i], state->engine.map_kpa,
                  ambient_temp_c, &sync->thermal_config, cool_index,
                  sync->sim_time_s, dt);
    heat_total_w += state->engine.cyl_thermal[i].heat_w;
    cht_max = state->cyl[i].cht_c > cht_max ? state->cyl[i].cht_c : cht_max;
    egt_sum += state->cyl[i].egt_c;
  }
  state->thermal.cht_c = cht_max;
  state->thermal.egt_c = n_cyl > 0 ? egt_sum / n_cyl : ambient_temp_c;
  thermal_oil_step(&state->thermal, &sync->thermal_config,
                   state->engine.friction_w, heat_total_w, cool_index,
                   ambient_temp_c, sync->sim_time_s, dt);

  sync->sim_time_s += dt;

  model_state_refresh_derived(state);

  /* Fuel path reads the settled operating point (rpm just refreshed above);
   * intake temp is the ambient proxy until MAT lands. */
  fuel_step(&state->fuel, &sync->fuel_config, sync->cyl_config,
            sync->engine_config.num_cylinders, state->engine.map_kpa,
            state->rpm, ambient_temp_c);

  /* Oil pressure from the settled crank speed and current oil temperature. */
  lube_step(&state->lube, &sync->lube_config, state->rpm,
            state->thermal.oil_temp_c);

  /* Charging system from the settled crank speed. Cranking draws starter
   * current from the battery on top of the usual accessory load */
  elec_step(&state->elec, &sync->elec_config, state->rpm,
            state->engine.run_state == ENGINE_CRANKING, dt);
}
