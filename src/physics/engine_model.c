#include "physics/engine_model.h"

#include "math/units.h"
#include "physics/combustion.h"
#include "physics/integrator.h"

enum {
  ENGINE_STATE_OMEGA = 0, /* crank angular velocity, rad/s */
  ENGINE_STATE_MAP = 1,   /* manifold absolute pressure, kPa */
  ENGINE_STATE_COUNT
};

typedef struct {
  double throttle;
  double load_torque_nm;
  double ambient_pressure_kpa;
  double inertia_kg_m2;
  double map_tau_s;
  double friction_coeff_nm_per_rad_s;
  const CylinderConfig *cylinders;
  int num_cylinders;
} EngineDerivParams;

static double map_target_kpa(double throttle, double ambient_pressure_kpa) {
  const double idle_vacuum_drop_kpa = 71.3; /* below ambient, closed throttle */

  double map_idle_kpa = ambient_pressure_kpa - idle_vacuum_drop_kpa;
  if (map_idle_kpa < 0.0) {
    map_idle_kpa = 0.0;
  }
  double map_wot_kpa = ambient_pressure_kpa;

  return map_idle_kpa + throttle * (map_wot_kpa - map_idle_kpa);
}

static void engine_derivative(const double *state, double *dstate, double t,
                              void *user_data) {
  (void)t;
  const EngineDerivParams *p = (const EngineDerivParams *)user_data;

  double omega = state[ENGINE_STATE_OMEGA];
  double map_kpa = state[ENGINE_STATE_MAP];

  double torque_indicated =
      cylinders_total_torque_nm(p->cylinders, p->num_cylinders, map_kpa, omega);
  double torque_friction = p->friction_coeff_nm_per_rad_s * omega;
  double torque_net = torque_indicated - torque_friction - p->load_torque_nm;

  dstate[ENGINE_STATE_OMEGA] = torque_net / p->inertia_kg_m2;
  dstate[ENGINE_STATE_MAP] =
      (map_target_kpa(p->throttle, p->ambient_pressure_kpa) - map_kpa) /
      p->map_tau_s;
}

EngineConfig engine_config_default(void) {
  EngineConfig cfg;
  cfg.inertia_kg_m2 = 0.6;
  cfg.map_tau_s = 0.25;
  cfg.friction_coeff_nm_per_rad_s = 0.12;

  /* Inline-four, 1-3-4-2 firing order */
  cfg.num_cylinders = 4;
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg.firing_order[i] = 0;
  }
  cfg.firing_order[0] = 1;
  cfg.firing_order[1] = 3;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 2;
  return cfg;
}

void engine_model_init(EngineState *state, const EngineConfig *config) {
  (void)config;
  state->omega_rad_s = rpm_to_rad_s(700.0); /* cold idle guess */
  state->map_kpa = 30.0;                    /* idle vacuum */
}

/* Internal integration sub-step size*/
#define ENGINE_SUB_STEP_S 0.0003

void engine_model_step(EngineState *state, const EngineConfig *config,
                       const EngineInput *input,
                       const CylinderConfig *cylinders, double t, double dt) {
  double vec[ENGINE_STATE_COUNT] = {state->omega_rad_s, state->map_kpa};

  EngineDerivParams params;
  params.throttle = input->throttle;
  params.load_torque_nm = input->load_torque_nm;
  params.ambient_pressure_kpa = input->ambient_pressure_kpa;
  params.inertia_kg_m2 = config->inertia_kg_m2;
  params.map_tau_s = config->map_tau_s;
  params.friction_coeff_nm_per_rad_s = config->friction_coeff_nm_per_rad_s;
  params.cylinders = cylinders;
  params.num_cylinders = config->num_cylinders;

  double t_local = t;
  double remaining = dt;
  while (remaining > 1e-12) {
    double h = remaining < ENGINE_SUB_STEP_S ? remaining : ENGINE_SUB_STEP_S;
    integrator_rk4_step(vec, ENGINE_STATE_COUNT, t_local, h, engine_derivative,
                        &params);
    t_local += h;
    remaining -= h;
  }

  state->omega_rad_s = vec[ENGINE_STATE_OMEGA];
  state->map_kpa = vec[ENGINE_STATE_MAP];
}

double engine_model_rpm(const EngineState *state) {
  return rad_s_to_rpm(state->omega_rad_s);
}

double engine_model_torque_nm(const EngineState *state) {
  return combustion_indicated_torque_nm(state->map_kpa, state->omega_rad_s);
}
