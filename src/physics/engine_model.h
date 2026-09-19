#ifndef PHYSICS_ENGINE_MODEL_H
#define PHYSICS_ENGINE_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal lumped-parameter model of a piston engine's rotating and intake
 * dynamics: crank angular velocity and manifold pressure, both carried as
 * ODE state. Indicated torque is the sum of per-cylinder contributions
 * (physics/cylinder.h); ambient pressure (which sets how high MAP can
 * actually reach) comes from physics/environment.h and is passed in via
 * EngineInput since it changes with altitude during a mission, not a fixed
 * engine parameter. */

#include <stdio.h>

#include "physics/crank_thermo.h"
#include "physics/cylinder.h"
#include "physics/fuel.h"

#define ENGINE_MAX_CYLINDERS 6

/* ENGINE_STOPPED: stalled/off, holding still, no torque of any kind.
 * ENGINE_CRANKING: the starter motor is turning the crank
 * ENGINE_RUNNING: normal self-sustaining combustion */
typedef enum {
  ENGINE_STOPPED = 0,
  ENGINE_CRANKING = 1,
  ENGINE_RUNNING = 2
} EngineRunState;

typedef struct {
  double omega_rad_s; /* crank angular velocity */
  double map_kpa;     /* manifold absolute pressure */
  double theta_deg;   /* crank angle, [0,720) -- see crank_thermo.h */
  double torque_nm;   /* mean total indicated torque */
  EngineRunState run_state;
  int ignition_on;
} EngineState;

typedef struct {
  double throttle;             /* 0.0 (closed) .. 1.0 (wide open) */
  double load_torque_nm;       /* external load (prop, generator, etc), N*m */
  double ambient_pressure_kpa; /* physics/environment.h's environment_isa() */
} EngineInput;

typedef struct {
  double inertia_kg_m2;               /* effective rotating inertia */
  double map_tau_s;                   /* manifold filling time constant, s */
  double friction_coeff_nm_per_rad_s; /* simple viscous friction coefficient */

  /* Engine geometry. num_cylinders is the active count (<=
   * ENGINE_MAX_CYLINDERS); firing_order lists 1-based cylinder numbers in the
   * order they fire, with unused trailing slots left 0. */
  int num_cylinders;
  int firing_order[ENGINE_MAX_CYLINDERS];

  EngineGeometry geom;

  double starter_torque_nm;
  double starter_catch_rpm;
} EngineConfig;

EngineConfig engine_config_default(void);

/* Sets state to a cold-idle starting point */
void engine_model_init(EngineState *state, const EngineConfig *config);

/* Begins starting a STOPPED engine (state->run_state == ENGINE_STOPPED) */
void engine_model_start(EngineState *state, const EngineConfig *config,
                        CylinderState *cyl_states);

/* Deliberate manual shutdown */
void engine_model_stop(EngineState *state);

void engine_model_step(EngineState *state, const EngineConfig *config,
                       const EngineInput *input, const FuelConfig *fuel_cfg,
                       const CylinderConfig *cylinders,
                       CylinderState *cyl_states, double intake_temp_c,
                       double t, double dt);

/* Convenience readouts. engine_model_torque_nm() returns
 * EngineState.torque_nm -- the mean crank-angle-resolved torque from the
 * last engine_model_step() call (real per-cylinder physics, reflecting
 * whatever faults are set on `cylinders`), not a re-derived lumped curve. */
double engine_model_rpm(const EngineState *state);
double engine_model_torque_nm(const EngineState *state);

/* Checks *cfg for physically-implausible values
 * Prints one line per issue found to `out` (pass NULL to check
 * silently). Returns the number of issues found; 0 means the config is
 * clean. */
int engine_config_validate(const EngineConfig *cfg, FILE *out);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_MODEL_H */
