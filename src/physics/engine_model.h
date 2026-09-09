#ifndef PHYSICS_ENGINE_MODEL_H
#define PHYSICS_ENGINE_MODEL_H

/* Minimal lumped-parameter model of a piston engine's rotating and intake
 * dynamics: crank angular velocity and manifold pressure, both carried as
 * ODE state. Indicated torque comes from physics/combustion.h; ambient
 * pressure (which sets how high MAP can actually reach) comes from
 * physics/environment.h and is passed in via EngineInput since it changes
 * with altitude during a mission, not a fixed engine parameter. */

#define ENGINE_MAX_CYLINDERS 6

typedef struct {
  double omega_rad_s; /* crank angular velocity */
  double map_kpa;     /* manifold absolute pressure */
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
} EngineConfig;

/* Plausible placeholder parameters for a small aero piston engine, to use
 * until real performance-map data replaces them (data/perf_maps.c, later
 * phase). Not sourced from a specific engine's spec sheet -- tune these
 * once you have real numbers to match against. */
EngineConfig engine_config_default(void);

/* Sets state to a cold-idle starting point. */
void engine_model_init(EngineState *state, const EngineConfig *config);

/* Advances state by dt seconds under the given input, using RK4
 * t is the current simulation time in seconds
 */
void engine_model_step(EngineState *state, const EngineConfig *config,
                       const EngineInput *input, double t, double dt);

/* Convenience readouts. */
double engine_model_rpm(const EngineState *state);
double engine_model_torque_nm(const EngineState *state);

#endif /* PHYSICS_ENGINE_MODEL_H */
