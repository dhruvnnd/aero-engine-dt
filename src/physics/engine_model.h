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
#include "physics/propeller.h"

#define ENGINE_MAX_CYLINDERS 6

/* ENGINE_STOPPED: stalled/off, holding still, no torque of any kind.
 * ENGINE_CRANKING: the starter motor is turning the crank
 * ENGINE_RUNNING: normal self-sustaining combustion */
typedef enum {
  ENGINE_STOPPED = 0,
  ENGINE_CRANKING = 1,
  ENGINE_RUNNING = 2
} EngineRunState;

typedef struct EngineTrace EngineTrace;

typedef struct {
  double omega_rad_s; /* crank angular velocity */
  double map_kpa;     /* manifold absolute pressure */
  double theta_deg;   /* crank angle, [0,720) -- see crank_thermo.h */
  double torque_nm;   /* mean total indicated torque */
  EngineRunState run_state;
  int ignition_on;

  /* Idle governor */
  double governor_throttle;
  double idle_integral;

  EngineTrace *trace;
} EngineState;

typedef struct {
  double throttle;       /* 0.0 (closed) .. 1.0 (wide open) */
  double load_torque_nm; /* external load on top of the propeller (accessories,
                            a dyno), N*m; model_sync_step() adds the prop's own
                            torque to this */
  double ambient_pressure_kpa; /* physics/environment.h's environment_isa() */
} EngineInput;

typedef struct {
  double inertia_kg_m2; /* effective rotating inertia */
  double map_tau_s;     /* manifold filling time constant, s */
  double friction_coeff_nm_per_rad_s;
  double friction_fmep_const_kpa;
  double friction_fmep_per_ms_kpa;

  /* Idle governor: an ECU-style closed loop that holds idle_target_rpm by
   * adding throttle while the engine runs */
  double idle_target_rpm;
  double idle_kp;           /* throttle per rpm of error */
  double idle_ki;           /* throttle per rpm of error per second */
  double idle_max_throttle; /* governor authority, 0..1 of throttle */

  /* Engine geometry. num_cylinders is the active count (<=
   * ENGINE_MAX_CYLINDERS); firing_order lists 1-based cylinder numbers in the
   * order they fire, with unused trailing slots left 0. */
  int num_cylinders;
  int firing_order[ENGINE_MAX_CYLINDERS];

  EngineGeometry geom;

  double starter_torque_nm;
  double starter_catch_rpm;

  PropConfig prop; /* direct-drive fixed-pitch propeller, see propeller.h */
} EngineConfig;

EngineConfig engine_config_default(void);

/* Sets state to a cold-idle starting point */
void engine_model_init(EngineState *state, const EngineConfig *config);

/* Begins starting a STOPPED engine (state->run_state == ENGINE_STOPPED) */
void engine_model_start(EngineState *state, const EngineConfig *config,
                        CylinderState *cyl_states);

/* Deliberate manual shutdown */
void engine_model_stop(EngineState *state);

/*  `dt` is a frame time; the integration is sub-stepped internally (~1 deg of
 * crank rotation each). Cylinder i runs its cycle offset from the crank by
 * engine_cylinder_phase_offsets(), so cylinders fire in firing_order,
 * 720/num_cylinders degrees apart. */
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

/* Total friction torque at crank speed omega_rad_s (0 at or below rest) */
double engine_friction_torque_nm(const EngineConfig *cfg, double omega_rad_s);
double engine_model_torque_nm(const EngineState *state);

/* Checks *cfg for physically-implausible values
 * Prints one line per issue found to `out` (pass NULL to check
 * silently). Returns the number of issues found; 0 means the config is
 * clean. */
int engine_config_validate(const EngineConfig *cfg, FILE *out);

/* Same checks as engine_config_validate(), but the messages come back in
 * `msgs` (up to `max_msgs`, each at most ENGINE_CONFIG_ISSUE_LEN chars, no
 * trailing newline) instead of being printed. Returns the total number of
 * issues, which can exceed `max_msgs`. `msgs` may be NULL to just count. */
#define ENGINE_CONFIG_MAX_ISSUES 32
#define ENGINE_CONFIG_ISSUE_LEN 160
int engine_config_check(const EngineConfig *cfg,
                        char msgs[][ENGINE_CONFIG_ISSUE_LEN], int max_msgs);

/* Figures worth showing next to a config; safe on a not-yet-valid config. */
typedef struct {
  double displacement_per_cyl_l;
  double total_displacement_l;
  double clearance_cc;        /* per cylinder */
  double firing_interval_deg; /* 720 / num_cylinders */
  double bore_stroke_ratio;
  double rod_ratio;               /* conrod length / stroke */
  double piston_speed_3000rpm_ms; /* mean piston speed */
  double friction_1000rpm_nm;     /* total friction torque at 1000 rpm */

  /* sea-level ISA, still air, at PROP_REF_RPM */
  double prop_tip_speed_ms;
  double prop_tip_mach;
  double prop_static_torque_nm;
  double prop_static_thrust_n;
} EngineDerived;
#define PROP_REF_RPM 2500.0
EngineDerived engine_config_derived(const EngineConfig *cfg);

/* A conventional firing order for 1..ENGINE_MAX_CYLINDERS cylinders (e.g.
 * 1-3-4-2 for four), zero-filled beyond `num_cylinders`. Out-of-range counts
 * are clamped. */
void engine_default_firing_order(int num_cylinders,
                                 int out[ENGINE_MAX_CYLINDERS]);

/* Crank-angle phase of each cylinder's cycle relative to the crank, from
 * firing_order */
void engine_cylinder_phase_offsets(const EngineConfig *cfg,
                                   double out[ENGINE_MAX_CYLINDERS]);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_MODEL_H */
