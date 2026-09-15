#ifndef PHYSICS_ENGINE_MODEL_H
#define PHYSICS_ENGINE_MODEL_H

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

/* Plausible placeholder parameters for a small aero piston engine, to use
 * until real performance-map data replaces them (data/perf_maps.c, later
 * phase). Not sourced from a specific engine's spec sheet -- tune these
 * once you have real numbers to match against. */
EngineConfig engine_config_default(void);

/* Sets state to a cold-idle starting point, engine already ENGINE_RUNNING
 * (rotation; thermal-adjacent fields the caller owns elsewhere are
 * untouched -- this only sets the EngineState fields above). This is the
 * "app just booted, pretend the engine is already up" convenience used by
 * model_state_init() and most tests -- it does NOT model a start sequence.
 * For that, from ENGINE_STOPPED, use engine_model_start() instead. */
void engine_model_init(EngineState *state, const EngineConfig *config);

/* Begins starting a STOPPED engine (state->run_state == ENGINE_STOPPED):
 * clears every active cylinder's pressure to a sane ambient-ish baseline and
 * moves to ENGINE_CRANKING. Deliberately does NOT touch omega_rad_s/theta_deg
 * or anything outside EngineState/CylinderState.cyl_pressure_kpa (in
 * particular thermal/fuel/electrical state stays exactly as it was -- a
 * stall doesn't cool the engine down or drain the battery) -- crank speed
 * builds up from wherever it already is (normally ~0, at rest) via the
 * starter torque applied in engine_model_step() while ENGINE_CRANKING,
 * exactly like a real starter motor spinning up a stopped engine. A no-op if
 * the engine isn't ENGINE_STOPPED (mirrors a real ignition switch: cranking
 * an already-running engine does nothing). `cyl_states` is an array of at
 * least config->num_cylinders entries, same as engine_model_step(). Whether
 * a start attempt should even be allowed (e.g. battery too weak) is the
 * caller's call -- this function doesn't know about ElecState. */
void engine_model_start(EngineState *state, const EngineConfig *config,
                        CylinderState *cyl_states);

/* Deliberate manual shutdown -- an ignition-off key turn, distinct from an
 * involuntary stall. If ENGINE_RUNNING: cuts ignition (state->ignition_on =
 * 0), which forces every cylinder to full misfire in engine_derivative() --
 * no more combustion energy is added, so the engine coasts down under
 * friction and load exactly like a stall does, and engine_model_step()'s
 * existing stall-floor logic transitions it to ENGINE_STOPPED once speed
 * decays below ENGINE_STALL_RPM. Deliberately does NOT teleport straight to
 * ENGINE_STOPPED -- a real engine keeps spinning for a moment after key-off.
 * If ENGINE_CRANKING: the starter is on the same ignition switch, so this
 * aborts the crank immediately back to ENGINE_STOPPED (no coast-down --
 * there's no momentum to speak of yet). A no-op if already ENGINE_STOPPED. */
void engine_model_stop(EngineState *state);

/* Advances state by dt seconds under the given input, using RK4. `dt` is a
 * *frame* time (whatever the caller's render/tick interval is) -- internally
 * this sub-steps at a fixed, much finer resolution so crank-angle-scale
 * dynamics integrate accurately regardless of frame rate; that sub-stepping
 * is purely an implementation detail and callers don't need to do anything
 * differently. Each active cylinder (config->num_cylinders of them) gets its
 * own crank-angle-resolved pressure trace (physics/crank_thermo.h), sharing
 * the crank's theta_deg -- true per-cylinder phase staggering is Phase 2.
 * `cylinders` (trims) and `cyl_states` (persistent per-cylinder state,
 * notably cyl_pressure_kpa) are each arrays of at least config->num_cylinders
 * entries. `fuel_cfg` supplies AFR for the combustion-energy calculation.
 * `intake_temp_c` is the charge temperature used for the trapped-mass
 * calculation (ambient temperature is the caller's usual proxy for this,
 * same as physics/fuel.h's air_flow_gps()). `t` is the current sim time, s.
 *
 * State machine: if `state->run_state` is ENGINE_STOPPED, this is a no-op
 * other than zeroing torque_nm -- call engine_model_start() to begin
 * cranking. If ENGINE_CRANKING, `config->starter_torque_nm` is added to the
 * torque balance on top of whatever combustion is doing (both are active
 * simultaneously -- a real starter doesn't wait for combustion to be silent
 * either), and once crank speed exceeds `config->starter_catch_rpm` the
 * state moves to ENGINE_RUNNING and the starter torque stops applying. If
 * ENGINE_RUNNING, behaves as a normal combustion-only engine. In any active
 * state, crank speed is not allowed to cross zero into reverse rotation: the
 * combustion model assumes forward rotation only, and nothing in it was
 * designed to behave sensibly run backward. Once speed would drop below a
 * small stall threshold, the engine is marked ENGINE_STOPPED instead
 * (everything holds at rest) rather than continuing to integrate -- this
 * applies during cranking too, e.g. if starter_torque_nm can't overcome the
 * load.
 */
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

#endif /* PHYSICS_ENGINE_MODEL_H */
