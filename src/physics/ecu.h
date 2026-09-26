#ifndef PHYSICS_ECU_H
#define PHYSICS_ECU_H

#ifdef __cplusplus
extern "C" {
#endif

/* The engine control unit's logic and state */

typedef enum {
  ECU_IDLE_DISABLED = 0, /* not configured: target 0, or no authority */
  ECU_IDLE_OFF,          /* switched off by the operator */
  ECU_IDLE_STANDBY,      /* engine not running, or ignition off */
  ECU_IDLE_PILOT,  /* pilot throttle at/above the authority: stepped aside */
  ECU_IDLE_ACTIVE, /* regulating */
  ECU_IDLE_LIMITED /* full authority in use and still below the target */
} EcuIdleMode;

typedef struct {
  double target_rpm; /* 0 disables the governor */
  double kp;         /* throttle per rpm of error */
  double ki;         /* throttle per rpm of error per second */
  double max_throttle;
} EcuIdleParams;

typedef struct {
  int idle_enabled; /* operator switch: 1 = governor allowed to act */
  EcuIdleMode idle_mode;

  /* the loop, as of the last ecu_step() (all throttle terms are 0..1) */
  double idle_target_rpm; /* configured target (0 = none) */
  double idle_error_rpm;  /* target - measured, 0 when there is no target */
  double idle_p_term;     /* kp * error */
  double idle_i_term;     /* the integrator */
  double idle_raw;        /* p + i, before the authority clamp */
  double idle_throttle;   /* governor output, clamped to [0, max_throttle] */
  double pilot_throttle;  /* what the pilot asked for */
  double throttle_cmd;    /* max(pilot, governor): what the engine gets */
} EcuState;

/* Governor on, everything else zero. */
void ecu_init(EcuState *e);

/* Clears the governor's loop state (integrator and terms), keeping the operator
 * switch: an engine start begins from a clean loop. */
void ecu_reset_idle(EcuState *e);

/* Operator switch. Either way the loop starts fresh. */
void ecu_set_idle_enabled(EcuState *e, int enabled);

/* One control step. Returns the throttle command (also stored in
 * e->throttle_cmd). `engine_running` = the engine is in its running state (not
 * stopped or cranking) with `ignition_on`. */
double ecu_step(EcuState *e, const EcuIdleParams *p, int engine_running,
                int ignition_on, double rpm, double pilot_throttle, double dt);

/* Short display name, e.g. "REGULATING". */
const char *ecu_idle_mode_name(EcuIdleMode mode);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ECU_H */
