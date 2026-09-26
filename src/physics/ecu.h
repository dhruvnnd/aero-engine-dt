#ifndef PHYSICS_ECU_H
#define PHYSICS_ECU_H

#ifdef __cplusplus
extern "C" {
#endif

/* The engine control unit */

typedef struct {
  double idle_target_rpm;   /* 0 disables the governor */
  double idle_kp;           /* throttle per rpm of error */
  double idle_ki;           /* throttle per rpm of error per second */
  double idle_max_throttle; /* governor authority, 0..1 of throttle */
} EcuConfig;

/* What the ECU measures */
typedef struct {
  double rpm;
  int engine_running; /* running (not stopped or cranking) */
  int ignition_on;
} EcuSensors;

/* What the pilot / autopilot asks for */
typedef struct {
  double throttle; /* 0..1 */
} EcuPilotCmd;

/* What the ECU drives */
typedef struct {
  double throttle; /* throttle-plate command, 0..1 */
} EcuActuators;

typedef enum {
  ECU_IDLE_DISABLED = 0, /* not configured: target 0, or no authority */
  ECU_IDLE_OFF,          /* switched off by the operator */
  ECU_IDLE_STANDBY,      /* engine not running, or ignition off */
  ECU_IDLE_PILOT,  /* pilot throttle at/above the authority: stepped aside */
  ECU_IDLE_ACTIVE, /* regulating */
  ECU_IDLE_LIMITED /* full authority in use and still below the target */
} EcuIdleMode;

typedef struct {
  int fitted;       /* an ECU is in the loop (set by the caller) */
  int idle_enabled; /* operator switch: 1 = governor allowed to act */
  EcuIdleMode idle_mode;

  /* the loop, as of the last step (throttle terms are 0..1) */
  double idle_target_rpm; /* configured target (0 = none) */
  double idle_error_rpm;  /* target - measured, 0 when there is no target */
  double idle_p_term;     /* kp * error */
  double idle_i_term;     /* the integrator */
  double idle_raw;        /* p + i, before the authority clamp */
  double idle_throttle;   /* governor output, clamped to [0, max_throttle] */
  double pilot_throttle;  /* what the pilot asked for */
  double throttle_cmd;    /* max(pilot, governor): what the engine gets */
} EcuState;

EcuConfig ecu_config_default(void);

/* Fitted, governor on, everything else zero. */
void ecu_init(EcuState *e);

/* Clears the governor's loop state, keeping the operator switch. */
void ecu_reset_idle(EcuState *e);

/* Operator switch. Either way the loop starts fresh. */
void ecu_set_idle_enabled(EcuState *e, int enabled);

/* One control step. */
void ecu_step(EcuState *e, const EcuConfig *cfg, const EcuSensors *sensors,
              const EcuPilotCmd *pilot, EcuActuators *out, double dt);

/* No ECU in the loop: the pilot's throttle passes straight through and the
 * state records that. Equivalent to a step of an ECU that does nothing. */
void ecu_bypass(EcuState *e, const EcuPilotCmd *pilot, EcuActuators *out);

/* Short display name, e.g. "REGULATING". */
const char *ecu_idle_mode_name(EcuIdleMode mode);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ECU_H */
