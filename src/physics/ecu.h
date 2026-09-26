#ifndef PHYSICS_ECU_H
#define PHYSICS_ECU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "physics/ecu_diag.h"

/* The engine control unit, kept apart from the engine physics: it knows nothing
 * about the engine model and the engine knows nothing about it. They meet only
 * through signals, wired together in model_sync_step():
 *
 *   pilot command --+
 *                   +--> ECU --> actuator commands --> engine
 *   sensors --------+
 *
 * An engine may have no ECU at all (EngineConfig.ecu_fitted = 0): the pilot's
 * throttle then goes straight to the engine.
 *
 * Idle governor: a PI loop on crank speed that adds throttle, up to
 * idle_max_throttle, to hold idle_target_rpm. The command is max(pilot,
 * governor), so it can only raise RPM. Above its authority the pilot is in
 * control and the integrator is held (not reset); the integrator is clamped for
 * anti-windup.
 *
 * Diagnostics (ecu_diag.h): the ECU reads the engine speed from two sensors and
 * checks them. It runs the governor on the primary while it is clean, on the
 * secondary if only that one is, and otherwise falls back to limp-home: the
 * governor is dropped for a fixed throttle (limp_throttle) and the loop cleared.
 * The operator can switch diagnostics off, and then the ECU trusts the primary
 * blindly. */

typedef struct {
  double idle_target_rpm;   /* 0 disables the governor */
  double idle_kp;           /* throttle per rpm of error */
  double idle_ki;           /* throttle per rpm of error per second */
  double idle_max_throttle; /* governor authority, 0..1 of throttle */
  double limp_throttle;     /* fixed idle throttle in limp-home, 0..1 */
  EcuDiagConfig diag;
} EcuConfig;

/* What the ECU measures */
typedef struct {
  double rpm;  /* primary engine-speed sensor (crank) */
  double rpm2; /* redundant one (alternator-derived) */
  int engine_running; /* running (not stopped or cranking) */
  int ignition_on;
} EcuSensors;

/* A fault on one of the ECU's sensor inputs: what the ECU reads instead of the
 * truth. Injected between the engine and the ECU, so the engine itself is
 * untouched. */
typedef enum {
  ECU_FAULT_NONE = 0,
  ECU_FAULT_OFFSET,  /* reads truth + value */
  ECU_FAULT_SCALE,   /* reads truth * value */
  ECU_FAULT_STUCK,   /* holds the first value it saw after the fault began */
  ECU_FAULT_DROPOUT, /* reads 0 */
  ECU_FAULT_KIND_COUNT
} EcuFaultKind;

typedef struct {
  EcuFaultKind kind;
  double value; /* offset (rpm) or scale factor; unused for STUCK / DROPOUT */
  double held;  /* the STUCK reading */
  int held_valid;
} EcuSensorFault;

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
  ECU_IDLE_PILOT,        /* pilot throttle at/above the authority: stepped aside */
  ECU_IDLE_ACTIVE,       /* regulating */
  ECU_IDLE_LIMITED,      /* full authority in use and still below the target */
  ECU_IDLE_LIMP          /* no trusted speed: fixed limp-home throttle */
} EcuIdleMode;

typedef struct {
  int fitted;        /* an ECU is in the loop (set by the caller) */
  int idle_enabled;  /* operator switch: 1 = governor allowed to act */
  int diag_enabled;  /* operator switch: 1 = diagnostics and fallback on */
  EcuIdleMode idle_mode;
  double time_s;     /* ECU clock: time it has been stepped */

  /* speed inputs */
  double rpm1_seen, rpm2_seen; /* what each sensor read, faults included */
  EcuSpeedSource speed_source; /* which the governor is using */
  double rpm_seen;             /* the speed it uses (0 with no source) */
  EcuDiag diag;

  /* the loop, as of the last step (throttle terms are 0..1) */
  double idle_target_rpm; /* configured target (0 = none) */
  double idle_error_rpm;  /* target - measured, 0 when there is no target */
  double idle_p_term;     /* kp * error */
  double idle_i_term;     /* the integrator */
  double idle_raw;        /* p + i, before the authority clamp */
  double idle_throttle;   /* governor output (the limp throttle in limp) */
  double pilot_throttle;  /* what the pilot asked for */
  double throttle_cmd;    /* max(pilot, governor): what the engine gets */
} EcuState;

EcuConfig ecu_config_default(void);

/* Fitted, governor and diagnostics on, everything else zero. */
void ecu_init(EcuState *e);

/* Clears the governor's loop state, keeping the operator switches. */
void ecu_reset_idle(EcuState *e);

/* Operator switches. The governor's restarts its loop; switching diagnostics
 * off makes the ECU trust the primary sensor blindly again. */
void ecu_set_idle_enabled(EcuState *e, int enabled);
void ecu_set_diag_enabled(EcuState *e, int enabled);

/* Maintenance clear of the fault codes. */
void ecu_clear_codes(EcuState *e);

/* One control step. */
void ecu_step(EcuState *e, const EcuConfig *cfg, const EcuSensors *sensors,
              const EcuPilotCmd *pilot, EcuActuators *out, double dt);

/* No ECU in the loop: the pilot's throttle passes straight through and the
 * state records that. Equivalent to a step of an ECU that does nothing. */
void ecu_bypass(EcuState *e, const EcuPilotCmd *pilot, EcuActuators *out);

/* Sets the fault. Changing the kind restarts it (a STUCK sensor re-latches);
 * changing only the value leaves a running fault in place. */
void ecu_sensor_fault_set(EcuSensorFault *f, EcuFaultKind kind, double value);

/* What the ECU reads for a true value `truth`; never negative. */
double ecu_sensor_fault_apply(EcuSensorFault *f, double truth);

/* Short display names, e.g. "REGULATING", "stuck", "secondary". */
const char *ecu_idle_mode_name(EcuIdleMode mode);
const char *ecu_fault_kind_name(EcuFaultKind kind);
const char *ecu_speed_source_name(EcuSpeedSource source);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ECU_H */
