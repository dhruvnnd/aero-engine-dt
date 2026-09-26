#include "physics/ecu.h"

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void ecu_init(EcuState *e) {
  e->idle_enabled = 1;
  e->idle_mode = ECU_IDLE_STANDBY;
  ecu_reset_idle(e);
  e->idle_target_rpm = 0.0;
  e->pilot_throttle = 0.0;
  e->throttle_cmd = 0.0;
}

void ecu_reset_idle(EcuState *e) {
  e->idle_error_rpm = 0.0;
  e->idle_p_term = 0.0;
  e->idle_i_term = 0.0;
  e->idle_raw = 0.0;
  e->idle_throttle = 0.0;
}

void ecu_set_idle_enabled(EcuState *e, int enabled) {
  e->idle_enabled = enabled ? 1 : 0;
  ecu_reset_idle(e);
}

double ecu_step(EcuState *e, const EcuIdleParams *p, int engine_running,
                int ignition_on, double rpm, double pilot_throttle, double dt) {
  const double u_max = p->max_throttle;
  e->pilot_throttle = pilot_throttle;
  e->idle_target_rpm = p->target_rpm > 0.0 ? p->target_rpm : 0.0;

  if (p->target_rpm <= 0.0 || u_max <= 0.0) {
    e->idle_mode = ECU_IDLE_DISABLED;
    ecu_reset_idle(e);
  } else if (!e->idle_enabled) {
    e->idle_mode = ECU_IDLE_OFF;
    ecu_reset_idle(e);
    e->idle_error_rpm = p->target_rpm - rpm;
  } else if (!engine_running || !ignition_on) {
    e->idle_mode = ECU_IDLE_STANDBY;
    ecu_reset_idle(e);
    e->idle_error_rpm = p->target_rpm - rpm;
  } else {
    const double error_rpm = p->target_rpm - rpm;
    if (pilot_throttle < u_max) {
      e->idle_i_term = clamp(e->idle_i_term + p->ki * error_rpm * dt, 0.0, u_max);
    }
    e->idle_error_rpm = error_rpm;
    e->idle_p_term = p->kp * error_rpm;
    e->idle_raw = e->idle_p_term + e->idle_i_term;
    e->idle_throttle = clamp(e->idle_raw, 0.0, u_max);

    if (pilot_throttle >= u_max) {
      e->idle_mode = ECU_IDLE_PILOT;
    } else if (e->idle_raw >= u_max && rpm < p->target_rpm) {
      e->idle_mode = ECU_IDLE_LIMITED;
    } else {
      e->idle_mode = ECU_IDLE_ACTIVE;
    }
  }

  e->throttle_cmd =
      pilot_throttle > e->idle_throttle ? pilot_throttle : e->idle_throttle;
  return e->throttle_cmd;
}

const char *ecu_idle_mode_name(EcuIdleMode mode) {
  switch (mode) {
  case ECU_IDLE_DISABLED:
    return "DISABLED";
  case ECU_IDLE_OFF:
    return "OFF";
  case ECU_IDLE_STANDBY:
    return "STANDBY";
  case ECU_IDLE_PILOT:
    return "PILOT IN CONTROL";
  case ECU_IDLE_ACTIVE:
    return "REGULATING";
  case ECU_IDLE_LIMITED:
    return "AT AUTHORITY LIMIT";
  }
  return "?";
}
