#include "physics/ecu.h"

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

EcuConfig ecu_config_default(void) {
  /* Not tuned against a real ECU: enough authority to hold idle against the
   * prop and accessory load at sea level, gentle enough not to hunt. */
  EcuConfig c;
  c.idle_target_rpm = 800.0;
  c.idle_kp = 0.0003;
  c.idle_ki = 0.0003;
  c.idle_max_throttle = 0.15;
  return c;
}

void ecu_init(EcuState *e) {
  e->fitted = 1;
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

void ecu_step(EcuState *e, const EcuConfig *cfg, const EcuSensors *sensors,
              const EcuPilotCmd *pilot, EcuActuators *out, double dt) {
  const double u_max = cfg->idle_max_throttle;
  const double rpm = sensors->rpm;
  const double pilot_throttle = pilot->throttle;
  e->fitted = 1;
  e->pilot_throttle = pilot_throttle;
  e->idle_target_rpm = cfg->idle_target_rpm > 0.0 ? cfg->idle_target_rpm : 0.0;

  if (cfg->idle_target_rpm <= 0.0 || u_max <= 0.0) {
    e->idle_mode = ECU_IDLE_DISABLED;
    ecu_reset_idle(e);
  } else if (!e->idle_enabled) {
    e->idle_mode = ECU_IDLE_OFF;
    ecu_reset_idle(e);
    e->idle_error_rpm = cfg->idle_target_rpm - rpm;
  } else if (!sensors->engine_running || !sensors->ignition_on) {
    e->idle_mode = ECU_IDLE_STANDBY;
    ecu_reset_idle(e);
    e->idle_error_rpm = cfg->idle_target_rpm - rpm;
  } else {
    const double error_rpm = cfg->idle_target_rpm - rpm;
    if (pilot_throttle < u_max) {
      e->idle_i_term =
          clamp(e->idle_i_term + cfg->idle_ki * error_rpm * dt, 0.0, u_max);
    }
    e->idle_error_rpm = error_rpm;
    e->idle_p_term = cfg->idle_kp * error_rpm;
    e->idle_raw = e->idle_p_term + e->idle_i_term;
    e->idle_throttle = clamp(e->idle_raw, 0.0, u_max);

    if (pilot_throttle >= u_max) {
      e->idle_mode = ECU_IDLE_PILOT;
    } else if (e->idle_raw >= u_max && rpm < cfg->idle_target_rpm) {
      e->idle_mode = ECU_IDLE_LIMITED;
    } else {
      e->idle_mode = ECU_IDLE_ACTIVE;
    }
  }

  e->throttle_cmd =
      pilot_throttle > e->idle_throttle ? pilot_throttle : e->idle_throttle;
  out->throttle = e->throttle_cmd;
}

void ecu_bypass(EcuState *e, const EcuPilotCmd *pilot, EcuActuators *out) {
  e->fitted = 0;
  e->idle_mode = ECU_IDLE_DISABLED;
  e->idle_target_rpm = 0.0;
  ecu_reset_idle(e);
  e->pilot_throttle = pilot->throttle;
  e->throttle_cmd = pilot->throttle;
  out->throttle = pilot->throttle;
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
