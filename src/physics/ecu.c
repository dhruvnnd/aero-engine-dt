#include "physics/ecu.h"

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

EcuConfig ecu_config_default(void) {
  /* Not tuned against a real ECU: enough authority to hold idle against the
   * prop and accessory load at sea level, gentle enough not to hunt. The limp
   * throttle is about what a healthy 4-cylinder needs to idle. */
  EcuConfig c;
  c.idle_target_rpm = 800.0;
  c.idle_kp = 0.0003;
  c.idle_ki = 0.0003;
  c.idle_max_throttle = 0.15;
  c.limp_throttle = 0.07;
  c.diag = ecu_diag_config_default();
  return c;
}

void ecu_init(EcuState *e) {
  e->fitted = 1;
  e->idle_enabled = 1;
  e->diag_enabled = 1;
  e->idle_mode = ECU_IDLE_STANDBY;
  e->time_s = 0.0;
  ecu_reset_idle(e);
  e->rpm1_seen = e->rpm2_seen = e->rpm_seen = 0.0;
  e->speed_source = ECU_SRC_PRIMARY;
  ecu_diag_init(&e->diag);
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

void ecu_set_diag_enabled(EcuState *e, int enabled) {
  e->diag_enabled = enabled ? 1 : 0;
  ecu_reset_idle(e);
  if (!e->diag_enabled) {
    e->speed_source = ECU_SRC_PRIMARY; /* blind trust in the primary */
  }
}

void ecu_clear_codes(EcuState *e) { ecu_diag_clear_codes(&e->diag); }

void ecu_step(EcuState *e, const EcuConfig *cfg, const EcuSensors *sensors,
              const EcuPilotCmd *pilot, EcuActuators *out, double dt) {
  const double u_max = cfg->idle_max_throttle;
  const double pilot_throttle = pilot->throttle;
  const int running = sensors->engine_running && sensors->ignition_on;
  e->fitted = 1;
  e->time_s += dt;
  e->pilot_throttle = pilot_throttle;
  e->rpm1_seen = sensors->rpm;
  e->rpm2_seen = sensors->rpm2;
  e->idle_target_rpm = cfg->idle_target_rpm > 0.0 ? cfg->idle_target_rpm : 0.0;

  /* which speed reading to act on */
  if (e->diag_enabled) {
    EcuDiagInput in;
    in.now_s = e->time_s;
    in.dt = dt;
    in.rpm[0] = sensors->rpm;
    in.rpm[1] = sensors->rpm2;
    in.running = running;
    in.pilot_throttle = pilot_throttle;
    in.throttle_cmd = e->throttle_cmd; /* the previous step's */
    EcuDiagResult r;
    ecu_diag_step(&e->diag, &cfg->diag, &in, &r);
    e->speed_source = r.source;
  } else {
    e->speed_source = ECU_SRC_PRIMARY;
  }
  const double rpm = e->speed_source == ECU_SRC_PRIMARY     ? sensors->rpm
                     : e->speed_source == ECU_SRC_SECONDARY ? sensors->rpm2
                                                            : 0.0;
  e->rpm_seen = rpm;
  const int limp = e->diag_enabled && e->speed_source == ECU_SRC_NONE;

  if (cfg->idle_target_rpm <= 0.0 || u_max <= 0.0) {
    e->idle_mode = ECU_IDLE_DISABLED;
    ecu_reset_idle(e);
  } else if (!e->idle_enabled) {
    e->idle_mode = ECU_IDLE_OFF;
    ecu_reset_idle(e);
    e->idle_error_rpm = cfg->idle_target_rpm - rpm;
  } else if (!running) {
    e->idle_mode = ECU_IDLE_STANDBY;
    ecu_reset_idle(e);
    e->idle_error_rpm = cfg->idle_target_rpm - rpm;
  } else if (limp) {
    /* no trusted speed: drop the loop for a fixed, safe idle throttle */
    e->idle_mode = ECU_IDLE_LIMP;
    ecu_reset_idle(e);
    e->idle_throttle = clamp(cfg->limp_throttle, 0.0, 1.0);
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
  e->rpm1_seen = e->rpm2_seen = e->rpm_seen = 0.0;
  e->speed_source = ECU_SRC_NONE;
  e->idle_target_rpm = 0.0;
  ecu_reset_idle(e);
  e->pilot_throttle = pilot->throttle;
  e->throttle_cmd = pilot->throttle;
  out->throttle = pilot->throttle;
}

void ecu_sensor_fault_set(EcuSensorFault *f, EcuFaultKind kind, double value) {
  if (kind != f->kind) {
    f->held_valid = 0;
  }
  f->kind = kind;
  f->value = value;
}

double ecu_sensor_fault_apply(EcuSensorFault *f, double truth) {
  double v = truth;
  switch (f->kind) {
  case ECU_FAULT_OFFSET:
    v = truth + f->value;
    break;
  case ECU_FAULT_SCALE:
    v = truth * f->value;
    break;
  case ECU_FAULT_STUCK:
    if (!f->held_valid) {
      f->held = truth;
      f->held_valid = 1;
    }
    v = f->held;
    break;
  case ECU_FAULT_DROPOUT:
    v = 0.0;
    break;
  case ECU_FAULT_NONE:
  case ECU_FAULT_KIND_COUNT:
    break;
  }
  return v < 0.0 ? 0.0 : v;
}

const char *ecu_fault_kind_name(EcuFaultKind kind) {
  switch (kind) {
  case ECU_FAULT_NONE:
    return "none";
  case ECU_FAULT_OFFSET:
    return "offset";
  case ECU_FAULT_SCALE:
    return "scale";
  case ECU_FAULT_STUCK:
    return "stuck";
  case ECU_FAULT_DROPOUT:
    return "dropout";
  case ECU_FAULT_KIND_COUNT:
    break;
  }
  return "?";
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
  case ECU_IDLE_LIMP:
    return "LIMP HOME";
  }
  return "?";
}

const char *ecu_speed_source_name(EcuSpeedSource source) {
  switch (source) {
  case ECU_SRC_NONE:
    return "none";
  case ECU_SRC_PRIMARY:
    return "crank";
  case ECU_SRC_SECONDARY:
    return "alternator";
  }
  return "?";
}
