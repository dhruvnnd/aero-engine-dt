#include "physics/engine_model.h"
#include "physics/engine_config_fields.h"

#include <math.h>

#include "math/units.h"
#include "physics/engine_trace.h"
#include "physics/integrator.h"

enum {
  ENGINE_STATE_OMEGA = 0,
  ENGINE_STATE_MAP = 1,
  ENGINE_STATE_THETA = 2,
  ENGINE_STATE_PRESSURE_BASE = 3,
};
#define ENGINE_STATE_MAX (ENGINE_STATE_PRESSURE_BASE + ENGINE_MAX_CYLINDERS)

#define ENGINE_POLYTROPIC_N 1.3

typedef struct {
  double throttle;
  double load_torque_nm;
  double ambient_pressure_kpa;
  double inertia_kg_m2;
  double map_tau_s;
  double fric_const_nm;         /* friction at any forward speed, N*m */
  double fric_lin_nm_per_rad_s; /* friction growing with crank speed */
  const CylinderConfig *cylinders;
  int num_cylinders;
  const EngineGeometry *geom;
  const FuelConfig *fuel_cfg;
  double intake_temp_c;
  double eff_cr[ENGINE_MAX_CYLINDERS];
  double v_ivc_m3[ENGINE_MAX_CYLINDERS];
  double
      phase_offset_deg[ENGINE_MAX_CYLINDERS]; /* cylinder's cycle lag behind
                                                 the crank, from firing_order */
  int cranking;
  double starter_torque_nm;
  int ignition_on;
} EngineDerivParams;

/* Friction as `const + lin * omega`: the FMEP model turned into torque
 * (T = Vd_total * FMEP / 4pi, FMEP = a + b * mean piston speed, and mean piston
 * speed = stroke * omega / pi) plus the size-independent viscous term. */
static void friction_terms(const EngineConfig *cfg, double *fric_const_nm,
                           double *fric_lin_nm_per_rad_s) {
  const double vd_total_m3 =
      cylinder_displacement_m3(&cfg->geom) * (double)cfg->num_cylinders;
  const double torque_per_kpa = vd_total_m3 * 1000.0 / (4.0 * UNITS_PI);
  *fric_const_nm = torque_per_kpa * cfg->friction_fmep_const_kpa;
  *fric_lin_nm_per_rad_s = torque_per_kpa * cfg->friction_fmep_per_ms_kpa *
                               cfg->geom.stroke_m / UNITS_PI +
                           cfg->friction_coeff_nm_per_rad_s;
}

double engine_friction_torque_nm(const EngineConfig *cfg, double omega_rad_s) {
  if (omega_rad_s <= 0.0) {
    return 0.0;
  }
  double c, l;
  friction_terms(cfg, &c, &l);
  return c + l * omega_rad_s;
}

static double map_target_kpa(double throttle, double ambient_pressure_kpa) {
  /* Closed-throttle MAP as a fixed fraction of ambient (30 kPa at sea level).
   * A fixed vacuum drop below ambient went to zero at altitude, where a closed
   * throttle then left the engine with no air at all and stalled it. */
  const double idle_map_fraction = 0.2963;

  double map_idle_kpa = ambient_pressure_kpa * idle_map_fraction;
  double map_wot_kpa = ambient_pressure_kpa;

  return map_idle_kpa + throttle * (map_wot_kpa - map_idle_kpa);
}

/* Crank torque at the state in `vec`: each cylinder's gas + inertia torque,
 * evaluated at its own local crank angle, plus the starter while cranking.
 * Returns the total; `gas_nm` / `inertia_nm` (each may be NULL) receive the
 * per-cylinder components. */
static double cylinder_torques_from_vec(const double *vec,
                                        const EngineDerivParams *p,
                                        double *gas_nm, double *inertia_nm) {
  double omega = vec[ENGINE_STATE_OMEGA];
  double theta_deg = vec[ENGINE_STATE_THETA];
  double total = 0.0;
  for (int i = 0; i < p->num_cylinders; i++) {
    double pressure_kpa = vec[ENGINE_STATE_PRESSURE_BASE + i];
    double theta_local = theta_deg - p->phase_offset_deg[i];

    CylinderVolumeDeriv vd =
        cylinder_volume_and_deriv(theta_local, p->geom, p->eff_cr[i]);
    CylinderKinematics k = cylinder_kinematics(theta_local, p->geom);
    double accel = omega * omega * k.d2x_drad2;
    double f_inertia = -p->geom->m_recip_kg * accel;

    double gas = kpa_to_pa(pressure_kpa) * vd.dv_drad;
    double inertia = f_inertia * k.dx_drad;
    if (gas_nm) {
      gas_nm[i] = gas;
    }
    if (inertia_nm) {
      inertia_nm[i] = inertia;
    }
    total += gas + inertia;
  }
  if (p->cranking) {
    total += p->starter_torque_nm;
  }
  return total;
}

static void engine_derivative(const double *state, double *dstate, double t,
                              void *user_data) {
  (void)t;
  const EngineDerivParams *p = (const EngineDerivParams *)user_data;

  double omega = state[ENGINE_STATE_OMEGA];
  double map_kpa = state[ENGINE_STATE_MAP];
  double theta_deg = state[ENGINE_STATE_THETA];
  double rpm = rad_s_to_rpm(omega);
  double dtheta_dt_deg_per_s = omega * (180.0 / UNITS_PI);

  double torque_total = 0.0;
  for (int i = 0; i < p->num_cylinders; i++) {
    const CylinderConfig *c = &p->cylinders[i];
    double pressure_kpa = state[ENGINE_STATE_PRESSURE_BASE + i];

    /* Each cylinder sits at its own point in the four-stroke cycle: the crank
     * angle minus its firing-order phase. */
    double theta_local = theta_deg - p->phase_offset_deg[i];
    int closed = cylinder_valve_closed(theta_local, p->geom);

    CylinderVolumeDeriv vd =
        cylinder_volume_and_deriv(theta_local, p->geom, p->eff_cr[i]);
    CylinderKinematics k = cylinder_kinematics(theta_local, p->geom);
    double accel = omega * omega * k.d2x_drad2;
    double f_inertia = -p->geom->m_recip_kg * accel;
    torque_total +=
        kpa_to_pa(pressure_kpa) * vd.dv_drad + f_inertia * k.dx_drad;

    double dp_dtheta;
    if (!closed) {
      /* gas exchange: vent to exhaust pressure (ambient) on the exhaust
       * stroke, to manifold pressure on the intake stroke */
      const double relax_deg = 5.0;
      double target_kpa = cylinder_open_valve_target_kpa(
          theta_local, map_kpa, p->ambient_pressure_kpa);
      dp_dtheta = (target_kpa - pressure_kpa) / relax_deg;
    } else {
      double lambda = cylinder_lambda(c);
      double misfire = p->ignition_on ? misfire_fraction(lambda) : 1.0;

      double base_adv = spark_advance_curve(rpm, map_kpa, p->geom);
      double combined_btdc = base_adv - c->spark_offset_deg;
      double theta_start = fmod(720.0 - combined_btdc + 720.0, 720.0);

      double q_total_j = cylinder_charge_energy_j_with_vivc(
          map_kpa, p->intake_temp_c, p->v_ivc_m3[i], p->geom,
          p->fuel_cfg->afr_stoich, lambda, misfire);
      double dq_ddeg = q_total_j * wiebe_burn_rate_per_deg(
                                       theta_local, theta_start,
                                       p->geom->delta_theta_burn_deg,
                                       p->geom->wiebe_a, p->geom->wiebe_m);

      double p_pa = kpa_to_pa(pressure_kpa);
      double dp_ddeg_pa = -ENGINE_POLYTROPIC_N * (p_pa / vd.v_m3) * vd.dv_ddeg +
                          (ENGINE_POLYTROPIC_N - 1.0) / vd.v_m3 * dq_ddeg;
      dp_dtheta = pa_to_kpa(dp_ddeg_pa);
    }
    dstate[ENGINE_STATE_PRESSURE_BASE + i] = dp_dtheta * dtheta_dt_deg_per_s;
  }

  if (p->cranking) {
    torque_total += p->starter_torque_nm;
  }

  double torque_friction =
      omega > 0.0 ? p->fric_const_nm + p->fric_lin_nm_per_rad_s * omega : 0.0;
  double torque_net = torque_total - torque_friction - p->load_torque_nm;

  dstate[ENGINE_STATE_OMEGA] = torque_net / p->inertia_kg_m2;
  dstate[ENGINE_STATE_MAP] =
      (map_target_kpa(p->throttle, p->ambient_pressure_kpa) - map_kpa) /
      p->map_tau_s;
  dstate[ENGINE_STATE_THETA] = dtheta_dt_deg_per_s;
}

EngineConfig engine_config_default(void) {
  EngineConfig cfg;
  cfg.inertia_kg_m2 = 0.6;
  cfg.map_tau_s = 0.25;
  cfg.friction_coeff_nm_per_rad_s = 0.02;
  cfg.friction_fmep_const_kpa = 40.0;
  cfg.friction_fmep_per_ms_kpa = 14.0;

  /* Inline-four, 1-3-4-2 firing order */
  cfg.num_cylinders = 4;
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg.firing_order[i] = 0;
  }
  cfg.firing_order[0] = 1;
  cfg.firing_order[1] = 3;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 2;

  cfg.ecu_fitted = 1;

  cfg.geom = engine_geometry_default();

  cfg.starter_torque_nm = 25.0;
  cfg.starter_catch_rpm = 800.0;

  cfg.prop = prop_config_default();
  cfg.ecu = ecu_config_default();

  return cfg;
}

void engine_model_init(EngineState *state, const EngineConfig *config) {
  (void)config;
  state->omega_rad_s = rpm_to_rad_s(700.0); /* cold idle guess */
  state->map_kpa = 30.0;                    /* idle vacuum */
  state->theta_deg = 0.0;
  state->torque_nm = 0.0;
  state->run_state = ENGINE_RUNNING;
  state->ignition_on = 1;
  state->trace = NULL;
}

#define ENGINE_STALL_RPM 150.0

void engine_model_start(EngineState *state, const EngineConfig *config,
                        CylinderState *cyl_states) {
  if (state->run_state != ENGINE_STOPPED) {
    return; /* already cranking or running */
  }
  for (int i = 0; i < config->num_cylinders; i++) {
    cyl_states[i].cyl_pressure_kpa = 101.325;
  }
  state->theta_deg = 0.0;
  state->torque_nm = 0.0;
  state->run_state = ENGINE_CRANKING;
  state->ignition_on = 1;
  if (state->trace) {
    engine_trace_clear(state->trace); /* crank angle just jumped back to 0 */
  }
}

void engine_model_stop(EngineState *state) {
  if (state->run_state == ENGINE_CRANKING) {
    state->run_state = ENGINE_STOPPED;
    state->omega_rad_s = 0.0;
    state->torque_nm = 0.0;
    return;
  }
  if (state->run_state == ENGINE_RUNNING) {
    state->ignition_on = 0;
  }
  /* Already ENGINE_STOPPED: no-op. */
}

#define ENGINE_SUB_STEP_DEG 1.0
#define ENGINE_SUB_STEP_MAX_S 0.0005

void engine_model_step(EngineState *state, const EngineConfig *config,
                       const EngineInput *input, const FuelConfig *fuel_cfg,
                       const CylinderConfig *cylinders,
                       CylinderState *cyl_states, double intake_temp_c,
                       double t, double dt) {
  if (state->run_state == ENGINE_STOPPED) {
    state->torque_nm = 0.0;
    /* nothing is pumping the manifold any more: it equalises with ambient, so
     * a restart doesn't begin from the vacuum the stalled engine left behind */
    state->map_kpa = input->ambient_pressure_kpa;
    return;
  }
  int was_cranking = (state->run_state == ENGINE_CRANKING);

  int n_cyl = config->num_cylinders;
  int n_state = ENGINE_STATE_PRESSURE_BASE + n_cyl;

  double vec[ENGINE_STATE_MAX];
  vec[ENGINE_STATE_OMEGA] = state->omega_rad_s;
  vec[ENGINE_STATE_MAP] = state->map_kpa;
  vec[ENGINE_STATE_THETA] = state->theta_deg;
  for (int i = 0; i < n_cyl; i++) {
    vec[ENGINE_STATE_PRESSURE_BASE + i] = cyl_states[i].cyl_pressure_kpa;
  }

  EngineDerivParams params;
  params.throttle = input->throttle;
  params.load_torque_nm = input->load_torque_nm;
  params.ambient_pressure_kpa = input->ambient_pressure_kpa;
  params.inertia_kg_m2 = config->inertia_kg_m2;
  params.map_tau_s = config->map_tau_s;
  friction_terms(config, &params.fric_const_nm, &params.fric_lin_nm_per_rad_s);
  params.cylinders = cylinders;
  params.num_cylinders = n_cyl;
  params.geom = &config->geom;
  params.fuel_cfg = fuel_cfg;
  params.intake_temp_c = intake_temp_c;
  params.cranking = was_cranking;
  params.starter_torque_nm = config->starter_torque_nm;
  params.ignition_on = state->ignition_on;
  engine_cylinder_phase_offsets(config, params.phase_offset_deg);
  for (int i = 0; i < n_cyl; i++) {
    double trim = cylinders[i].compression_trim;
    params.eff_cr[i] =
        config->geom.compression_ratio * (trim > 0.0 ? trim : 0.0);
    params.v_ivc_m3[i] = cylinder_volume_m3(config->geom.ivc_deg, &config->geom,
                                            params.eff_cr[i]);
  }

  const double stall_omega_rad_s =
      was_cranking ? 0.0 : rpm_to_rad_s(ENGINE_STALL_RPM);

  double t_local = t;
  double remaining = dt;
  double torque_accum = 0.0;
  int substeps = 0;
  int stalled = 0;
  while (remaining > 1e-12) {
    double dtheta_dt_deg_per_s =
        fabs(vec[ENGINE_STATE_OMEGA]) * (180.0 / UNITS_PI);
    double dt_sub = dtheta_dt_deg_per_s > 1.0
                        ? ENGINE_SUB_STEP_DEG / dtheta_dt_deg_per_s
                        : ENGINE_SUB_STEP_MAX_S;
    if (dt_sub > ENGINE_SUB_STEP_MAX_S) {
      dt_sub = ENGINE_SUB_STEP_MAX_S;
    }
    double h = remaining < dt_sub ? remaining : dt_sub;
    integrator_rk4_step(vec, n_state, t_local, h, engine_derivative, &params);
    t_local += h;
    remaining -= h;
    if (state->trace) {
      EngineTraceSample s;
      double gas[ENGINE_MAX_CYLINDERS] = {0};
      double inertia[ENGINE_MAX_CYLINDERS] = {0};
      double total = cylinder_torques_from_vec(vec, &params, gas, inertia);
      torque_accum += total;
      s.theta_deg = (float)crank_wrap720_deg(vec[ENGINE_STATE_THETA]);
      s.omega_rad_s = (float)vec[ENGINE_STATE_OMEGA];
      s.torque_nm = (float)total;
      for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
        s.cyl_gas_nm[i] = (float)gas[i];
        s.cyl_inertia_nm[i] = (float)inertia[i];
        s.cyl_pressure_kpa[i] =
            i < n_cyl ? (float)vec[ENGINE_STATE_PRESSURE_BASE + i] : 0.0f;
      }
      engine_trace_push(state->trace, &s);
    } else {
      torque_accum += cylinder_torques_from_vec(vec, &params, NULL, NULL);
    }
    substeps++;

    if (vec[ENGINE_STATE_OMEGA] < stall_omega_rad_s) {
      vec[ENGINE_STATE_OMEGA] = 0.0;
      stalled = 1;
      break;
    }
  }

  state->omega_rad_s = vec[ENGINE_STATE_OMEGA];
  state->map_kpa = vec[ENGINE_STATE_MAP];
  state->theta_deg = crank_wrap720_deg(vec[ENGINE_STATE_THETA]);
  state->torque_nm = substeps > 0 ? torque_accum / substeps : 0.0;
  if (stalled) {
    state->run_state = ENGINE_STOPPED;
  } else if (was_cranking &&
             state->omega_rad_s >= rpm_to_rad_s(config->starter_catch_rpm)) {
    state->run_state = ENGINE_RUNNING;
  }

  for (int i = 0; i < n_cyl; i++) {
    cyl_states[i].cyl_pressure_kpa = vec[ENGINE_STATE_PRESSURE_BASE + i];
  }
}

double engine_model_rpm(const EngineState *state) {
  return rad_s_to_rpm(state->omega_rad_s);
}

double engine_model_torque_nm(const EngineState *state) {
  return state->torque_nm;
}

int engine_config_check(const EngineConfig *cfg,
                        char msgs[][ENGINE_CONFIG_ISSUE_LEN], int max_msgs) {
  int issues = 0;
#define ISSUE(...)                                                             \
  do {                                                                         \
    if (msgs && issues < max_msgs) {                                           \
      snprintf(msgs[issues], ENGINE_CONFIG_ISSUE_LEN, __VA_ARGS__);            \
    }                                                                          \
    issues++;                                                                  \
  } while (0)

  if (cfg->num_cylinders < 1 || cfg->num_cylinders > ENGINE_MAX_CYLINDERS) {
    ISSUE("num_cylinders = %d: must be between 1 and %d", cfg->num_cylinders,
          ENGINE_MAX_CYLINDERS);
  } else {
    /* firing_order[0 .. num_cylinders) must be a permutation of
     * 1..num_cylinders; trailing slots must be 0. */
    int seen[ENGINE_MAX_CYLINDERS + 1] = {0};
    for (int i = 0; i < cfg->num_cylinders; i++) {
      int f = cfg->firing_order[i];
      if (f < 1 || f > cfg->num_cylinders || seen[f]) {
        ISSUE("firing_order[%d] = %d: not a valid entry for a 1..%d "
              "permutation (out of range or repeated)",
              i, f, cfg->num_cylinders);
      } else {
        seen[f] = 1;
      }
    }
    for (int i = cfg->num_cylinders; i < ENGINE_MAX_CYLINDERS; i++) {
      if (cfg->firing_order[i] != 0) {
        ISSUE("firing_order[%d] = %d: slot beyond num_cylinders (%d) should "
              "be 0",
              i, cfg->firing_order[i], cfg->num_cylinders);
      }
    }
  }

  if (cfg->ecu_fitted != 0 && cfg->ecu_fitted != 1) {
    ISSUE("ecu_fitted = %d: must be 0 (no ECU) or 1 (ECU fitted)",
          cfg->ecu_fitted);
  }

  /* per-parameter ranges come from the field table */
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    const double v = *engine_config_field_cptr(cfg, f);
    if (!engine_config_field_in_range(f, v)) {
      char m[ENGINE_CONFIG_ISSUE_LEN];
      engine_config_field_range_message(f, v, m, sizeof m);
      ISSUE("%s", m);
    }
  }

  const EngineGeometry *g = &cfg->geom;
  if (g->conrod_len_m <= g->stroke_m / 2.0) {
    ISSUE("geom: conrod_len_m (%g) must exceed stroke_m/2 (%g) -- "
          "slider-crank geometry can't close otherwise",
          g->conrod_len_m, g->stroke_m / 2.0);
  }
  if (g->evo_deg >= g->ivc_deg) {
    ISSUE("geom: evo_deg (%g) must be less than ivc_deg (%g)", g->evo_deg,
          g->ivc_deg);
  }
#undef ISSUE
  return issues;
}

int engine_config_validate(const EngineConfig *cfg, FILE *out) {
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  const int n = engine_config_check(cfg, msgs, ENGINE_CONFIG_MAX_ISSUES);
  if (out) {
    const int shown =
        n < ENGINE_CONFIG_MAX_ISSUES ? n : ENGINE_CONFIG_MAX_ISSUES;
    for (int i = 0; i < shown; i++) {
      fprintf(out, "%s\n", msgs[i]);
    }
  }
  return n;
}

EngineDerived engine_config_derived(const EngineConfig *cfg) {
  EngineDerived d;
  const EngineGeometry *g = &cfg->geom;
  const double vd = cylinder_displacement_m3(g);
  d.displacement_per_cyl_l = vd * 1000.0;
  d.total_displacement_l = vd * (double)cfg->num_cylinders * 1000.0;
  d.clearance_cc = g->compression_ratio > 1.0
                       ? cylinder_clearance_m3(g, g->compression_ratio) * 1.0e6
                       : 0.0;
  d.firing_interval_deg =
      cfg->num_cylinders > 0 ? 720.0 / (double)cfg->num_cylinders : 0.0;
  d.bore_stroke_ratio = g->stroke_m > 0.0 ? g->bore_m / g->stroke_m : 0.0;
  d.rod_ratio = g->stroke_m > 0.0 ? g->conrod_len_m / g->stroke_m : 0.0;
  d.piston_speed_3000rpm_ms = 2.0 * g->stroke_m * 3000.0 / 60.0;
  d.friction_1000rpm_nm = engine_friction_torque_nm(cfg, rpm_to_rad_s(1000.0));

  const double sea_level_density = 1.225; /* kg/m^3, ISA */
  const double sea_level_sound_ms = 340.3;
  PropState ps;
  prop_step(&ps, &cfg->prop, PROP_REF_RPM, 0.0, sea_level_density);
  d.prop_tip_speed_ms = UNITS_PI * cfg->prop.diameter_m * (PROP_REF_RPM / 60.0);
  d.prop_tip_mach = d.prop_tip_speed_ms / sea_level_sound_ms;
  d.prop_static_torque_nm = ps.torque_nm;
  d.prop_static_thrust_n = ps.thrust_n;
  return d;
}

void engine_cylinder_phase_offsets(const EngineConfig *cfg,
                                   double out[ENGINE_MAX_CYLINDERS]) {
  const int n =
      cfg->num_cylinders < 0
          ? 0
          : (cfg->num_cylinders > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS
                                                       : cfg->num_cylinders);

  int seen[ENGINE_MAX_CYLINDERS + 1] = {0};
  int valid = 1;
  for (int k = 0; k < n; k++) {
    int f = cfg->firing_order[k];
    if (f < 1 || f > n || seen[f]) {
      valid = 0;
    } else {
      seen[f] = 1;
    }
  }

  int order[ENGINE_MAX_CYLINDERS];
  if (valid) {
    for (int k = 0; k < ENGINE_MAX_CYLINDERS; k++) {
      order[k] = cfg->firing_order[k];
    }
  } else {
    engine_default_firing_order(n, order);
  }

  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    out[i] = 0.0;
  }
  for (int k = 0; k < n; k++) {
    out[order[k] - 1] = (double)k * 720.0 / (double)n;
  }
}

void engine_default_firing_order(int num_cylinders,
                                 int out[ENGINE_MAX_CYLINDERS]) {
  /* Common orders: inline-4 1-3-4-2, inline-6 1-5-3-6-2-4; the others are the
   * usual conventions for their cylinder counts. */
  static const int ORDERS[ENGINE_MAX_CYLINDERS + 1][ENGINE_MAX_CYLINDERS] = {
      {0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 2, 0, 0, 0, 0},
      {1, 3, 2, 0, 0, 0}, {1, 3, 4, 2, 0, 0}, {1, 2, 4, 5, 3, 0},
      {1, 5, 3, 6, 2, 4}};
  const int n = num_cylinders < 0 ? 0
                                  : (num_cylinders > ENGINE_MAX_CYLINDERS
                                         ? ENGINE_MAX_CYLINDERS
                                         : num_cylinders);
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    out[i] = ORDERS[n][i];
  }
}
