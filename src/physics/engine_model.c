#include "physics/engine_model.h"

#include <math.h>

#include "math/units.h"
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
  double friction_coeff_nm_per_rad_s;
  const CylinderConfig *cylinders;
  int num_cylinders;
  const EngineGeometry *geom;
  const FuelConfig *fuel_cfg;
  double intake_temp_c;
  double eff_cr[ENGINE_MAX_CYLINDERS];
  double v_ivc_m3[ENGINE_MAX_CYLINDERS];
  int cranking;
  double starter_torque_nm;
  int ignition_on;
} EngineDerivParams;

static double map_target_kpa(double throttle, double ambient_pressure_kpa) {
  const double idle_vacuum_drop_kpa = 71.3; /* below ambient, closed throttle */

  double map_idle_kpa = ambient_pressure_kpa - idle_vacuum_drop_kpa;
  if (map_idle_kpa < 0.0) {
    map_idle_kpa = 0.0;
  }
  double map_wot_kpa = ambient_pressure_kpa;

  return map_idle_kpa + throttle * (map_wot_kpa - map_idle_kpa);
}

static double cylinders_total_torque_from_vec(const double *vec,
                                              const EngineDerivParams *p) {
  double omega = vec[ENGINE_STATE_OMEGA];
  double theta_deg = vec[ENGINE_STATE_THETA];
  double total = 0.0;
  for (int i = 0; i < p->num_cylinders; i++) {
    double pressure_kpa = vec[ENGINE_STATE_PRESSURE_BASE + i];

    CylinderVolumeDeriv vd =
        cylinder_volume_and_deriv(theta_deg, p->geom, p->eff_cr[i]);
    CylinderKinematics k = cylinder_kinematics(theta_deg, p->geom);
    double accel = omega * omega * k.d2x_drad2;
    double f_inertia = -p->geom->m_recip_kg * accel;

    total += kpa_to_pa(pressure_kpa) * vd.dv_drad + f_inertia * k.dx_drad;
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
  int closed = cylinder_valve_closed(theta_deg, p->geom);

  double torque_total = 0.0;
  for (int i = 0; i < p->num_cylinders; i++) {
    const CylinderConfig *c = &p->cylinders[i];
    double pressure_kpa = state[ENGINE_STATE_PRESSURE_BASE + i];

    CylinderVolumeDeriv vd =
        cylinder_volume_and_deriv(theta_deg, p->geom, p->eff_cr[i]);
    CylinderKinematics k = cylinder_kinematics(theta_deg, p->geom);
    double accel = omega * omega * k.d2x_drad2;
    double f_inertia = -p->geom->m_recip_kg * accel;
    torque_total +=
        kpa_to_pa(pressure_kpa) * vd.dv_drad + f_inertia * k.dx_drad;

    double dp_dtheta;
    if (!closed) {
      const double relax_deg = 5.0;
      dp_dtheta = (map_kpa - pressure_kpa) / relax_deg;
    } else {
      double lambda = cylinder_lambda(c);
      double misfire = p->ignition_on ? misfire_fraction(lambda) : 1.0;

      double base_adv = spark_advance_curve(rpm, map_kpa, p->geom);
      double combined_btdc = base_adv - c->spark_offset_deg;
      double theta_start = fmod(720.0 - combined_btdc + 720.0, 720.0);

      double q_total_j = cylinder_charge_energy_j_with_vivc(
          map_kpa, p->intake_temp_c, p->v_ivc_m3[i], p->geom,
          p->fuel_cfg->afr_stoich, lambda, misfire);
      double dq_ddeg =
          q_total_j * wiebe_burn_rate_per_deg(
                          theta_deg, theta_start, p->geom->delta_theta_burn_deg,
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

  double torque_friction = p->friction_coeff_nm_per_rad_s * omega;
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
  cfg.friction_coeff_nm_per_rad_s = 0.12;

  /* Inline-four, 1-3-4-2 firing order */
  cfg.num_cylinders = 4;
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg.firing_order[i] = 0;
  }
  cfg.firing_order[0] = 1;
  cfg.firing_order[1] = 3;
  cfg.firing_order[2] = 4;
  cfg.firing_order[3] = 2;

  cfg.geom = engine_geometry_default();

  cfg.starter_torque_nm = 25.0;
  cfg.starter_catch_rpm = 800.0;

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
  params.friction_coeff_nm_per_rad_s = config->friction_coeff_nm_per_rad_s;
  params.cylinders = cylinders;
  params.num_cylinders = n_cyl;
  params.geom = &config->geom;
  params.fuel_cfg = fuel_cfg;
  params.intake_temp_c = intake_temp_c;
  params.cranking = was_cranking;
  params.starter_torque_nm = config->starter_torque_nm;
  params.ignition_on = state->ignition_on;
  for (int i = 0; i < n_cyl; i++) {
    double trim = cylinders[i].compression_trim;
    params.eff_cr[i] =
        config->geom.compression_ratio * (trim > 0.0 ? trim : 0.0);
    params.v_ivc_m3[i] = cylinder_volume_m3(config->geom.ivc_deg, &config->geom,
                                            params.eff_cr[i]);
  }

  /* While cranking, the engine legitimately starts at/near 0 RPM and climbs
   * gradually as the starter (and, once combustion contributes, the
   * cylinders) spin it up -- it has no business being anywhere near
   * ENGINE_STALL_RPM yet, so that floor doesn't apply. The only thing that
   * actually needs guarding against here is the same reverse-rotation
   * failure mode as running (e.g. the starter genuinely can't overcome the
   * load), so the floor while cranking is just "don't go negative." */
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
    torque_accum += cylinders_total_torque_from_vec(vec, &params);
    substeps++;

    /* Crank speed dropping below the stall threshold (or, from a bad
     * transient, all the way through zero into reverse) stops the engine
     * here rather than letting it continue: the combustion model assumes
     * forward rotation only, and if speed keeps dropping unchecked, the
     * angle-based sub-step size above (inversely proportional to |omega|)
     * would keep shrinking as speed swings more negative, making this loop
     * take pathologically many iterations per frame -- this is what froze
     * the whole program before this check existed. */
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
    /* Caught: combustion has carried speed past the catch threshold, so the
     * starter disengages -- checked once per frame (not mid-sub-step-loop)
     * for simplicity; being off by a fraction of a frame doesn't matter
     * physically. */
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

int engine_config_validate(const EngineConfig *cfg, FILE *out) {
  int issues = 0;

  if (cfg->num_cylinders < 1 || cfg->num_cylinders > ENGINE_MAX_CYLINDERS) {
    if (out) {
      fprintf(out, "num_cylinders = %d: must be between 1 and %d\n",
              cfg->num_cylinders, ENGINE_MAX_CYLINDERS);
    }
    issues++;
  } else {
    /* firing_order[0 .. num_cylinders) must be a permutation of
     * 1..num_cylinders; trailing slots must be 0. */
    int seen[ENGINE_MAX_CYLINDERS + 1] = {0};
    for (int i = 0; i < cfg->num_cylinders; i++) {
      int f = cfg->firing_order[i];
      if (f < 1 || f > cfg->num_cylinders || seen[f]) {
        if (out) {
          fprintf(out,
                  "firing_order[%d] = %d: not a valid entry for a "
                  "1..%d permutation (out of range or repeated)\n",
                  i, f, cfg->num_cylinders);
        }
        issues++;
      } else {
        seen[f] = 1;
      }
    }
    for (int i = cfg->num_cylinders; i < ENGINE_MAX_CYLINDERS; i++) {
      if (cfg->firing_order[i] != 0) {
        if (out) {
          fprintf(out,
                  "firing_order[%d] = %d: slot beyond num_cylinders (%d) "
                  "should be 0\n",
                  i, cfg->firing_order[i], cfg->num_cylinders);
        }
        issues++;
      }
    }
  }

  if (cfg->inertia_kg_m2 <= 0.0) {
    if (out) {
      fprintf(out, "inertia_kg_m2 = %g: must be positive\n",
              cfg->inertia_kg_m2);
    }
    issues++;
  }
  if (cfg->map_tau_s <= 0.0) {
    if (out) {
      fprintf(out, "map_tau_s = %g: must be positive\n", cfg->map_tau_s);
    }
    issues++;
  }
  if (cfg->friction_coeff_nm_per_rad_s <= 0.0) {
    if (out) {
      fprintf(out, "friction_coeff_nm_per_rad_s = %g: must be positive\n",
              cfg->friction_coeff_nm_per_rad_s);
    }
    issues++;
  }

  const EngineGeometry *g = &cfg->geom;
  if (g->bore_m <= 0.0 || g->stroke_m <= 0.0) {
    if (out) {
      fprintf(out, "geom: bore_m/stroke_m must be positive\n");
    }
    issues++;
  }
  if (g->conrod_len_m <= g->stroke_m / 2.0) {
    if (out) {
      fprintf(out,
              "geom: conrod_len_m (%g) must exceed stroke_m/2 (%g) -- "
              "slider-crank geometry can't close otherwise\n",
              g->conrod_len_m, g->stroke_m / 2.0);
    }
    issues++;
  }
  if (g->compression_ratio <= 1.0) {
    if (out) {
      fprintf(out, "geom: compression_ratio must exceed 1.0\n");
    }
    issues++;
  }
  if (g->m_recip_kg <= 0.0) {
    if (out) {
      fprintf(out, "geom: m_recip_kg must be positive\n");
    }
    issues++;
  }
  if (g->delta_theta_burn_deg <= 0.0) {
    if (out) {
      fprintf(out, "geom: delta_theta_burn_deg must be positive\n");
    }
    issues++;
  }
  if (g->combustion_efficiency <= 0.0 || g->combustion_efficiency > 1.0) {
    if (out) {
      fprintf(out, "geom: combustion_efficiency must be in (0,1]\n");
    }
    issues++;
  }
  if (g->evo_deg < 0.0 || g->evo_deg >= 720.0 || g->ivc_deg < 0.0 ||
      g->ivc_deg >= 720.0 || g->evo_deg >= g->ivc_deg) {
    if (out) {
      fprintf(out,
              "geom: evo_deg (%g) and ivc_deg (%g) must both be in [0,720) "
              "with evo_deg < ivc_deg\n",
              g->evo_deg, g->ivc_deg);
    }
    issues++;
  }

  return issues;
}
