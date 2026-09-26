#include "physics/engine_config_fields.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

const ConfigGroup ENGINE_CONFIG_GROUPS[] = {
    {"Dynamics", 0},
    {"Geometry", 0},
    {"Propeller", 0},
    {"ECU", 0},
    {"Combustion", 1},
};
const int ENGINE_CONFIG_GROUP_COUNT =
    (int)(sizeof ENGINE_CONFIG_GROUPS / sizeof ENGINE_CONFIG_GROUPS[0]);

enum { G_DYN = 0, G_GEOM = 1, G_PROP = 2, G_ECU = 3, G_COMB = 4 };
_Static_assert(G_ECU == ENGINE_CONFIG_GROUP_ECU,
               "ENGINE_CONFIG_GROUP_ECU must match the ECU group's index");

#define POS_EXCL (0.0), (INFINITY), CFG_MIN_EXCL /* strictly positive */
#define NON_NEG (0.0), (INFINITY), 0             /* zero allowed */
#define ANY (-INFINITY), (INFINITY), 0           /* unbounded */
#define NO_TYP 0.0, 0.0
#define OFF(member) offsetof(EngineConfig, member)

/* Order matters: it is the order of the spec file, the panels and the
 * validator's messages. */
const ConfigField ENGINE_CONFIG_FIELDS[] = {
    /* ---- dynamics ---- */
    {"inertia_kg_m2", "crank inertia", "kg*m^2", G_DYN, OFF(inertia_kg_m2),
     POS_EXCL, 0.3, 1.0, 0.05, 3.0, "%.3f", 1.0, "kg*m^2",
     "Effective rotating inertia of crank + flywheel + prop (the prop's\n"
     "aerodynamic load is separate, see the propeller group).\n"
     "Typical range for a small 4-cyl aero piston engine: 0.3 - 1.0."},
    {"map_tau_s", "MAP time constant", "s", G_DYN, OFF(map_tau_s), POS_EXCL,
     0.1, 0.4, 0.02, 1.0, "%.3f", 1.0, "s",
     "Manifold filling time constant -- how fast MAP chases its\n"
     "throttle target. Typical range: 0.1 - 0.4."},
    {"friction_coeff_nm_per_rad_s", "viscous friction", "N*m/(rad/s)", G_DYN,
     OFF(friction_coeff_nm_per_rad_s), POS_EXCL, 0.005, 0.05, 0.001, 0.5,
     "%.4f", 1.0, "N*m/(rad/s)",
     "Size-independent viscous drag (shaft seals, accessories), N*m per\n"
     "rad/s of crank speed. The bulk of the friction is the FMEP model\n"
     "below, which scales with displacement. Typical range: 0.005 - 0.05."},
    {"friction_fmep_const_kpa", "friction, constant part", "kPa", G_DYN,
     OFF(friction_fmep_const_kpa), NON_NEG, 30.0, 80.0, 0.0, 200.0, "%.1f",
     1.0, "kPa",
     "Speed-independent friction mean effective pressure (rings, valve\n"
     "train, bearings). Friction torque = displacement / 4pi * FMEP, so it\n"
     "grows with engine size. Typical range: 30 - 80."},
    {"friction_fmep_per_ms_kpa", "friction, speed part", "kPa per m/s", G_DYN,
     OFF(friction_fmep_per_ms_kpa), NON_NEG, 8.0, 20.0, 0.0, 40.0, "%.2f",
     1.0, "kPa per m/s",
     "Friction mean effective pressure added per m/s of mean piston speed\n"
     "(2 * stroke * rpm / 60). Typical range: 8 - 20."},
    {"starter_torque_nm", "starter torque", "N*m", G_DYN,
     OFF(starter_torque_nm), POS_EXCL, NO_TYP, 5.0, 100.0, "%.1f", 1.0, "N*m",
     "Starter motor torque while cranking."},
    {"starter_catch_rpm", "starter catch speed", "rpm", G_DYN,
     OFF(starter_catch_rpm), POS_EXCL, NO_TYP, 200.0, 1500.0, "%.0f", 1.0,
     "rpm",
     "Crank speed at which the engine is considered to have caught\n"
     "after cranking."},

    /* ---- geometry ---- */
    {"bore_m", "bore", "m", G_GEOM, OFF(geom.bore_m), POS_EXCL, 0.07, 0.10,
     0.04, 0.14, "%.4f", 1000.0, "mm",
     "Cylinder bore. Typical small aero/auto range: 0.07 - 0.10."},
    {"stroke_m", "stroke", "m", G_GEOM, OFF(geom.stroke_m), POS_EXCL, 0.07,
     0.10, 0.04, 0.14, "%.4f", 1000.0, "mm",
     "Piston stroke. Typical range: 0.07 - 0.10."},
    {"conrod_len_m", "conrod length", "m", G_GEOM, OFF(geom.conrod_len_m),
     POS_EXCL, 0.12, 0.20, 0.08, 0.25, "%.4f", 1000.0, "mm",
     "Connecting rod length. Must exceed stroke_m/2 for the\n"
     "slider-crank geometry to close. Typical range: 0.12 - 0.20."},
    {"compression_ratio", "compression ratio", "", G_GEOM,
     OFF(geom.compression_ratio), 1.0, INFINITY, CFG_MIN_EXCL, 8.5, 11.0, 5.0,
     15.0, "%.2f", 1.0, ":1",
     "Nominal compression ratio (Vd/Vc + 1), before a cylinder's own\n"
     "compression_trim. Typical NA gasoline range: 8.5 - 11."},
    {"evo_deg", "exhaust valve opens", "deg", G_GEOM, OFF(geom.evo_deg), 0.0,
     720.0, CFG_MAX_EXCL, 110.0, 150.0, 60.0, 180.0, "%.0f", 1.0, "deg",
     "Exhaust valve open, crank degrees in the [0,720) convention\n"
     "documented in physics/crank_thermo.h (0 = TDC at start of power\n"
     "stroke). Must satisfy evo_deg < ivc_deg. Typical range: 110 - 150\n"
     "(BBDC of the power stroke)."},
    {"ivc_deg", "intake valve closes", "deg", G_GEOM, OFF(geom.ivc_deg), 0.0,
     720.0, CFG_MAX_EXCL, 570.0, 610.0, 500.0, 700.0, "%.0f", 1.0, "deg",
     "Intake valve close, crank degrees, same convention as evo_deg.\n"
     "Typical range: 570 - 610 (ABDC of the intake stroke)."},
    {"m_recip_kg", "reciprocating mass", "kg", G_GEOM, OFF(geom.m_recip_kg),
     POS_EXCL, 0.3, 0.6, 0.1, 1.0, "%.3f", 1.0, "kg/cyl",
     "Reciprocating mass per cylinder (piston + pin + ~1/3 conrod\n"
     "mass) -- drives the inertia-torque term. Typical range: 0.3 - 0.6."},

    /* ---- propeller ---- */
    {"prop_diameter_m", "prop diameter", "m", G_PROP, OFF(prop.diameter_m),
     POS_EXCL, 1.0, 1.8, 0.3, 3.0, "%.3f", 1000.0, "mm",
     "Propeller diameter. Load torque scales with D^5, so this is the\n"
     "biggest lever on RPM. Typical range for a small UAV: 1.0 - 1.8."},
    {"prop_j_zero_thrust", "zero-thrust advance ratio", "", G_PROP,
     OFF(prop.j_zero_thrust), POS_EXCL, 0.6, 1.1, 0.2, 2.0, "%.3f", 1.0, "",
     "Advance ratio J = V/(n*D) at which thrust falls to zero, roughly\n"
     "0.9 * pitch / diameter -- a coarser pitch unloads later.\n"
     "Typical range: 0.6 - 1.1."},
    {"prop_ct_static", "static thrust coefficient", "", G_PROP,
     OFF(prop.ct_static), NON_NEG, 0.08, 0.13, 0.02, 0.25, "%.4f", 1.0, "",
     "Thrust coefficient at J = 0 (thrust = Ct * rho * n^2 * D^4).\n"
     "Typical range: 0.08 - 0.13."},
    {"prop_cq_static", "static torque coefficient", "", G_PROP,
     OFF(prop.cq_static), NON_NEG, 0.005, 0.010, 0.0, 0.03, "%.5f", 1.0, "",
     "Torque coefficient at J = 0 (torque = Cq * rho * n^2 * D^5).\n"
     "0 removes the propeller entirely (unloaded engine).\n"
     "Typical range: 0.005 - 0.010."},
    {"prop_cq_unload", "torque unloading", "", G_PROP, OFF(prop.cq_unload), 0.0,
     1.0, 0, 0.3, 0.6, 0.0, 1.0, "%.3f", 1.0, "",
     "Fraction of the static torque lost by the time J reaches the\n"
     "zero-thrust advance ratio (airspeed unloads the blades).\n"
     "Typical range: 0.3 - 0.6; must be in [0, 1]."},

    /* ---- ECU ---- */
    {"idle_target_rpm", "idle target speed", "rpm", G_ECU,
     OFF(ecu.idle_target_rpm), NON_NEG, 600.0, 1400.0, 0.0, 2000.0, "%.0f", 1.0,
     "rpm",
     "Crank speed the idle governor holds by adding throttle while the\n"
     "engine runs. 0 disables the governor. Typical range for a direct-drive\n"
     "aircraft engine: 600 - 1400."},
    {"idle_kp", "governor proportional gain", "throttle/rpm", G_ECU,
     OFF(ecu.idle_kp), NON_NEG, 0.0001, 0.001, 0.0, 0.003, "%.5f", 1.0,
     "throttle/rpm",
     "Throttle added per rpm of speed error. Too high makes idle hunt.\n"
     "Typical range: 0.0001 - 0.001."},
    {"idle_ki", "governor integral gain", "throttle/rpm/s", G_ECU,
     OFF(ecu.idle_ki), NON_NEG, 0.0001, 0.001, 0.0, 0.003, "%.5f", 1.0,
     "throttle/rpm/s",
     "Throttle added per rpm of error per second; removes the steady-state\n"
     "error left by the proportional term. Typical range: 0.0001 - 0.001."},
    {"idle_max_throttle", "governor authority", "", G_ECU,
     OFF(ecu.idle_max_throttle), 0.0, 1.0, 0, 0.1, 0.25, 0.0, 0.5, "%.3f", 1.0, "",
     "Most throttle the governor may add, and the pilot throttle above which\n"
     "it steps aside. Typical range: 0.1 - 0.25; must be in [0, 1]."},

    /* ---- combustion model tuning ---- */
    {"wiebe_a", "Wiebe a", "", G_COMB, OFF(geom.wiebe_a), POS_EXCL, 3.0, 6.0,
     1.0, 10.0, "%.2f", 1.0, "",
     "Wiebe heat-release efficiency parameter. Typical range: 3 - 6."},
    {"wiebe_m", "Wiebe m", "", G_COMB, OFF(geom.wiebe_m), POS_EXCL, 2.0, 3.0,
     0.5, 4.0, "%.2f", 1.0, "",
     "Wiebe heat-release shape parameter. Typical range: 2 - 3."},
    {"delta_theta_burn_deg", "burn duration", "deg", G_COMB,
     OFF(geom.delta_theta_burn_deg), POS_EXCL, 40.0, 60.0, 20.0, 90.0, "%.0f",
     1.0, "deg", "Total burn duration in crank degrees. Typical range: 40 - 60."},
    {"spark_base_btdc_deg", "spark base advance", "deg BTDC", G_COMB,
     OFF(geom.spark_base_btdc_deg), ANY, 5.0, 20.0, 0.0, 30.0, "%.1f", 1.0,
     "deg BTDC",
     "Spark advance curve: deg BTDC = base + rpm_gain*(rpm/1000) -\n"
     "map_retard*(map_kpa-30), clamped to [5,35] deg BTDC.\n"
     "Base advance typical range: 5 - 20."},
    {"spark_rpm_gain_deg_per_1000rpm", "spark rpm gain", "deg/1000rpm", G_COMB,
     OFF(geom.spark_rpm_gain_deg_per_1000rpm), ANY, 3.0, 8.0, 0.0, 12.0,
     "%.2f", 1.0, "deg/1000rpm",
     "Spark advance added per 1000 rpm. Typical range: 3 - 8."},
    {"spark_map_retard_deg_per_kpa", "spark MAP retard", "deg/kPa", G_COMB,
     OFF(geom.spark_map_retard_deg_per_kpa), ANY, 0.05, 0.25, 0.0, 0.5, "%.3f",
     1.0, "deg/kPa",
     "Spark advance removed per kPa of MAP above 30.\n"
     "Typical range: 0.05 - 0.25."},
    {"combustion_efficiency", "combustion efficiency", "", G_COMB,
     OFF(geom.combustion_efficiency), 0.0, 1.0, CFG_MIN_EXCL, 0.25, 0.35, 0.05,
     1.0, "%.3f", 1.0, "",
     "Fraction of the fuel's chemical energy that manifests as effective\n"
     "in-cylinder heat (the rest is lost to cylinder walls and exhaust --\n"
     "this single-zone model has no heat-transfer correlation, so this\n"
     "stands in for it). Typical range: 0.25 - 0.35; must be in (0,1]."},
};
const int ENGINE_CONFIG_FIELD_COUNT =
    (int)(sizeof ENGINE_CONFIG_FIELDS / sizeof ENGINE_CONFIG_FIELDS[0]);

const ConfigField *engine_config_find_field(const char *key) {
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    if (strcmp(ENGINE_CONFIG_FIELDS[i].key, key) == 0) {
      return &ENGINE_CONFIG_FIELDS[i];
    }
  }
  return NULL;
}

double *engine_config_field_ptr(EngineConfig *cfg, const ConfigField *f) {
  return (double *)((char *)cfg + f->offset);
}

const double *engine_config_field_cptr(const EngineConfig *cfg,
                                       const ConfigField *f) {
  return (const double *)((const char *)cfg + f->offset);
}

int engine_config_field_in_range(const ConfigField *f, double v) {
  if (isnan(v)) {
    return 0;
  }
  if (isfinite(f->valid_min)) {
    if ((f->flags & CFG_MIN_EXCL) ? !(v > f->valid_min) : !(v >= f->valid_min)) {
      return 0;
    }
  }
  if (isfinite(f->valid_max)) {
    if ((f->flags & CFG_MAX_EXCL) ? !(v < f->valid_max) : !(v <= f->valid_max)) {
      return 0;
    }
  }
  return 1;
}

void engine_config_field_range_message(const ConfigField *f, double v,
                                       char *out, size_t out_size) {
  const int has_min = isfinite(f->valid_min);
  const int has_max = isfinite(f->valid_max);
  const int min_excl = (f->flags & CFG_MIN_EXCL) != 0;
  const int max_excl = (f->flags & CFG_MAX_EXCL) != 0;

  if (has_min && !has_max && f->valid_min == 0.0 && min_excl) {
    snprintf(out, out_size, "%s = %g: must be positive", f->key, v);
  } else if (has_min && has_max) {
    snprintf(out, out_size, "%s = %g: must be in %c%g, %g%c", f->key, v,
             min_excl ? '(' : '[', f->valid_min, f->valid_max,
             max_excl ? ')' : ']');
  } else if (has_min) {
    snprintf(out, out_size, "%s = %g: must be %s %g", f->key, v,
             min_excl ? "greater than" : "at least", f->valid_min);
  } else if (has_max) {
    snprintf(out, out_size, "%s = %g: must be %s %g", f->key, v,
             max_excl ? "less than" : "at most", f->valid_max);
  } else {
    snprintf(out, out_size, "%s = %g: is not a valid number", f->key, v);
  }
}
