#include "physics/engine_spec_io.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define ENGINE_SPEC_LINE_MAX 256

static char *trim(char *s) {
  while (*s && isspace((unsigned char)*s)) {
    s++;
  }
  if (*s == '\0') {
    return s;
  }
  char *end = s + strlen(s) - 1;
  while (end > s && isspace((unsigned char)*end)) {
    *end-- = '\0';
  }
  return s;
}

/* Parses a decimal integer from `s` into *out. Returns 0 on success, -1 if
 * `s` (after trimming) isn't purely a valid integer */
static int parse_int_strict(const char *s, int *out) {
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') {
    return -1;
  }
  *out = (int)v;
  return 0;
}

static int parse_double_strict(const char *s, double *out) {
  char *end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0') {
    return -1;
  }
  *out = v;
  return 0;
}

/* Parses a comma-separated list of 1-based cylinder numbers, e.g.
 * "1,3,4,2", into cfg->firing_order (zero-filling the rest). Returns 0 on
 * success, -1 on a malformed entry or more entries than
 * ENGINE_MAX_CYLINDERS. */
static int parse_firing_order(char *value, EngineConfig *cfg) {
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    cfg->firing_order[i] = 0;
  }
  int n = 0;
  for (char *tok = strtok(value, ","); tok != NULL; tok = strtok(NULL, ",")) {
    if (n >= ENGINE_MAX_CYLINDERS) {
      return -1;
    }
    int v;
    if (parse_int_strict(trim(tok), &v) != 0) {
      return -1;
    }
    cfg->firing_order[n++] = v;
  }
  return 0;
}

EngineSpecResult engine_spec_load(const char *path, EngineConfig *out) {
  EngineSpecResult res = {ENGINE_SPEC_OK, 0, 0};
  *out = engine_config_default();

  FILE *f = fopen(path, "r");
  if (!f) {
    fprintf(stderr,
            "engine_spec: couldn't open '%s' for reading -- using defaults\n",
            path);
    res.status = ENGINE_SPEC_ERR_OPEN;
    return res;
  }

  char line[ENGINE_SPEC_LINE_MAX];
  int lineno = 0;
  while (fgets(line, sizeof(line), f)) {
    lineno++;
    char *trimmed = trim(line);
    if (*trimmed == '\0' || *trimmed == '#') {
      continue;
    }

    char *eq = strchr(trimmed, '=');
    if (!eq) {
      fprintf(stderr, "engine_spec: %s:%d: expected 'key = value', got: %s\n",
              path, lineno, trimmed);
      res.status = ENGINE_SPEC_ERR_PARSE;
      res.error_line = lineno;
      fclose(f);
      return res;
    }
    *eq = '\0';
    char *key = trim(trimmed);
    char *value = trim(eq + 1);

    int parse_ok = 1;
    if (strcmp(key, "num_cylinders") == 0) {
      parse_ok = (parse_int_strict(value, &out->num_cylinders) == 0);
    } else if (strcmp(key, "firing_order") == 0) {
      parse_ok = (parse_firing_order(value, out) == 0);
    } else if (strcmp(key, "inertia_kg_m2") == 0) {
      parse_ok = (parse_double_strict(value, &out->inertia_kg_m2) == 0);
    } else if (strcmp(key, "map_tau_s") == 0) {
      parse_ok = (parse_double_strict(value, &out->map_tau_s) == 0);
    } else if (strcmp(key, "friction_coeff_nm_per_rad_s") == 0) {
      parse_ok =
          (parse_double_strict(value, &out->friction_coeff_nm_per_rad_s) == 0);
    } else if (strcmp(key, "bore_m") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.bore_m) == 0);
    } else if (strcmp(key, "stroke_m") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.stroke_m) == 0);
    } else if (strcmp(key, "conrod_len_m") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.conrod_len_m) == 0);
    } else if (strcmp(key, "compression_ratio") == 0) {
      parse_ok =
          (parse_double_strict(value, &out->geom.compression_ratio) == 0);
    } else if (strcmp(key, "ivc_deg") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.ivc_deg) == 0);
    } else if (strcmp(key, "evo_deg") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.evo_deg) == 0);
    } else if (strcmp(key, "m_recip_kg") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.m_recip_kg) == 0);
    } else if (strcmp(key, "wiebe_a") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.wiebe_a) == 0);
    } else if (strcmp(key, "wiebe_m") == 0) {
      parse_ok = (parse_double_strict(value, &out->geom.wiebe_m) == 0);
    } else if (strcmp(key, "delta_theta_burn_deg") == 0) {
      parse_ok =
          (parse_double_strict(value, &out->geom.delta_theta_burn_deg) == 0);
    } else if (strcmp(key, "spark_base_btdc_deg") == 0) {
      parse_ok =
          (parse_double_strict(value, &out->geom.spark_base_btdc_deg) == 0);
    } else if (strcmp(key, "spark_rpm_gain_deg_per_1000rpm") == 0) {
      parse_ok = (parse_double_strict(
                      value, &out->geom.spark_rpm_gain_deg_per_1000rpm) == 0);
    } else if (strcmp(key, "spark_map_retard_deg_per_kpa") == 0) {
      parse_ok = (parse_double_strict(
                      value, &out->geom.spark_map_retard_deg_per_kpa) == 0);
    } else if (strcmp(key, "combustion_efficiency") == 0) {
      parse_ok =
          (parse_double_strict(value, &out->geom.combustion_efficiency) == 0);
    } else {
      fprintf(stderr, "engine_spec: %s:%d: unknown key '%s' -- ignored\n", path,
              lineno, key);
      res.unknown_keys++;
      continue;
    }

    if (!parse_ok) {
      fprintf(stderr, "engine_spec: %s:%d: bad value for '%s': %s\n", path,
              lineno, key, value);
      res.status = ENGINE_SPEC_ERR_PARSE;
      res.error_line = lineno;
      fclose(f);
      return res;
    }
  }

  fclose(f);
  return res;
}

int engine_spec_save(const char *path, const EngineConfig *cfg) {
  FILE *f = fopen(path, "w");
  if (!f) {
    fprintf(stderr, "engine_spec: couldn't open '%s' for writing\n", path);
    return -1;
  }

  fprintf(f, "# Engine spec for the aero-engine-dt digital twin.\n"
             "# Flat 'key = value' text; '#' starts a whole-line comment;\n"
             "# blank lines are ignored. Any field left out keeps\n"
             "# engine_config_default()'s value, so a spec file only needs to\n"
             "# override what's different for this engine.\n\n");

  fprintf(f,
          "# Number of active cylinders (1..%d / ENGINE_MAX_CYLINDERS).\n"
          "num_cylinders = %d\n\n",
          ENGINE_MAX_CYLINDERS, cfg->num_cylinders);

  fprintf(f, "# 1-based cylinder firing order, comma-separated, one entry per\n"
             "# active cylinder above. Consecutive entries fire\n"
             "# 720/num_cylinders crank degrees apart.\n"
             "firing_order = ");
  for (int i = 0; i < cfg->num_cylinders && i < ENGINE_MAX_CYLINDERS; i++) {
    fprintf(f, "%s%d", i > 0 ? "," : "", cfg->firing_order[i]);
  }
  fprintf(f, "\n\n");

  fprintf(f,
          "# Effective rotating inertia of crank + flywheel + prop, kg*m^2.\n"
          "# Typical range for a small 4-cyl aero piston engine: 0.3 - 1.0.\n"
          "inertia_kg_m2 = %.6g\n\n",
          cfg->inertia_kg_m2);

  fprintf(f,
          "# Manifold filling time constant, s -- how fast MAP chases its\n"
          "# throttle target. Typical range: 0.1 - 0.4.\n"
          "map_tau_s = %.6g\n\n",
          cfg->map_tau_s);

  fprintf(f,
          "# Simple viscous friction coefficient, N*m per rad/s of crank\n"
          "# speed. Typical range: 0.05 - 0.2.\n"
          "friction_coeff_nm_per_rad_s = %.6g\n\n",
          cfg->friction_coeff_nm_per_rad_s);

  fprintf(f, "# crank-angle-resolved combustion geometry, shared by\n"
             "# every cylinder (a cylinder's own compression_trim etc. scale\n"
             "# off this)\n\n");

  fprintf(f,
          "# Cylinder bore, m. Typical small aero/auto range: 0.07 - 0.10.\n"
          "bore_m = %.6g\n\n"
          "# Piston stroke, m. Typical range: 0.07 - 0.10.\n"
          "stroke_m = %.6g\n\n"
          "# Connecting rod length, m. Must exceed stroke_m/2 for the\n"
          "# slider-crank geometry to close. Typical range: 0.12 - 0.20.\n"
          "conrod_len_m = %.6g\n\n"
          "# Nominal compression ratio (Vd/Vc + 1), before a cylinder's own\n"
          "# compression_trim. Typical NA gasoline range: 8.5 - 11.\n"
          "compression_ratio = %.6g\n\n",
          cfg->geom.bore_m, cfg->geom.stroke_m, cfg->geom.conrod_len_m,
          cfg->geom.compression_ratio);

  fprintf(f,
          "# Intake valve close and exhaust valve open, crank degrees in\n"
          "# the [0,720) convention documented in physics/crank_thermo.h\n"
          "# (0 = TDC at start of power stroke). Must satisfy\n"
          "# evo_deg < ivc_deg. Typical ivc_deg range: 570 - 610 (ABDC of\n"
          "# the intake stroke); typical evo_deg range: 110 - 150 (BBDC of\n"
          "# the power stroke).\n"
          "evo_deg = %.6g\n"
          "ivc_deg = %.6g\n\n",
          cfg->geom.evo_deg, cfg->geom.ivc_deg);

  fprintf(f,
          "# Reciprocating mass per cylinder (piston + pin + ~1/3 conrod\n"
          "# mass), kg -- drives the inertia-torque term. Typical range:\n"
          "# 0.3 - 0.6.\n"
          "m_recip_kg = %.6g\n\n",
          cfg->geom.m_recip_kg);

  fprintf(f,
          "# Wiebe combustion heat-release shape: efficiency parameter\n"
          "# (typical 3 - 6), shape parameter (typical 2 - 3), and total\n"
          "# burn duration in crank degrees (typical 40 - 60).\n"
          "wiebe_a = %.6g\n"
          "wiebe_m = %.6g\n"
          "delta_theta_burn_deg = %.6g\n\n",
          cfg->geom.wiebe_a, cfg->geom.wiebe_m, cfg->geom.delta_theta_burn_deg);

  fprintf(f,
          "# Spark advance curve (deg BTDC = base + rpm_gain*(rpm/1000) -\n"
          "# map_retard*(map_kpa-30)), clamped to [5,35] deg BTDC. Typical\n"
          "# ranges: base 5-20, rpm_gain 3-8, map_retard 0.05-0.25.\n"
          "spark_base_btdc_deg = %.6g\n"
          "spark_rpm_gain_deg_per_1000rpm = %.6g\n"
          "spark_map_retard_deg_per_kpa = %.6g\n\n",
          cfg->geom.spark_base_btdc_deg,
          cfg->geom.spark_rpm_gain_deg_per_1000rpm,
          cfg->geom.spark_map_retard_deg_per_kpa);

  fprintf(f,
          "# Fraction of the fuel's chemical energy that manifests as\n"
          "# effective in-cylinder heat (the rest lost to cylinder walls\n"
          "# and exhaust -- this single-zone model has no Woschni-style\n"
          "# heat-transfer correlation, so this stands in for it). Typical\n"
          "# range: 0.25 - 0.35; must be in (0,1].\n"
          "combustion_efficiency = %.6g\n",
          cfg->geom.combustion_efficiency);

  fclose(f);
  return 0;
}
