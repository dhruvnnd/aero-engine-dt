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
          "# throttle target. Typical range: 0.1 - 0.4. (Replaced by a real\n"
          "# mass-balance ODE in a later phase; still the live MAP dynamics\n"
          "# for now.)\n"
          "map_tau_s = %.6g\n\n",
          cfg->map_tau_s);

  fprintf(f,
          "# Simple viscous friction coefficient, N*m per rad/s of crank\n"
          "# speed. Typical range: 0.05 - 0.2.\n"
          "friction_coeff_nm_per_rad_s = %.6g\n\n",
          cfg->friction_coeff_nm_per_rad_s);

  fclose(f);
  return 0;
}
