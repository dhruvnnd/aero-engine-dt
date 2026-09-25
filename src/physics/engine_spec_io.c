#include "physics/engine_spec_io.h"

#include "physics/engine_config_fields.h"

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
    } else if (engine_config_find_field(key) != NULL) {
      parse_ok = (parse_double_strict(
                      value, engine_config_field_ptr(
                                 out, engine_config_find_field(key))) == 0);
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

/* Writes `doc` (lines separated by '\n') as '#' comment lines. */
static void write_comment(FILE *f, const char *doc) {
  const char *line = doc;
  while (*line) {
    const char *nl = strchr(line, '\n');
    const int len = nl ? (int)(nl - line) : (int)strlen(line);
    fprintf(f, "# %.*s\n", len, line);
    line += len;
    if (*line == '\n') {
      line++;
    }
  }
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

  /* every other parameter comes from the field table, grouped */
  for (int g = 0; g < ENGINE_CONFIG_GROUP_COUNT; g++) {
    fprintf(f, "# --- %s ---\n\n", ENGINE_CONFIG_GROUPS[g].name);
    for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
      const ConfigField *fld = &ENGINE_CONFIG_FIELDS[i];
      if (fld->group != g) {
        continue;
      }
      write_comment(f, fld->doc);
      fprintf(f, "%s = %.6g\n\n", fld->key,
              *engine_config_field_cptr(cfg, fld));
    }
  }

  fclose(f);
  return 0;
}
