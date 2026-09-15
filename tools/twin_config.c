/* Engine-spec authoring/validation tool for the digital-twin core.
 *
 * Scaffolds a new engine spec file pre-filled with defaults and documentation
 * comments, and checks an existing one for errors
 *
 *   twin_config --new PATH [--cylinders N] [--firing-order LIST]
 *   twin_config --check PATH
 *   twin_config --help
 *
 * Examples:
 *   twin_config --new engines/my_v4.cfg
 *   twin_config --new engines/inline6.cfg --cylinders 6 \
 *       --firing-order 1,5,3,6,2,4
 *   twin_config --check engines/my_v4.cfg
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "physics/crank_thermo.h"
#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"

static void usage(FILE *out, const char *argv0) {
  fprintf(out,
          "usage: %s --new PATH [--cylinders N] [--firing-order LIST]\n"
          "       %s --check PATH\n"
          "       %s --help\n\n"
          "Creates or validates an engine spec file (flat key=value text)\n"
          "used to configure the digital twin's engine model without\n"
          "recompiling -- point twin_sim or the dashboard at the result.\n\n"
          "--new PATH            write a template spec file to PATH,\n"
          "                      pre-filled from engine_config_default()\n"
          "                      with inline comments documenting every\n"
          "                      field, its unit, and a typical range.\n"
          "--cylinders N         override the template's cylinder count\n"
          "                      (default 4; identity firing order 1..N\n"
          "                      unless --firing-order is also given)\n"
          "--firing-order LIST   comma-separated 1-based firing order,\n"
          "                      e.g. 1,3,4,2\n"
          "--check PATH          load PATH, report parse errors and\n"
          "                      unknown keys, then validate the result\n"
          "                      (range/permutation checks) and print a\n"
          "                      derived summary if it loads cleanly\n"
          "--help                show this message\n",
          argv0, argv0, argv0);
}

static int cmd_new(const char *path, int cylinders_set, int cylinders,
                   const char *firing_order_arg) {
  EngineConfig cfg = engine_config_default();

  if (cylinders_set) {
    if (cylinders < 1 || cylinders > ENGINE_MAX_CYLINDERS) {
      fprintf(stderr, "twin_config: --cylinders must be 1..%d\n",
              ENGINE_MAX_CYLINDERS);
      return 2;
    }
    cfg.num_cylinders = cylinders;
    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
      cfg.firing_order[i] = (i < cylinders) ? i + 1 : 0; /* identity order */
    }
  }

  if (firing_order_arg) {
    char buf[128];
    if (strlen(firing_order_arg) >= sizeof(buf)) {
      fprintf(stderr, "twin_config: --firing-order value too long\n");
      return 2;
    }
    strcpy(buf, firing_order_arg);

    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
      cfg.firing_order[i] = 0;
    }
    int n = 0;
    for (char *tok = strtok(buf, ","); tok != NULL; tok = strtok(NULL, ",")) {
      if (n >= ENGINE_MAX_CYLINDERS) {
        fprintf(stderr,
                "twin_config: --firing-order has more than %d "
                "entries\n",
                ENGINE_MAX_CYLINDERS);
        return 2;
      }
      cfg.firing_order[n++] = atoi(tok);
    }
    if (!cylinders_set) {
      cfg.num_cylinders = n; /* firing order implies the cylinder count */
    }
  }

  int issues = engine_config_validate(&cfg, stderr);
  if (issues > 0) {
    fprintf(stderr,
            "twin_config: %d issue(s) with the requested --cylinders/"
            "--firing-order combination -- not writing %s\n",
            issues, path);
    return 1;
  }

  if (engine_spec_save(path, &cfg) != 0) {
    return 2;
  }
  fprintf(stderr, "twin_config: wrote %s (%d cylinder%s)\n", path,
          cfg.num_cylinders, cfg.num_cylinders == 1 ? "" : "s");
  return 0;
}

static int cmd_check(const char *path) {
  EngineConfig cfg;
  EngineSpecResult r = engine_spec_load(path, &cfg);

  if (r.status == ENGINE_SPEC_ERR_OPEN) {
    return 2; /* engine_spec_load already reported this to stderr */
  }
  if (r.status == ENGINE_SPEC_ERR_PARSE) {
    fprintf(stderr, "twin_config: %s: parse error at line %d\n", path,
            r.error_line);
    return 2;
  }
  if (r.unknown_keys > 0) {
    fprintf(stderr, "twin_config: %s: %d unknown key(s) ignored (see above)\n",
            path, r.unknown_keys);
  }

  int issues = engine_config_validate(&cfg, stdout);
  if (issues > 0) {
    printf("%s: %d issue(s) found\n", path, issues);
    return 1;
  }

  printf("%s: OK\n\n", path);
  printf("num_cylinders                = %d\n", cfg.num_cylinders);
  printf("firing_order                 = ");
  for (int i = 0; i < cfg.num_cylinders; i++) {
    printf("%s%d", i > 0 ? "," : "", cfg.firing_order[i]);
  }
  printf("\n");
  printf("firing interval              = %.1f deg (720 / num_cylinders)\n",
         720.0 / cfg.num_cylinders);
  printf("inertia_kg_m2                = %.6g\n", cfg.inertia_kg_m2);
  printf("map_tau_s                    = %.6g\n", cfg.map_tau_s);
  printf("friction_coeff_nm_per_rad_s  = %.6g\n",
         cfg.friction_coeff_nm_per_rad_s);

  printf("bore_m / stroke_m / conrod_len_m = %.6g / %.6g / %.6g\n",
         cfg.geom.bore_m, cfg.geom.stroke_m, cfg.geom.conrod_len_m);
  printf("compression_ratio            = %.6g\n", cfg.geom.compression_ratio);
  printf("evo_deg / ivc_deg             = %.6g / %.6g\n", cfg.geom.evo_deg,
         cfg.geom.ivc_deg);
  printf("m_recip_kg                    = %.6g\n", cfg.geom.m_recip_kg);
  printf("wiebe_a / wiebe_m / burn_deg  = %.6g / %.6g / %.6g\n",
         cfg.geom.wiebe_a, cfg.geom.wiebe_m, cfg.geom.delta_theta_burn_deg);
  printf("spark base/rpm_gain/map_retard = %.6g / %.6g / %.6g\n",
         cfg.geom.spark_base_btdc_deg, cfg.geom.spark_rpm_gain_deg_per_1000rpm,
         cfg.geom.spark_map_retard_deg_per_kpa);
  printf("combustion_efficiency         = %.6g\n",
         cfg.geom.combustion_efficiency);

  double displacement_l =
      cylinder_displacement_m3(&cfg.geom) * cfg.num_cylinders * 1000.0;
  double clearance_cc =
      cylinder_clearance_m3(&cfg.geom, cfg.geom.compression_ratio) * 1.0e6;
  printf("\ntotal displacement            = %.3f L (%d cyl)\n", displacement_l,
         cfg.num_cylinders);
  printf("clearance volume (per cyl)    = %.2f cc\n", clearance_cc);
  return 0;
}

int main(int argc, char **argv) {
  const char *new_path = NULL;
  const char *check_path = NULL;
  int cylinders_set = 0;
  int cylinders = 0;
  const char *firing_order_arg = NULL;

  for (int i = 1; i < argc; i++) {
    const char *a = argv[i];
    if (!strcmp(a, "--new") && i + 1 < argc) {
      new_path = argv[++i];
    } else if (!strcmp(a, "--check") && i + 1 < argc) {
      check_path = argv[++i];
    } else if (!strcmp(a, "--cylinders") && i + 1 < argc) {
      cylinders = atoi(argv[++i]);
      cylinders_set = 1;
    } else if (!strcmp(a, "--firing-order") && i + 1 < argc) {
      firing_order_arg = argv[++i];
    } else if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
      usage(stdout, argv[0]);
      return 0;
    } else {
      fprintf(stderr, "twin_config: unknown or incomplete argument: %s\n\n", a);
      usage(stderr, argv[0]);
      return 2;
    }
  }

  if (new_path && check_path) {
    fprintf(stderr, "twin_config: --new and --check are mutually exclusive\n");
    return 2;
  }
  if (new_path) {
    return cmd_new(new_path, cylinders_set, cylinders, firing_order_arg);
  }
  if (check_path) {
    return cmd_check(check_path);
  }

  usage(stderr, argv[0]);
  return 2;
}
