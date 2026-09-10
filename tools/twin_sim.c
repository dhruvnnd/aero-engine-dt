/* Headless CSV sim-runner for the digital-twin core.
 *
 * Steps model_sync over a scripted throttle/altitude profile and writes a
 * CSV time-series to stdout -- for plotting, eyeballing transient response,
 * and mission-profile experiments, all without the SDL dashboard. Links
 * engine_core only; contains no SDL.
 *
 *   twin_sim [--profile NAME] [--dt SECONDS] [--duration SECONDS]
 *            [--load NM] [--seed N] [--sensor] [--list] [--help]
 *
 * Examples:
 *   twin_sim --list
 *   twin_sim --profile cruise-climb > run.csv
 *   twin_sim --profile rapid-throttle --sensor --dt 0.01 --duration 90 >
 * run.csv
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "model/channels.h"
#include "model/state.h"
#include "model/sync.h"
#include "physics/environment.h"
#include "telemetry/sensor.h"

typedef struct {
  const char *name;
  const char *description;
  double duration_s; /* default; overridable with --duration */
  double (*throttle)(double t);
  double (*altitude_m)(double t);
  double (*airspeed_ms)(double t); /* true airspeed; sets ram-air cooling */
  double ambient_offset_c; /* added to ISA temperature (hot-day, etc.) */
} SimProfile;

static double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

static double alt_ground(double t) {
  (void)t;
  return 0.0;
}

static double thr_idle(double t) {
  (void)t;
  return 0.0;
}

static double spd_zero(double t) {
  (void)t;
  return 0.0; /* parked / test stand: no ram air, prop wash only */
}

static double thr_takeoff(double t) { return clamp01(t / 5.0); }
static double alt_takeoff(double t) {
  double climb = (t - 10.0) * 10.0; /* 10 m/s, after a 10 s ground roll */
  return climb < 0.0 ? 0.0 : climb;
}
static double spd_takeoff(double t) {
  double v = 3.5 * t;         /* accelerate down the runway */
  return v > 38.0 ? 38.0 : v; /* then hold the climb-out speed */
}

static double thr_cruise(double t) {
  (void)t;
  return 0.75;
}
static double alt_cruiseclimb(double t) {
  double a = t * 5.0; /* 5 m/s */
  return a > 4500.0 ? 4500.0 : a;
}
static double spd_cruiseclimb(double t) {
  double v = 6.0 * t; /* spool up to the best-climb speed */
  return v > 46.0 ? 46.0 : v;
}

static double thr_hotday(double t) {
  (void)t;
  return 0.80;
}

static double thr_rapid(double t) {
  return fmod(t, 20.0) < 10.0 ? 0.20 : 1.00; /* square wave, 20 s period */
}

static const SimProfile PROFILES[] = {
    {"idle", "closed throttle, sea level, static", 120.0, thr_idle, alt_ground,
     spd_zero, 0.0},
    {"takeoff", "throttle ramp to WOT, ground roll then 10 m/s climb", 120.0,
     thr_takeoff, alt_takeoff, spd_takeoff, 0.0},
    {"cruise-climb", "0.75 throttle, 5 m/s climb capped at 4500 m", 900.0,
     thr_cruise, alt_cruiseclimb, spd_cruiseclimb, 0.0},
    {"hot-day", "0.80 throttle, sea level, static, ISA +25 C", 300.0,
     thr_hotday, alt_ground, spd_zero, 25.0},
    {"rapid-throttle",
     "0.20 <-> 1.00 square wave, 20 s period, sea level static", 120.0,
     thr_rapid, alt_ground, spd_zero, 0.0},
};
static const int PROFILE_COUNT = (int)(sizeof(PROFILES) / sizeof(PROFILES[0]));

static const SimProfile *find_profile(const char *name) {
  for (int i = 0; i < PROFILE_COUNT; i++) {
    if (strcmp(PROFILES[i].name, name) == 0) {
      return &PROFILES[i];
    }
  }
  return NULL;
}

static void print_profiles(FILE *out) {
  fprintf(out, "profiles:\n");
  for (int i = 0; i < PROFILE_COUNT; i++) {
    fprintf(out, "  %-15s %s (default %.0f s)\n", PROFILES[i].name,
            PROFILES[i].description, PROFILES[i].duration_s);
  }
}

static void usage(FILE *out, const char *argv0) {
  fprintf(out,
          "usage: %s [--profile NAME] [--dt SECONDS] [--duration SECONDS]\n"
          "          [--load NM] [--seed N] [--sensor]\n"
          "          [--cyl N:KEY=VAL ...] [--fault-at SECONDS]\n"
          "          [--list] [--help]\n\n"
          "Writes a CSV engine/thermal time-series to stdout; progress and\n"
          "run parameters go to stderr. Add --sensor for the noisy sensor\n"
          "channels alongside ground truth.\n\n"
          "--cyl sets one cylinder trim (repeatable). N is 1-based; KEY is\n"
          "  inj   injector flow trim   (1.0 nominal)\n"
          "  comp  compression trim     (1.0 nominal)\n"
          "  spark spark offset, deg    (0.0 nominal, + = retard)\n"
          "  leak  intake leak fraction (0.0 nominal, + = leaner)\n"
          "  cool  cooling trim         (1.0 nominal, < 1 = hotter head)\n"
          "--fault-at delays every --cyl trim until that time (default 0).\n\n",
          argv0);
  print_profiles(out);
}

typedef enum {
  CYL_KEY_INJ,
  CYL_KEY_COMP,
  CYL_KEY_SPARK,
  CYL_KEY_LEAK,
  CYL_KEY_COOL
} CylKey;

typedef struct {
  int cyl0; /* 0-based cylinder index */
  CylKey key;
  double value;
} CylFault;

#define MAX_CYL_FAULTS 16
static CylFault g_faults[MAX_CYL_FAULTS];
static int g_nfaults;

/* Parses "N:KEY=VAL" and appends it to g_faults. Returns 0 on success. */
static int parse_cyl_fault(const char *spec) {
  const char *colon = strchr(spec, ':');
  const char *eq = colon ? strchr(colon, '=') : NULL;
  if (!colon || !eq || g_nfaults >= MAX_CYL_FAULTS) {
    return -1;
  }
  int cyl = atoi(spec);
  if (cyl < 1 || cyl > ENGINE_MAX_CYLINDERS) {
    return -1;
  }

  char key[16] = {0};
  size_t klen = (size_t)(eq - colon - 1);
  if (klen == 0 || klen >= sizeof(key)) {
    return -1;
  }
  memcpy(key, colon + 1, klen);

  CylKey k;
  if (!strcmp(key, "inj")) {
    k = CYL_KEY_INJ;
  } else if (!strcmp(key, "comp")) {
    k = CYL_KEY_COMP;
  } else if (!strcmp(key, "spark")) {
    k = CYL_KEY_SPARK;
  } else if (!strcmp(key, "leak")) {
    k = CYL_KEY_LEAK;
  } else if (!strcmp(key, "cool")) {
    k = CYL_KEY_COOL;
  } else {
    return -1;
  }

  g_faults[g_nfaults].cyl0 = cyl - 1;
  g_faults[g_nfaults].key = k;
  g_faults[g_nfaults].value = atof(eq + 1);
  g_nfaults++;
  return 0;
}

static void apply_cyl_faults(ModelSync *sync) {
  for (int i = 0; i < g_nfaults; i++) {
    CylinderConfig *c = &sync->cyl_config[g_faults[i].cyl0];
    double v = g_faults[i].value;
    switch (g_faults[i].key) {
    case CYL_KEY_INJ:
      c->injector_flow_trim = v;
      break;
    case CYL_KEY_COMP:
      c->compression_trim = v;
      break;
    case CYL_KEY_SPARK:
      c->spark_offset_deg = v;
      break;
    case CYL_KEY_LEAK:
      c->intake_leak_frac = v;
      break;
    case CYL_KEY_COOL:
      c->cooling_trim = v;
      break;
    }
  }
}

int main(int argc, char **argv) {
  const char *profile_name = "idle";
  double dt = 0.02;
  double duration_s = -1.0; /* < 0 => use the profile's default */
  double load_nm = 40.0;
  uint32_t seed = 1u;
  int with_sensor = 0;
  double fault_at_s = 0.0;

  for (int i = 1; i < argc; i++) {
    const char *a = argv[i];
    if ((!strcmp(a, "--profile") || !strcmp(a, "-p")) && i + 1 < argc) {
      profile_name = argv[++i];
    } else if (!strcmp(a, "--dt") && i + 1 < argc) {
      dt = atof(argv[++i]);
    } else if (!strcmp(a, "--duration") && i + 1 < argc) {
      duration_s = atof(argv[++i]);
    } else if (!strcmp(a, "--load") && i + 1 < argc) {
      load_nm = atof(argv[++i]);
    } else if (!strcmp(a, "--seed") && i + 1 < argc) {
      seed = (uint32_t)strtoul(argv[++i], NULL, 10);
    } else if (!strcmp(a, "--sensor")) {
      with_sensor = 1;
    } else if (!strcmp(a, "--cyl") && i + 1 < argc) {
      if (parse_cyl_fault(argv[++i]) != 0) {
        fprintf(stderr, "twin_sim: bad --cyl spec: %s\n\n", argv[i]);
        usage(stderr, argv[0]);
        return 2;
      }
    } else if (!strcmp(a, "--fault-at") && i + 1 < argc) {
      fault_at_s = atof(argv[++i]);
    } else if (!strcmp(a, "--list")) {
      print_profiles(stdout);
      return 0;
    } else if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
      usage(stdout, argv[0]);
      return 0;
    } else {
      fprintf(stderr, "twin_sim: unknown or incomplete argument: %s\n\n", a);
      usage(stderr, argv[0]);
      return 2;
    }
  }

  const SimProfile *profile = find_profile(profile_name);
  if (!profile) {
    fprintf(stderr, "twin_sim: no such profile: %s\n\n", profile_name);
    print_profiles(stderr);
    return 2;
  }
  if (dt <= 0.0) {
    fprintf(stderr, "twin_sim: --dt must be positive\n");
    return 2;
  }
  if (duration_s < 0.0) {
    duration_s = profile->duration_s;
  }

  ModelSync sync;
  model_sync_init(&sync);

  double alt0 = profile->altitude_m(0.0);
  AtmosphereState atm0 = environment_isa(alt0);
  double ambient0_c = (atm0.temperature_k - 273.15) + profile->ambient_offset_c;

  ModelState st;
  model_state_init(&st, &sync.engine_config, ambient0_c);

  Sensor sensor;
  SensorConfig scfg = sensor_config_default();
  sensor_init(&sensor, &scfg, seed);

  fprintf(stderr,
          "# twin_sim profile=%s dt=%.4f duration=%.1f load=%.1f seed=%u%s",
          profile->name, dt, duration_s, load_nm, seed,
          with_sensor ? " +sensor" : "");
  if (g_nfaults > 0) {
    fprintf(stderr, " faults=%d@%.0fs", g_nfaults, fault_at_s);
  }
  fprintf(stderr, "\n");

  int faults_applied = (g_nfaults == 0);

  int nchan = 0;
  const ModelChannel *chans = model_channels(&nchan);

  printf("t,throttle,alt_m,ambient_c,airspeed_ms,cool_index");
  for (int c = 0; c < nchan; c++) {
    printf(",%s", chans[c].name);
  }
  if (with_sensor) {
    printf(",s_rpm,s_rpm_ok,s_map_kpa,s_map_ok,s_cht_c,s_cht_ok,"
           "s_egt_c,s_egt_ok,s_oil_c,s_oil_ok");
  }
  printf("\n");

  long nsteps = (long)(duration_s / dt + 0.5);
  for (long i = 0; i <= nsteps; i++) {
    double t = (double)i * dt;

    if (!faults_applied && t >= fault_at_s) {
      apply_cyl_faults(&sync);
      faults_applied = 1;
    }

    double alt_m = profile->altitude_m(t);
    double airspeed_ms = profile->airspeed_ms(t);
    AtmosphereState atm = environment_isa(alt_m);
    double ambient_c = (atm.temperature_k - 273.15) + profile->ambient_offset_c;
    double throttle = profile->throttle(t);

    EnvState env_now;
    environment_state(&env_now, alt_m, profile->ambient_offset_c, airspeed_ms);
    double cool_index =
        environment_cool_index(env_now.density_kg_m3, airspeed_ms, st.rpm);

    printf("%.3f,%.4f,%.1f,%.2f,%.2f,%.4f", t, throttle, alt_m, ambient_c,
           airspeed_ms, cool_index);
    for (int c = 0; c < nchan; c++) {
      printf(",%.*f", chans[c].precision, chans[c].get(&st, chans[c].index));
    }

    if (with_sensor) {
      SensorReading r = sensor_read(&sensor, &st);
      printf(",%.2f,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f,%d", r.value.rpm, r.ok.rpm,
             r.value.map_kpa, r.ok.map_kpa, r.value.cht_c, r.ok.cht_c,
             r.value.egt_c, r.ok.egt_c, r.value.oil_temp_c, r.ok.oil_temp_c);
    }
    printf("\n");

    if (i < nsteps) {
      EngineInput in;
      in.throttle = throttle;
      in.load_torque_nm = load_nm;
      in.ambient_pressure_kpa = atm.pressure_kpa;

      EnvInput env_in;
      env_in.altitude_m = alt_m;
      env_in.airspeed_ms = airspeed_ms;
      env_in.oat_offset_c = profile->ambient_offset_c;
      model_sync_step(&sync, &st, &in, &env_in, dt);
    }
  }

  return 0;
}
