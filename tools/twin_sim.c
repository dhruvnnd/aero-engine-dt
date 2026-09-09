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

#include "physics/environment.h"
#include "telemetry/sensor.h"
#include "model/channels.h"
#include "model/sync.h"
#include "model/state.h"

typedef struct {
  const char *name;
  const char *description;
  double duration_s; /* default; overridable with --duration */
  double (*throttle)(double t);
  double (*altitude_m)(double t);
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

static double thr_takeoff(double t) { return clamp01(t / 5.0); }
static double alt_takeoff(double t) {
  double climb = (t - 10.0) * 10.0; /* 10 m/s, after a 10 s ground roll */
  return climb < 0.0 ? 0.0 : climb;
}

static double thr_cruise(double t) {
  (void)t;
  return 0.75;
}
static double alt_cruiseclimb(double t) {
  double a = t * 5.0; /* 5 m/s */
  return a > 4500.0 ? 4500.0 : a;
}

static double thr_hotday(double t) {
  (void)t;
  return 0.80;
}

static double thr_rapid(double t) {
  return fmod(t, 20.0) < 10.0 ? 0.20 : 1.00; /* square wave, 20 s period */
}

static const SimProfile PROFILES[] = {
    {"idle", "closed throttle, sea level", 120.0, thr_idle, alt_ground, 0.0},
    {"takeoff", "throttle ramp to WOT, ground roll then 10 m/s climb", 120.0,
     thr_takeoff, alt_takeoff, 0.0},
    {"cruise-climb", "0.75 throttle, 5 m/s climb capped at 4500 m", 900.0,
     thr_cruise, alt_cruiseclimb, 0.0},
    {"hot-day", "0.80 throttle, sea level, ISA +25 C", 300.0, thr_hotday,
     alt_ground, 25.0},
    {"rapid-throttle", "0.20 <-> 1.00 square wave, 20 s period, sea level",
     120.0, thr_rapid, alt_ground, 0.0},
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
          "          [--load NM] [--seed N] [--sensor] [--list] [--help]\n\n"
          "Writes a CSV engine/thermal time-series to stdout; progress and\n"
          "run parameters go to stderr. Add --sensor for the noisy sensor\n"
          "channels alongside ground truth.\n\n",
          argv0);
  print_profiles(out);
}

int main(int argc, char **argv) {
  const char *profile_name = "idle";
  double dt = 0.02;
  double duration_s = -1.0; /* < 0 => use the profile's default */
  double load_nm = 40.0;
  uint32_t seed = 1u;
  int with_sensor = 0;

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
          "# twin_sim profile=%s dt=%.4f duration=%.1f load=%.1f seed=%u%s\n",
          profile->name, dt, duration_s, load_nm, seed,
          with_sensor ? " +sensor" : "");

  int nchan = 0;
  const ModelChannel *chans = model_channels(&nchan);

  printf("t,throttle,alt_m,ambient_c");
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

    double alt_m = profile->altitude_m(t);
    AtmosphereState atm = environment_isa(alt_m);
    double ambient_c = (atm.temperature_k - 273.15) + profile->ambient_offset_c;
    double throttle = profile->throttle(t);

    printf("%.3f,%.4f,%.1f,%.2f", t, throttle, alt_m, ambient_c);
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
      model_sync_step(&sync, &st, &in, ambient_c, dt);
    }
  }

  return 0;
}
