#include "physics/intake.h"

#include <math.h>

#include "math/units.h"

#define AIR_K 1.4 /* ratio of specific heats */
#define AIR_R_J_PER_KGK 287.05
#define THROTTLE_SHAPE_POWER                                                   \
  1.4 /* tunes how front/back-loaded the plate's                               \
       * open-area curve is; see below */

IntakeConfig intake_config_default(void) {
  IntakeConfig c;
  c.plenum_vol_m3 = 0.0025; /* ~2.5 L, a small four-cylinder's manifold */
  c.throttle_bore_m = 0.068;
  c.throttle_leak_frac = 0.005;
  c.throttle_cd = 0.85;
  return c;
}

double throttle_flow_kg_s(double throttle, double upstream_kpa,
                          double downstream_kpa, double upstream_temp_c,
                          const IntakeConfig *cfg) {
  if (downstream_kpa >= upstream_kpa) {
    return 0.0;
  }

  double thr = throttle < 0.0 ? 0.0 : (throttle > 1.0 ? 1.0 : throttle);
  double shape = 1.0 - cos(pow(thr, THROTTLE_SHAPE_POWER) * UNITS_PI / 2.0);
  double area_frac =
      cfg->throttle_leak_frac + (1.0 - cfg->throttle_leak_frac) * shape;
  double area_m2 = area_frac * (UNITS_PI / 4.0) * cfg->throttle_bore_m *
                   cfg->throttle_bore_m;

  double p_up_pa = kpa_to_pa(upstream_kpa);
  double t_up_k = celsius_to_kelvin(upstream_temp_c);
  double pr = kpa_to_pa(downstream_kpa) / p_up_pa;

  const double pr_crit = pow(2.0 / (AIR_K + 1.0), AIR_K / (AIR_K - 1.0));
  double flow_fn;
  if (pr <= pr_crit) {
    /* choked: downstream pressure no longer affects the flow rate */
    flow_fn = sqrt(AIR_K / (AIR_R_J_PER_KGK * t_up_k)) *
              pow(2.0 / (AIR_K + 1.0), (AIR_K + 1.0) / (2.0 * (AIR_K - 1.0)));
  } else {
    flow_fn = sqrt((2.0 * AIR_K) / (AIR_R_J_PER_KGK * t_up_k * (AIR_K - 1.0)) *
                   (pow(pr, 2.0 / AIR_K) - pow(pr, (AIR_K + 1.0) / AIR_K)));
  }
  return cfg->throttle_cd * area_m2 * p_up_pa * flow_fn;
}
