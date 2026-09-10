#include "physics/lubrication.h"

LubeConfig lube_config_default(void) {
  LubeConfig c;
  c.relief_valve_kpa = 500.0;    /* ~72 psi relief setting */
  c.k_pump_kpa_per_rpm = 0.42;   /* tuned: relief at cold cruise, ~150 kPa hot idle */
  c.visc_ref_temp_c = 15.0;      /* cold-start ambient */
  c.visc_falloff_per_c = 0.006;  /* ~half viscosity by ~100 degC */
  c.bearing_wear = 0.0;
  return c;
}

void lube_state_init(LubeState *state) { state->oil_press_kpa = 0.0; }

/* Relative oil viscosity: 1.0 at the reference temperature, falling as the
 * oil warms and thins. Clamped so it stays positive and bounded. */
static double viscosity_factor(const LubeConfig *c, double oil_temp_c) {
  double v = 1.0 - c->visc_falloff_per_c * (oil_temp_c - c->visc_ref_temp_c);
  if (v < 0.05) {
    v = 0.05;
  }
  if (v > 1.5) {
    v = 1.5;
  }
  return v;
}

void lube_step(LubeState *state, const LubeConfig *config, double rpm,
               double oil_temp_c) {
  double wear = config->bearing_wear > 0.0 ? config->bearing_wear : 0.0;

  double p = config->k_pump_kpa_per_rpm * rpm *
             viscosity_factor(config, oil_temp_c) / (1.0 + wear);

  if (p > config->relief_valve_kpa) {
    p = config->relief_valve_kpa;
  }
  if (p < 0.0) {
    p = 0.0;
  }
  state->oil_press_kpa = p;
}
