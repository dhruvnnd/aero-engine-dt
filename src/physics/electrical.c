#include "physics/electrical.h"

ElecConfig elec_config_default(void) {
  ElecConfig c;
  c.bus_nominal_v = 14.2;
  c.alt_rated_a = 40.0;
  c.alt_cutin_rpm = 800.0;
  c.alt_full_output_rpm = 1800.0;
  c.alt_health = 1.0;
  c.load_base_a = 15.0;
  c.batt_capacity_ah = 15.0;
  c.batt_open_v = 12.6;
  c.batt_internal_r_ohm = 0.03;
  return c;
}

void elec_state_init(ElecState *state) {
  state->bus_v = 0.0;
  state->alt_current_a = 0.0;
  state->alt_field_a = 0.0;
  state->batt_soc = 1.0;
}

/* Current the alternator can deliver at this speed and health. */
static double alt_capacity_a(const ElecConfig *c, double rpm) {
  double frac;
  if (rpm <= c->alt_cutin_rpm) {
    frac = 0.0;
  } else if (rpm >= c->alt_full_output_rpm) {
    frac = 1.0;
  } else {
    frac = (rpm - c->alt_cutin_rpm) /
           (c->alt_full_output_rpm - c->alt_cutin_rpm);
  }
  double h = c->alt_health < 0.0 ? 0.0 : (c->alt_health > 1.0 ? 1.0 : c->alt_health);
  return c->alt_rated_a * frac * h;
}

void elec_step(ElecState *state, const ElecConfig *config, double rpm,
               double dt) {
  double alt_cap_a = alt_capacity_a(config, rpm);
  double load_a = config->load_base_a;

  double batt_i_a; /* + discharging, - charging */
  double alt_out_a;

  if (alt_cap_a >= load_a) {
    /* Alternator carries the load and tops up the battery, tapered. */
    double taper_a = 0.3 * config->batt_capacity_ah;
    double surplus_a = alt_cap_a - load_a;
    double charge_a = state->batt_soc < 1.0
                          ? (surplus_a < taper_a ? surplus_a : taper_a)
                          : 0.0;
    batt_i_a = -charge_a;
    alt_out_a = load_a + charge_a;
    state->bus_v = config->bus_nominal_v;
  } else {
    /* Shortfall comes off the battery. */
    batt_i_a = load_a - alt_cap_a;
    alt_out_a = alt_cap_a;
    double ocv = config->batt_open_v * (0.92 + 0.08 * state->batt_soc);
    state->bus_v = ocv - batt_i_a * config->batt_internal_r_ohm;
    if (state->bus_v < 0.0) {
      state->bus_v = 0.0;
    }
  }

  double dsoc = -batt_i_a * dt / (config->batt_capacity_ah * 3600.0);
  double soc = state->batt_soc + dsoc;
  state->batt_soc = soc < 0.0 ? 0.0 : (soc > 1.0 ? 1.0 : soc);

  state->alt_current_a = alt_out_a;
  state->alt_field_a =
      config->alt_rated_a > 0.0 ? 2.5 * alt_out_a / config->alt_rated_a : 0.0;
}
