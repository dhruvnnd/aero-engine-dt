#include "physics/fuel.h"

#include "math/units.h"

FuelConfig fuel_config_default(void) {
  FuelConfig c;
  c.afr_stoich = 14.7;      /* gasoline */
  c.lambda_target = 1.0;    /* stoichiometric */
  c.vol_eff = 0.85;         /* naturally aspirated, mid-range */
  c.displacement_l = 2.0;   /* small aero piston engine, placeholder */
  c.pump_press_kpa = 300.0; /* ~3 bar rail */
  return c;
}

void fuel_state_init(FuelState *state) {
  state->air_flow_gps = 0.0;
  state->fuel_flow_kgph = 0.0;
  state->fuel_press_kpa = 0.0;
}

/* Engine air mass flow, g/s. Four-stroke: each cylinder ingests one full
 * charge every two crank revolutions. */
static double air_flow_gps(const FuelConfig *c, double map_kpa,
                           double intake_temp_c, double rpm) {
  const double air_r_j_per_kg_k = 287.05;

  double rho_kg_m3 = kpa_to_pa(map_kpa) /
                     (air_r_j_per_kg_k * celsius_to_kelvin(intake_temp_c));
  double disp_m3 = c->displacement_l * 1.0e-3;
  double volume_flow_m3_s = c->vol_eff * disp_m3 * (rpm / 60.0) / 2.0;
  return rho_kg_m3 * volume_flow_m3_s * 1000.0; /* kg -> g */
}

void fuel_step(FuelState *state, const FuelConfig *config,
               const CylinderConfig *cyl, int num_cylinders, double map_kpa,
               double rpm, double intake_temp_c) {
  int n = num_cylinders > 0 ? num_cylinders : 1;

  double air_gps = air_flow_gps(config, map_kpa, intake_temp_c, rpm);

  /* ECU meters total fuel to the commanded lambda against total air... */
  double lambda_cmd = config->lambda_target > 0.1 ? config->lambda_target : 0.1;
  double fuel_cmd_gps = air_gps / (config->afr_stoich * lambda_cmd);
  /* ...then each injector delivers its trimmed share. */
  double trim_sum = 0.0;
  for (int i = 0; i < n; i++) {
    trim_sum += cyl[i].injector_flow_trim;
  }
  double fuel_gps = (fuel_cmd_gps / n) * trim_sum;

  state->air_flow_gps = air_gps;
  state->fuel_flow_kgph = fuel_gps * 3.6; /* g/s -> kg/h */
  state->fuel_press_kpa =
      config->pump_press_kpa - 0.8 * state->fuel_flow_kgph; /* demand droop */
}
