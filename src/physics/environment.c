#include "physics/environment.h"

#include <math.h>

#include "math/units.h"

/* Standard troposphere constants (ISA). */
#define ISA_SEA_LEVEL_TEMP_K 288.15
#define ISA_SEA_LEVEL_PRESSURE_KPA 101.325
#define ISA_LAPSE_RATE_K_PER_M 0.0065
#define ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K 287.05
#define ISA_GRAVITY_M_S2 9.80665

AtmosphereState environment_isa(double altitude_m) {
  AtmosphereState atm;

  atm.temperature_k =
      ISA_SEA_LEVEL_TEMP_K - ISA_LAPSE_RATE_K_PER_M * altitude_m;

  double temp_ratio = atm.temperature_k / ISA_SEA_LEVEL_TEMP_K;
  double pressure_exponent =
      ISA_GRAVITY_M_S2 /
      (ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K * ISA_LAPSE_RATE_K_PER_M);
  atm.pressure_kpa =
      ISA_SEA_LEVEL_PRESSURE_KPA * pow(temp_ratio, pressure_exponent);

  atm.density_kg_m3 =
      kpa_to_pa(atm.pressure_kpa) /
      (ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K * atm.temperature_k);

  return atm;
}

double environment_density_altitude_m(double density_kg_m3) {
  const double rho0 = ISA_SEA_LEVEL_PRESSURE_KPA * 1000.0 /
                      (ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K * ISA_SEA_LEVEL_TEMP_K);
  /* rho/rho0 = (1 - L h / T0)^n,  n = g/(R L) - 1  (ISA troposphere) */
  const double n = ISA_GRAVITY_M_S2 / (ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K *
                                       ISA_LAPSE_RATE_K_PER_M) -
                   1.0;

  double ratio = density_kg_m3 / rho0;
  if (ratio <= 0.0) {
    ratio = 1.0e-6;
  }
  return (ISA_SEA_LEVEL_TEMP_K / ISA_LAPSE_RATE_K_PER_M) *
         (1.0 - pow(ratio, 1.0 / n));
}

void environment_state(EnvState *env, double altitude_m, double oat_offset_c,
                       double airspeed_ms) {
  AtmosphereState isa = environment_isa(altitude_m);

  env->altitude_m = altitude_m;
  env->airspeed_ms = airspeed_ms;
  env->oat_c = (isa.temperature_k - 273.15) + oat_offset_c;
  env->ambient_kpa = isa.pressure_kpa;
  env->density_kg_m3 =
      kpa_to_pa(isa.pressure_kpa) /
      (ISA_SPECIFIC_GAS_CONSTANT_J_PER_KG_K * celsius_to_kelvin(env->oat_c));
  env->density_alt_m = environment_density_altitude_m(env->density_kg_m3);
  env->dynamic_press_pa =
      0.5 * env->density_kg_m3 * airspeed_ms * airspeed_ms;
}

double environment_cool_index(double density_kg_m3, double airspeed_ms,
                              double rpm) {
  const double rho0 = 1.225;
  const double v_ref_ms = 50.0;   /* reference cruise true airspeed */
  const double rpm_ref = 1900.0;  /* reference cruise crank speed */
  const double k_prop = 48.6;     /* propwash weight; sets ground-idle cooling */

  double v = airspeed_ms > 0.0 ? airspeed_ms : 0.0;
  double n = rpm > 0.0 ? rpm : 0.0;

  double ram_and_prop = density_kg_m3 * v + k_prop * (n / rpm_ref);
  double reference = rho0 * v_ref_ms + k_prop;
  double idx = ram_and_prop / reference;

  if (idx < 0.05) {
    idx = 0.05;
  }
  if (idx > 3.0) {
    idx = 3.0;
  }
  return idx;
}
