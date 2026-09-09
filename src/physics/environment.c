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
