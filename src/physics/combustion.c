#include "physics/combustion.h"

/* Fraction of combustion chemical energy that becomes indicated work; the
 * rest becomes waste heat, split between cylinder-wall heat transfer
 * (-> CHT) and hot exhaust gas (-> EGT). ~0.30 is in
 * the right ballpark for a naturally aspirated gasoline piston engine's
 * indicated efficiency */
#define COMBUSTION_INDICATED_EFFICIENCY 0.30

double combustion_indicated_torque_nm(double map_kpa, double omega_rad_s) {
  const double k_torque_nm_per_kpa = 2.2;
  const double falloff_scale_rad_s = 300.0;
  const double falloff_min = 0.15;

  double falloff = 1.0 - (omega_rad_s / falloff_scale_rad_s);
  if (falloff < falloff_min) {
    falloff = falloff_min;
  }
  if (falloff > 1.0) {
    falloff = 1.0;
  }

  return k_torque_nm_per_kpa * map_kpa * falloff;
}

double combustion_indicated_power_w(double map_kpa, double omega_rad_s) {
  return combustion_indicated_torque_nm(map_kpa, omega_rad_s) * omega_rad_s;
}

double combustion_waste_heat_w(double map_kpa, double omega_rad_s) {
  double indicated_power_w = combustion_indicated_power_w(map_kpa, omega_rad_s);
  double total_energy_w = indicated_power_w / COMBUSTION_INDICATED_EFFICIENCY;
  return total_energy_w - indicated_power_w;
}

double combustion_load_fraction(double map_kpa, double omega_rad_s) {
  /* Roughly the peak indicated power this torque curve can make, for a
   * placeholder ~2 L engine. Tune with the rest of the perf map. */
  const double rated_indicated_power_w = 16000.0;

  double lf = combustion_indicated_power_w(map_kpa, omega_rad_s) /
              rated_indicated_power_w;
  if (lf < 0.0) {
    lf = 0.0;
  }
  if (lf > 1.0) {
    lf = 1.0;
  }
  return lf;
}
