#ifndef PHYSICS_THERMAL_H
#define PHYSICS_THERMAL_H

typedef struct {
  double cht_c;      /* cylinder head temperature */
  double egt_c;      /* exhaust gas temperature */
  double oil_temp_c; /* oil temperature */
} ThermalState;

typedef struct {
  double cht_tau_s;
  double egt_tau_s;
  double oil_tau_s;

  /* CHT/EGT chase a temperature *rise* above ambient that saturates with
   * engine load: rise = rise_rated_c * sqrt(load_fraction). Oil still tracks
   * a linear function of raw waste heat (it's a slow bulk sink). */
  double cht_rise_rated_c;
  double egt_rise_rated_c;
  double oil_gain_c_per_w;
} ThermalConfig;

ThermalConfig thermal_config_default(void);

/* Load-saturating temperature rise above ambient:
 *   rise_rated_c * sqrt(clamp(load_frac, 0, 1))
 * Shared by the engine-wide nodes here and the per-cylinder nodes in
 * physics/cylinder.c so they stay consistent. */
double thermal_rise_c(double rise_rated_c, double load_frac);

/* Sets all three temperatures to ambient (cold start). */
void thermal_init(ThermalState *state, double ambient_temp_c);

/* Advances state by dt seconds. `waste_heat_w` drives the oil node;
 * `load_frac` (0..1, from combustion_load_fraction) drives CHT and EGT. */
void thermal_step(ThermalState *state, const ThermalConfig *config,
                  double waste_heat_w, double load_frac, double ambient_temp_c,
                  double t, double dt);

#endif /* PHYSICS_THERMAL_H */
