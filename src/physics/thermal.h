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
} ThermalConfig;

ThermalConfig thermal_config_default(void);

/* Sets all three temperatures to ambient (cold start). */
void thermal_init(ThermalState *state, double ambient_temp_c);

/* Advances state by dt seconds given the current waste heat release rate and
 * ambient temperature */
void thermal_step(ThermalState *state, const ThermalConfig *config,
                  double waste_heat_w, double ambient_temp_c, double t,
                  double dt);

#endif /* PHYSICS_THERMAL_H */
