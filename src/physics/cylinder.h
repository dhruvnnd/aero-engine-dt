#ifndef PHYSICS_CYLINDER_H
#define PHYSICS_CYLINDER_H

#include "physics/thermal.h"

/* Per-cylinder overlay on the lumped engine model. */

typedef struct {
  double injector_flow_trim; /* 1.0 = nominal delivered fuel mass */
  double compression_trim;   /* 1.0 = nominal effective compression */
  double spark_offset_deg;   /* 0.0 = fires at the commanded advance */
  double intake_leak_frac;   /* 0.0 = sealed; >0 leans this cylinder */
  double cooling_trim;       /* 1.0 = nominal head heat rejection */
} CylinderConfig;

typedef struct {
  double cht_c;    /* cylinder head temperature */
  double egt_c;    /* exhaust gas temperature at this port */
  double lambda;   /* air/fuel equivalence ratio, 1.0 = stoichiometric */
  double imep_bar; /* indicated mean effective pressure (load proxy) */
  double ca50_deg; /* crank angle of 50% burn, deg ATDC (combustion phasing) */
  double fuel_pw_ms;   /* injector pulse width */
  double misfire_rate; /* fraction of recent cycles that failed to fire, 0..1 */
} CylinderState;

/* All trims nominal (a healthy cylinder). */
CylinderConfig cylinder_config_default(void);

/* Cold start: temperatures at ambient, lambda stoichiometric, every other
 * output zero. */
void cylinder_state_init(CylinderState *state, double ambient_temp_c);

/* Advances this cylinder's cht_c and egt_c nodes by dt and refreshes its
 * algebraic outputs (lambda, imep_bar, fuel_pw_ms, ca50_deg, misfire_rate).
 */
void cylinder_step(CylinderState *state, const CylinderConfig *config,
                   double map_kpa, double omega_rad_s, double ambient_c,
                   const ThermalConfig *thermal_cfg, int num_cylinders,
                   double t, double dt);

#endif /* PHYSICS_CYLINDER_H */
