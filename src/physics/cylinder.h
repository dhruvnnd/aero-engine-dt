#ifndef PHYSICS_CYLINDER_H
#define PHYSICS_CYLINDER_H

#ifdef __cplusplus
extern "C" {
#endif

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
  double cyl_pressure_kpa; /* in-cylinder pressure */
} CylinderState;

/* All trims nominal (a healthy cylinder). */
CylinderConfig cylinder_config_default(void);

/* Cold start: temperatures at ambient, lambda stoichiometric, every other
 * output zero. */
void cylinder_state_init(CylinderState *state, double ambient_temp_c);

/* What the crank-angle model says one cylinder did over the last step,
 * averaged over about two engine cycles; the cylinder's thermal nodes are
 * driven from this. */
typedef struct {
  double heat_w;       /* mean heat released into the gas by combustion, W */
  double imep_kpa;     /* net indicated mean effective pressure (incl. pumping) */
  double friction_w;   /* this cylinder's share of the engine's friction power */
  double gas_flow_kg_s; /* charge (air + fuel) passing through it, kg/s */
  double blowdown_c;   /* gas temperature when the exhaust valve last opened */
} CylinderThermalInput;

/* Advances this cylinder's cht_c and egt_c nodes by dt and refreshes its
 * algebraic outputs (lambda, imep_bar, fuel_pw_ms, ca50_deg, misfire_rate).
 * The head chases ambient + head heat * thermal_cfg->cht_k_per_kw / cooling;
 * the exhaust port chases ambient + egt_port_factor * (blowdown - ambient).
 * `cool_index` (from environment_cool_index) scales head cooling. */
void cylinder_step(CylinderState *state, const CylinderConfig *config,
                   const CylinderThermalInput *in, double map_kpa,
                   double ambient_c, const ThermalConfig *thermal_cfg,
                   double cool_index, double t, double dt);

/* This cylinder's air/fuel equivalence ratio from its trims alone (>1.0 =
 * lean, from injector_flow_trim < 1.0 and/or intake_leak_frac > 0) */
double cylinder_lambda(const CylinderConfig *c);

/* Fraction of cycles that fail to fire outright */
double misfire_fraction(double lambda);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_CYLINDER_H */
