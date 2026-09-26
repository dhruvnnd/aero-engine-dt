#ifndef PHYSICS_PROPELLER_H
#define PHYSICS_PROPELLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed-pitch propeller, direct-drive (prop speed = crank speed) */

typedef struct {
  double diameter_m;
  double j_zero_thrust; /* advance ratio at which thrust falls to zero,
                           roughly 0.9 * pitch / diameter */
  double ct_static;     /* thrust coefficient at J = 0 */
  double cq_static;     /* torque coefficient at J = 0; 0 = no propeller
                           (the engine runs unloaded, dyno-style) */
  double cq_unload; /* fraction of static torque lost by J = j_zero_thrust */
} PropConfig;

typedef struct {
  double advance_ratio; /* J, after the PROP_X_MAX cap */
  double thrust_n;
  double torque_nm; /* load on the crank, >= 0 */
} PropState;

PropConfig prop_config_default(void);

/* Advance ratio V / (n * D); a stopped prop reports the capped maximum. */
double prop_advance_ratio(const PropConfig *cfg, double rpm,
                          double airspeed_ms);

/* Thrust and torque at the given crank speed, true airspeed and air density.
 * Negative rpm / airspeed are treated as zero. */
void prop_step(PropState *state, const PropConfig *cfg, double rpm,
               double airspeed_ms, double density_kg_m3);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_PROPELLER_H */
