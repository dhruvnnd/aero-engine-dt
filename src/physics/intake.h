#ifndef PHYSICS_INTAKE_H
#define PHYSICS_INTAKE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Throttle-plate flow and intake plenum sizing */

typedef struct {
  double plenum_vol_m3;      /* throttle body + manifold volume upstream of
                                the intake ports */
  double throttle_bore_m;    /* throttle plate diameter at full open */
  double throttle_leak_frac; /* flow area open even at closed throttle
                                (idle bypass / imperfect seal), fraction of
                                the full-open area */
  double throttle_cd;        /* discharge coefficient */
} IntakeConfig;

IntakeConfig intake_config_default(void);

/* Mass flow through the throttle plate, kg/s, from upstream (ambient) to
 * downstream (manifold) pressure */
double throttle_flow_kg_s(double throttle, double upstream_kpa,
                          double downstream_kpa, double upstream_temp_c,
                          const IntakeConfig *cfg);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_INTAKE_H */
