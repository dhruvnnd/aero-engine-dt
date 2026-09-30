#ifndef TELEMETRY_PV_DIAGRAM_H
#define TELEMETRY_PV_DIAGRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Pressure-volume (indicator) diagram of one cylinder
 *
 * Cylinder-local angle convention (as in crank_thermo.h): 0 = firing TDC,
 * expansion 0-180, exhaust 180-360, intake 360-540, compression 540-720.
 */

#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "physics/engine_trace.h"

/* Polytropic index the engine model integrates with */
#define PV_MODEL_POLYTROPIC_N 1.3

typedef struct {
  int n; /* points in the loop, in increasing cylinder-local angle */
  float angle_deg[ENGINE_TRACE_CAPACITY]; /* cylinder-local, [0,720) */
  float v_cc[ENGINE_TRACE_CAPACITY];
  float p_kpa[ENGINE_TRACE_CAPACITY];
} PvLoop;

typedef struct {
  int valid; /* 0 if the trace had no complete cycle to work from */

  double v_min_cc;   /* clearance volume at TDC */
  double v_max_cc;   /* clearance + displacement at BDC */
  double comp_ratio; /* v_max / v_min, the effective compression ratio */

  double peak_kpa;       /* highest in-cylinder pressure over the cycle */
  double peak_angle_deg; /* where it occurred, deg ATDC in (-360, 360] */

  double work_net_j;   /* integral of P dV over the whole 720 deg cycle */
  double work_gross_j; /* ... over the compression + expansion strokes only */
  double work_pump_j;  /* ... over the exhaust + intake strokes (usually < 0) */
  double imep_net_kpa; /* work_net / displacement */
  double imep_gross_kpa; /* work_gross / displacement */
  double pmep_kpa;       /* -work_pump / displacement; pumping loss */

  double rpm;      /* mean crank speed over the cycle */
  double power_kw; /* net indicated power: work_net * rpm / 120 */

  /* Fitted polytropic index of the compression stroke */
  double compression_n;

  double ivc_p_kpa;
  double ivc_v_cc;
} PvStats;

int pv_loop_build(const EngineTrace *trace, const EngineConfig *cfg,
                  const CylinderConfig *cyl_cfg, int cyl, int cycles_back,
                  PvLoop *loop, PvStats *stats);

/* Cylinder volume in cc at a cylinder-local angle, for the same geometry and
 * compression trim the loop uses */
double pv_volume_cc(const EngineConfig *cfg, const CylinderConfig *cyl_cfg,
                    double local_deg);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PV_DIAGRAM_H */
