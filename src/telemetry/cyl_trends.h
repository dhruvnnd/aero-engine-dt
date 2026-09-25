#ifndef TELEMETRY_CYL_TRENDS_H
#define TELEMETRY_CYL_TRENDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/trends.h"
#include "util/history.h"

/*
 * Rolling per-cylinder time-series: one History per (metric, cylinder). Each
 * History points into the matching buf slot, so a CylTrends must not be copied
 * or moved after cyl_trends_init() -- use cyl_trends_snapshot() for a copy.
 */

#define CYL_TRENDS_CAP TRENDS_CAP

typedef enum {
  CYLM_CHT = 0, /* head temperature, degC */
  CYLM_EGT,     /* exhaust gas temperature, degC */
  CYLM_LAMBDA,  /* air/fuel equivalence ratio */
  CYLM_MISFIRE, /* misfire rate, percent */
  CYLM_COUNT
} CylMetric;

typedef struct {
  double buf[CYLM_COUNT][ENGINE_MAX_CYLINDERS][CYL_TRENDS_CAP];
  History hist[CYLM_COUNT][ENGINE_MAX_CYLINDERS];
} CylTrends;

void cyl_trends_init(CylTrends *t);

/* Append the first `num_cyl` cylinders' readings (clamped to
 * ENGINE_MAX_CYLINDERS); the other slots stay empty. */
void cyl_trends_sample(CylTrends *t, const ModelState *s, int num_cyl);

/* Rebuilds `dst` (already a valid, separate CylTrends) as an independent copy
 * of `src`'s current contents. */
void cyl_trends_snapshot(CylTrends *dst, const CylTrends *src);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_CYL_TRENDS_H */
