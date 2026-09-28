#ifndef TELEMETRY_INTAKE_TRENDS_H
#define TELEMETRY_INTAKE_TRENDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/trends.h"
#include "util/history.h"

#define INTAKE_TRENDS_CAP TRENDS_CAP

typedef enum {
  INTM_MAP = 0,     /* manifold pressure, kPa */
  INTM_MDOT_IN,     /* throttle-plate flow in, g/s */
  INTM_MDOT_OUT,    /* cylinder induction flow out, g/s */
  INTM_THROTTLE,    /* throttle command the engine received, percent */
  INTM_COUNT
} IntakeMetric;

typedef struct {
  double buf[INTM_COUNT][INTAKE_TRENDS_CAP];
  History hist[INTM_COUNT];
} IntakeTrends;

void intake_trends_init(IntakeTrends *t);

/* Append the current plenum mass-balance readings to every series. */
void intake_trends_sample(IntakeTrends *t, const ModelState *s,
                          const EngineConfig *cfg);

void intake_trends_snapshot(IntakeTrends *dst, const IntakeTrends *src);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_INTAKE_TRENDS_H */
