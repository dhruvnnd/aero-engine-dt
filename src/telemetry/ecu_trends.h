#ifndef TELEMETRY_ECU_TRENDS_H
#define TELEMETRY_ECU_TRENDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/trends.h"
#include "util/history.h"

#define ECU_TRENDS_CAP TRENDS_CAP

typedef enum {
  ECUM_RPM = 0,  /* measured crank speed, rpm */
  ECUM_TARGET,   /* idle target, rpm (0 when none) */
  ECUM_PILOT,    /* pilot throttle, percent */
  ECUM_GOVERNOR, /* governor output (what it adds), percent */
  ECUM_COMMAND,  /* throttle command the engine received, percent */
  ECUM_P_TERM,   /* proportional term, percent throttle */
  ECUM_I_TERM,   /* integrator, percent throttle */
  ECUM_COUNT
} EcuMetric;

typedef struct {
  double buf[ECUM_COUNT][ECU_TRENDS_CAP];
  History hist[ECUM_COUNT];
} EcuTrends;

void ecu_trends_init(EcuTrends *t);

/* Append the current ECU readings to every series. */
void ecu_trends_sample(EcuTrends *t, const ModelState *s);

void ecu_trends_snapshot(EcuTrends *dst, const EcuTrends *src);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_ECU_TRENDS_H */
