#ifndef TELEMETRY_ECU_COMPARE_H
#define TELEMETRY_ECU_COMPARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/trends.h"
#include "util/history.h"

#define ECU_COMPARE_CAP TRENDS_CAP

typedef enum {
  ECUC_RPM_ECU = 0, /* crank speed with the ECU, rpm */
  ECUC_RPM_BARE,    /* crank speed without it, rpm */
  ECUC_THR_ECU,     /* throttle the engine got with the ECU, percent */
  ECUC_THR_BARE,    /* ... without it (the pilot's), percent */
  ECUC_COUNT
} EcuCompareMetric;

typedef struct {
  double buf[ECUC_COUNT][ECU_COMPARE_CAP];
  History hist[ECUC_COUNT];
} EcuCompareTrends;

void ecu_compare_init(EcuCompareTrends *t);

/* Append the current readings of both runs. */
void ecu_compare_sample(EcuCompareTrends *t, const ModelState *with_ecu,
                        const ModelState *without_ecu);

/* Rebuilds `dst` (already a valid, separate EcuCompareTrends) as an independent
 * copy of `src`'s current contents. */
void ecu_compare_snapshot(EcuCompareTrends *dst, const EcuCompareTrends *src);

typedef struct {
  double rpm_gain;           /* rpm with the ECU minus rpm without it */
  double extra_throttle_pct; /* throttle the ECU adds beyond the pilot's, % */
  int compensating;          /* the ECU is adding throttle right now */
  int bare_stalled;          /* the engine without the ECU has stalled while the
                                one with it is still going */
} EcuCompareSummary;

EcuCompareSummary ecu_compare_summarize(const ModelState *with_ecu,
                                        const ModelState *without_ecu);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_ECU_COMPARE_H */
