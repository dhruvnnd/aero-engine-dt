#include "telemetry/ecu_compare.h"

/* the ECU counts as compensating above this much added throttle, percent */
#define COMPENSATING_MIN_PCT 0.1

void ecu_compare_init(EcuCompareTrends *t) {
  for (int m = 0; m < ECUC_COUNT; m++) {
    history_init(&t->hist[m], t->buf[m], ECU_COMPARE_CAP);
  }
}

void ecu_compare_sample(EcuCompareTrends *t, const ModelState *with_ecu,
                        const ModelState *without_ecu) {
  history_push(&t->hist[ECUC_RPM_ECU], with_ecu->rpm);
  history_push(&t->hist[ECUC_RPM_BARE], without_ecu->rpm);
  history_push(&t->hist[ECUC_THR_ECU], with_ecu->ecu.throttle_cmd * 100.0);
  history_push(&t->hist[ECUC_THR_BARE], without_ecu->ecu.throttle_cmd * 100.0);
}

void ecu_compare_snapshot(EcuCompareTrends *dst, const EcuCompareTrends *src) {
  ecu_compare_init(dst);
  for (int m = 0; m < ECUC_COUNT; m++) {
    const int n = history_count(&src->hist[m]);
    for (int i = 0; i < n; i++) {
      history_push(&dst->hist[m], history_at(&src->hist[m], i));
    }
  }
}

EcuCompareSummary ecu_compare_summarize(const ModelState *with_ecu,
                                        const ModelState *without_ecu) {
  EcuCompareSummary s;
  s.rpm_gain = with_ecu->rpm - without_ecu->rpm;
  s.extra_throttle_pct =
      (with_ecu->ecu.throttle_cmd - with_ecu->ecu.pilot_throttle) * 100.0;
  if (s.extra_throttle_pct < 0.0) {
    s.extra_throttle_pct = 0.0;
  }
  s.compensating = s.extra_throttle_pct > COMPENSATING_MIN_PCT;
  s.bare_stalled = without_ecu->engine.run_state == ENGINE_STOPPED &&
                   with_ecu->engine.run_state != ENGINE_STOPPED;
  return s;
}
