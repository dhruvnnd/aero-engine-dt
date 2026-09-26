#include "telemetry/ecu_trends.h"

void ecu_trends_init(EcuTrends *t) {
  for (int m = 0; m < ECUM_COUNT; m++) {
    history_init(&t->hist[m], t->buf[m], ECU_TRENDS_CAP);
  }
}

void ecu_trends_sample(EcuTrends *t, const ModelState *s) {
  const EcuState *e = &s->engine.ecu;
  history_push(&t->hist[ECUM_RPM], s->rpm);
  history_push(&t->hist[ECUM_TARGET], e->idle_target_rpm);
  history_push(&t->hist[ECUM_PILOT], e->pilot_throttle * 100.0);
  history_push(&t->hist[ECUM_GOVERNOR], e->idle_throttle * 100.0);
  history_push(&t->hist[ECUM_COMMAND], e->throttle_cmd * 100.0);
  history_push(&t->hist[ECUM_P_TERM], e->idle_p_term * 100.0);
  history_push(&t->hist[ECUM_I_TERM], e->idle_i_term * 100.0);
}

void ecu_trends_snapshot(EcuTrends *dst, const EcuTrends *src) {
  ecu_trends_init(dst);
  for (int m = 0; m < ECUM_COUNT; m++) {
    const int n = history_count(&src->hist[m]);
    for (int i = 0; i < n; i++) {
      history_push(&dst->hist[m], history_at(&src->hist[m], i));
    }
  }
}
