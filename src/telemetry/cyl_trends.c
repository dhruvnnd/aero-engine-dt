#include "telemetry/cyl_trends.h"

void cyl_trends_init(CylTrends *t) {
  for (int m = 0; m < CYLM_COUNT; m++) {
    for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
      history_init(&t->hist[m][c], t->buf[m][c], CYL_TRENDS_CAP);
    }
  }
}

void cyl_trends_sample(CylTrends *t, const ModelState *s, int num_cyl) {
  const int n = num_cyl < 0
                    ? 0
                    : (num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS
                                                      : num_cyl);
  for (int c = 0; c < n; c++) {
    const CylinderState *cs = &s->cyl[c];
    history_push(&t->hist[CYLM_CHT][c], cs->cht_c);
    history_push(&t->hist[CYLM_EGT][c], cs->egt_c);
    history_push(&t->hist[CYLM_LAMBDA][c], cs->lambda);
    history_push(&t->hist[CYLM_MISFIRE][c], cs->misfire_rate * 100.0);
  }
}

void cyl_trends_snapshot(CylTrends *dst, const CylTrends *src) {
  cyl_trends_init(dst);
  for (int m = 0; m < CYLM_COUNT; m++) {
    for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
      const int n = history_count(&src->hist[m][c]);
      for (int i = 0; i < n; i++) {
        history_push(&dst->hist[m][c], history_at(&src->hist[m][c], i));
      }
    }
  }
}
