#include "telemetry/trends.h"

void trends_init(Trends *t) {
  history_init(&t->rpm, t->rpm_buf, TRENDS_CAP);
  history_init(&t->cht, t->cht_buf, TRENDS_CAP);
  history_init(&t->egt, t->egt_buf, TRENDS_CAP);
  history_init(&t->oil_press, t->oil_press_buf, TRENDS_CAP);
  history_init(&t->batt_pct, t->batt_pct_buf, TRENDS_CAP);
}

void trends_sample(Trends *t, const ModelState *s) {
  history_push(&t->rpm, s->rpm);
  history_push(&t->cht, s->thermal.cht_c);
  history_push(&t->egt, s->thermal.egt_c);
  history_push(&t->oil_press, s->lube.oil_press_kpa);
  history_push(&t->batt_pct, s->elec.batt_soc * 100.0);
}
