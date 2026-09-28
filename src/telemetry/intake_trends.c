#include "telemetry/intake_trends.h"

#include "physics/crank_thermo.h"
#include "physics/intake.h"

void intake_trends_init(IntakeTrends *t) {
  for (int m = 0; m < INTM_COUNT; m++) {
    history_init(&t->hist[m], t->buf[m], INTAKE_TRENDS_CAP);
  }
}

void intake_trends_sample(IntakeTrends *t, const ModelState *s,
                          const EngineConfig *cfg) {
  const double total_disp_m3 =
      cylinder_displacement_m3(&cfg->geom) * (double)cfg->num_cylinders;
  const double mdot_in_kg_s =
      throttle_flow_kg_s(s->ecu.throttle_cmd, s->env.ambient_kpa,
                        s->engine.map_kpa, s->env.oat_c, &cfg->intake);
  const double mdot_out_kg_s = engine_induction_air_flow_kg_s(
      s->rpm, s->engine.map_kpa, s->env.oat_c, &cfg->geom, total_disp_m3);

  history_push(&t->hist[INTM_MAP], s->engine.map_kpa);
  history_push(&t->hist[INTM_MDOT_IN], mdot_in_kg_s * 1000.0);
  history_push(&t->hist[INTM_MDOT_OUT], mdot_out_kg_s * 1000.0);
  history_push(&t->hist[INTM_THROTTLE], s->ecu.throttle_cmd * 100.0);
}

void intake_trends_snapshot(IntakeTrends *dst, const IntakeTrends *src) {
  intake_trends_init(dst);
  for (int m = 0; m < INTM_COUNT; m++) {
    const int n = history_count(&src->hist[m]);
    for (int i = 0; i < n; i++) {
      history_push(&dst->hist[m], history_at(&src->hist[m], i));
    }
  }
}
