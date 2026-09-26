#include "telemetry/engine_faults.h"
#include "physics/engine_trace.h"

#include <math.h>
#include <stdio.h>

#define TRIM_TOLERANCE 1e-6

/* Same rule as the Cylinders panel's misfire column. */
static ChannelStatus misfire_status(double rate) {
  return rate > 0.5 ? CHANNEL_ALERT : (rate > 0.0 ? CHANNEL_WARN : CHANNEL_OK);
}

static ChannelStatus worse(ChannelStatus a, ChannelStatus b) {
  return (int)b > (int)a ? b : a; /* OK < WARN < ALERT */
}

void engine_faults_init(EngineFaultTracker *t) {
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    CylFaultTrack z = {0};
    t->cyl[i] = z;
  }
}

int cylinder_has_fault(const CylinderConfig *c) {
  return fabs(c->injector_flow_trim - 1.0) > TRIM_TOLERANCE ||
         fabs(c->compression_trim - 1.0) > TRIM_TOLERANCE ||
         fabs(c->spark_offset_deg) > TRIM_TOLERANCE ||
         fabs(c->intake_leak_frac) > TRIM_TOLERANCE ||
         fabs(c->cooling_trim - 1.0) > TRIM_TOLERANCE;
}

void cylinder_fault_text(const CylinderConfig *c, char *out, int out_size) {
  int n = 0;
  out[0] = '\0';
#define ADD(cond, ...)                                                         \
  do {                                                                         \
    if (cond) {                                                                \
      n += snprintf(out + n, (size_t)(out_size - n), "%s", n > 0 ? ", " : ""); \
      n += snprintf(out + n, (size_t)(out_size - n), __VA_ARGS__);             \
      if (n >= out_size) {                                                     \
        return;                                                                \
      }                                                                        \
    }                                                                          \
  } while (0)
  ADD(fabs(c->injector_flow_trim - 1.0) > TRIM_TOLERANCE, "injector %.2f",
      c->injector_flow_trim);
  ADD(fabs(c->compression_trim - 1.0) > TRIM_TOLERANCE, "compression %.2f",
      c->compression_trim);
  ADD(fabs(c->spark_offset_deg) > TRIM_TOLERANCE, "spark %+.0f deg",
      c->spark_offset_deg);
  ADD(fabs(c->intake_leak_frac) > TRIM_TOLERANCE, "intake leak %.2f",
      c->intake_leak_frac);
  ADD(fabs(c->cooling_trim - 1.0) > TRIM_TOLERANCE, "cooling %.2f",
      c->cooling_trim);
#undef ADD
}

ChannelStatus cylinder_symptom_status(const CylinderState *c) {
  ChannelStatus st = channel_status_for(c->cht_c, CHANNEL_RANGE_CHT);
  st = worse(st, channel_status_for(c->egt_c, CHANNEL_RANGE_EGT));
  st = worse(st, channel_status_for(c->lambda, CHANNEL_RANGE_LAMBDA));
  st = worse(st, misfire_status(c->misfire_rate));
  return st;
}

void engine_faults_update(EngineFaultTracker *t, const CylinderConfig *cfg,
                          const ModelState *s, int num_cyl, double now_s) {
  const int n = num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS : num_cyl;
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    CylFaultTrack *k = &t->cyl[i];
    if (i >= n || !cylinder_has_fault(&cfg[i])) {
      CylFaultTrack z = {0};
      *k = z;
      continue;
    }
    if (!k->faulty) {
      CylFaultTrack z = {0};
      *k = z;
      k->faulty = 1;
      k->since_s = now_s;
    }
    k->now = cylinder_symptom_status(&s->cyl[i]);
    k->worst = worse(k->worst, k->now);
    if (k->now != CHANNEL_OK && !k->flagged) {
      k->flagged = 1;
      k->first_flag_s = now_s;
    }
  }
}

CylSymptoms engine_faults_symptoms(const ModelState *s, int cyl, int num_cyl,
                                   const EngineTrace *trace) {
  CylSymptoms out = {0.0, 0.0, 0.0, 0.0, -1.0};
  const int n = num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS : num_cyl;
  if (cyl < 0 || cyl >= n) {
    return out;
  }
  const CylinderState *c = &s->cyl[cyl];
  out.lambda = c->lambda;
  out.misfire_pct = c->misfire_rate * 100.0;
  if (n < 2) {
    return out;
  }

  double cht = 0.0, egt = 0.0;
  for (int i = 0; i < n; i++) {
    if (i != cyl) {
      cht += s->cyl[i].cht_c;
      egt += s->cyl[i].egt_c;
    }
  }
  out.d_cht_c = c->cht_c - cht / (n - 1);
  out.d_egt_c = c->egt_c - egt / (n - 1);

  if (trace && engine_trace_count(trace) > 1) {
    const int m = engine_trace_last_cycle_count(trace);
    const int first = engine_trace_count(trace) - m;
    double mine = 0.0, others = 0.0;
    for (int k = 0; k < m; k++) {
      const EngineTraceSample *x = engine_trace_at(trace, first + k);
      for (int i = 0; i < n; i++) {
        const double q =
            (double)x->cyl_gas_nm[i] + (double)x->cyl_inertia_nm[i];
        if (i == cyl) {
          mine += q;
        } else {
          others += q;
        }
      }
    }
    mine /= m;
    others /= (m * (n - 1));
    if (others > 1.0) { /* the others are actually making torque */
      out.power_pct = mine / others * 100.0;
    }
  }
  return out;
}
