#include "physics/engine_trace.h"

#include <math.h>

void engine_trace_clear(EngineTrace *t) {
  t->head = 0;
  t->count = 0;
}

void engine_trace_push(EngineTrace *t, const EngineTraceSample *s) {
  t->samples[t->head] = *s;
  t->head = (t->head + 1) % ENGINE_TRACE_CAPACITY;
  if (t->count < ENGINE_TRACE_CAPACITY) {
    t->count++;
  }
}

int engine_trace_count(const EngineTrace *t) { return t->count; }

const EngineTraceSample *engine_trace_at(const EngineTrace *t, int i) {
  if (i < 0 || i >= t->count) {
    return NULL;
  }
  int oldest = (t->head - t->count + ENGINE_TRACE_CAPACITY) %
               ENGINE_TRACE_CAPACITY;
  return &t->samples[(oldest + i) % ENGINE_TRACE_CAPACITY];
}

int engine_trace_last_cycle_count(const EngineTrace *t) {
  if (t->count == 0) {
    return 0;
  }
  double travelled = 0.0;
  int n = 1;
  for (int i = t->count - 1; i > 0; i--) {
    double d = (double)engine_trace_at(t, i)->theta_deg -
               (double)engine_trace_at(t, i - 1)->theta_deg;
    d = fmod(d + 720.0, 720.0); /* forward crank travel between samples */
    if (travelled + d >= 720.0) {
      break;
    }
    travelled += d;
    n++;
  }
  return n;
}
