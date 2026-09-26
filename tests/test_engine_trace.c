#include "test_util.h"

#include "physics/engine_trace.h"

static EngineTrace g_trace; /* ~700 KB: keep it off the stack */

static EngineTraceSample sample_at(double theta_deg) {
  EngineTraceSample s = {0};
  s.theta_deg = (float)theta_deg;
  return s;
}

static void test_starts_empty_and_out_of_range_is_null(void) {
  engine_trace_clear(&g_trace);
  CHECK(engine_trace_count(&g_trace) == 0);
  CHECK(engine_trace_at(&g_trace, 0) == NULL);
  CHECK(engine_trace_last_cycle_count(&g_trace) == 0);
}

static void test_samples_come_back_oldest_first(void) {
  engine_trace_clear(&g_trace);
  for (int i = 0; i < 5; i++) {
    EngineTraceSample s = sample_at(i);
    s.torque_nm = (float)(10 * i);
    engine_trace_push(&g_trace, &s);
  }
  CHECK(engine_trace_count(&g_trace) == 5);
  for (int i = 0; i < 5; i++) {
    CHECK_NEAR(engine_trace_at(&g_trace, i)->torque_nm, 10.0 * i, 1e-6);
  }
  CHECK(engine_trace_at(&g_trace, 5) == NULL);
  CHECK(engine_trace_at(&g_trace, -1) == NULL);
}

static void test_ring_keeps_the_newest_capacity_samples(void) {
  engine_trace_clear(&g_trace);
  const int total = ENGINE_TRACE_CAPACITY + 100;
  for (int i = 0; i < total; i++) {
    EngineTraceSample s = sample_at(0.0);
    s.torque_nm = (float)i;
    engine_trace_push(&g_trace, &s);
  }
  CHECK(engine_trace_count(&g_trace) == ENGINE_TRACE_CAPACITY);
  CHECK_NEAR(engine_trace_at(&g_trace, 0)->torque_nm, 100.0, 1e-6);
  CHECK_NEAR(engine_trace_at(&g_trace, ENGINE_TRACE_CAPACITY - 1)->torque_nm,
             total - 1, 1e-6);
}

static void test_clear_empties_the_trace(void) {
  EngineTraceSample s = sample_at(1.0);
  engine_trace_push(&g_trace, &s);
  engine_trace_clear(&g_trace);
  CHECK(engine_trace_count(&g_trace) == 0);
  CHECK(engine_trace_at(&g_trace, 0) == NULL);
}

/* Pushes `n` samples advancing `step` deg each, starting at `start`. */
static void push_sweep(double start, double step, int n) {
  for (int i = 0; i < n; i++) {
    EngineTraceSample s = sample_at(fmod(start + step * i, 720.0));
    engine_trace_push(&g_trace, &s);
  }
}

static void test_last_cycle_spans_720_degrees_across_the_wrap(void) {
  engine_trace_clear(&g_trace);
  push_sweep(300.0, 1.0, 2000); /* wraps at 720 more than once */
  CHECK(engine_trace_last_cycle_count(&g_trace) == 720);

  engine_trace_clear(&g_trace);
  push_sweep(0.0, 0.5, 4000); /* finer steps -> more samples per cycle */
  CHECK(engine_trace_last_cycle_count(&g_trace) == 1440);
}

static void test_last_cycle_is_everything_when_less_than_a_cycle(void) {
  engine_trace_clear(&g_trace);
  push_sweep(100.0, 1.0, 300);
  CHECK(engine_trace_last_cycle_count(&g_trace) == 300);
}

static const TestCase CASES[] = {
    {"engine_trace.starts_empty_and_out_of_range_is_null",
     test_starts_empty_and_out_of_range_is_null},
    {"engine_trace.samples_come_back_oldest_first",
     test_samples_come_back_oldest_first},
    {"engine_trace.ring_keeps_the_newest_capacity_samples",
     test_ring_keeps_the_newest_capacity_samples},
    {"engine_trace.clear_empties_the_trace", test_clear_empties_the_trace},
    {"engine_trace.last_cycle_spans_720_degrees_across_the_wrap",
     test_last_cycle_spans_720_degrees_across_the_wrap},
    {"engine_trace.last_cycle_is_everything_when_less_than_a_cycle",
     test_last_cycle_is_everything_when_less_than_a_cycle},
};

RUN_TESTS(CASES)
