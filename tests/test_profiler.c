#include "test_util.h"

#include <string.h>

#include "util/profiler.h"

static Profiler g_p; /* ~120 KB: keep it off the stack */

static void test_names_register_and_truncate(void) {
  profiler_init(&g_p);
  CHECK(profiler_add_stage(&g_p, "physics") == 0);
  CHECK(profiler_add_stage(&g_p, "ui") == 1);
  CHECK(profiler_add_counter(&g_p, "steps") == 0);
  CHECK(strcmp(g_p.stage_name[1], "ui") == 0);
  CHECK(strcmp(g_p.counter_name[0], "steps") == 0);

  const int id = profiler_add_stage(
      &g_p, "a stage name that is far longer than the name buffer allows");
  CHECK(id == 2);
  CHECK(strlen(g_p.stage_name[id]) == PROF_NAME_LEN - 1);
}

static void test_tables_report_full(void) {
  profiler_init(&g_p);
  for (int i = 0; i < PROF_MAX_STAGES; i++) {
    CHECK(profiler_add_stage(&g_p, "s") == i);
  }
  CHECK(profiler_add_stage(&g_p, "one too many") == -1);
  for (int i = 0; i < PROF_MAX_COUNTERS; i++) {
    CHECK(profiler_add_counter(&g_p, "c") == i);
  }
  CHECK(profiler_add_counter(&g_p, "one too many") == -1);
}

static void test_a_frame_is_the_period_between_begins(void) {
  profiler_init(&g_p);
  CHECK(profiler_frame_count(&g_p) == 0);
  profiler_frame_begin(&g_p, 1.000);
  CHECK(profiler_frame_count(&g_p) == 0); /* nothing closed yet */
  profiler_frame_begin(&g_p, 1.020);
  CHECK(profiler_frame_count(&g_p) == 1);
  profiler_frame_begin(&g_p, 1.050);
  CHECK(profiler_frame_count(&g_p) == 2);

  CHECK_NEAR(profiler_frame_at(&g_p, 0)->frame_ms, 20.0, 1e-3);
  CHECK_NEAR(profiler_frame_at(&g_p, 1)->frame_ms, 30.0, 1e-3);
  CHECK(profiler_frame_at(&g_p, 2) == NULL);
  CHECK(profiler_frame_at(&g_p, -1) == NULL);
  CHECK(g_p.frames_total == 2);
}

static void test_stage_time_accumulates_within_a_frame(void) {
  profiler_init(&g_p);
  const int phys = profiler_add_stage(&g_p, "physics");
  const int ui = profiler_add_stage(&g_p, "ui");

  profiler_frame_begin(&g_p, 0.0);
  /* entered twice in one frame: the two spans add up */
  profiler_begin(&g_p, phys, 0.001);
  profiler_end(&g_p, phys, 0.003);
  profiler_begin(&g_p, phys, 0.004);
  profiler_end(&g_p, phys, 0.009);
  profiler_begin(&g_p, ui, 0.010);
  profiler_end(&g_p, ui, 0.014);
  profiler_frame_begin(&g_p, 0.016);

  const ProfFrame *f = profiler_frame_at(&g_p, 0);
  CHECK_NEAR(f->stage_ms[phys], 7.0, 1e-3);
  CHECK_NEAR(f->stage_ms[ui], 4.0, 1e-3);
  CHECK_NEAR(f->frame_ms, 16.0, 1e-3);

  /* the next frame starts from zero */
  profiler_frame_begin(&g_p, 0.032);
  CHECK_NEAR(profiler_frame_at(&g_p, 1)->stage_ms[phys], 0.0, 1e-9);
}

static void test_unmatched_and_bad_calls_are_ignored(void) {
  profiler_init(&g_p);
  const int s = profiler_add_stage(&g_p, "s");
  profiler_frame_begin(&g_p, 0.0);
  profiler_end(&g_p, s, 0.005);       /* end without begin */
  profiler_begin(&g_p, 99, 0.0);      /* unregistered stage */
  profiler_end(&g_p, 99, 0.001);
  profiler_begin(&g_p, -1, 0.0);
  profiler_count(&g_p, 42, 1.0);      /* unregistered counter */
  profiler_begin(&g_p, s, 0.006);     /* begun but never ended */
  profiler_frame_begin(&g_p, 0.010);
  CHECK_NEAR(profiler_frame_at(&g_p, 0)->stage_ms[s], 0.0, 1e-9);
  /* a stage left open doesn't leak into the next frame */
  profiler_end(&g_p, s, 0.012);
  profiler_frame_begin(&g_p, 0.020);
  CHECK_NEAR(profiler_frame_at(&g_p, 1)->stage_ms[s], 0.0, 1e-9);
}

static void test_counters_sum_per_frame(void) {
  profiler_init(&g_p);
  const int steps = profiler_add_counter(&g_p, "steps");
  profiler_frame_begin(&g_p, 0.0);
  profiler_count(&g_p, steps, 3.0);
  profiler_count(&g_p, steps, 2.0);
  profiler_frame_begin(&g_p, 0.016);
  profiler_count(&g_p, steps, 7.0);
  profiler_frame_begin(&g_p, 0.032);
  CHECK_NEAR(profiler_frame_at(&g_p, 0)->counter[steps], 5.0, 1e-6);
  CHECK_NEAR(profiler_frame_at(&g_p, 1)->counter[steps], 7.0, 1e-6);
  CHECK_NEAR(profiler_counter_avg(&g_p, steps, 0), 6.0, 1e-6);
  CHECK_NEAR(profiler_counter_max(&g_p, steps, 0), 7.0, 1e-6);
  CHECK_NEAR(profiler_counter_avg(&g_p, steps, 1), 7.0, 1e-6); /* newest only */
}

static void test_ring_keeps_the_newest_frames(void) {
  profiler_init(&g_p);
  const int total = PROF_HISTORY + 50;
  double t = 0.0;
  profiler_frame_begin(&g_p, t);
  for (int i = 0; i < total; i++) {
    t += 0.001 * (i + 1); /* frame i is (i + 1) ms long */
    profiler_frame_begin(&g_p, t);
  }
  CHECK(profiler_frame_count(&g_p) == PROF_HISTORY);
  CHECK(g_p.frames_total == total);
  CHECK_NEAR(profiler_frame_at(&g_p, 0)->frame_ms, 51.0, 1e-2);
  CHECK_NEAR(profiler_frame_at(&g_p, PROF_HISTORY - 1)->frame_ms, total, 1e-2);
}

static void test_summary_statistics(void) {
  profiler_init(&g_p);
  /* frames of 1..100 ms */
  double t = 0.0;
  profiler_frame_begin(&g_p, t);
  for (int i = 1; i <= 100; i++) {
    t += 0.001 * i;
    profiler_frame_begin(&g_p, t);
  }
  ProfSummary s = profiler_frame_summary(&g_p, 0);
  CHECK(s.n == 100);
  CHECK_NEAR(s.avg, 50.5, 1e-2);
  CHECK_NEAR(s.min, 1.0, 1e-3);
  CHECK_NEAR(s.max, 100.0, 1e-3);
  CHECK_NEAR(s.p50, 50.5, 0.1);
  CHECK_NEAR(s.p95, 95.05, 0.2);
  CHECK_NEAR(s.p99, 99.01, 0.2);
  CHECK(s.p50 <= s.p95 && s.p95 <= s.p99 && s.p99 <= s.max);

  /* only the newest ten: 91..100 */
  s = profiler_frame_summary(&g_p, 10);
  CHECK(s.n == 10);
  CHECK_NEAR(s.min, 91.0, 1e-3);
  CHECK_NEAR(s.avg, 95.5, 1e-2);

  /* more than exist -> everything */
  CHECK(profiler_frame_summary(&g_p, 5000).n == 100);
}

static void test_summary_of_nothing_is_zero(void) {
  profiler_init(&g_p);
  const ProfSummary s = profiler_frame_summary(&g_p, 0);
  CHECK(s.n == 0);
  CHECK_NEAR(s.avg, 0.0, 1e-12);
  CHECK_NEAR(s.p99, 0.0, 1e-12);
  CHECK_NEAR(profiler_stage_avg_ms(&g_p, 0, 0), 0.0, 1e-12);
  CHECK_NEAR(profiler_stage_max_ms(&g_p, 0, 0), 0.0, 1e-12);
}

static void test_single_frame_summary(void) {
  profiler_init(&g_p);
  profiler_frame_begin(&g_p, 0.0);
  profiler_frame_begin(&g_p, 0.010);
  const ProfSummary s = profiler_frame_summary(&g_p, 0);
  CHECK(s.n == 1);
  CHECK_NEAR(s.p50, 10.0, 1e-3);
  CHECK_NEAR(s.p99, 10.0, 1e-3);
}

static void test_stage_average_and_worst(void) {
  profiler_init(&g_p);
  const int s = profiler_add_stage(&g_p, "physics");
  double t = 0.0;
  const double cost_ms[4] = {2.0, 4.0, 12.0, 6.0};
  profiler_frame_begin(&g_p, t);
  for (int i = 0; i < 4; i++) {
    profiler_begin(&g_p, s, t);
    profiler_end(&g_p, s, t + cost_ms[i] / 1000.0);
    t += 0.016;
    profiler_frame_begin(&g_p, t);
  }
  CHECK_NEAR(profiler_stage_avg_ms(&g_p, s, 0), 6.0, 1e-3);
  CHECK_NEAR(profiler_stage_max_ms(&g_p, s, 0), 12.0, 1e-3);
  CHECK_NEAR(profiler_stage_avg_ms(&g_p, s, 2), 9.0, 1e-3); /* 12 and 6 */
  CHECK_NEAR(profiler_stage_max_ms(&g_p, s, 1), 6.0, 1e-3);
}

static void test_reset_clears_history_but_keeps_registrations(void) {
  profiler_init(&g_p);
  const int s = profiler_add_stage(&g_p, "physics");
  profiler_frame_begin(&g_p, 0.0);
  profiler_frame_begin(&g_p, 0.016);
  CHECK(profiler_frame_count(&g_p) == 1);

  profiler_reset(&g_p);
  CHECK(profiler_frame_count(&g_p) == 0);
  CHECK(g_p.frames_total == 0);
  CHECK(g_p.stage_count == 1);
  CHECK(strcmp(g_p.stage_name[s], "physics") == 0);

  /* the first frame after a reset doesn't span the gap before it */
  profiler_frame_begin(&g_p, 100.0);
  CHECK(profiler_frame_count(&g_p) == 0);
  profiler_frame_begin(&g_p, 100.016);
  CHECK_NEAR(profiler_frame_at(&g_p, 0)->frame_ms, 16.0, 1e-3);
}

static void test_csv_export_has_header_and_a_row_per_frame(void) {
  profiler_init(&g_p);
  const int phys = profiler_add_stage(&g_p, "physics");
  profiler_add_stage(&g_p, "ui");
  const int steps = profiler_add_counter(&g_p, "steps");
  profiler_frame_begin(&g_p, 0.0);
  for (int i = 0; i < 3; i++) {
    profiler_begin(&g_p, phys, i * 0.02);
    profiler_end(&g_p, phys, i * 0.02 + 0.005);
    profiler_count(&g_p, steps, i + 1);
    profiler_frame_begin(&g_p, (i + 1) * 0.02);
  }

  FILE *f = tmpfile();
  CHECK(f != NULL);
  if (!f) {
    return;
  }
  CHECK(profiler_write_csv(&g_p, f) == 3);
  rewind(f);
  char line[256];
  CHECK(fgets(line, sizeof line, f) != NULL);
  line[strcspn(line, "\r\n")] = '\0'; /* text-mode tmpfile may add CR */
  CHECK(strcmp(line, "frame,frame_ms,physics_ms,ui_ms,steps") == 0);
  int rows = 0;
  while (fgets(line, sizeof line, f)) {
    rows++;
    if (rows == 1) {
      CHECK(strncmp(line, "0,20.0000,5.0000,0.0000,1", 25) == 0);
    }
  }
  CHECK(rows == 3);
  fclose(f);
}

static const TestCase tests[] = {
    {"names_register_and_truncate", test_names_register_and_truncate},
    {"tables_report_full", test_tables_report_full},
    {"a_frame_is_the_period_between_begins",
     test_a_frame_is_the_period_between_begins},
    {"stage_time_accumulates_within_a_frame",
     test_stage_time_accumulates_within_a_frame},
    {"unmatched_and_bad_calls_are_ignored",
     test_unmatched_and_bad_calls_are_ignored},
    {"counters_sum_per_frame", test_counters_sum_per_frame},
    {"ring_keeps_the_newest_frames", test_ring_keeps_the_newest_frames},
    {"summary_statistics", test_summary_statistics},
    {"summary_of_nothing_is_zero", test_summary_of_nothing_is_zero},
    {"single_frame_summary", test_single_frame_summary},
    {"stage_average_and_worst", test_stage_average_and_worst},
    {"csv_export_has_header_and_a_row_per_frame",
     test_csv_export_has_header_and_a_row_per_frame},
    {"reset_clears_history_but_keeps_registrations",
     test_reset_clears_history_but_keeps_registrations},
};

RUN_TESTS(tests)
