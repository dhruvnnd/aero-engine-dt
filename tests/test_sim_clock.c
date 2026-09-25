#include "model/sim_clock.h"

#include "test_util.h"

static void test_running_scales_by_speed(void) {
  SimClock c;
  sim_clock_init(&c);
  CHECK_NEAR(sim_clock_speed(&c), 1.0, 1e-12);
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.016, 1e-12);

  sim_clock_set_speed(&c, 4); /* 4x */
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.064, 1e-12);
  sim_clock_set_speed(&c, 0); /* 0.25x */
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.004, 1e-12);
}

static void test_speed_index_is_clamped(void) {
  SimClock c;
  sim_clock_init(&c);
  sim_clock_set_speed(&c, -3);
  CHECK(c.speed_idx == 0);
  sim_clock_set_speed(&c, 99);
  CHECK(c.speed_idx == SIM_CLOCK_NUM_SPEEDS - 1);
}

static void test_pause_freezes_time(void) {
  SimClock c;
  sim_clock_init(&c);
  c.paused = true;
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.0, 1e-12);
  CHECK_NEAR(sim_clock_advance(&c, 0.5), 0.0, 1e-12);
}

static void test_step_advances_once_while_paused(void) {
  SimClock c;
  sim_clock_init(&c);
  c.paused = true;
  sim_clock_request_step(&c);
  CHECK_NEAR(sim_clock_advance(&c, 0.016), SIM_CLOCK_STEP_S, 1e-12);
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.0, 1e-12); /* consumed */

  /* the step size is fixed: independent of frame time and speed */
  sim_clock_set_speed(&c, 4);
  sim_clock_request_step(&c);
  CHECK_NEAR(sim_clock_advance(&c, 0.5), SIM_CLOCK_STEP_S, 1e-12);
}

static void test_step_ignored_while_running(void) {
  SimClock c;
  sim_clock_init(&c);
  sim_clock_request_step(&c); /* not paused: no effect */
  CHECK(!c.step_pending);

  /* a step requested, then resumed before it was served, is dropped */
  c.paused = true;
  sim_clock_request_step(&c);
  c.paused = false;
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.016, 1e-12);
  c.paused = true;
  CHECK_NEAR(sim_clock_advance(&c, 0.016), 0.0, 1e-12);
}

static void test_substeps_cover_the_interval(void) {
  double rem = 0.067; /* one 4x frame at 60 fps */
  double sum = 0.0;
  int n = 0;
  double h;
  while ((h = sim_clock_take_step(&rem)) > 0.0) {
    CHECK(h <= SIM_CLOCK_MAX_STEP_S + 1e-12);
    sum += h;
    n++;
  }
  CHECK_NEAR(sum, 0.067, 1e-12);
  CHECK(n == 4); /* 0.02 x3 + 0.007 */
  CHECK_NEAR(rem, 0.0, 1e-12);
}

static void test_small_and_empty_intervals(void) {
  double rem = 0.005;
  CHECK_NEAR(sim_clock_take_step(&rem), 0.005, 1e-12);
  CHECK_NEAR(sim_clock_take_step(&rem), 0.0, 1e-12);

  double none = 0.0;
  CHECK_NEAR(sim_clock_take_step(&none), 0.0, 1e-12);

  double exact = 0.04;
  CHECK_NEAR(sim_clock_take_step(&exact), 0.02, 1e-12);
  CHECK_NEAR(sim_clock_take_step(&exact), 0.02, 1e-12);
  CHECK_NEAR(sim_clock_take_step(&exact), 0.0, 1e-12);
}

static const TestCase cases[] = {
    {"running scales by speed", test_running_scales_by_speed},
    {"speed index is clamped", test_speed_index_is_clamped},
    {"pause freezes time", test_pause_freezes_time},
    {"step advances once while paused", test_step_advances_once_while_paused},
    {"step ignored while running", test_step_ignored_while_running},
    {"substeps cover the interval", test_substeps_cover_the_interval},
    {"small and empty intervals", test_small_and_empty_intervals},
};

RUN_TESTS(cases)
