#include "model/sim_clock.h"

const double SIM_CLOCK_SPEEDS[SIM_CLOCK_NUM_SPEEDS] = {0.25, 0.5, 1.0, 2.0, 4.0};

void sim_clock_init(SimClock *c) {
  c->paused = false;
  c->speed_idx = SIM_CLOCK_DEFAULT_SPEED_IDX;
  c->step_pending = false;
}

double sim_clock_speed(const SimClock *c) {
  return SIM_CLOCK_SPEEDS[c->speed_idx];
}

void sim_clock_set_speed(SimClock *c, int idx) {
  if (idx < 0) {
    idx = 0;
  }
  if (idx >= SIM_CLOCK_NUM_SPEEDS) {
    idx = SIM_CLOCK_NUM_SPEEDS - 1;
  }
  c->speed_idx = idx;
}

void sim_clock_request_step(SimClock *c) {
  if (c->paused) {
    c->step_pending = true;
  }
}

double sim_clock_advance(SimClock *c, double wall_dt) {
  if (c->paused) {
    if (c->step_pending) {
      c->step_pending = false;
      return SIM_CLOCK_STEP_S;
    }
    return 0.0;
  }
  c->step_pending = false;
  return wall_dt > 0.0 ? wall_dt * sim_clock_speed(c) : 0.0;
}

double sim_clock_take_step(double *remaining) {
  if (*remaining <= 1e-12) {
    *remaining = 0.0;
    return 0.0;
  }
  const double h =
      *remaining < SIM_CLOCK_MAX_STEP_S ? *remaining : SIM_CLOCK_MAX_STEP_S;
  *remaining -= h;
  return h;
}
