#ifndef MODEL_SIM_CLOCK_H
#define MODEL_SIM_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*
 * Turns wall-clock frame time into simulated time: pause, single-step and a
 * speed multiplier, plus fixed sub-stepping so the physics never takes a step
 * bigger than SIM_CLOCK_MAX_STEP_S no matter the frame rate or speed.
 */

/* Largest physics step, seconds (the same 0.02 s twin_sim runs at). */
#define SIM_CLOCK_MAX_STEP_S 0.02

/* Simulated time advanced by one "step" while paused, seconds. */
#define SIM_CLOCK_STEP_S 0.05

#define SIM_CLOCK_NUM_SPEEDS 5
#define SIM_CLOCK_DEFAULT_SPEED_IDX 2

/* Speed multipliers: 0.25x 0.5x 1x 2x 4x. */
extern const double SIM_CLOCK_SPEEDS[SIM_CLOCK_NUM_SPEEDS];

typedef struct {
  bool paused;
  int speed_idx;     /* index into SIM_CLOCK_SPEEDS */
  bool step_pending; /* one SIM_CLOCK_STEP_S advance owed (only while paused) */
} SimClock;

void sim_clock_init(SimClock *c);

/* Current multiplier. */
double sim_clock_speed(const SimClock *c);

/* Selects a speed; the index is clamped to the table. */
void sim_clock_set_speed(SimClock *c, int idx);

/* Asks for one fixed step. Only takes effect while paused. */
void sim_clock_request_step(SimClock *c);

/* Simulated seconds to advance this frame for `wall_dt` seconds of wall time:
 * wall_dt * speed when running; 0 when paused, except one SIM_CLOCK_STEP_S
 * (consuming the request) after sim_clock_request_step(). */
double sim_clock_advance(SimClock *c, double wall_dt);

/* Takes the next physics step out of `*remaining` simulated seconds: at most
 * SIM_CLOCK_MAX_STEP_S, never more than what is left. Returns 0 when nothing
 * is left. Use as: while ((h = sim_clock_take_step(&rem)) > 0) step(h); */
double sim_clock_take_step(double *remaining);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_SIM_CLOCK_H */
