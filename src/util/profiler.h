#ifndef UTIL_PROFILER_H
#define UTIL_PROFILER_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROF_MAX_STAGES 40
#define PROF_MAX_COUNTERS 6
#define PROF_HISTORY 600
#define PROF_NAME_LEN 28

typedef struct {
  float frame_ms; /* wall-clock length of the frame */
  float stage_ms[PROF_MAX_STAGES];
  float counter[PROF_MAX_COUNTERS];
} ProfFrame;

typedef struct {
  char stage_name[PROF_MAX_STAGES][PROF_NAME_LEN];
  int stage_count;
  char counter_name[PROF_MAX_COUNTERS][PROF_NAME_LEN];
  int counter_count;

  double stage_start_s[PROF_MAX_STAGES];
  int stage_open[PROF_MAX_STAGES];
  double frame_start_s;
  int frame_open;
  ProfFrame cur; /* the frame being timed */

  ProfFrame hist[PROF_HISTORY];
  int head;  /* next write slot */
  int count; /* valid frames, <= PROF_HISTORY */
  long frames_total;
} Profiler;

void profiler_init(Profiler *p);

/* Drops the recorded history (registered stages/counters are kept). */
void profiler_reset(Profiler *p);

/* Register a stage / counter; returns its id, or -1 if the table is full. The
 * name is truncated to PROF_NAME_LEN - 1 characters. */
int profiler_add_stage(Profiler *p, const char *name);
int profiler_add_counter(Profiler *p, const char *name);

void profiler_frame_begin(Profiler *p, double now_s);

void profiler_begin(Profiler *p, int stage, double now_s);
void profiler_end(Profiler *p, int stage, double now_s);

/* Adds `value` to a counter for the current frame. */
void profiler_count(Profiler *p, int counter, double value);

/* Completed frames, oldest first (0 = oldest); NULL if out of range. */
int profiler_frame_count(const Profiler *p);
const ProfFrame *profiler_frame_at(const Profiler *p, int i);

typedef struct {
  int n; /* frames summarised */
  double avg, min, max;
  double p50, p95, p99; /* percentiles of frame_ms */
} ProfSummary;

/* Summary of frame_ms over the newest `last_n` completed frames (all of them if
 * fewer, or last_n <= 0). */
ProfSummary profiler_frame_summary(const Profiler *p, int last_n);

/* Mean and worst value over the same window. */
double profiler_stage_avg_ms(const Profiler *p, int stage, int last_n);
double profiler_stage_max_ms(const Profiler *p, int stage, int last_n);
double profiler_counter_avg(const Profiler *p, int counter, int last_n);
double profiler_counter_max(const Profiler *p, int counter, int last_n);

int profiler_write_csv(const Profiler *p, FILE *out);

#ifdef __cplusplus
}
#endif

#endif /* UTIL_PROFILER_H */
