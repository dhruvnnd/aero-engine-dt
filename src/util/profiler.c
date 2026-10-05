#include "util/profiler.h"

#include <stdlib.h>
#include <string.h>

void profiler_reset(Profiler *p) {
  p->head = 0;
  p->count = 0;
  p->frames_total = 0;
  p->frame_open = 0;
  memset(&p->cur, 0, sizeof p->cur);
  memset(p->stage_open, 0, sizeof p->stage_open);
}

void profiler_init(Profiler *p) {
  memset(p, 0, sizeof *p);
}

static int add_name(char names[][PROF_NAME_LEN], int *count, int cap,
                    const char *name) {
  if (*count >= cap) {
    return -1;
  }
  strncpy(names[*count], name ? name : "", PROF_NAME_LEN - 1);
  names[*count][PROF_NAME_LEN - 1] = '\0';
  return (*count)++;
}

int profiler_add_stage(Profiler *p, const char *name) {
  return add_name(p->stage_name, &p->stage_count, PROF_MAX_STAGES, name);
}

int profiler_add_counter(Profiler *p, const char *name) {
  return add_name(p->counter_name, &p->counter_count, PROF_MAX_COUNTERS, name);
}

void profiler_frame_begin(Profiler *p, double now_s) {
  if (p->frame_open) {
    p->cur.frame_ms = (float)((now_s - p->frame_start_s) * 1000.0);
    p->hist[p->head] = p->cur;
    p->head = (p->head + 1) % PROF_HISTORY;
    if (p->count < PROF_HISTORY) {
      p->count++;
    }
    p->frames_total++;
  }
  memset(&p->cur, 0, sizeof p->cur);
  memset(p->stage_open, 0, sizeof p->stage_open);
  p->frame_start_s = now_s;
  p->frame_open = 1;
}

void profiler_begin(Profiler *p, int stage, double now_s) {
  if (stage < 0 || stage >= p->stage_count) {
    return;
  }
  p->stage_start_s[stage] = now_s;
  p->stage_open[stage] = 1;
}

void profiler_end(Profiler *p, int stage, double now_s) {
  if (stage < 0 || stage >= p->stage_count || !p->stage_open[stage]) {
    return;
  }
  p->cur.stage_ms[stage] +=
      (float)((now_s - p->stage_start_s[stage]) * 1000.0);
  p->stage_open[stage] = 0;
}

void profiler_count(Profiler *p, int counter, double value) {
  if (counter < 0 || counter >= p->counter_count) {
    return;
  }
  p->cur.counter[counter] += (float)value;
}

int profiler_frame_count(const Profiler *p) { return p->count; }

const ProfFrame *profiler_frame_at(const Profiler *p, int i) {
  if (i < 0 || i >= p->count) {
    return NULL;
  }
  const int oldest = (p->head - p->count + PROF_HISTORY) % PROF_HISTORY;
  return &p->hist[(oldest + i) % PROF_HISTORY];
}

/* Index of the first frame in the newest-`last_n` window. */
static int window_start(const Profiler *p, int last_n) {
  if (last_n <= 0 || last_n > p->count) {
    return 0;
  }
  return p->count - last_n;
}

static int cmp_float(const void *a, const void *b) {
  const float x = *(const float *)a;
  const float y = *(const float *)b;
  return (x > y) - (x < y);
}

/* Linear interpolation between closest ranks, q in [0,1]; `v` sorted. */
static double percentile(const float *v, int n, double q) {
  if (n == 1) {
    return v[0];
  }
  const double pos = q * (n - 1);
  const int lo = (int)pos;
  const int hi = lo + 1 < n ? lo + 1 : lo;
  const double frac = pos - lo;
  return v[lo] + (v[hi] - v[lo]) * frac;
}

ProfSummary profiler_frame_summary(const Profiler *p, int last_n) {
  ProfSummary s;
  memset(&s, 0, sizeof s);
  const int first = window_start(p, last_n);
  const int n = p->count - first;
  if (n <= 0) {
    return s;
  }
  float sorted[PROF_HISTORY];
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sorted[i] = profiler_frame_at(p, first + i)->frame_ms;
    sum += sorted[i];
  }
  qsort(sorted, (size_t)n, sizeof sorted[0], cmp_float);
  s.n = n;
  s.avg = sum / n;
  s.min = sorted[0];
  s.max = sorted[n - 1];
  s.p50 = percentile(sorted, n, 0.50);
  s.p95 = percentile(sorted, n, 0.95);
  s.p99 = percentile(sorted, n, 0.99);
  return s;
}

double profiler_stage_avg_ms(const Profiler *p, int stage, int last_n) {
  if (stage < 0 || stage >= PROF_MAX_STAGES) {
    return 0.0;
  }
  const int first = window_start(p, last_n);
  const int n = p->count - first;
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += profiler_frame_at(p, first + i)->stage_ms[stage];
  }
  return n > 0 ? sum / n : 0.0;
}

double profiler_stage_max_ms(const Profiler *p, int stage, int last_n) {
  if (stage < 0 || stage >= PROF_MAX_STAGES) {
    return 0.0;
  }
  const int first = window_start(p, last_n);
  double m = 0.0;
  for (int i = first; i < p->count; i++) {
    const double v = profiler_frame_at(p, i)->stage_ms[stage];
    if (v > m) {
      m = v;
    }
  }
  return m;
}

double profiler_counter_avg(const Profiler *p, int counter, int last_n) {
  if (counter < 0 || counter >= PROF_MAX_COUNTERS) {
    return 0.0;
  }
  const int first = window_start(p, last_n);
  const int n = p->count - first;
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += profiler_frame_at(p, first + i)->counter[counter];
  }
  return n > 0 ? sum / n : 0.0;
}

double profiler_counter_max(const Profiler *p, int counter, int last_n) {
  if (counter < 0 || counter >= PROF_MAX_COUNTERS) {
    return 0.0;
  }
  const int first = window_start(p, last_n);
  double m = 0.0;
  for (int i = first; i < p->count; i++) {
    const double v = profiler_frame_at(p, i)->counter[counter];
    if (v > m) {
      m = v;
    }
  }
  return m;
}

int profiler_write_csv(const Profiler *p, FILE *out) {
  fprintf(out, "frame,frame_ms");
  for (int s = 0; s < p->stage_count; s++) {
    fprintf(out, ",%s_ms", p->stage_name[s]);
  }
  for (int c = 0; c < p->counter_count; c++) {
    fprintf(out, ",%s", p->counter_name[c]);
  }
  fputc(10, out); /* newline */
  for (int i = 0; i < p->count; i++) {
    const ProfFrame *f = profiler_frame_at(p, i);
    fprintf(out, "%ld,%.4f", p->frames_total - p->count + i, f->frame_ms);
    for (int s = 0; s < p->stage_count; s++) {
      fprintf(out, ",%.4f", f->stage_ms[s]);
    }
    for (int c = 0; c < p->counter_count; c++) {
      fprintf(out, ",%.6g", f->counter[c]);
    }
    fputc(10, out); /* newline */
  }
  return p->count;
}
