#include "data/run_recorder.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#endif

/* Rows between commits: ~5 s of a 10 Hz recording. */
#define FLUSH_EVERY_ROWS 50

static void ensure_parent_dir(const char *path) {
  const char *slash = strrchr(path, '/');
#ifdef _WIN32
  const char *bslash = strrchr(path, '\\');
  if (bslash && (!slash || bslash > slash)) {
    slash = bslash;
  }
#endif
  if (!slash) {
    return;
  }
  char dir[512];
  const size_t len = (size_t)(slash - path);
  if (len == 0 || len >= sizeof dir) {
    return;
  }
  memcpy(dir, path, len);
  dir[len] = '\0';
#ifdef _WIN32
  _mkdir(dir);
#else
  mkdir(dir, 0755);
#endif
}

void run_recorder_init(RunRecorder *r) { memset(r, 0, sizeof *r); }

int run_recorder_start(RunRecorder *r, const char *db_path,
                       const RunLogMeta *meta, double t0_s) {
  if (r->active) {
    return -1;
  }
  ensure_parent_dir(db_path);
  if (run_log_open(&r->log, db_path) != RUN_LOG_OK) {
    return -1;
  }
  const int64_t id = run_log_begin_run(&r->log, meta, r->uuid);
  if (id < 0) {
    run_log_close(&r->log);
    return -1;
  }
  r->run_id = id;
  r->active = 1;
  r->rows = 0;
  r->t0_s = t0_s;
  snprintf(r->path, sizeof r->path, "%s", db_path);
  return 0;
}

int run_recorder_active(const RunRecorder *r) { return r->active; }

void run_recorder_write(RunRecorder *r, const RunLogSample *sample) {
  if (!r->active) {
    return;
  }
  RunLogSample s = *sample;
  s.t = sample->t - r->t0_s;
  run_log_write_sample(&r->log, r->run_id, &s);
  r->rows++;
  if (r->rows % FLUSH_EVERY_ROWS == 0) {
    run_log_flush(&r->log);
  }
}

void run_recorder_stop(RunRecorder *r, const char *status) {
  if (!r->active) {
    return;
  }
  run_log_end_run(&r->log, r->run_id, status);
  run_log_close(&r->log);
  r->active = 0;
}
