#ifndef DATA_RUN_RECORDER_H
#define DATA_RUN_RECORDER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "data/run_log.h"

/*
 * Start/stop recording of an interactive session into the shared SQLite run
 * log (the same database the twin_sim tool and the sync tool use). One run at
 * a time; samples are timestamped relative to the moment recording started.
 */

typedef struct {
  RunLog log;
  int64_t run_id;
  int active;
  char uuid[37];
  char path[512];
  long rows;
  double t0_s; /* simulated time at start, subtracted from every sample */
} RunRecorder;

void run_recorder_init(RunRecorder *r);

/* Opens (creating the parent directory and file if needed) `db_path` and begins
 * a run described by `meta`. `t0_s` is the current simulated time. Returns 0 on
 * success, -1 on failure (nothing left open), or if already recording. */
int run_recorder_start(RunRecorder *r, const char *db_path,
                       const RunLogMeta *meta, double t0_s);

int run_recorder_active(const RunRecorder *r);

/* Appends one row. `sample->t` is absolute simulated time; the recorder makes
 * it relative to the start. Commits to disk every few dozen rows. No-op when
 * not recording. */
void run_recorder_write(RunRecorder *r, const RunLogSample *sample);

/* Ends the run with `status` ("completed" or "aborted") and closes the
 * database. No-op when not recording. */
void run_recorder_stop(RunRecorder *r, const char *status);

#ifdef __cplusplus
}
#endif

#endif /* DATA_RUN_RECORDER_H */
