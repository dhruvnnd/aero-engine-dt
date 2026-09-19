#ifndef DATA_RUN_LOG_H
#define DATA_RUN_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "model/state.h"
#include "sqlite3.h"
#include "telemetry/sensor.h"

/* Logs simulation runs to a single shared SQLite database */

typedef enum {
  RUN_LOG_OK = 0,
  RUN_LOG_ERR_OPEN,   /* couldn't open/create the database file */
  RUN_LOG_ERR_SCHEMA, /* couldn't create or verify the runs/samples tables */
} RunLogStatus;

typedef struct {
  sqlite3 *db;
  sqlite3_stmt *insert_run_stmt;
  sqlite3_stmt *insert_sample_stmt;
  sqlite3_stmt *end_run_stmt;
  uint32_t uuid_rng_state; /* xorshift32, seeded once in run_log_open() */
  int in_txn;              /* 1 while a run's write transaction is open */
} RunLog;

typedef struct {
  const char *source;  /* "twin_sim" | "dashboard" */
  const char *profile; /* mission profile name, or NULL */
  double dt;
  double duration_s; /* planned duration; 0 if open-ended (dashboard) */
  double load_nm;
  uint32_t seed;
  const char *engine_spec; /* spec file path, or "default" */
  int with_sensor;         /* 1 if this run's samples will carry sensor data */
} RunLogMeta;

typedef struct {
  double t;
  double throttle;
  double alt_m;
  double ambient_c;
  double airspeed_ms;
  double cool_index;
  const ModelState *state;
  const SensorReading *sensor;
} RunLogSample;

RunLogStatus run_log_open(RunLog *log, const char *path);

/* Inserts a `runs` row, starts that run's write transaction, and returns its
 * row id (>= 1), or -1 on error */
int64_t run_log_begin_run(RunLog *log, const RunLogMeta *meta, char *uuid_out);

/* Appends one `samples` row for `run_id` */
void run_log_write_sample(RunLog *log, int64_t run_id,
                          const RunLogSample *sample);

/* Commits the run's samples written so far and opens a fresh transaction */
void run_log_flush(RunLog *log);

/* Marks the run ended (`status`: "completed" or "aborted"), stamps
 * ended_at, and commits its transaction. */
void run_log_end_run(RunLog *log, int64_t run_id, const char *status);

/* Finalizes prepared statements and closes the database */
void run_log_close(RunLog *log);

#ifdef __cplusplus
}
#endif

#endif /* DATA_RUN_LOG_H */
