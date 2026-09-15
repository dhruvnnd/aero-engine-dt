#include "data/run_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "model/channels.h"

#define RUN_LOG_SQL_BUF 8192

static const char *const RUNS_DDL = "CREATE TABLE IF NOT EXISTS runs ("
                                    "  id INTEGER PRIMARY KEY,"
                                    "  uuid TEXT NOT NULL UNIQUE,"
                                    "  source TEXT NOT NULL,"
                                    "  profile TEXT,"
                                    "  dt REAL,"
                                    "  duration_s REAL,"
                                    "  load_nm REAL,"
                                    "  seed INTEGER,"
                                    "  engine_spec TEXT,"
                                    "  with_sensor INTEGER NOT NULL DEFAULT 0,"
                                    "  started_at TEXT NOT NULL DEFAULT "
                                    "(strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),"
                                    "  ended_at TEXT,"
                                    "  status TEXT NOT NULL DEFAULT 'running',"
                                    "  synced_at TEXT"
                                    ");";

static const char *const RUNS_INSERT_SQL =
    "INSERT INTO runs (uuid, source, profile, dt, duration_s, load_nm, seed,"
    " engine_spec, with_sensor) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9);";

static const char *const RUNS_END_SQL =
    "UPDATE runs SET ended_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'),"
    " status = ?1 WHERE id = ?2;";

static const char *const SAMPLE_BASE_COLS[] = {
    "run_id",    "t",           "throttle",   "alt_m",
    "ambient_c", "airspeed_ms", "cool_index",
};
#define SAMPLE_BASE_COUNT                                                      \
  (int)(sizeof(SAMPLE_BASE_COLS) / sizeof(SAMPLE_BASE_COLS[0]))

static const char *const SENSOR_COLS[] = {
    "s_rpm",    "s_rpm_ok", "s_map_kpa", "s_map_ok", "s_cht_c",
    "s_cht_ok", "s_egt_c",  "s_egt_ok",  "s_oil_c",  "s_oil_ok",
};
#define SENSOR_COL_COUNT (int)(sizeof(SENSOR_COLS) / sizeof(SENSOR_COLS[0]))

static void appendf(char *buf, size_t bufsz, size_t *off, const char *fmt,
                    ...) {
  if (*off >= bufsz) {
    return;
  }
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf + *off, bufsz - *off, fmt, ap);
  va_end(ap);
  if (n > 0) {
    *off += (size_t)n;
  }
}

static void build_samples_ddl(char *buf, size_t bufsz) {
  size_t off = 0;
  int nchan;
  const ModelChannel *chans = model_channels(&nchan);

  appendf(buf, bufsz, &off,
          "CREATE TABLE IF NOT EXISTS samples ("
          "id INTEGER PRIMARY KEY,"
          "run_id INTEGER NOT NULL REFERENCES runs(id),"
          "t REAL NOT NULL,"
          "throttle REAL,alt_m REAL,ambient_c REAL,airspeed_ms REAL,"
          "cool_index REAL,");
  for (int c = 0; c < nchan; c++) {
    appendf(buf, bufsz, &off, "%s REAL,", chans[c].name);
  }
  for (int i = 0; i < SENSOR_COL_COUNT; i++) {
    appendf(buf, bufsz, &off, "%s %s,", SENSOR_COLS[i],
            (i % 2 == 1) ? "INTEGER" : "REAL"); /* *_ok flags are INTEGER */
  }
  /* Trim the trailing comma and close. */
  if (off > 0 && buf[off - 1] == ',') {
    off--;
  }
  appendf(buf, bufsz, &off, ");");
}

static void build_samples_insert(char *buf, size_t bufsz) {
  size_t off = 0;
  int nchan;
  const ModelChannel *chans = model_channels(&nchan);
  int total = SAMPLE_BASE_COUNT + nchan + SENSOR_COL_COUNT;

  appendf(buf, bufsz, &off, "INSERT INTO samples (");
  for (int i = 0; i < SAMPLE_BASE_COUNT; i++) {
    appendf(buf, bufsz, &off, "%s,", SAMPLE_BASE_COLS[i]);
  }
  for (int c = 0; c < nchan; c++) {
    appendf(buf, bufsz, &off, "%s,", chans[c].name);
  }
  for (int i = 0; i < SENSOR_COL_COUNT; i++) {
    appendf(buf, bufsz, &off, "%s,", SENSOR_COLS[i]);
  }
  if (off > 0 && buf[off - 1] == ',') {
    off--;
  }
  appendf(buf, bufsz, &off, ") VALUES (");
  for (int i = 0; i < total; i++) {
    appendf(buf, bufsz, &off, i + 1 < total ? "?," : "?");
  }
  appendf(buf, bufsz, &off, ");");
}

/* Bumped on every run_log_open() call so two opens can never seed the uuid
 * rng identically -- time(NULL) and a stack address alone repeat when the
 * same RunLog variable is reopened within the same second (this bit an
 * early version of the reopen test). */
static uint32_t g_run_log_open_seq;

static int exec_or_warn(sqlite3 *db, const char *sql, const char *what) {
  char *err = NULL;
  if (sqlite3_exec(db, sql, NULL, NULL, &err) != SQLITE_OK) {
    fprintf(stderr, "run_log: %s failed: %s\n", what,
            err ? err : "unknown error");
    sqlite3_free(err);
    return -1;
  }
  return 0;
}

RunLogStatus run_log_open(RunLog *log, const char *path) {
  memset(log, 0, sizeof(*log));

  if (sqlite3_open(path, &log->db) != SQLITE_OK) {
    fprintf(stderr, "run_log: couldn't open %s: %s\n", path,
            sqlite3_errmsg(log->db));
    sqlite3_close(log->db);
    log->db = NULL;
    return RUN_LOG_ERR_OPEN;
  }

  if (exec_or_warn(log->db, "PRAGMA journal_mode=WAL;", "enabling WAL") ||
      exec_or_warn(log->db, "PRAGMA synchronous=NORMAL;",
                   "setting synchronous") ||
      exec_or_warn(log->db, "PRAGMA foreign_keys=ON;",
                   "enabling foreign keys") ||
      exec_or_warn(log->db, RUNS_DDL, "creating runs table")) {
    sqlite3_close(log->db);
    log->db = NULL;
    return RUN_LOG_ERR_SCHEMA;
  }

  char samples_ddl[RUN_LOG_SQL_BUF];
  build_samples_ddl(samples_ddl, sizeof(samples_ddl));
  if (exec_or_warn(log->db, samples_ddl, "creating samples table") ||
      exec_or_warn(log->db,
                   "CREATE INDEX IF NOT EXISTS idx_samples_run_id "
                   "ON samples(run_id);",
                   "creating samples index")) {
    sqlite3_close(log->db);
    log->db = NULL;
    return RUN_LOG_ERR_SCHEMA;
  }

  char insert_sql[RUN_LOG_SQL_BUF];
  build_samples_insert(insert_sql, sizeof(insert_sql));

  if (sqlite3_prepare_v2(log->db, RUNS_INSERT_SQL, -1, &log->insert_run_stmt,
                         NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(log->db, insert_sql, -1, &log->insert_sample_stmt,
                         NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(log->db, RUNS_END_SQL, -1, &log->end_run_stmt, NULL) !=
          SQLITE_OK) {
    fprintf(stderr, "run_log: preparing statements failed: %s\n",
            sqlite3_errmsg(log->db));
    run_log_close(log);
    return RUN_LOG_ERR_SCHEMA;
  }

  uint32_t seq = ++g_run_log_open_seq;
  log->uuid_rng_state =
      (uint32_t)time(NULL) ^ (uint32_t)(uintptr_t)log ^ (seq * 0x9E3779B9u);
  if (log->uuid_rng_state == 0) {
    log->uuid_rng_state = 0x9E3779B9u;
  }
  return RUN_LOG_OK;
}

static uint32_t uuid_rng_next(RunLog *log) {
  uint32_t x = log->uuid_rng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  log->uuid_rng_state = x;
  return x;
}

/* Formats a random RFC-4122-shaped (version 4, variant 1) uuid string into
 * out[37]. Not cryptographically random -- xorshift32 */
static void gen_uuid(RunLog *log, char out[37]) {
  uint32_t w[4];
  for (int i = 0; i < 4; i++) {
    w[i] = uuid_rng_next(log);
  }
  w[1] = (w[1] & 0xFFFF0FFFu) | 0x00004000u; /* version 4 */
  w[2] = (w[2] & 0x3FFFFFFFu) | 0x80000000u; /* variant 1 */
  snprintf(out, 37, "%08x-%04x-%04x-%04x-%04x%08x", w[0], w[1] >> 16,
           w[1] & 0xFFFF, w[2] >> 16, w[2] & 0xFFFF, w[3]);
}

int64_t run_log_begin_run(RunLog *log, const RunLogMeta *meta, char *uuid_out) {
  char uuid[37];
  gen_uuid(log, uuid);
  if (uuid_out) {
    memcpy(uuid_out, uuid, sizeof(uuid));
  }

  sqlite3_stmt *s = log->insert_run_stmt;
  sqlite3_reset(s);
  sqlite3_clear_bindings(s);
  sqlite3_bind_text(s, 1, uuid, -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(s, 2, meta->source, -1, SQLITE_TRANSIENT);
  if (meta->profile) {
    sqlite3_bind_text(s, 3, meta->profile, -1, SQLITE_TRANSIENT);
  } else {
    sqlite3_bind_null(s, 3);
  }
  sqlite3_bind_double(s, 4, meta->dt);
  sqlite3_bind_double(s, 5, meta->duration_s);
  sqlite3_bind_double(s, 6, meta->load_nm);
  sqlite3_bind_int64(s, 7, (sqlite3_int64)meta->seed);
  sqlite3_bind_text(s, 8, meta->engine_spec ? meta->engine_spec : "default", -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_int(s, 9, meta->with_sensor ? 1 : 0);

  if (sqlite3_step(s) != SQLITE_DONE) {
    fprintf(stderr, "run_log: inserting run failed: %s\n",
            sqlite3_errmsg(log->db));
    return -1;
  }

  int64_t run_id = (int64_t)sqlite3_last_insert_rowid(log->db);
  if (!log->in_txn) {
    exec_or_warn(log->db, "BEGIN;", "starting run transaction");
    log->in_txn = 1;
  }
  return run_id;
}

void run_log_write_sample(RunLog *log, int64_t run_id,
                          const RunLogSample *sample) {
  sqlite3_stmt *s = log->insert_sample_stmt;
  sqlite3_reset(s);
  sqlite3_clear_bindings(s);

  int idx = 1;
  sqlite3_bind_int64(s, idx++, (sqlite3_int64)run_id);
  sqlite3_bind_double(s, idx++, sample->t);
  sqlite3_bind_double(s, idx++, sample->throttle);
  sqlite3_bind_double(s, idx++, sample->alt_m);
  sqlite3_bind_double(s, idx++, sample->ambient_c);
  sqlite3_bind_double(s, idx++, sample->airspeed_ms);
  sqlite3_bind_double(s, idx++, sample->cool_index);

  int nchan;
  const ModelChannel *chans = model_channels(&nchan);
  for (int c = 0; c < nchan; c++) {
    sqlite3_bind_double(s, idx++, chans[c].get(sample->state, chans[c].index));
  }

  const SensorReading *r = sample->sensor;
  if (r) {
    sqlite3_bind_double(s, idx++, r->value.rpm);
    sqlite3_bind_int(s, idx++, r->ok.rpm ? 1 : 0);
    sqlite3_bind_double(s, idx++, r->value.map_kpa);
    sqlite3_bind_int(s, idx++, r->ok.map_kpa ? 1 : 0);
    sqlite3_bind_double(s, idx++, r->value.cht_c);
    sqlite3_bind_int(s, idx++, r->ok.cht_c ? 1 : 0);
    sqlite3_bind_double(s, idx++, r->value.egt_c);
    sqlite3_bind_int(s, idx++, r->ok.egt_c ? 1 : 0);
    sqlite3_bind_double(s, idx++, r->value.oil_temp_c);
    sqlite3_bind_int(s, idx++, r->ok.oil_temp_c ? 1 : 0);
  } else {
    for (int i = 0; i < SENSOR_COL_COUNT; i++) {
      sqlite3_bind_null(s, idx++);
    }
  }

  if (sqlite3_step(s) != SQLITE_DONE) {
    fprintf(stderr, "run_log: writing sample failed: %s\n",
            sqlite3_errmsg(log->db));
  }
}

void run_log_flush(RunLog *log) {
  if (!log->in_txn) {
    return;
  }
  exec_or_warn(log->db, "COMMIT;", "flushing samples");
  exec_or_warn(log->db, "BEGIN;", "reopening run transaction");
}

void run_log_end_run(RunLog *log, int64_t run_id, const char *status) {
  sqlite3_stmt *s = log->end_run_stmt;
  sqlite3_reset(s);
  sqlite3_clear_bindings(s);
  sqlite3_bind_text(s, 1, status, -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(s, 2, (sqlite3_int64)run_id);
  if (sqlite3_step(s) != SQLITE_DONE) {
    fprintf(stderr, "run_log: ending run failed: %s\n",
            sqlite3_errmsg(log->db));
  }

  if (log->in_txn) {
    exec_or_warn(log->db, "COMMIT;", "committing run");
    log->in_txn = 0;
  }
}

void run_log_close(RunLog *log) {
  if (!log->db) {
    return;
  }
  if (log->in_txn) {
    exec_or_warn(log->db, "COMMIT;", "committing on close");
    log->in_txn = 0;
  }
  sqlite3_finalize(log->insert_run_stmt);
  sqlite3_finalize(log->insert_sample_stmt);
  sqlite3_finalize(log->end_run_stmt);
  sqlite3_close(log->db);
  memset(log, 0, sizeof(*log));
}
