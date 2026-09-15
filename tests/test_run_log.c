#include "test_util.h"

#include <string.h>

#include "data/run_log.h"
#include "model/state.h"
#include "model/sync.h"
#include "physics/engine_model.h"

#define TMP_PATH "test_run_log_tmp.db"

static ModelState make_state(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  return st;
}

/* Runs `sql` against `db` and returns the single integer result of a
 * `SELECT count(...)`-shaped query. Fails the test (via CHECK) rather than
 * crashing if the query itself is malformed. */
static int64_t query_count(sqlite3 *db, const char *sql) {
  sqlite3_stmt *s = NULL;
  CHECK(sqlite3_prepare_v2(db, sql, -1, &s, NULL) == SQLITE_OK);
  int64_t v = -1;
  if (sqlite3_step(s) == SQLITE_ROW) {
    v = sqlite3_column_int64(s, 0);
  }
  sqlite3_finalize(s);
  return v;
}

static void test_open_creates_schema(void) {
  RunLog log;
  CHECK(run_log_open(&log, ":memory:") == RUN_LOG_OK);

  CHECK(query_count(log.db,
                    "SELECT count(*) FROM sqlite_master WHERE type='table' "
                    "AND name='runs';") == 1);
  CHECK(query_count(log.db,
                    "SELECT count(*) FROM sqlite_master WHERE type='table' "
                    "AND name='samples';") == 1);
  /* Every model_channels() column landed in the samples table. */
  CHECK(query_count(log.db,
                    "SELECT count(*) FROM pragma_table_info('samples') "
                    "WHERE name='rpm';") == 1);
  CHECK(query_count(log.db,
                    "SELECT count(*) FROM pragma_table_info('samples') "
                    "WHERE name='cht_c_1';") == 1);

  run_log_close(&log);
}

static void test_begin_write_end_round_trip(void) {
  RunLog log;
  CHECK(run_log_open(&log, ":memory:") == RUN_LOG_OK);

  RunLogMeta meta = {0};
  meta.source = "test";
  meta.profile = "idle";
  meta.dt = 0.02;
  meta.duration_s = 1.0;
  meta.load_nm = 8.0;
  meta.seed = 1;
  meta.engine_spec = NULL; /* should fall back to "default" */
  meta.with_sensor = 0;

  char uuid[37] = {0};
  int64_t run_id = run_log_begin_run(&log, &meta, uuid);
  CHECK(run_id > 0);
  CHECK(strlen(uuid) == 36);

  ModelState st = make_state();
  for (int i = 0; i < 10; i++) {
    RunLogSample sample = {0};
    sample.t = i * meta.dt;
    sample.throttle = 0.5;
    sample.alt_m = 0.0;
    sample.ambient_c = 15.0;
    sample.airspeed_ms = 0.0;
    sample.cool_index = 0.1;
    sample.state = &st;
    sample.sensor = NULL;
    run_log_write_sample(&log, run_id, &sample);
  }

  run_log_end_run(&log, run_id, "completed");

  CHECK(query_count(log.db, "SELECT count(*) FROM samples WHERE run_id=1;") ==
       10);
  CHECK(query_count(log.db,
                    "SELECT count(*) FROM runs WHERE id=1 AND "
                    "status='completed' AND ended_at IS NOT NULL AND "
                    "engine_spec='default';") == 1);
  /* No sensor this run -- those columns must be NULL, not 0. */
  CHECK(query_count(log.db,
                    "SELECT count(*) FROM samples WHERE run_id=1 AND "
                    "s_rpm_ok IS NOT NULL;") == 0);

  run_log_close(&log);
}

static void test_sensor_columns_populated_when_present(void) {
  RunLog log;
  CHECK(run_log_open(&log, ":memory:") == RUN_LOG_OK);

  RunLogMeta meta = {0};
  meta.source = "test";
  meta.with_sensor = 1;
  int64_t run_id = run_log_begin_run(&log, &meta, NULL);

  ModelState st = make_state();
  SensorReading r = {0};
  r.value.rpm = 850.0;
  r.ok.rpm = true;
  r.ok.map_kpa = false; /* dropped-out channel */

  RunLogSample sample = {0};
  sample.state = &st;
  sample.sensor = &r;
  run_log_write_sample(&log, run_id, &sample);

  CHECK(query_count(log.db,
                    "SELECT count(*) FROM samples WHERE s_rpm=850.0 AND "
                    "s_rpm_ok=1 AND s_map_ok=0;") == 1);

  run_log_close(&log);
}

static void test_reopen_appends_not_overwrites(void) {
  remove(TMP_PATH);

  RunLog log;
  CHECK(run_log_open(&log, TMP_PATH) == RUN_LOG_OK);
  RunLogMeta meta = {0};
  meta.source = "test";
  int64_t run1 = run_log_begin_run(&log, &meta, NULL);
  run_log_end_run(&log, run1, "completed");
  run_log_close(&log);

  /* Reopening an existing file must not drop the first run -- the schema
   * DDL is CREATE TABLE IF NOT EXISTS, never CREATE/DROP. */
  CHECK(run_log_open(&log, TMP_PATH) == RUN_LOG_OK);
  CHECK(query_count(log.db, "SELECT count(*) FROM runs;") == 1);
  int64_t run2 = run_log_begin_run(&log, &meta, NULL);
  run_log_end_run(&log, run2, "completed");
  CHECK(query_count(log.db, "SELECT count(*) FROM runs;") == 2);
  CHECK(run2 != run1);
  run_log_close(&log);

  remove(TMP_PATH);
}

static void test_uuids_are_unique(void) {
  RunLog log;
  CHECK(run_log_open(&log, ":memory:") == RUN_LOG_OK);
  RunLogMeta meta = {0};
  meta.source = "test";

  char uuid1[37], uuid2[37];
  int64_t run1 = run_log_begin_run(&log, &meta, uuid1);
  run_log_end_run(&log, run1, "completed");
  run_log_begin_run(&log, &meta, uuid2);
  CHECK(strcmp(uuid1, uuid2) != 0);

  run_log_close(&log);
}

static const TestCase CASES[] = {
    {"run_log.open_creates_schema", test_open_creates_schema},
    {"run_log.begin_write_end_round_trip", test_begin_write_end_round_trip},
    {"run_log.sensor_columns_populated_when_present",
     test_sensor_columns_populated_when_present},
    {"run_log.reopen_appends_not_overwrites",
     test_reopen_appends_not_overwrites},
    {"run_log.uuids_are_unique", test_uuids_are_unique},
};

RUN_TESTS(CASES)
