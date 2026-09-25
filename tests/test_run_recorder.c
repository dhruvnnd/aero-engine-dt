#include "data/run_recorder.h"

#include <stdio.h>
#include <string.h>

#include "model/state.h"
#include "model/sync.h"
#include "sqlite3.h"
#include "test_util.h"

#define TMP_PATH "test_run_recorder_tmp.db"

static ModelState make_state(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);
  return st;
}

static RunLogMeta make_meta(void) {
  RunLogMeta meta;
  memset(&meta, 0, sizeof meta);
  meta.source = "dashboard";
  meta.dt = 0.1;
  meta.load_nm = 8.0;
  meta.engine_spec = "default";
  return meta;
}

static double query_double(sqlite3 *db, const char *sql) {
  sqlite3_stmt *s = NULL;
  double v = -1.0;
  CHECK(sqlite3_prepare_v2(db, sql, -1, &s, NULL) == SQLITE_OK);
  if (sqlite3_step(s) == SQLITE_ROW) {
    v = sqlite3_column_double(s, 0);
  }
  sqlite3_finalize(s);
  return v;
}

static void write_rows(RunRecorder *r, const ModelState *st, int n,
                       double t_start) {
  for (int i = 0; i < n; i++) {
    RunLogSample s;
    memset(&s, 0, sizeof s);
    s.t = t_start + 0.1 * i;
    s.state = st;
    run_recorder_write(r, &s);
  }
}

static void test_records_a_run(void) {
  remove(TMP_PATH);
  ModelState st = make_state();
  RunLogMeta meta = make_meta();

  RunRecorder r;
  run_recorder_init(&r);
  CHECK(!run_recorder_active(&r));

  CHECK(run_recorder_start(&r, TMP_PATH, &meta, 42.0) == 0);
  CHECK(run_recorder_active(&r));
  CHECK(run_recorder_start(&r, TMP_PATH, &meta, 0.0) == -1); /* already recording */

  write_rows(&r, &st, 120, 42.0); /* crosses a flush boundary */
  CHECK(r.rows == 120);
  run_recorder_stop(&r, "completed");
  CHECK(!run_recorder_active(&r));

  sqlite3 *db = NULL;
  CHECK(sqlite3_open(TMP_PATH, &db) == SQLITE_OK);
  CHECK_NEAR(query_double(db, "SELECT count(*) FROM samples;"), 120.0, 0.0);
  CHECK_NEAR(query_double(db, "SELECT count(*) FROM runs WHERE source='dashboard' "
                              "AND status='completed' AND ended_at IS NOT NULL;"),
             1.0, 0.0);
  /* timestamps are relative to the start of recording */
  CHECK_NEAR(query_double(db, "SELECT min(t) FROM samples;"), 0.0, 1e-9);
  CHECK_NEAR(query_double(db, "SELECT max(t) FROM samples;"), 11.9, 1e-6);
  sqlite3_close(db);
  remove(TMP_PATH);
}

static void test_aborted_status_and_second_run_appends(void) {
  remove(TMP_PATH);
  ModelState st = make_state();
  RunLogMeta meta = make_meta();
  RunRecorder r;
  run_recorder_init(&r);

  CHECK(run_recorder_start(&r, TMP_PATH, &meta, 0.0) == 0);
  write_rows(&r, &st, 5, 0.0);
  run_recorder_stop(&r, "aborted");

  CHECK(run_recorder_start(&r, TMP_PATH, &meta, 100.0) == 0);
  write_rows(&r, &st, 7, 100.0);
  run_recorder_stop(&r, "completed");

  sqlite3 *db = NULL;
  CHECK(sqlite3_open(TMP_PATH, &db) == SQLITE_OK);
  CHECK_NEAR(query_double(db, "SELECT count(*) FROM runs;"), 2.0, 0.0);
  CHECK_NEAR(query_double(db, "SELECT count(*) FROM runs WHERE status='aborted';"),
             1.0, 0.0);
  CHECK_NEAR(query_double(db, "SELECT count(*) FROM samples;"), 12.0, 0.0);
  sqlite3_close(db);
  remove(TMP_PATH);
}

static void test_write_and_stop_are_noops_when_idle(void) {
  ModelState st = make_state();
  RunRecorder r;
  run_recorder_init(&r);
  write_rows(&r, &st, 3, 0.0); /* must not crash or write anything */
  CHECK(r.rows == 0);
  run_recorder_stop(&r, "completed");
  CHECK(!run_recorder_active(&r));
}

static void test_bad_path_fails_cleanly(void) {
  RunLogMeta meta = make_meta();
  RunRecorder r;
  run_recorder_init(&r);
  CHECK(run_recorder_start(&r, "no_such_dir_xyz/a/b/c.db", &meta, 0.0) == -1);
  CHECK(!run_recorder_active(&r));
}

static const TestCase cases[] = {
    {"records a run", test_records_a_run},
    {"aborted status, second run appends",
     test_aborted_status_and_second_run_appends},
    {"write/stop are no-ops when idle", test_write_and_stop_are_noops_when_idle},
    {"bad path fails cleanly", test_bad_path_fails_cleanly},
};

RUN_TESTS(cases)
