package syncer

import (
	"context"
	"database/sql"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"testing"
	"time"

	_ "modernc.org/sqlite"

	"aero-engine-dt/sync/internal/backend"
	"aero-engine-dt/sync/internal/store"
)

// Mirrors src/data/run_log.c's schema -- same fixture approach as
// internal/store's own tests, kept separate rather than shared since each
// package's tests should stand alone.
const schemaDDL = `
CREATE TABLE runs (
  id INTEGER PRIMARY KEY,
  uuid TEXT NOT NULL UNIQUE,
  source TEXT NOT NULL,
  profile TEXT,
  dt REAL, duration_s REAL, load_nm REAL, seed INTEGER,
  engine_spec TEXT, with_sensor INTEGER NOT NULL DEFAULT 0,
  started_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
  ended_at TEXT, status TEXT NOT NULL DEFAULT 'running',
  synced_at TEXT, backend_run_id TEXT
);
CREATE TABLE samples (
  id INTEGER PRIMARY KEY,
  run_id INTEGER NOT NULL REFERENCES runs(id),
  t REAL NOT NULL,
  throttle REAL, alt_m REAL, ambient_c REAL, airspeed_ms REAL, cool_index REAL,
  rpm REAL, map_kpa REAL, torque_nm REAL, cht_c REAL, egt_c REAL, oil_c REAL,
  cht_c_1 REAL, cht_c_2 REAL, cht_c_3 REAL, cht_c_4 REAL,
  egt_c_1 REAL, egt_c_2 REAL, egt_c_3 REAL, egt_c_4 REAL,
  air_gps REAL, fuel_kgph REAL, fuel_press_kpa REAL,
  lambda_1 REAL, lambda_2 REAL, lambda_3 REAL, lambda_4 REAL,
  oil_press_kpa REAL, bus_v REAL, alt_a REAL, alt_field_a REAL, batt_soc REAL,
  s_rpm REAL, s_rpm_ok INTEGER, s_map_kpa REAL, s_map_ok INTEGER,
  s_cht_c REAL, s_cht_ok INTEGER, s_egt_c REAL, s_egt_ok INTEGER,
  s_oil_c REAL, s_oil_ok INTEGER
);
`

func newFixtureStore(t *testing.T) (*store.Store, *sql.DB) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "fixture.db")

	seed, err := sql.Open("sqlite", path)
	if err != nil {
		t.Fatalf("opening fixture for seeding: %v", err)
	}
	if _, err := seed.Exec(schemaDDL); err != nil {
		t.Fatalf("creating fixture schema: %v", err)
	}
	t.Cleanup(func() { seed.Close() })

	s, err := store.Open(path)
	if err != nil {
		t.Fatalf("store.Open: %v", err)
	}
	t.Cleanup(func() { s.Close() })

	return s, seed
}

func insertCompletedRun(t *testing.T, db *sql.DB, uuid string) int64 {
	t.Helper()
	res, err := db.Exec(`
		INSERT INTO runs (uuid, source, profile, dt, duration_s, load_nm, seed,
		                   engine_spec, with_sensor, ended_at, status)
		VALUES (?, 'test', 'idle', 0.02, 1.0, 8.0, 1, 'default', 0,
		        '2026-01-01T00:00:00Z', 'completed');
	`, uuid)
	if err != nil {
		t.Fatalf("inserting run: %v", err)
	}
	id, err := res.LastInsertId()
	if err != nil {
		t.Fatalf("LastInsertId: %v", err)
	}

	// Every base/channel column populated (0.0 default) -- mirrors the real
	// invariant run_log_write_sample guarantees: only s_* sensor columns
	// are ever legitimately NULL, never these.
	_, err = db.Exec(`
		INSERT INTO samples (
			run_id, t, throttle, alt_m, ambient_c, airspeed_ms, cool_index,
			rpm, map_kpa, torque_nm, cht_c, egt_c, oil_c,
			cht_c_1, cht_c_2, cht_c_3, cht_c_4,
			egt_c_1, egt_c_2, egt_c_3, egt_c_4,
			air_gps, fuel_kgph, fuel_press_kpa,
			lambda_1, lambda_2, lambda_3, lambda_4,
			oil_press_kpa, bus_v, alt_a, alt_field_a, batt_soc
		) VALUES (
			?, 0.0, 0,0,0,0,0,
			700.0, 0,0, 15.0, 0,0,
			0,0,0,0,
			0,0,0,0,
			0,0,0,
			0,0,0,0,
			0,0,0,0,0
		);
	`, id)
	if err != nil {
		t.Fatalf("inserting sample: %v", err)
	}
	return id
}

func TestSyncPending_Success_MarksRunSynced(t *testing.T) {
	s, db := newFixtureStore(t)
	runID := insertCompletedRun(t, db, "local-uuid-1")

	var gotSessionName string
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		r.ParseMultipartForm(1 << 20)
		gotSessionName = r.FormValue("session_name")
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		w.Write([]byte(`{"run_id":"backend-uuid-1","session_name":"` + gotSessionName +
			`","rows_ingested":1,"archived":true}`))
	}))
	defer srv.Close()

	syncer := New(s, backend.New(srv.URL, 5*time.Second))
	results, err := syncer.SyncPending(context.Background())
	if err != nil {
		t.Fatalf("SyncPending: %v", err)
	}
	if len(results) != 1 {
		t.Fatalf("got %d results, want 1", len(results))
	}
	r := results[0]
	if r.Status != StatusSynced {
		t.Fatalf("Status = %v, want StatusSynced (err: %v)", r.Status, r.Err)
	}
	if r.RunID != runID || r.UUID != "local-uuid-1" {
		t.Errorf("Result = %+v, want RunID=%d UUID=local-uuid-1", r, runID)
	}
	if gotSessionName != "local-uuid-1" {
		t.Errorf("server saw session_name=%q, want the run's local uuid", gotSessionName)
	}

	pending, err := s.PendingRuns()
	if err != nil {
		t.Fatalf("PendingRuns: %v", err)
	}
	if len(pending) != 0 {
		t.Errorf("got %d pending runs after sync, want 0", len(pending))
	}

	var backendRunID string
	if err := db.QueryRow(`SELECT backend_run_id FROM runs WHERE id = ?;`, runID).
		Scan(&backendRunID); err != nil {
		t.Fatalf("reading back backend_run_id: %v", err)
	}
	if backendRunID != "backend-uuid-1" {
		t.Errorf("backend_run_id = %q, want backend-uuid-1", backendRunID)
	}
}

func TestSyncPending_CleanRejection_LeavesRunPending(t *testing.T) {
	s, db := newFixtureStore(t)
	runID := insertCompletedRun(t, db, "local-uuid-2")

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusUnprocessableEntity)
		w.Write([]byte(`{"detail":"bad data"}`))
	}))
	defer srv.Close()

	syncer := New(s, backend.New(srv.URL, 5*time.Second))
	results, err := syncer.SyncPending(context.Background())
	if err != nil {
		t.Fatalf("SyncPending: %v", err)
	}
	if len(results) != 1 || results[0].Status != StatusFailed {
		t.Fatalf("got %+v, want one StatusFailed result", results)
	}

	pending, err := s.PendingRuns()
	if err != nil || len(pending) != 1 || pending[0].ID != runID {
		t.Fatalf("PendingRuns() = %+v, %v; want run %d still pending", pending, err, runID)
	}
}

func TestSyncPending_AmbiguousFailure_LeavesRunPendingAndIsMarkedAmbiguous(t *testing.T) {
	s, db := newFixtureStore(t)
	runID := insertCompletedRun(t, db, "local-uuid-3")

	// A closed server guarantees a connection-level failure, never a real
	// HTTP response -- the "outcome unknown" case.
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {}))
	srv.Close()

	syncer := New(s, backend.New(srv.URL, 2*time.Second))
	results, err := syncer.SyncPending(context.Background())
	if err != nil {
		t.Fatalf("SyncPending: %v", err)
	}
	if len(results) != 1 || results[0].Status != StatusAmbiguous {
		t.Fatalf("got %+v, want one StatusAmbiguous result", results)
	}

	pending, err := s.PendingRuns()
	if err != nil || len(pending) != 1 || pending[0].ID != runID {
		t.Fatalf("PendingRuns() = %+v, %v; want run %d still pending", pending, err, runID)
	}
}

func TestSyncPending_OneFailureDoesNotBlockOtherRuns(t *testing.T) {
	s, db := newFixtureStore(t)
	failID := insertCompletedRun(t, db, "local-uuid-fail")
	okID := insertCompletedRun(t, db, "local-uuid-ok")

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		r.ParseMultipartForm(1 << 20)
		if r.FormValue("session_name") == "local-uuid-fail" {
			w.WriteHeader(http.StatusUnprocessableEntity)
			w.Write([]byte(`{"detail":"bad"}`))
			return
		}
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		w.Write([]byte(`{"run_id":"backend-uuid-ok","session_name":"local-uuid-ok","rows_ingested":1,"archived":true}`))
	}))
	defer srv.Close()

	syncer := New(s, backend.New(srv.URL, 5*time.Second))
	results, err := syncer.SyncPending(context.Background())
	if err != nil {
		t.Fatalf("SyncPending: %v", err)
	}
	if len(results) != 2 {
		t.Fatalf("got %d results, want 2", len(results))
	}

	byID := map[int64]Result{}
	for _, r := range results {
		byID[r.RunID] = r
	}
	if byID[failID].Status != StatusFailed {
		t.Errorf("fail run: got %v, want StatusFailed", byID[failID].Status)
	}
	if byID[okID].Status != StatusSynced {
		t.Errorf("ok run: got %v, want StatusSynced", byID[okID].Status)
	}

	pending, err := s.PendingRuns()
	if err != nil || len(pending) != 1 || pending[0].ID != failID {
		t.Fatalf("PendingRuns() = %+v, %v; want only run %d still pending", pending, err, failID)
	}
}
