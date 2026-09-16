package store

import (
	"database/sql"
	"path/filepath"
	"testing"

	_ "modernc.org/sqlite"
)

// Mirrors src/data/run_log.c's schema closely enough to be a real contract
// check: if the C side ever renames/drops a column this package selects by
// name, this fixture (or, better, a real run_log.db) is where that would
// need to change too.
const schemaDDL = `
CREATE TABLE runs (
  id INTEGER PRIMARY KEY,
  uuid TEXT NOT NULL UNIQUE,
  source TEXT NOT NULL,
  profile TEXT,
  dt REAL,
  duration_s REAL,
  load_nm REAL,
  seed INTEGER,
  engine_spec TEXT,
  with_sensor INTEGER NOT NULL DEFAULT 0,
  started_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
  ended_at TEXT,
  status TEXT NOT NULL DEFAULT 'running',
  synced_at TEXT,
  backend_run_id TEXT
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

// openFixture creates a fresh on-disk sqlite db with the schema above and
// returns a Store over it plus the raw *sql.DB for seeding test rows.
func openFixture(t *testing.T) (*Store, *sql.DB) {
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

	s, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	t.Cleanup(func() { s.Close() })

	return s, seed
}

func insertRun(t *testing.T, db *sql.DB, uuid, status string, syncedAt any) int64 {
	t.Helper()
	res, err := db.Exec(`
		INSERT INTO runs (uuid, source, profile, dt, duration_s, load_nm, seed,
		                   engine_spec, with_sensor, ended_at, status, synced_at)
		VALUES (?, 'test', 'idle', 0.02, 10.0, 8.0, 1, 'default', 0,
		        CASE WHEN ?='running' THEN NULL ELSE '2026-01-01T00:00:00Z' END,
		        ?, ?);
	`, uuid, status, status, syncedAt)
	if err != nil {
		t.Fatalf("inserting run: %v", err)
	}
	id, err := res.LastInsertId()
	if err != nil {
		t.Fatalf("LastInsertId: %v", err)
	}
	return id
}

func TestPendingRuns_FiltersByStatusAndSyncedAt(t *testing.T) {
	s, db := openFixture(t)

	stillRunning := insertRun(t, db, "uuid-running", "running", nil)
	alreadySynced := insertRun(t, db, "uuid-synced", "completed", "2026-01-02T00:00:00Z")
	pending := insertRun(t, db, "uuid-pending", "completed", nil)
	_ = stillRunning
	_ = alreadySynced

	runs, err := s.PendingRuns()
	if err != nil {
		t.Fatalf("PendingRuns: %v", err)
	}
	if len(runs) != 1 {
		t.Fatalf("got %d pending runs, want 1: %+v", len(runs), runs)
	}
	if runs[0].ID != pending {
		t.Errorf("got run id %d, want %d", runs[0].ID, pending)
	}
	if runs[0].UUID != "uuid-pending" {
		t.Errorf("got uuid %q, want uuid-pending", runs[0].UUID)
	}
	if !runs[0].Profile.Valid || runs[0].Profile.String != "idle" {
		t.Errorf("got profile %+v, want valid \"idle\"", runs[0].Profile)
	}
	if runs[0].EndedAt == "" {
		t.Error("EndedAt is empty for a completed run")
	}
}

// insertSample inserts one samples row with every base/channel column
// populated (0.0 default), overriding t/rpm/cht_c_1 -- mirrors the real
// invariant run_log_write_sample guarantees: only the s_* sensor columns
// are ever legitimately NULL, never these.
func insertSample(t *testing.T, db *sql.DB, runID int64, ts, rpm, chtC1 float64) {
	t.Helper()
	_, err := db.Exec(`
		INSERT INTO samples (
			run_id, t, throttle, alt_m, ambient_c, airspeed_ms, cool_index,
			rpm, map_kpa, torque_nm, cht_c, egt_c, oil_c,
			cht_c_1, cht_c_2, cht_c_3, cht_c_4,
			egt_c_1, egt_c_2, egt_c_3, egt_c_4,
			air_gps, fuel_kgph, fuel_press_kpa,
			lambda_1, lambda_2, lambda_3, lambda_4,
			oil_press_kpa, bus_v, alt_a, alt_field_a, batt_soc
		) VALUES (
			?, ?, 0,0,0,0,0,
			?, 0,0,0,0,0,
			?, 0,0,0,
			0,0,0,0,
			0,0,0,
			0,0,0,0,
			0,0,0,0,0
		);
	`, runID, ts, rpm, chtC1)
	if err != nil {
		t.Fatalf("inserting sample: %v", err)
	}
}

func TestSamples_ReturnsRowsInTimeOrder(t *testing.T) {
	s, db := openFixture(t)
	runID := insertRun(t, db, "uuid-1", "completed", nil)

	for _, row := range []struct {
		t, rpm, chtC1 float64
	}{
		{0.0, 700.0, 15.0},
		{0.02, 705.5, 15.2},
		{0.04, 711.0, 15.4},
	} {
		insertSample(t, db, runID, row.t, row.rpm, row.chtC1)
	}

	rows, err := s.Samples(runID)
	if err != nil {
		t.Fatalf("Samples: %v", err)
	}
	if len(rows) != 3 {
		t.Fatalf("got %d samples, want 3", len(rows))
	}
	for i, want := range []float64{0.0, 0.02, 0.04} {
		if rows[i].T != want {
			t.Errorf("row %d: got t=%v, want %v", i, rows[i].T, want)
		}
	}
	if rows[1].RPM != 705.5 {
		t.Errorf("row 1: got rpm=%v, want 705.5", rows[1].RPM)
	}
	if rows[2].CHTC1 != 15.4 {
		t.Errorf("row 2: got cht_c_1=%v, want 15.4", rows[2].CHTC1)
	}
}

func TestMarkSynced_RemovesRunFromPending(t *testing.T) {
	s, db := openFixture(t)
	runID := insertRun(t, db, "uuid-1", "completed", nil)

	runs, err := s.PendingRuns()
	if err != nil || len(runs) != 1 {
		t.Fatalf("precondition: PendingRuns() = %v, %v; want 1 run", runs, err)
	}

	if err := s.MarkSynced(runID, "backend-uuid-xyz"); err != nil {
		t.Fatalf("MarkSynced: %v", err)
	}

	runs, err = s.PendingRuns()
	if err != nil {
		t.Fatalf("PendingRuns after sync: %v", err)
	}
	if len(runs) != 0 {
		t.Fatalf("got %d pending runs after MarkSynced, want 0: %+v", len(runs), runs)
	}

	var gotBackendRunID string
	row := db.QueryRow(`SELECT backend_run_id FROM runs WHERE id = ?;`, runID)
	if err := row.Scan(&gotBackendRunID); err != nil {
		t.Fatalf("reading back backend_run_id: %v", err)
	}
	if gotBackendRunID != "backend-uuid-xyz" {
		t.Errorf("backend_run_id = %q, want backend-uuid-xyz", gotBackendRunID)
	}
}

func TestMarkSynced_UnknownRunIsAnError(t *testing.T) {
	s, _ := openFixture(t)
	if err := s.MarkSynced(999, "irrelevant"); err == nil {
		t.Fatal("MarkSynced(999) on an empty db: got nil error, want one")
	}
}
