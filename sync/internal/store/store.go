// Package store reads the local run_log database that twin_sim and the dashboard log runs to
package store

import (
	"database/sql"
	"fmt"

	_ "modernc.org/sqlite"
)

// Run is one row from the `runs` table
type Run struct {
	ID         int64
	UUID       string
	Profile    sql.NullString 
	Dt         float64
	DurationS  float64
	LoadNm     float64
	Seed       int64
	EngineSpec string
	WithSensor bool
	StartedAt  string
	EndedAt    string // non-NULL: PendingRuns only returns completed/aborted runs
}

// SampleRow is one row of a run's `samples` table 
type SampleRow struct {
	T            float64
	Throttle     float64
	AltM         float64
	AmbientC     float64
	AirspeedMs   float64
	CoolIndex    float64
	RPM          float64
	MapKPa       float64
	TorqueNm     float64
	CHTC         float64
	EGTC         float64
	OilC         float64
	CHTC1        float64
	CHTC2        float64
	CHTC3        float64
	CHTC4        float64
	EGTC1        float64
	EGTC2        float64
	EGTC3        float64
	EGTC4        float64
	AirGps       float64
	FuelKgph     float64
	FuelPressKPa float64
	Lambda1      float64
	Lambda2      float64
	Lambda3      float64
	Lambda4      float64
	OilPressKPa  float64
	BusV         float64
	AltA         float64
	AltFieldA    float64
	BattSoc      float64
}

type Store struct {
	db *sql.DB
}

// Open opens the run_log database at path
func Open(path string) (*Store, error) {
	db, err := sql.Open("sqlite", path)
	if err != nil {
		return nil, fmt.Errorf("store: opening %s: %w", path, err)
	}
	if err := db.Ping(); err != nil {
		db.Close()
		return nil, fmt.Errorf("store: opening %s: %w", path, err)
	}
	return &Store{db: db}, nil
}

func (s *Store) Close() error {
	return s.db.Close()
}

// PendingRuns returns every completed run that hasn't been synced yet,
// oldest first. A run still in progress (status='running') is never
// returned 
func (s *Store) PendingRuns() ([]Run, error) {
	rows, err := s.db.Query(`
		SELECT id, uuid, profile, dt, duration_s, load_nm, seed,
		       engine_spec, with_sensor, started_at, ended_at
		FROM runs
		WHERE status = 'completed' AND synced_at IS NULL
		ORDER BY id;
	`)
	if err != nil {
		return nil, fmt.Errorf("store: listing pending runs: %w", err)
	}
	defer rows.Close()

	var runs []Run
	for rows.Next() {
		var r Run
		var withSensor int
		if err := rows.Scan(&r.ID, &r.UUID, &r.Profile, &r.Dt, &r.DurationS,
			&r.LoadNm, &r.Seed, &r.EngineSpec, &withSensor, &r.StartedAt,
			&r.EndedAt); err != nil {
			return nil, fmt.Errorf("store: scanning run row: %w", err)
		}
		r.WithSensor = withSensor != 0
		runs = append(runs, r)
	}
	return runs, rows.Err()
}

// Samples returns every sample row for runID, in time order.
func (s *Store) Samples(runID int64) ([]SampleRow, error) {
	rows, err := s.db.Query(`
		SELECT t, throttle, alt_m, ambient_c, airspeed_ms, cool_index,
		       rpm, map_kpa, torque_nm, cht_c, egt_c, oil_c,
		       cht_c_1, cht_c_2, cht_c_3, cht_c_4,
		       egt_c_1, egt_c_2, egt_c_3, egt_c_4,
		       air_gps, fuel_kgph, fuel_press_kpa,
		       lambda_1, lambda_2, lambda_3, lambda_4,
		       oil_press_kpa, bus_v, alt_a, alt_field_a, batt_soc
		FROM samples
		WHERE run_id = ?
		ORDER BY id;
	`, runID)
	if err != nil {
		return nil, fmt.Errorf("store: querying samples for run %d: %w", runID, err)
	}
	defer rows.Close()

	var out []SampleRow
	for rows.Next() {
		var r SampleRow
		if err := rows.Scan(&r.T, &r.Throttle, &r.AltM, &r.AmbientC, &r.AirspeedMs,
			&r.CoolIndex, &r.RPM, &r.MapKPa, &r.TorqueNm, &r.CHTC, &r.EGTC, &r.OilC,
			&r.CHTC1, &r.CHTC2, &r.CHTC3, &r.CHTC4,
			&r.EGTC1, &r.EGTC2, &r.EGTC3, &r.EGTC4,
			&r.AirGps, &r.FuelKgph, &r.FuelPressKPa,
			&r.Lambda1, &r.Lambda2, &r.Lambda3, &r.Lambda4,
			&r.OilPressKPa, &r.BusV, &r.AltA, &r.AltFieldA, &r.BattSoc); err != nil {
			return nil, fmt.Errorf("store: scanning sample row: %w", err)
		}
		out = append(out, r)
	}
	return out, rows.Err()
}

// MarkSynced stamps synced_at and records the backend's assigned run id
// after a successful upload, so PendingRuns won't return this run again.
// backendRunID is the server's own uuid from its /ingest response -- it is
// not the same value as this run's local uuid, since the backend assigns
// its own id rather than accepting a client-supplied one.
func (s *Store) MarkSynced(runID int64, backendRunID string) error {
	res, err := s.db.Exec(
		`UPDATE runs SET synced_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'),
		                 backend_run_id = ? WHERE id = ?;`,
		backendRunID, runID)
	if err != nil {
		return fmt.Errorf("store: marking run %d synced: %w", runID, err)
	}
	n, err := res.RowsAffected()
	if err != nil {
		return fmt.Errorf("store: marking run %d synced: %w", runID, err)
	}
	if n == 0 {
		return fmt.Errorf("store: marking run %d synced: no such run", runID)
	}
	return nil
}
