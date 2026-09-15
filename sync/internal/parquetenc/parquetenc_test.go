package parquetenc

import (
	"bytes"
	"testing"

	"github.com/parquet-go/parquet-go"

	"aero-engine-dt/sync/internal/store"
)

// The exact column list from AeroEngineTwin's SimulationRow
// (src/schemas/sim_table.py), in its declared order. If this ever drifts
// from Row's parquet tags, uploads will start failing the backend's schema
// check -- this test is the tripwire for that, not something to loosen.
var backendColumns = []string{
	"t", "throttle", "alt_m", "ambient_c", "airspeed_ms", "cool_index",
	"rpm", "map_kpa", "torque_nm", "cht_c", "egt_c", "oil_c",
	"cht_c_1", "cht_c_2", "cht_c_3", "cht_c_4",
	"egt_c_1", "egt_c_2", "egt_c_3", "egt_c_4",
	"air_gps", "fuel_kgph", "fuel_press_kpa",
	"lambda_1", "lambda_2", "lambda_3", "lambda_4",
	"oil_press_kpa", "bus_v", "alt_a", "alt_field_a", "batt_soc",
}

func TestRowSchema_MatchesBackendColumns(t *testing.T) {
	schema := parquet.SchemaOf(Row{})
	var got []string
	for _, f := range schema.Fields() {
		got = append(got, f.Name())
	}
	if len(got) != len(backendColumns) {
		t.Fatalf("got %d columns, want %d\ngot:  %v\nwant: %v",
			len(got), len(backendColumns), got, backendColumns)
	}
	for i, want := range backendColumns {
		if got[i] != want {
			t.Errorf("column %d: got %q, want %q", i, got[i], want)
		}
	}
}

func sampleFixture() []store.SampleRow {
	return []store.SampleRow{
		{
			T: 0.0, Throttle: 0.0, AltM: 0, AmbientC: 15.0, AirspeedMs: 0, CoolIndex: 0.1,
			RPM: 700.123, MapKPa: 45.5, TorqueNm: -3.2, CHTC: 15.0, EGTC: 20.0, OilC: 15.0,
			CHTC1: 15.1, CHTC2: 14.9, CHTC3: 15.0, CHTC4: 15.05,
			EGTC1: 20.1, EGTC2: 19.8, EGTC3: 20.0, EGTC4: 20.2,
			AirGps: 2.5, FuelKgph: 1.1, FuelPressKPa: 300.0,
			Lambda1: 1.0, Lambda2: 0.98, Lambda3: 1.02, Lambda4: 1.0,
			OilPressKPa: 250.0, BusV: 13.8, AltA: 5.0, AltFieldA: 1.2, BattSoc: 0.95,
		},
		{
			T: 0.5, Throttle: 0.75, AltM: 100, AmbientC: 14.5, AirspeedMs: 10, CoolIndex: 0.3,
			RPM: 1500.5, MapKPa: 80.0, TorqueNm: 40.0, CHTC: 90.0, EGTC: 650.0, OilC: 70.0,
			CHTC1: 91.0, CHTC2: 89.0, CHTC3: 90.5, CHTC4: 89.5,
			EGTC1: 655.0, EGTC2: 648.0, EGTC3: 650.0, EGTC4: 651.0,
			AirGps: 8.0, FuelKgph: 5.5, FuelPressKPa: 295.0,
			Lambda1: 0.95, Lambda2: 0.96, Lambda3: 0.94, Lambda4: 0.97,
			OilPressKPa: 260.0, BusV: 14.1, AltA: 20.0, AltFieldA: 2.0, BattSoc: 0.96,
		},
	}
}

func TestEncode_RoundTripsValues(t *testing.T) {
	samples := sampleFixture()
	data, err := Encode(samples)
	if err != nil {
		t.Fatalf("Encode: %v", err)
	}
	if len(data) < 8 || string(data[:4]) != "PAR1" || string(data[len(data)-4:]) != "PAR1" {
		t.Fatal("output doesn't look like a real Parquet file (missing PAR1 magic)")
	}

	reader := parquet.NewGenericReader[Row](bytes.NewReader(data))
	defer reader.Close()
	got := make([]Row, len(samples))
	n, err := reader.Read(got)
	if err != nil && n != len(samples) {
		t.Fatalf("reading back: %v (got %d rows)", err, n)
	}
	if n != len(samples) {
		t.Fatalf("got %d rows back, want %d", n, len(samples))
	}

	for i, s := range samples {
		want := FromSample(s)
		if got[i] != want {
			t.Errorf("row %d: got %+v, want %+v", i, got[i], want)
		}
	}
}

func TestEncode_EmptyInputProducesValidFile(t *testing.T) {
	data, err := Encode(nil)
	if err != nil {
		t.Fatalf("Encode(nil): %v", err)
	}
	if len(data) < 8 || string(data[:4]) != "PAR1" {
		t.Fatal("empty-input output doesn't look like a real Parquet file")
	}

	reader := parquet.NewGenericReader[Row](bytes.NewReader(data))
	defer reader.Close()
	if reader.NumRows() != 0 {
		t.Errorf("got %d rows for empty input, want 0", reader.NumRows())
	}
}
