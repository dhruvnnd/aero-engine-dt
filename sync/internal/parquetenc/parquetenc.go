package parquetenc

import (
	"bytes"
	"fmt"

	"github.com/parquet-go/parquet-go"

	"aero-engine-dt/sync/internal/store"
)

// Row is one line of the payload
type Row struct {
	T            float64 `parquet:"t"`
	Throttle     float32 `parquet:"throttle"`
	AltM         float32 `parquet:"alt_m"`
	AmbientC     float32 `parquet:"ambient_c"`
	AirspeedMs   float32 `parquet:"airspeed_ms"`
	CoolIndex    float32 `parquet:"cool_index"`
	RPM          float32 `parquet:"rpm"`
	MapKPa       float32 `parquet:"map_kpa"`
	TorqueNm     float32 `parquet:"torque_nm"`
	CHTC         float32 `parquet:"cht_c"`
	EGTC         float32 `parquet:"egt_c"`
	OilC         float32 `parquet:"oil_c"`
	CHTC1        float32 `parquet:"cht_c_1"`
	CHTC2        float32 `parquet:"cht_c_2"`
	CHTC3        float32 `parquet:"cht_c_3"`
	CHTC4        float32 `parquet:"cht_c_4"`
	EGTC1        float32 `parquet:"egt_c_1"`
	EGTC2        float32 `parquet:"egt_c_2"`
	EGTC3        float32 `parquet:"egt_c_3"`
	EGTC4        float32 `parquet:"egt_c_4"`
	AirGps       float32 `parquet:"air_gps"`
	FuelKgph     float32 `parquet:"fuel_kgph"`
	FuelPressKPa float32 `parquet:"fuel_press_kpa"`
	Lambda1      float32 `parquet:"lambda_1"`
	Lambda2      float32 `parquet:"lambda_2"`
	Lambda3      float32 `parquet:"lambda_3"`
	Lambda4      float32 `parquet:"lambda_4"`
	OilPressKPa  float32 `parquet:"oil_press_kpa"`
	BusV         float32 `parquet:"bus_v"`
	AltA         float32 `parquet:"alt_a"`
	AltFieldA    float32 `parquet:"alt_field_a"`
	BattSoc      float32 `parquet:"batt_soc"`
}

// FromSample narrows one store.SampleRow into the backend's float32 wire representation.
func FromSample(s store.SampleRow) Row {
	return Row{
		T:            s.T,
		Throttle:     float32(s.Throttle),
		AltM:         float32(s.AltM),
		AmbientC:     float32(s.AmbientC),
		AirspeedMs:   float32(s.AirspeedMs),
		CoolIndex:    float32(s.CoolIndex),
		RPM:          float32(s.RPM),
		MapKPa:       float32(s.MapKPa),
		TorqueNm:     float32(s.TorqueNm),
		CHTC:         float32(s.CHTC),
		EGTC:         float32(s.EGTC),
		OilC:         float32(s.OilC),
		CHTC1:        float32(s.CHTC1),
		CHTC2:        float32(s.CHTC2),
		CHTC3:        float32(s.CHTC3),
		CHTC4:        float32(s.CHTC4),
		EGTC1:        float32(s.EGTC1),
		EGTC2:        float32(s.EGTC2),
		EGTC3:        float32(s.EGTC3),
		EGTC4:        float32(s.EGTC4),
		AirGps:       float32(s.AirGps),
		FuelKgph:     float32(s.FuelKgph),
		FuelPressKPa: float32(s.FuelPressKPa),
		Lambda1:      float32(s.Lambda1),
		Lambda2:      float32(s.Lambda2),
		Lambda3:      float32(s.Lambda3),
		Lambda4:      float32(s.Lambda4),
		OilPressKPa:  float32(s.OilPressKPa),
		BusV:         float32(s.BusV),
		AltA:         float32(s.AltA),
		AltFieldA:    float32(s.AltFieldA),
		BattSoc:      float32(s.BattSoc),
	}
}

// Encode renders a run's samples into a Snappy-compressed Parquet file, in
// memory. Empty input still produces a valid
// zero-row Parquet file rather than erroring 
func Encode(samples []store.SampleRow) ([]byte, error) {
	rows := make([]Row, len(samples))
	for i, s := range samples {
		rows[i] = FromSample(s)
	}

	var buf bytes.Buffer
	writer := parquet.NewGenericWriter[Row](&buf, parquet.Compression(&parquet.Snappy))
	if _, err := writer.Write(rows); err != nil {
		return nil, fmt.Errorf("parquetenc: writing rows: %w", err)
	}
	if err := writer.Close(); err != nil {
		return nil, fmt.Errorf("parquetenc: closing writer: %w", err)
	}
	return buf.Bytes(), nil
}
