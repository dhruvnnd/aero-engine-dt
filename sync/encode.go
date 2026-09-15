//go:build ignore

// Dev tool: encodes one run from a run_log database into a Parquet file on
// disk, so you can inspect what the sync tool would actually send.
// Not part of the sync tool itself (go:build ignore keeps it out of normal
// builds/tests) -- run directly with `go run encode.go [flags]`.
//
// Only shows completed, not-yet-synced runs (same query the real sync tool
// will use) -- a run already marked synced won't be picked by -run 0, but
// you can still target it directly with an explicit -run ID since Samples()
// doesn't care about sync status.
package main

import (
	"flag"
	"fmt"
	"log"
	"os"

	"aero-engine-dt/sync/internal/parquetenc"
	"aero-engine-dt/sync/internal/store"
)

func main() {
	dbPath := flag.String("db", "../runs/twin_sim.db", "run_log database to read")
	runID := flag.Int64("run", 0, "run id to encode (default: the first pending run)")
	outPath := flag.String("out", "out.parquet", "where to write the encoded Parquet file")
	flag.Parse()

	s, err := store.Open(*dbPath)
	if err != nil {
		log.Fatal(err)
	}
	defer s.Close()

	id := *runID
	var uuid string
	if id == 0 {
		runs, err := s.PendingRuns()
		if err != nil {
			log.Fatal(err)
		}
		if len(runs) == 0 {
			log.Fatalf("no pending (completed, unsynced) runs in %s -- pass -run ID to target a specific run", *dbPath)
		}
		id = runs[0].ID
		uuid = runs[0].UUID
	}

	samples, err := s.Samples(id)
	if err != nil {
		log.Fatal(err)
	}
	if len(samples) == 0 {
		log.Fatalf("run %d has no samples -- check the id and db path", id)
	}

	data, err := parquetenc.Encode(samples)
	if err != nil {
		log.Fatal(err)
	}
	if err := os.WriteFile(*outPath, data, 0644); err != nil {
		log.Fatal(err)
	}

	if uuid != "" {
		fmt.Printf("run %d (uuid=%s): %d samples -> %d bytes -> %s\n",
			id, uuid, len(samples), len(data), *outPath)
	} else {
		fmt.Printf("run %d: %d samples -> %d bytes -> %s\n",
			id, len(samples), len(data), *outPath)
	}
}
