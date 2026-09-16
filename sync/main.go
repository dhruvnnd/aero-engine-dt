package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"time"

	"aero-engine-dt/sync/internal/backend"
	"aero-engine-dt/sync/internal/store"
	"aero-engine-dt/sync/internal/syncer"
)

func main() {
	dbPath := flag.String("db", "runs/twin_sim.db", "run_log database to sync from")
	serverURL := flag.String("server", "",
		"AeroEngineTwin backend base URL (e.g. https://host:port) -- required unless -dry-run")
	timeout := flag.Duration("timeout", 60*time.Second, "per-upload HTTP timeout")
	dryRun := flag.Bool("dry-run", false,
		"list pending runs and sample counts without uploading or contacting a server")
	flag.Parse()

	if *serverURL == "" && !*dryRun {
		fmt.Fprintln(os.Stderr, "aero-sync: -server is required (or pass -dry-run)")
		flag.Usage()
		os.Exit(2)
	}

	s, err := store.Open(*dbPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "aero-sync: %v\n", err)
		os.Exit(1)
	}
	defer s.Close()

	if *dryRun {
		os.Exit(runDryRun(s))
	}
	os.Exit(runSync(s, *serverURL, *timeout))
}

func runDryRun(s *store.Store) int {
	runs, err := s.PendingRuns()
	if err != nil {
		fmt.Fprintf(os.Stderr, "aero-sync: %v\n", err)
		return 1
	}
	if len(runs) == 0 {
		fmt.Println("aero-sync: no pending runs")
		return 0
	}

	for _, r := range runs {
		samples, err := s.Samples(r.ID)
		if err != nil {
			fmt.Fprintf(os.Stderr, "aero-sync: run %d: %v\n", r.ID, err)
			continue
		}
		profile := "--"
		if r.Profile.Valid {
			profile = r.Profile.String
		}
		fmt.Printf("run %d (uuid=%s profile=%s): %d samples, would upload\n",
			r.ID, r.UUID, profile, len(samples))
	}
	return 0
}

func runSync(s *store.Store, serverURL string, timeout time.Duration) int {
	client := backend.New(serverURL, timeout)
	sy := syncer.New(s, client)

	results, err := sy.SyncPending(context.Background())
	if err != nil {
		fmt.Fprintf(os.Stderr, "aero-sync: %v\n", err)
		return 1
	}
	if len(results) == 0 {
		fmt.Println("aero-sync: no pending runs")
		return 0
	}

	var failed, ambiguous int
	for _, r := range results {
		switch r.Status {
		case syncer.StatusSynced:
			fmt.Printf("run %d (uuid=%s): synced\n", r.RunID, r.UUID)
		case syncer.StatusFailed:
			failed++
			fmt.Fprintf(os.Stderr, "run %d (uuid=%s): FAILED: %v\n", r.RunID, r.UUID, r.Err)
		case syncer.StatusAmbiguous:
			ambiguous++
			fmt.Fprintf(os.Stderr,
				"run %d (uuid=%s): AMBIGUOUS -- outcome unknown, left pending; "+
					"check %s/runs for a matching session_name before assuming it needs a retry: %v\n",
				r.RunID, r.UUID, serverURL, r.Err)
		}
	}

	fmt.Printf("aero-sync: %d synced, %d failed, %d ambiguous\n",
		len(results)-failed-ambiguous, failed, ambiguous)

	if failed > 0 || ambiguous > 0 {
		return 1
	}
	return 0
}
