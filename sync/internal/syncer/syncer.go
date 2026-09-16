package syncer

import (
	"context"
	"errors"
	"fmt"

	"aero-engine-dt/sync/internal/backend"
	"aero-engine-dt/sync/internal/parquetenc"
	"aero-engine-dt/sync/internal/store"
)

type Status int

const (
	// StatusSynced: uploaded and marked synced locally. Done.
	StatusSynced Status = iota
	// StatusFailed: a clean failure
	StatusFailed
	// StatusAmbiguous: backend.ErrAmbiguousOutcome 
	StatusAmbiguous
)

func (s Status) String() string {
	switch s {
	case StatusSynced:
		return "synced"
	case StatusFailed:
		return "failed"
	case StatusAmbiguous:
		return "ambiguous"
	default:
		return "unknown"
	}
}

// Result is one run's sync outcome.
type Result struct {
	RunID  int64
	UUID   string
	Status Status
	Err    error // nil only when Status == StatusSynced
}

type Syncer struct {
	store   *store.Store
	backend *backend.Client
}

func New(s *store.Store, b *backend.Client) *Syncer {
	return &Syncer{store: s, backend: b}
}

// SyncPending uploads every completed, not-yet-synced run and returns one
// Result per run attempted, in the order PendingRuns() lists them
func (s *Syncer) SyncPending(ctx context.Context) ([]Result, error) {
	runs, err := s.store.PendingRuns()
	if err != nil {
		return nil, fmt.Errorf("syncer: listing pending runs: %w", err)
	}

	results := make([]Result, 0, len(runs))
	for _, r := range runs {
		results = append(results, s.syncOne(ctx, r))
	}
	return results, nil
}

func (s *Syncer) syncOne(ctx context.Context, r store.Run) Result {
	res := Result{RunID: r.ID, UUID: r.UUID}

	samples, err := s.store.Samples(r.ID)
	if err != nil {
		res.Status = StatusFailed
		res.Err = fmt.Errorf("reading samples for run %d: %w", r.ID, err)
		return res
	}

	data, err := parquetenc.Encode(samples)
	if err != nil {
		res.Status = StatusFailed
		res.Err = fmt.Errorf("encoding run %d: %w", r.ID, err)
		return res
	}

	resp, err := s.backend.Ingest(ctx, r.UUID, data)
	if err != nil {
		if errors.Is(err, backend.ErrAmbiguousOutcome) {
			res.Status = StatusAmbiguous
		} else {
			res.Status = StatusFailed
		}
		res.Err = fmt.Errorf("uploading run %d (uuid=%s): %w", r.ID, r.UUID, err)
		return res
	}

	if err := s.store.MarkSynced(r.ID, resp.RunID); err != nil {
		res.Status = StatusFailed
		res.Err = fmt.Errorf(
			"run %d uploaded (backend_run_id=%s) but marking synced failed: %w",
			r.ID, resp.RunID, err)
		return res
	}

	res.Status = StatusSynced
	return res
}
