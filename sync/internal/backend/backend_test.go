package backend

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

func TestIngest_Success_SendsExpectedFieldsAndParsesResponse(t *testing.T) {
	const wantBody = "not real parquet bytes, just checking they arrive intact"
	var gotSessionName string
	var gotFileBytes []byte
	var gotContentType string

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		gotContentType = r.Header.Get("Content-Type")
		if err := r.ParseMultipartForm(1 << 20); err != nil {
			t.Fatalf("server: ParseMultipartForm: %v", err)
		}
		gotSessionName = r.FormValue("session_name")

		f, _, err := r.FormFile("file")
		if err != nil {
			t.Fatalf("server: FormFile: %v", err)
		}
		defer f.Close()
		buf := make([]byte, len(wantBody)+10)
		n, _ := f.Read(buf)
		gotFileBytes = buf[:n]

		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		w.Write([]byte(`{"run_id":"11111111-2222-3333-4444-555555555555","session_name":"` +
			gotSessionName + `","rows_ingested":401,"archived":true}`))
	}))
	defer srv.Close()

	client := New(srv.URL, 5*time.Second)
	resp, err := client.Ingest(context.Background(), "local-uuid-abc", []byte(wantBody))
	if err != nil {
		t.Fatalf("Ingest: %v", err)
	}

	if gotSessionName != "local-uuid-abc" {
		t.Errorf("server saw session_name=%q, want local-uuid-abc", gotSessionName)
	}
	if string(gotFileBytes) != wantBody {
		t.Errorf("server saw file bytes=%q, want %q", gotFileBytes, wantBody)
	}
	if len(gotContentType) < 19 || gotContentType[:19] != "multipart/form-data" {
		t.Errorf("Content-Type = %q, want multipart/form-data...", gotContentType)
	}

	if resp.RunID != "11111111-2222-3333-4444-555555555555" {
		t.Errorf("RunID = %q, want the server-assigned uuid", resp.RunID)
	}
	if resp.RowsIngested != 401 {
		t.Errorf("RowsIngested = %d, want 401", resp.RowsIngested)
	}
	if !resp.Archived {
		t.Error("Archived = false, want true")
	}
}

func TestIngest_NonSuccessStatus_ReturnsStatusErrorNotAmbiguous(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusUnprocessableEntity)
		w.Write([]byte(`{"detail":"File is not valid Parquet data"}`))
	}))
	defer srv.Close()

	client := New(srv.URL, 5*time.Second)
	_, err := client.Ingest(context.Background(), "s", []byte("bad data"))
	if err == nil {
		t.Fatal("got nil error for a 422 response")
	}

	var statusErr *StatusError
	if !errors.As(err, &statusErr) {
		t.Fatalf("got %v (%T), want a *StatusError", err, err)
	}
	if statusErr.StatusCode != 422 {
		t.Errorf("StatusCode = %d, want 422", statusErr.StatusCode)
	}
	if errors.Is(err, ErrAmbiguousOutcome) {
		t.Error("a clean 422 response should not be ErrAmbiguousOutcome -- " +
			"the server definitely rejected it")
	}
}

func TestIngest_NetworkFailure_IsAmbiguous(t *testing.T) {
	// A server that's already closed guarantees a connection-level failure
	// (refused), never a real HTTP response -- exactly the "outcome
	// unknown" case ErrAmbiguousOutcome exists for.
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {}))
	srv.Close()

	client := New(srv.URL, 2*time.Second)
	_, err := client.Ingest(context.Background(), "s", []byte("data"))
	if err == nil {
		t.Fatal("got nil error posting to a closed server")
	}
	if !errors.Is(err, ErrAmbiguousOutcome) {
		t.Errorf("got %v, want it to wrap ErrAmbiguousOutcome", err)
	}
}
