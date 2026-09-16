// Package backend is the HTTP client for AeroEngineTwin's /ingest API.
package backend

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"mime/multipart"
	"net/http"
	"time"
)

var ErrAmbiguousOutcome = errors.New("backend: outcome unknown")

// StatusError is a clean non-2xx HTTP response
type StatusError struct {
	StatusCode int
	Status     string
	Body       string
}

func (e *StatusError) Error() string {
	return fmt.Sprintf("backend: /ingest returned %s: %s", e.Status, e.Body)
}

// IngestResponse mirrors /ingest's JSON response body.
type IngestResponse struct {
	RunID        string `json:"run_id"`
	SessionName  string `json:"session_name"`
	RowsIngested int    `json:"rows_ingested"`
	Archived     bool   `json:"archived"`
	ArchiveError string `json:"archive_error,omitempty"`
}

// Client talks to one AeroEngineTwin backend instance.
type Client struct {
	baseURL    string
	httpClient *http.Client
}

// New returns a Client for baseURL. timeout applies per HTTP call; pass 0 to rely on the
// caller's context deadline instead.
func New(baseURL string, timeout time.Duration) *Client {
	return &Client{
		baseURL:    baseURL,
		httpClient: &http.Client{Timeout: timeout},
	}
}

// Ingest uploads one run's Parquet-encoded samples. sessionName is the
// label the backend stores alongside its own generated run_id
func (c *Client) Ingest(ctx context.Context, sessionName string, parquetData []byte) (*IngestResponse, error) {
	var body bytes.Buffer
	writer := multipart.NewWriter(&body)

	if err := writer.WriteField("session_name", sessionName); err != nil {
		return nil, fmt.Errorf("backend: building request: %w", err)
	}
	part, err := writer.CreateFormFile("file", "run.parquet")
	if err != nil {
		return nil, fmt.Errorf("backend: building request: %w", err)
	}
	if _, err := part.Write(parquetData); err != nil {
		return nil, fmt.Errorf("backend: building request: %w", err)
	}
	if err := writer.Close(); err != nil {
		return nil, fmt.Errorf("backend: building request: %w", err)
	}

	req, err := http.NewRequestWithContext(ctx, http.MethodPost, c.baseURL+"/ingest", &body)
	if err != nil {
		return nil, fmt.Errorf("backend: building request: %w", err)
	}
	req.Header.Set("Content-Type", writer.FormDataContentType())

	resp, err := c.httpClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("%w: request failed: %w", ErrAmbiguousOutcome, err)
	}
	defer resp.Body.Close()

	respBody, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("%w: reading response: %w", ErrAmbiguousOutcome, err)
	}

	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		return nil, &StatusError{StatusCode: resp.StatusCode, Status: resp.Status, Body: string(respBody)}
	}

	var result IngestResponse
	if err := json.Unmarshal(respBody, &result); err != nil {
		// A 2xx with an unparseable body is itself ambiguous
		return nil, fmt.Errorf("%w: parsing response: %w", ErrAmbiguousOutcome, err)
	}
	return &result, nil
}
