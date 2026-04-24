//go:build !darwin

package main

import (
	"os"
	"testing"
)

func TestParseAscHeader(t *testing.T) {
	p := "/home/eryk/workspace/warsaw-commute-game/example-map/83232_1744593_N-34-138-B-b-3-2.asc"
	if _, err := os.Stat(p); err != nil {
		t.Skip("example asc not available")
	}
	h, err := parseAscHeader(p)
	if err != nil {
		t.Fatal(err)
	}
	if h.NCols != 4385 || h.NRows != 4745 {
		t.Fatalf("unexpected dims: %+v", h)
	}
	if h.CellSize != 0.5 {
		t.Fatalf("unexpected cellsize: %v", h.CellSize)
	}
	if h.LLIsCorner {
		t.Fatalf("expected center not corner")
	}
}

func TestFindSmallestIFDIndex(t *testing.T) {
	p := "/home/eryk/workspace/warsaw-commute-game/example-map/73019_868009_N-34-138-B-b-4-1.tif"
	if _, err := os.Stat(p); err != nil {
		t.Skip("example tif not available")
	}
	idx, w, h, err := findSmallestIFDIndex(p)
	if err != nil {
		t.Fatal(err)
	}
	if idx <= 0 {
		t.Fatalf("expected a non-first IFD to be smallest, got idx=%d", idx)
	}
	if w > 8000 || h > 8000 {
		t.Fatalf("sub-image too large for GPU: %dx%d", w, h)
	}
	if w < 1500 || h < 1500 {
		t.Fatalf("sub-image too low res: %dx%d", w, h)
	}
}
