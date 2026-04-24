package main

import (
	"testing"

	simpkg "github.com/errornesttorn/mini-traffic-simulation-core"
)

func assertVecNear(t *testing.T, got, want Vec2) {
	t.Helper()
	if distSq(got, want) > 1e-6 {
		t.Fatalf("got %+v, want %+v", got, want)
	}
}

func TestApplyMoveTargetMovesWholeJunction(t *testing.T) {
	s1 := simpkg.NewSpline(1, NewVec2(0, 0), NewVec2(1, 0), NewVec2(9, 0), NewVec2(10, 0))
	s1.Priority = true
	s1.SpeedLimitKmh = 30
	s1.HardCoupledIDs = []int{2}
	s2 := simpkg.NewSpline(2, NewVec2(10, 0), NewVec2(11, 0), NewVec2(19, 0), NewVec2(20, 0))
	s3 := simpkg.NewSpline(3, NewVec2(10, 0), NewVec2(10, 1), NewVec2(10, 9), NewVec2(10, 10))
	original := []Spline{s1, s2, s3}

	target := MoveTarget{
		Kind:    MoveHandleJunction,
		NodeKey: nodeKeyFromVec2(NewVec2(10, 0)),
		Point:   NewVec2(10, 0),
	}
	movedTo := NewVec2(12, 2)
	out, changed := applyMoveTarget(original, target, movedTo)
	if !changed {
		t.Fatal("expected move to report changed geometry")
	}

	assertVecNear(t, out[0].P3, movedTo)
	assertVecNear(t, out[0].P2, NewVec2(11, 2))
	assertVecNear(t, out[1].P0, movedTo)
	assertVecNear(t, out[1].P1, NewVec2(13, 2))
	assertVecNear(t, out[2].P0, movedTo)
	assertVecNear(t, out[2].P1, NewVec2(12, 3))

	if !out[0].Priority || out[0].SpeedLimitKmh != 30 || len(out[0].HardCoupledIDs) != 1 || out[0].HardCoupledIDs[0] != 2 {
		t.Fatalf("authored spline metadata was not preserved: %+v", out[0])
	}
}

func TestApplyMoveTargetMovesQuadraticM(t *testing.T) {
	p0 := NewVec2(0, 0)
	m := NewVec2(5, 5)
	p3 := NewVec2(10, 0)
	spline := newQuadraticSpline(7, p0, m, p3)
	newM := NewVec2(6, 4)

	out, changed := applyMoveTarget([]Spline{spline}, MoveTarget{
		Kind:     MoveHandleM,
		SplineID: spline.ID,
		Point:    m,
	}, newM)
	if !changed {
		t.Fatal("expected move to report changed geometry")
	}

	assertVecNear(t, out[0].P0, p0)
	assertVecNear(t, out[0].P3, p3)
	gotM, ok := quadraticControlPointForSpline(out[0])
	if !ok {
		t.Fatal("moved spline should still be representable as a quadratic")
	}
	assertVecNear(t, gotM, newM)
}

func TestApplyMoveTargetWithConstraintsRotatesConnectedHandles(t *testing.T) {
	incoming := simpkg.NewSpline(1, NewVec2(0, 0), NewVec2(2, 0), NewVec2(8, 0), NewVec2(10, 0))
	outgoing := simpkg.NewSpline(2, NewVec2(10, 0), NewVec2(11, 0), NewVec2(20, 0), NewVec2(21, 0))
	branch := simpkg.NewSpline(3, NewVec2(10, 0), NewVec2(10, 3), NewVec2(14, 8), NewVec2(15, 9))
	original := []Spline{incoming, outgoing, branch}

	out, changed := applyMoveTargetWithConstraints(original, MoveTarget{
		Kind:     MoveHandleP1,
		SplineID: outgoing.ID,
		Point:    outgoing.P1,
	}, NewVec2(10, 4))
	if !changed {
		t.Fatal("expected constrained move to report changed geometry")
	}

	assertVecNear(t, out[1].P1, NewVec2(10, 4))
	assertVecNear(t, out[0].P2, NewVec2(10, -2))
	assertVecNear(t, out[2].P1, NewVec2(10, 3))
}
