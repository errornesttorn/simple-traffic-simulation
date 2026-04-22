package sim

import (
	"math"
	"math/rand"
	"testing"
)

func TestUpdatePedestriansSpawnsFromDeadEnds(t *testing.T) {
	rand.Seed(1)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(12, 0)},
	}
	nextID := 1
	pedestrians, timers := updatePedestrians(nil, paths, &nextID, nil, pedestrianSpawnIntervalS*(1+pedestrianSpawnJitterFrac)+1, nil, nil)

	if len(pedestrians) != 2 {
		t.Fatalf("expected one pedestrian from each dead end, got %d", len(pedestrians))
	}
	if len(timers) != 2 {
		t.Fatalf("expected two spawn timers, got %d", len(timers))
	}
	for i, ped := range pedestrians {
		if ped.ID != i+1 {
			t.Fatalf("expected pedestrian ID %d, got %d", i+1, ped.ID)
		}
		if ped.PathIndex != 0 {
			t.Fatalf("expected pedestrian on path 0, got path %d", ped.PathIndex)
		}
		if ped.Distance < pedestrianSpawnInsetM {
			t.Fatalf("expected spawned pedestrian to start inside path, got distance %.3f", ped.Distance)
		}
	}
}

func TestUpdatePedestriansBranchAndDespawnAtDeadEnd(t *testing.T) {
	rand.Seed(2)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(2, 0)},
		{P0: NewVec2(2, 0), P1: NewVec2(3, 1)},
		{P0: NewVec2(2, 0), P1: NewVec2(3, -1)},
	}
	pedestrians := []Pedestrian{
		{
			ID:        1,
			PathIndex: 0,
			Distance:  1.85,
			Forward:   true,
			Speed:     1.4,
			BaseSpeed: 1.4,
			Radius:    pedestrianRadiusM,
		},
	}

	pedestrians, timers := updatePedestrians(pedestrians, paths, nil, nil, 0.3, nil, nil)
	if len(pedestrians) != 1 {
		t.Fatalf("expected pedestrian to survive branch, got %d", len(pedestrians))
	}
	if !pedestrians[0].TransitionActive {
		t.Fatal("expected pedestrian to enter a junction transition")
	}
	chosenPath := pedestrians[0].TransitionNextPath
	if chosenPath != 1 && chosenPath != 2 {
		t.Fatalf("expected pedestrian to choose one of the branch edges, got %d", chosenPath)
	}

	for step := 0; step < 5 && len(pedestrians) > 0; step++ {
		pedestrians, timers = updatePedestrians(pedestrians, paths, nil, timers, 0.4, nil, nil)
	}
	if len(pedestrians) != 0 {
		t.Fatalf("expected pedestrian to despawn at branch dead end, still have %d", len(pedestrians))
	}
}

func TestUpdatePedestriansSoftAvoidanceKeepsThemMoving(t *testing.T) {
	rand.Seed(3)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	pedestrians := []Pedestrian{
		{
			ID:            1,
			PathIndex:     0,
			Distance:      9.0,
			Forward:       true,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: 0,
		},
		{
			ID:            2,
			PathIndex:     0,
			Distance:      9.0,
			Forward:       false,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: 0,
		},
	}

	pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.5, nil, nil)
	if len(pedestrians) != 2 {
		t.Fatalf("expected both pedestrians to remain active, got %d", len(pedestrians))
	}
	if pedestrians[0].Distance <= 9.0 || pedestrians[1].Distance <= 9.0 {
		t.Fatalf("expected both pedestrians to keep moving, got distances %.3f and %.3f", pedestrians[0].Distance, pedestrians[1].Distance)
	}
	if pedestrians[0].LateralOffset <= 0.1 {
		t.Fatalf("expected forward pedestrian to shift to one side, got lateral offset %.3f", pedestrians[0].LateralOffset)
	}
	if pedestrians[1].LateralOffset >= -0.1 {
		t.Fatalf("expected reverse pedestrian to shift to the opposite side, got lateral offset %.3f", pedestrians[1].LateralOffset)
	}
	if pedestrians[0].Speed <= 0 || pedestrians[1].Speed <= 0 {
		t.Fatalf("expected soft avoidance to preserve forward motion, got speeds %.3f and %.3f", pedestrians[0].Speed, pedestrians[1].Speed)
	}
}

func TestPedestrianPoseTraversesTurnConnector(t *testing.T) {
	rand.Seed(4)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(2, 0)},
		{P0: NewVec2(2, 0), P1: NewVec2(2, 2)},
	}
	pedestrians := []Pedestrian{
		{
			ID:            1,
			PathIndex:     0,
			Distance:      1.95,
			Forward:       true,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: pedestrianPreferredOffsetM,
		},
	}

	foundMidTransition := false
	ped := Pedestrian{}
	for step := 0; step < 12; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.15, nil, nil)
		if len(pedestrians) != 1 {
			t.Fatalf("expected pedestrian to remain active, got %d", len(pedestrians))
		}
		ped = pedestrians[0]
		if ped.TransitionActive && ped.TransitionLength > 0 && ped.TransitionDistance/ped.TransitionLength >= 0.35 {
			foundMidTransition = true
			break
		}
	}
	if !foundMidTransition {
		t.Fatal("expected pedestrian to reach the middle of a turn transition")
	}
	pos, heading, ok := PedestrianPose(paths, ped)
	if !ok {
		t.Fatal("expected turn-connector pedestrian pose")
	}
	if pos.X <= 1.2 || pos.X >= 2.75 || pos.Y <= -0.8 || pos.Y >= 1.8 {
		t.Fatalf("expected connector position near the corner, got (%.3f, %.3f)", pos.X, pos.Y)
	}
	if absf(heading.X) <= 0.2 || absf(heading.Y) <= 0.2 {
		t.Fatalf("expected connector heading between old and new edges, got (%.3f, %.3f)", heading.X, heading.Y)
	}
}

func buildCrossingTestSpline(id int) Spline {
	return NewSpline(id,
		NewVec2(10, -10),
		NewVec2(10, -3),
		NewVec2(10, 3),
		NewVec2(10, 10),
	)
}

func TestComputePedestrianCrossingsFindsIntersection(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	if len(crossings) != 1 {
		t.Fatalf("expected 1 crossing, got %d", len(crossings))
	}
	c := crossings[0]
	if c.SplineID != 1 || c.PathIndex != 0 {
		t.Fatalf("unexpected crossing ids: %+v", c)
	}
	if absf(c.DistOnPath-10) > 0.1 {
		t.Fatalf("expected crossing at path distance ~10, got %.3f", c.DistOnPath)
	}
	if absf(c.DistOnSpline-10) > 0.5 {
		t.Fatalf("expected crossing at spline distance ~10, got %.3f", c.DistOnSpline)
	}
}

func TestApproachingPedestrianBlocksCarsAtCrossing(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})

	approaching := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 8.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	blocked := buildPedestrianBlockedSplineDists(crossings, approaching)
	if len(blocked[1]) == 0 {
		t.Fatal("expected spline 1 to be blocked by pedestrian near the crossing")
	}

	far := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 1.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	if got := buildPedestrianBlockedSplineDists(crossings, far); len(got[1]) != 0 {
		t.Fatalf("expected no block when pedestrian is well before the crossing, got %v", got)
	}
}

func TestCarWithRoomBrakesForBlockedCrossing(t *testing.T) {
	blocked := map[int][]float32{1: {25}}
	spline := NewSpline(1,
		NewVec2(0, 0),
		NewVec2(10, 0),
		NewVec2(20, 0),
		NewVec2(30, 0),
	)
	graph := NewRoadGraph([]Spline{spline}, nil)
	car := Car{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 0, Accel: 3.0, Speed: 5.0, MaxSpeed: 20.0}
	storedSpline, _ := graph.splinePtrByID(1)
	cap := computePedestrianCrossingSpeedCap(car, storedSpline, graph, blocked)
	if cap >= float32(math.MaxFloat32) {
		t.Fatal("expected a speed cap when approaching a blocked crossing with plenty of room")
	}

	close := Car{ID: 2, CurrentSplineID: 1, DistanceOnSpline: 24, Accel: 3.0, Speed: 12.0, MaxSpeed: 20.0}
	capClose := computePedestrianCrossingSpeedCap(close, storedSpline, graph, blocked)
	if capClose != 0 {
		t.Fatalf("expected a zero cap (cannot stop in time) when within the stop buffer, got %.3f", capClose)
	}
}

func TestPedestrianWaitsForCarOccupyingCrossing(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	cars := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 10, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 7.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 30; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, cars)
		if len(pedestrians) != 1 {
			t.Fatalf("expected pedestrian to remain active, got %d", len(pedestrians))
		}
	}
	stopAt := float32(10.0) - pedestrianCrossingStopBufferM
	if pedestrians[0].Distance > stopAt+0.1 {
		t.Fatalf("expected pedestrian to stop before crossing (<= %.3f), got %.3f", stopAt, pedestrians[0].Distance)
	}
	if pedestrians[0].Distance < stopAt-0.8 {
		t.Fatalf("expected pedestrian to reach the stop line (~%.3f), got %.3f", stopAt, pedestrians[0].Distance)
	}
	if pedestrians[0].Speed > 0.05 {
		t.Fatalf("expected pedestrian speed to settle near zero while waiting, got %.3f", pedestrians[0].Speed)
	}
}

func TestPedestrianResumesAfterCarClears(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	cars := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 10, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 7.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 15; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, cars)
	}
	stoppedAt := pedestrians[0].Distance
	// Car drives away; no longer occupies the crossing.
	carsClear := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 30, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	for step := 0; step < 10; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, carsClear)
	}
	if pedestrians[0].Distance <= stoppedAt+0.5 {
		t.Fatalf("expected pedestrian to resume moving after car clears, still at %.3f (was %.3f)", pedestrians[0].Distance, stoppedAt)
	}
}
