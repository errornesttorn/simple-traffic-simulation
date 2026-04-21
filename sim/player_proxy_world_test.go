package sim

import "testing"

func testCarOnSpline(spline *Spline, id, routeID int, controlMode CarControlMode, distance, speed float32) Car {
	frontPos, tangent := sampleSplineAtDistance(spline, distance)
	return Car{
		ID:                   id,
		RouteID:              routeID,
		ControlMode:          controlMode,
		CurrentSplineID:      spline.ID,
		DestinationSplineID:  spline.ID,
		PrevSplineIDs:        [2]int{-1, -1},
		DistanceOnSpline:     distance,
		RearPosition:         vecSub(frontPos, vecScale(tangent, 4.6*wheelbaseFrac)),
		Speed:                speed,
		MaxSpeed:             maxf(speed, 12),
		Accel:                4,
		Length:               4.6,
		Width:                1.9,
		CurveSpeedMultiplier: 1,
		LaneChangeSplineID:   -1,
		AfterSplineID:        -1,
		DesiredLaneSplineID:  -1,
		VehicleKind:          VehicleCar,
	}
}

func TestWorldUpdateAndClearPlayerProxy(t *testing.T) {
	world := NewWorld()
	world.Splines = []Spline{
		straightSpline(1, 0, 0, 40, 0),
	}

	ok := world.UpdatePlayerProxy(PlayerProxyFitInput{
		Position:            NewVec2(10, 0.5),
		Heading:             NewVec2(1, 0),
		Speed:               6,
		Length:              4.6,
		Width:               1.9,
		CarID:               42,
		DestinationSplineID: 1,
	})
	if !ok {
		t.Fatal("expected world player proxy update to succeed")
	}
	if !world.HasPlayerProxy {
		t.Fatal("expected world to own a player proxy")
	}
	if world.PlayerProxyCar.ID != 42 {
		t.Fatalf("expected proxy car id 42, got %d", world.PlayerProxyCar.ID)
	}
	if world.PlayerProxyCar.DestinationSplineID != 1 {
		t.Fatalf("expected destination spline 1, got %d", world.PlayerProxyCar.DestinationSplineID)
	}
	if world.PlayerProxyCar.ControlMode != CarControlExternal {
		t.Fatalf("expected external control mode, got %v", world.PlayerProxyCar.ControlMode)
	}
	if len(world.Cars) != 1 {
		t.Fatalf("expected proxy to be inserted into world cars, got %d cars", len(world.Cars))
	}
	if world.Cars[0].ID != 42 {
		t.Fatalf("expected proxy car to be stored in world cars, got id %d", world.Cars[0].ID)
	}

	clone := world.Clone()
	if !clone.HasPlayerProxy {
		t.Fatal("expected cloned world to preserve player proxy")
	}
	if clone.PlayerProxyCar.ID != world.PlayerProxyCar.ID {
		t.Fatal("expected cloned proxy car to match original")
	}

	world.ClearPlayerProxy()
	if world.HasPlayerProxy {
		t.Fatal("expected proxy to be cleared")
	}
	if len(world.Cars) != 0 {
		t.Fatalf("expected clearing proxy to remove it from world cars, got %d cars", len(world.Cars))
	}
}

func TestWorldStepLeavesExternalCarsUntouched(t *testing.T) {
	world := NewWorld()
	world.Splines = []Spline{
		straightSpline(1, 0, 0, 80, 0),
	}
	spline := &world.Splines[0]
	frontPos, tangent := sampleSplineAtDistance(spline, 12)
	external := Car{
		ID:                  77,
		RouteID:             -1,
		ControlMode:         CarControlExternal,
		CurrentSplineID:     1,
		DestinationSplineID: 1,
		DistanceOnSpline:    12,
		RearPosition:        vecSub(frontPos, vecScale(tangent, 4.6*wheelbaseFrac)),
		Speed:               8,
		MaxSpeed:            8,
		Length:              4.6,
		Width:               1.9,
		LaneChangeSplineID:  -1,
		AfterSplineID:       -1,
		DesiredLaneSplineID: -1,
		VehicleKind:         VehicleCar,
	}
	world.Cars = []Car{external}

	world.Step(0.25)

	if len(world.Cars) != 1 {
		t.Fatalf("expected one external car after step, got %d", len(world.Cars))
	}
	got := world.Cars[0]
	if got.ControlMode != CarControlExternal {
		t.Fatalf("expected external control mode after step, got %v", got.ControlMode)
	}
	if got.DistanceOnSpline != external.DistanceOnSpline {
		t.Fatalf("expected external car distance %.2f to remain unchanged, got %.2f", external.DistanceOnSpline, got.DistanceOnSpline)
	}
	if got.CurrentSplineID != external.CurrentSplineID {
		t.Fatalf("expected external car to stay on spline %d, got %d", external.CurrentSplineID, got.CurrentSplineID)
	}
}

func TestFollowingSpeedCapsIncludeExternalCars(t *testing.T) {
	splines := []Spline{
		straightSpline(1, 0, 0, 80, 0),
	}
	cars := []Car{
		testCarOnSpline(&splines[0], 1, 1, CarControlAI, 10, 14),
		testCarOnSpline(&splines[0], 77, -1, CarControlExternal, 18, 0),
	}

	graph := NewRoadGraph(splines, BuildVehicleCounts(cars))
	caps, _ := computeFollowingSpeedCaps(cars, graph)
	if len(caps) != len(cars) {
		t.Fatalf("expected %d follow caps, got %d", len(cars), len(caps))
	}
	if caps[0] > 0.01 {
		t.Fatalf("expected AI car to cap to the external car's speed, got %.3f", caps[0])
	}
}

func TestWorldStepSlowsAICarForExternalProxy(t *testing.T) {
	world := NewWorld()
	world.Splines = []Spline{
		straightSpline(1, 0, 0, 80, 0),
	}
	world.Routes = []Route{
		{
			ID:            1,
			StartSplineID: 1,
			EndSplineID:   1,
			PathIDs:       []int{1},
			Valid:         true,
			Color:         NewColor(120, 120, 200, 255),
			VehicleKind:   VehicleCar,
		},
	}
	world.NextCarID = 2

	ai := testCarOnSpline(&world.Splines[0], 1, 1, CarControlAI, 10, 14)
	external := testCarOnSpline(&world.Splines[0], 77, -1, CarControlExternal, 18, 0)
	world.Cars = []Car{ai, external}

	world.Step(0.1)

	aiIdx := FindCarIndexByID(world.Cars, ai.ID)
	if aiIdx < 0 {
		t.Fatal("expected AI car to remain in the world after stepping")
	}
	gotAI := world.Cars[aiIdx]
	if gotAI.Speed >= ai.Speed {
		t.Fatalf("expected AI car to slow down for the external proxy, speed stayed at %.3f", gotAI.Speed)
	}

	externalIdx := FindCarIndexByID(world.Cars, external.ID)
	if externalIdx < 0 {
		t.Fatal("expected external proxy to remain in the world after stepping")
	}
	gotExternal := world.Cars[externalIdx]
	if gotExternal.DistanceOnSpline != external.DistanceOnSpline {
		t.Fatalf("expected external proxy distance %.2f to remain unchanged, got %.2f", external.DistanceOnSpline, gotExternal.DistanceOnSpline)
	}
}

func TestSpawnBlockedByExternalCar(t *testing.T) {
	splines := []Spline{
		straightSpline(1, 0, 0, 80, 0),
	}
	route := Route{
		ID:            1,
		StartSplineID: 1,
		EndSplineID:   1,
		PathIDs:       []int{1},
		Valid:         true,
		Color:         NewColor(120, 120, 200, 255),
		VehicleKind:   VehicleCar,
	}
	blocker := testCarOnSpline(&splines[0], 77, -1, CarControlExternal, 1.5, 0)
	candidate := spawnVehicle(100, route, splines)

	if !spawnBlocked(candidate, []Car{blocker}, splines) {
		t.Fatal("expected spawn to be blocked by the external proxy car")
	}
}
