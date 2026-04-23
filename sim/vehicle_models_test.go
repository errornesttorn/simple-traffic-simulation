package sim

import (
	"math/rand"
	"testing"
)

func TestBundledVehicleModelsParse(t *testing.T) {
	if err := VehicleModelsError(); err != nil {
		t.Fatalf("bundled vehicle models failed to load: %v", err)
	}
	cars := CarModels()
	if len(cars) < 5 {
		t.Fatalf("expected at least 5 car models, got %d", len(cars))
	}
	buses := BusModels()
	if len(buses) < 2 {
		t.Fatalf("expected at least 2 bus models, got %d", len(buses))
	}
	// Articulated bus should have a trailer defined.
	var artic VehicleModel
	for _, b := range buses {
		if b.Trailer != nil {
			artic = b
			break
		}
	}
	if artic.ID == "" {
		t.Fatal("expected at least one articulated bus model with a trailer")
	}
	if artic.Trailer.LengthM <= 0 || artic.Trailer.WidthM <= 0 {
		t.Fatalf("articulated bus trailer dimensions invalid: %+v", artic.Trailer)
	}
	// At least one car model with a trailer as well.
	haveTrailerCar := false
	for _, c := range cars {
		if c.Trailer != nil {
			haveTrailerCar = true
			break
		}
	}
	if !haveTrailerCar {
		t.Fatal("expected at least one car model that tows a trailer")
	}
}

func TestLookupVehicleModelFindsKnownIDs(t *testing.T) {
	if _, ok := LookupVehicleModel("bus_standard"); !ok {
		t.Fatal("expected to find bus_standard model")
	}
	if _, ok := LookupVehicleModel("hatchback_a"); !ok {
		t.Fatal("expected to find hatchback_a model")
	}
	if _, ok := LookupVehicleModel("does_not_exist_xyz"); ok {
		t.Fatal("lookup of missing model should return false")
	}
}

func TestSpawnVehicleInheritsFromModel(t *testing.T) {
	rand.Seed(42)
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0))
	route := Route{
		ID:            1,
		StartSplineID: 1,
		VehicleKind:   VehicleCar,
		Color:         NewColor(100, 120, 140, 255),
	}
	car := spawnVehicle(1, route, []Spline{spline})
	if car.ModelID == "" {
		t.Fatal("spawnVehicle should set ModelID from the chosen model")
	}
	model, ok := LookupVehicleModel(car.ModelID)
	if !ok {
		t.Fatalf("spawned car has unknown ModelID %q", car.ModelID)
	}
	if absf(car.Length-model.LengthM/metersPerUnit) > 1e-3 {
		t.Fatalf("car length %.3f does not match model %.3f", car.Length, model.LengthM)
	}
	if absf(car.Width-model.WidthM/metersPerUnit) > 1e-3 {
		t.Fatalf("car width %.3f does not match model %.3f", car.Width, model.WidthM)
	}
	if absf(car.MaxSpeed-model.MaxSpeedMPS) > 1e-3 {
		t.Fatalf("car max speed %.3f does not match model %.3f", car.MaxSpeed, model.MaxSpeedMPS)
	}
	if absf(car.Accel-model.Accel) > 1e-3 {
		t.Fatalf("car accel %.3f does not match model %.3f", car.Accel, model.Accel)
	}
	if absf(car.CurveSpeedMultiplier-model.CurveSpeedMultiplier) > 1e-3 {
		t.Fatalf("curve multiplier %.3f does not match model %.3f", car.CurveSpeedMultiplier, model.CurveSpeedMultiplier)
	}
	if absf(car.FrontPivotFrac-model.FrontPivotFrac) > 1e-6 {
		t.Fatalf("front pivot frac %.4f does not match model %.4f", car.FrontPivotFrac, model.FrontPivotFrac)
	}
	if absf(car.RearPivotFrac-model.RearPivotFrac) > 1e-6 {
		t.Fatalf("rear pivot frac %.4f does not match model %.4f", car.RearPivotFrac, model.RearPivotFrac)
	}
	if absf(car.WheelbaseFrac()-(model.RearPivotFrac-model.FrontPivotFrac)) > 1e-6 {
		t.Fatalf("WheelbaseFrac %.4f does not match model %.4f", car.WheelbaseFrac(), model.RearPivotFrac-model.FrontPivotFrac)
	}
	if model.Trailer != nil && !car.Trailer.HasTrailer {
		t.Fatal("expected spawned car to carry a trailer based on model definition")
	}
	if model.Trailer == nil && car.Trailer.HasTrailer {
		t.Fatal("spawned car has a trailer but its model has none")
	}
}

func TestVehicleModelsHaveVariedPivots(t *testing.T) {
	models := append([]VehicleModel{}, CarModels()...)
	models = append(models, BusModels()...)
	if len(models) == 0 {
		t.Fatal("no vehicle models to inspect")
	}
	seenFront := map[float32]bool{}
	for _, m := range models {
		if m.FrontPivotFrac <= 0 || m.RearPivotFrac <= m.FrontPivotFrac || m.RearPivotFrac >= 1 {
			t.Fatalf("model %q has invalid pivot fractions front=%.3f rear=%.3f", m.ID, m.FrontPivotFrac, m.RearPivotFrac)
		}
		seenFront[m.FrontPivotFrac] = true
	}
	if len(seenFront) < 2 {
		t.Fatalf("expected at least two distinct front_pivot_frac values across models, saw %d", len(seenFront))
	}
}

func TestSpawnBusHonoursRouteModelID(t *testing.T) {
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0))
	route := Route{
		ID:            1,
		StartSplineID: 1,
		VehicleKind:   VehicleBus,
		Color:         NewColor(200, 200, 50, 255),
		BusModelID:    "bus_articulated",
	}
	for i := 0; i < 20; i++ {
		car := spawnVehicle(i+1, route, []Spline{spline})
		if car.ModelID != "bus_articulated" {
			t.Fatalf("pinned bus route should always spawn bus_articulated, got %q", car.ModelID)
		}
		if !car.Trailer.HasTrailer {
			t.Fatal("bus_articulated defines a trailer — spawned car should have one")
		}
	}
}

func TestSpawnCarRouteIgnoresBusModelID(t *testing.T) {
	// A malformed/legacy car route shouldn't be pinned to a bus model ID.
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0))
	route := Route{
		ID:            1,
		StartSplineID: 1,
		VehicleKind:   VehicleCar,
		Color:         NewColor(100, 120, 140, 255),
		BusModelID:    "bus_articulated", // should be ignored for car routes
	}
	for i := 0; i < 20; i++ {
		car := spawnVehicle(i+1, route, []Spline{spline})
		if _, ok := carModelByID[car.ModelID]; !ok {
			t.Fatalf("car route should spawn car models, got %q", car.ModelID)
		}
	}
}

func TestSpawnBusFallsBackWhenModelIDUnknown(t *testing.T) {
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0))
	route := Route{
		ID:            1,
		StartSplineID: 1,
		VehicleKind:   VehicleBus,
		BusModelID:    "totally_made_up_model",
	}
	car := spawnVehicle(1, route, []Spline{spline})
	if _, ok := busModelByID[car.ModelID]; !ok {
		t.Fatalf("unknown bus model ID should fall back to a random bus, got %q", car.ModelID)
	}
}

func TestSpawnBusPicksFromBusPool(t *testing.T) {
	rand.Seed(7)
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0))
	route := Route{
		ID:            1,
		StartSplineID: 1,
		VehicleKind:   VehicleBus,
		Color:         NewColor(200, 200, 50, 255),
	}
	seen := map[string]bool{}
	for i := 0; i < 30; i++ {
		car := spawnVehicle(i+1, route, []Spline{spline})
		if car.ModelID == "" {
			t.Fatal("bus spawn missing ModelID")
		}
		if _, ok := busModelByID[car.ModelID]; !ok {
			t.Fatalf("bus spawned with non-bus model %q", car.ModelID)
		}
		seen[car.ModelID] = true
	}
	if len(seen) < 1 {
		t.Fatal("expected at least one bus model to spawn across 30 attempts")
	}
}
