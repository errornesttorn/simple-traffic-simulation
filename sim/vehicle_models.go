package sim

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"math/rand"
	"sync"
)

// VehicleTrailerModel holds the physical dimensions of an attached trailer.
// Present for models that tow (pickup+trailer, articulated bus, etc.).
type VehicleTrailerModel struct {
	LengthM float32 `json:"length_m"`
	WidthM  float32 `json:"width_m"`
}

// VehicleModel describes a single car or bus "make"/variant. Spawned vehicles
// inherit every physical parameter from the model so future rendering code can
// pick a texture/mesh by ID without inferring from loose numeric fields.
//
// FrontPivotFrac and RearPivotFrac are the longitudinal positions of the
// spline-following point and the dragged rear point as fractions of the
// vehicle length, measured from the front bumper. Their difference is the
// effective wheelbase as a fraction of length. Smaller fractions mean shorter
// overhangs (more cab-forward designs like vans/buses); larger fractions mean
// longer overhangs (typical hatchbacks).
type VehicleModel struct {
	ID                   string               `json:"id"`
	DisplayName          string               `json:"display_name,omitempty"`
	LengthM              float32              `json:"length_m"`
	WidthM               float32              `json:"width_m"`
	MaxSpeedMPS          float32              `json:"max_speed_mps"`
	Accel                float32              `json:"accel"`
	CurveSpeedMultiplier float32              `json:"curve_speed_multiplier"`
	FrontPivotFrac       float32              `json:"front_pivot_frac"`
	RearPivotFrac        float32              `json:"rear_pivot_frac"`
	Trailer              *VehicleTrailerModel `json:"trailer,omitempty"`
}

type vehicleModelFile struct {
	Models []VehicleModel `json:"models"`
}

//go:embed assets/cars.json
var embeddedCarModelsJSON []byte

//go:embed assets/buses.json
var embeddedBusModelsJSON []byte

var (
	vehicleModelsOnce sync.Once
	carModels         []VehicleModel
	busModels         []VehicleModel
	carModelByID      map[string]VehicleModel
	busModelByID      map[string]VehicleModel
	vehicleModelsErr  error
)

func parseVehicleModels(data []byte, label string) ([]VehicleModel, error) {
	if len(data) == 0 {
		return nil, fmt.Errorf("%s: empty data", label)
	}
	var file vehicleModelFile
	if err := json.Unmarshal(data, &file); err != nil {
		return nil, fmt.Errorf("%s: %w", label, err)
	}
	if len(file.Models) == 0 {
		return nil, fmt.Errorf("%s: no models defined", label)
	}
	for i, m := range file.Models {
		if m.ID == "" {
			return nil, fmt.Errorf("%s: model %d missing id", label, i)
		}
		if m.LengthM <= 0 || m.WidthM <= 0 {
			return nil, fmt.Errorf("%s: model %q has non-positive dimensions", label, m.ID)
		}
		if m.MaxSpeedMPS <= 0 || m.Accel <= 0 {
			return nil, fmt.Errorf("%s: model %q has non-positive dynamics", label, m.ID)
		}
		if m.CurveSpeedMultiplier <= 0 {
			file.Models[i].CurveSpeedMultiplier = 1.0
		}
		if m.FrontPivotFrac <= 0 || m.FrontPivotFrac >= 1 ||
			m.RearPivotFrac <= 0 || m.RearPivotFrac >= 1 ||
			m.RearPivotFrac <= m.FrontPivotFrac {
			return nil, fmt.Errorf("%s: model %q has invalid pivot fractions front=%.3f rear=%.3f (need 0 < front < rear < 1)",
				label, m.ID, m.FrontPivotFrac, m.RearPivotFrac)
		}
	}
	return file.Models, nil
}

func loadVehicleModels() {
	vehicleModelsOnce.Do(func() {
		cars, err := parseVehicleModels(embeddedCarModelsJSON, "cars.json")
		if err != nil {
			vehicleModelsErr = err
			return
		}
		buses, err := parseVehicleModels(embeddedBusModelsJSON, "buses.json")
		if err != nil {
			vehicleModelsErr = err
			return
		}
		carModels = cars
		busModels = buses
		carModelByID = make(map[string]VehicleModel, len(cars))
		for _, m := range cars {
			carModelByID[m.ID] = m
		}
		busModelByID = make(map[string]VehicleModel, len(buses))
		for _, m := range buses {
			busModelByID[m.ID] = m
		}
	})
}

// CarModels returns the loaded list of car variants. The slice is shared; do
// not mutate.
func CarModels() []VehicleModel {
	loadVehicleModels()
	return carModels
}

// BusModels returns the loaded list of bus variants.
func BusModels() []VehicleModel {
	loadVehicleModels()
	return busModels
}

// VehicleModelsError returns the fatal error encountered while parsing the
// bundled vehicle model JSON, if any. A non-nil error here means the models
// couldn't be read — spawning will fall back to a synthetic default but the
// caller should surface this so the bundled assets can be fixed.
func VehicleModelsError() error {
	loadVehicleModels()
	return vehicleModelsErr
}

// LookupVehicleModel returns the model with the given ID across both car and
// bus pools, preferring cars because they are the common case.
func LookupVehicleModel(id string) (VehicleModel, bool) {
	loadVehicleModels()
	if m, ok := carModelByID[id]; ok {
		return m, true
	}
	if m, ok := busModelByID[id]; ok {
		return m, true
	}
	return VehicleModel{}, false
}

// RandomVehicleModel returns a random model for the given vehicle kind.
// Returns (model, true) on success. When the embedded assets failed to load
// or are empty it returns (zero, false) and callers should use a fallback.
func RandomVehicleModel(kind VehicleKind) (VehicleModel, bool) {
	loadVehicleModels()
	var pool []VehicleModel
	switch kind {
	case VehicleBus:
		pool = busModels
	default:
		pool = carModels
	}
	if len(pool) == 0 {
		return VehicleModel{}, false
	}
	return pool[rand.Intn(len(pool))], true
}

// fallbackVehicleModel is used when model JSON can't be loaded for some
// reason. It keeps behaviour close to the historic hard-coded randomisation
// so the simulation still runs.
func fallbackVehicleModel(kind VehicleKind) VehicleModel {
	if kind == VehicleBus {
		return VehicleModel{
			ID:                   "fallback_bus",
			DisplayName:          "Fallback Bus",
			LengthM:              11.5,
			WidthM:               2.55,
			MaxSpeedMPS:          17.0,
			Accel:                1.5,
			CurveSpeedMultiplier: 0.95,
			FrontPivotFrac:       defaultFrontPivotFrac,
			RearPivotFrac:        defaultRearPivotFrac,
		}
	}
	return VehicleModel{
		ID:                   "fallback_car",
		DisplayName:          "Fallback Car",
		LengthM:              4.4,
		WidthM:               1.85,
		MaxSpeedMPS:          30.0,
		Accel:                3.5,
		CurveSpeedMultiplier: 1.0,
		FrontPivotFrac:       defaultFrontPivotFrac,
		RearPivotFrac:        defaultRearPivotFrac,
	}
}
