package sim

import (
	"container/heap"
	"encoding/json"
	"fmt"
	"image/color"
	"math"
	"math/rand"
	"os"
	"runtime"
	"sort"
	"strings"
	"sync"
	"time"
)

type VehicleKind int

const (
	VehicleCar VehicleKind = iota
	VehicleBus
)

const (
	simSamples = 96

	spawnSliderMaxPerMinute float32 = 60.0

	metersPerUnit float32 = 1.0

	predictionHorizonSeconds float32 = 3.0
	predictionStepSeconds    float32 = 0.15
	blameAngleThresholdDeg   float32 = 45.0
	brakeDecelMultiplier     float32 = 2.5
)

const (
	busStopMinDwellSeconds float32 = 10.0
	busStopMaxDwellSeconds float32 = 20.0
	stopFrontGapM          float32 = 5.0
)

const (
	accelEscapeLookaheadSecs  float32 = 1.0
	accelEscapeTestShortSecs  float32 = 0.25
	accelEscapeTestMediumSecs float32 = 0.5
)

const (
	followTimeHeadwaySecs float32 = 1.5
	followMinGapM         float32 = 2.0
	followLookaheadM      float32 = 60.0
	followHeadingCos      float32 = 0.766
)

const (
	laneChangeHalfSecs        float32 = 1.0
	laneChangeMinSpeed        float32 = 3.0
	laneChangeDirCos          float32 = 0.71
	preferenceChangeCooldownS float32 = 7.5
	overtakeSlowThresholdS    float32 = 2.0
	overtakeCooldownS         float32 = 3.5
	maxCarSpeed               float32 = 36.1
	laneChangeForcedSpeedMPS  float32 = 20.0 / 3.6
	laneChangeForcedDistEnd   float32 = 15.0
	curveSpeedIntervalM       float32 = 10.0
	maxLateralAccelMPS2       float32 = 5.0
	collisionBroadPhaseSlackM float32 = 5.0
	frontPivotFrac            float32 = 0.20
	rearPivotFrac             float32 = 0.80
	wheelbaseFrac             float32 = rearPivotFrac - frontPivotFrac
)

type Vec2 struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
}

func NewVec2(x, y float32) Vec2 {
	return Vec2{X: x, Y: y}
}

type Color = color.RGBA

func NewColor(r, g, b, a uint8) Color {
	return color.RGBA{R: r, G: g, B: b, A: a}
}

type Spline struct {
	ID       int
	Priority bool
	BusOnly  bool

	P0 Vec2
	P1 Vec2
	P2 Vec2
	P3 Vec2

	Length         float32
	SpeedFactor    float32
	Samples        [simSamples + 1]Vec2
	CumLen         [simSamples + 1]float32
	HardCoupledIDs []int
	SoftCoupledIDs []int
	SpeedLimitKmh  float32
	LanePreference int
	CurveSpeedMPS  []float32
	TravelTimeS    float32
}

type BusStop struct {
	SplineID int
	WorldPos Vec2
}

type Route struct {
	ID int

	StartSplineID int
	EndSplineID   int

	PathIDs        []int
	PathLength     float32
	SpawnPerMinute float32
	NextSpawnIn    float32
	Valid          bool
	ColorIndex     int
	Color          Color
	VehicleKind    VehicleKind
	BusStops       []BusStop
}

type Trailer struct {
	HasTrailer   bool
	Length       float32
	Width        float32
	Color        Color
	RearPosition Vec2
}

type Car struct {
	RouteID int

	CurrentSplineID      int
	DestinationSplineID  int
	PrevSplineIDs        [2]int
	DistanceOnSpline     float32
	RearPosition         Vec2
	LateralOffset        float32
	Speed                float32
	MaxSpeed             float32
	Accel                float32
	Length               float32
	Width                float32
	CurveSpeedMultiplier float32
	Color                Color
	Braking              bool
	SoftSlowing          bool

	LaneChanging        bool
	LaneChangeSplineID  int
	AfterSplineID       int
	AfterSplineDist     float32
	DesiredLaneSplineID int
	DesiredLaneDeadline float32

	PreferenceCooldown float32
	SlowedTimer        float32
	OvertakeCooldown   float32
	VehicleKind        VehicleKind
	NextBusStopIndex   int
	BusStopTimer       float32
	BusStopDuration    float32

	Trailer Trailer
}

type TrajectorySample struct {
	Time            float32
	Position        Vec2
	Heading         Vec2
	Priority        bool
	SplineID        int
	HasTrailer      bool
	TrailerPosition Vec2
	TrailerHeading  Vec2
}

type CollisionPrediction struct {
	Time            float32
	PosA            Vec2
	PosB            Vec2
	PrevPosA        Vec2
	PrevPosB        Vec2
	HeadingA        Vec2
	HeadingB        Vec2
	AlreadyCollided bool
	PriorityA       bool
	PriorityB       bool
	SplineAID       int
	SplineBID       int
}

type collisionGeometry struct {
	bodyRadius     float32
	bodyOffsets    []float32
	trailerRadius  float32
	trailerOffsets []float32
	coarseRadius   float32
}

type BrakingProfile struct {
	Cars int

	BasePredictMS  float64
	ConflictScanMS float64
	BrakeProbeMS   float64
	HoldProbeMS    float64
	FinalizeMS     float64

	BasePredictions        int
	StationaryPredictions  int
	EscapePredictions      int
	FasterPredictions      int
	TotalPredictions       int
	TotalPredictionSamples int

	PrimaryPairCandidates  int
	PrimaryBroadPhasePairs int
	PrimaryCollisionChecks int
	PrimaryCollisionHits   int

	StationaryCollisionChecks int
	StationaryCollisionHits   int
	EscapeCollisionChecks     int
	EscapeCollisionHits       int
	HoldCollisionChecks       int
	HoldCollisionHits         int

	InitiallyBlamedCars int
	BrakingCars         int
	HoldCars            int
}

type holdProbeCarResult struct {
	shouldHold                bool
	holdLink                  DebugBlameLink
	hasHoldLink               bool
	candidateLinks            []DebugBlameLink
	fasterPredictions         int
	stationaryPredictions     int
	totalPredictionSamples    int
	holdCollisionChecks       int
	holdCollisionHits         int
	stationaryCollisionChecks int
	stationaryCollisionHits   int
}

type basePredictionCarResult struct {
	prediction   []TrajectorySample
	pose         carPose
	reach        float32
	geometry     collisionGeometry
	totalSamples int
}

type brakeProbeCarResult struct {
	shouldBrake            bool
	escapePredictions      int
	totalPredictionSamples int
	escapeCollisionChecks  int
	escapeCollisionHits    int
}

type pathCacheKey struct {
	StartID     int
	EndID       int
	VehicleKind VehicleKind
}

type pathCacheEntry struct {
	PathIDs []int
	Cost    float32
	OK      bool
}

type dijkstraItem struct {
	idx  int
	dist float32
}

type dijkstraHeap []dijkstraItem

func (h dijkstraHeap) Len() int            { return len(h) }
func (h dijkstraHeap) Less(i, j int) bool  { return h[i].dist < h[j].dist }
func (h dijkstraHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *dijkstraHeap) Push(x interface{}) { *h = append(*h, x.(dijkstraItem)) }
func (h *dijkstraHeap) Pop() interface{} {
	old := *h
	n := len(old)
	item := old[n-1]
	*h = old[:n-1]
	return item
}

type RoadGraph struct {
	splines         []Spline
	indexByID       map[int]int
	startsByNode    map[string][]int
	routeNeighbors  [][]int
	segmentCosts    []float32
	pathCacheMu     sync.RWMutex
	pathCache       map[pathCacheKey]pathCacheEntry
	pathCacheHits   int
	pathCacheMisses int
}

func NewRoadGraph(splines []Spline, vehicleCounts map[int]int) *RoadGraph {
	indexByID := buildSplineIndexByID(splines)
	startsByNode := buildStartsByNode(splines)
	routeNeighbors := make([][]int, len(splines))
	segmentCosts := make([]float32, len(splines))
	for i, spline := range splines {
		segmentCosts[i] = segmentTravelCost(spline, vehicleCounts)
		next := startsByNode[pointKey(spline.P3)]
		if len(next) > 0 {
			routeNeighbors[i] = append([]int(nil), expandWithCoupledNeighbors(next, splines, indexByID)...)
		}
	}
	return &RoadGraph{
		splines:        splines,
		indexByID:      indexByID,
		startsByNode:   startsByNode,
		routeNeighbors: routeNeighbors,
		segmentCosts:   segmentCosts,
		pathCache:      make(map[pathCacheKey]pathCacheEntry),
	}
}

func (g *RoadGraph) splineByID(id int) (Spline, bool) {
	idx, ok := g.indexByID[id]
	if !ok {
		return Spline{}, false
	}
	return g.splines[idx], true
}

func (g *RoadGraph) SplineByID(id int) (Spline, bool) {
	return g.splineByID(id)
}

func (g *RoadGraph) nextPhysicalIndices(currentSplineID int) []int {
	currentSpline, ok := g.splineByID(currentSplineID)
	if !ok {
		return nil
	}
	return g.startsByNode[pointKey(currentSpline.P3)]
}

type DebugBlameLink struct {
	FromCarIndex int
	ToCarIndex   int
}

type TrafficState int

const (
	TrafficRed TrafficState = iota
	TrafficYellow
	TrafficGreen
)

type TrafficLight struct {
	ID           int
	SplineID     int
	DistOnSpline float32
	WorldPos     Vec2
	CycleID      int
}

type TrafficPhase struct {
	DurationSecs          float32
	ClearanceDurationSecs float32
	GreenLightIDs         []int
}

type TrafficCycle struct {
	ID         int
	LightIDs   []int
	Phases     []TrafficPhase
	Timer      float32
	PhaseIndex int
	Enabled    bool
}

type World struct {
	Splines           []Spline
	LaneChangeSplines []Spline
	Routes            []Route
	Cars              []Car
	TrafficLights     []TrafficLight
	TrafficCycles     []TrafficCycle

	NextSplineID int
	NextRouteID  int
	NextLightID  int
	NextCycleID  int

	RouteVisualsTimer float32

	DebugBlameLinks      []DebugBlameLink
	HoldBlameLinks       []DebugBlameLink
	DebugSelectedCar     int
	DebugSelectedCarMode int // 0 = primary candidates, 1 = hold candidates
	DebugCandidateLinks  []DebugBlameLink
	BasePathHits         int
	BasePathMisses       int
	AllPathHits          int
	AllPathMisses        int
	BrakingProfile       BrakingProfile

	RouteVisualsMS float64
	LaneChangesMS  float64
	GraphBuildMS   float64
	BrakingMS      float64
	FollowMS       float64
	UpdateCarsMS   float64
}

func NewWorld() World {
	return World{
		Splines:           make([]Spline, 0, 128),
		LaneChangeSplines: make([]Spline, 0, 32),
		Routes:            make([]Route, 0, 32),
		Cars:              make([]Car, 0, 256),
		TrafficLights:     make([]TrafficLight, 0),
		TrafficCycles:     make([]TrafficCycle, 0),
		NextSplineID:      1,
		NextRouteID:       1,
		NextLightID:       1,
		NextCycleID:       1,
		DebugSelectedCar:  -1,
	}
}

func NewSpline(id int, p0, p1, p2, p3 Vec2) Spline {
	s := Spline{ID: id, P0: p0, P1: p1, P2: p2, P3: p3}
	cacheSpline(&s)
	return s
}

func (w *World) RebuildSpline(i int) {
	if i < 0 || i >= len(w.Splines) {
		return
	}
	cacheSpline(&w.Splines[i])
}

func RebuildSpline(s *Spline) {
	if s == nil {
		return
	}
	cacheSpline(s)
}

func (w *World) RebuildAllSplines() {
	for i := range w.Splines {
		cacheSpline(&w.Splines[i])
	}
}

func (w *World) RefreshRoutes() {
	w.Routes = UpdateRouteVisuals(w.Routes, w.Splines, map[int]int{})
}

func (w *World) ResetTransientState() {
	w.Cars = w.Cars[:0]
	w.LaneChangeSplines = w.LaneChangeSplines[:0]
	w.DebugBlameLinks = nil
	w.HoldBlameLinks = nil
}

func sinceMS(start time.Time) float64 {
	return float64(time.Since(start).Microseconds()) / 1000.0
}

func parallelWorkerCount(n int) int {
	if n <= 0 {
		return 0
	}
	workers := runtime.GOMAXPROCS(0)
	if workers < 1 {
		workers = 1
	}
	if workers > n {
		workers = n
	}
	return workers
}

func parallelFor(n int, fn func(start, end int)) {
	if n <= 0 {
		return
	}
	workers := parallelWorkerCount(n)
	if workers <= 1 {
		fn(0, n)
		return
	}
	chunkSize := (n + workers - 1) / workers
	var wg sync.WaitGroup
	for worker := 0; worker < workers; worker++ {
		start := worker * chunkSize
		if start >= n {
			break
		}
		end := start + chunkSize
		if end > n {
			end = n
		}
		wg.Add(1)
		go func(start, end int) {
			defer wg.Done()
			fn(start, end)
		}(start, end)
	}
	wg.Wait()
}

func (w *World) Step(dt float32) {
	w.RouteVisualsMS = 0
	w.LaneChangesMS = 0
	w.GraphBuildMS = 0
	w.BrakingMS = 0
	w.FollowMS = 0
	w.UpdateCarsMS = 0
	w.BrakingProfile = BrakingProfile{}

	graphStart := time.Now()
	vehicleCounts := BuildVehicleCounts(w.Cars)
	baseGraph := NewRoadGraph(w.Splines, vehicleCounts)
	w.GraphBuildMS += sinceMS(graphStart)

	w.RouteVisualsTimer -= dt
	if w.RouteVisualsTimer <= 0 {
		routeVisualsStart := time.Now()
		w.Routes = UpdateRouteVisualsWithGraph(w.Routes, baseGraph)
		w.RouteVisualsMS = sinceMS(routeVisualsStart)
		w.RouteVisualsTimer = 0.5
	}

	laneChangesStart := time.Now()
	w.LaneChangeSplines, w.Cars = computeLaneChanges(w.Cars, w.Splines, w.LaneChangeSplines, &w.NextSplineID, baseGraph, dt)
	w.LaneChangesMS = sinceMS(laneChangesStart)

	graphStart = time.Now()
	allSplines := mergedSplines(w.Splines, w.LaneChangeSplines)
	allGraph := NewRoadGraph(allSplines, vehicleCounts)
	w.GraphBuildMS += sinceMS(graphStart)

	brakingStart := time.Now()
	brakingDecisions, holdSpeedDecisions, debugBlameLinks, holdBlameLinks, candidateLinks, brakingProfile := computeBrakingDecisions(w.Cars, allGraph, w.DebugSelectedCar, w.DebugSelectedCarMode)
	w.BrakingMS = sinceMS(brakingStart)
	w.BrakingProfile = brakingProfile

	followStart := time.Now()
	followCaps := computeFollowingSpeedCaps(w.Cars, allGraph)
	w.FollowMS = sinceMS(followStart)

	updateCarsStart := time.Now()
	var indexRemap []int
	w.Cars, indexRemap = updateCars(w.Cars, w.Routes, allGraph, brakingDecisions, holdSpeedDecisions, followCaps, w.TrafficLights, w.TrafficCycles, dt)
	w.LaneChangeSplines = gcLaneChangeSplines(w.LaneChangeSplines, w.Cars)
	w.Routes, w.Cars = updateRouteSpawning(w.Routes, w.Cars, w.Splines, dt)
	w.TrafficCycles = UpdateTrafficCycles(w.TrafficCycles, dt)
	w.UpdateCarsMS = sinceMS(updateCarsStart)

	w.DebugBlameLinks = remapBlameLinks(debugBlameLinks, indexRemap)
	w.HoldBlameLinks = remapBlameLinks(holdBlameLinks, indexRemap)
	w.DebugCandidateLinks = remapBlameLinks(candidateLinks, indexRemap)
	if w.DebugSelectedCar >= 0 && w.DebugSelectedCar < len(indexRemap) {
		w.DebugSelectedCar = indexRemap[w.DebugSelectedCar]
	} else if w.DebugSelectedCar >= len(indexRemap) {
		w.DebugSelectedCar = -1
	}
	w.BasePathHits = baseGraph.pathCacheHits
	w.BasePathMisses = baseGraph.pathCacheMisses
	w.AllPathHits = allGraph.pathCacheHits
	w.AllPathMisses = allGraph.pathCacheMisses
}

type SavedSplineFile struct {
	Splines       []SavedSpline       `json:"splines"`
	Routes        []SavedRoute        `json:"routes,omitempty"`
	Cars          []SavedCar          `json:"cars,omitempty"`
	TrafficLights []SavedTrafficLight `json:"traffic_lights,omitempty"`
	TrafficCycles []SavedTrafficCycle `json:"traffic_cycles,omitempty"`
}

type SavedTrafficLight struct {
	ID           int     `json:"id"`
	SplineID     int     `json:"spline_id"`
	DistOnSpline float32 `json:"dist_on_spline"`
	WorldPosX    float32 `json:"world_pos_x"`
	WorldPosY    float32 `json:"world_pos_y"`
	CycleID      int     `json:"cycle_id"`
}

type SavedTrafficPhase struct {
	DurationSecs          float32 `json:"duration_secs"`
	ClearanceDurationSecs float32 `json:"clearance_duration_secs,omitempty"`
	GreenLightIDs         []int   `json:"green_light_ids,omitempty"`
}

type SavedTrafficCycle struct {
	ID                 int                 `json:"id"`
	Enabled            bool                `json:"enabled"`
	YellowDurationSecs float32             `json:"yellow_duration_secs,omitempty"`
	Phases             []SavedTrafficPhase `json:"phases,omitempty"`
}

type SavedSpline struct {
	ID             int     `json:"id"`
	Priority       bool    `json:"priority"`
	BusOnly        bool    `json:"bus_only,omitempty"`
	P0             Vec2    `json:"p0"`
	P1             Vec2    `json:"p1"`
	P2             Vec2    `json:"p2"`
	P3             Vec2    `json:"p3"`
	HardCoupledIDs []int   `json:"hard_coupled_ids,omitempty"`
	SoftCoupledIDs []int   `json:"soft_coupled_ids,omitempty"`
	SpeedLimitKmh  float32 `json:"speed_limit_kmh,omitempty"`
	LanePreference int     `json:"lane_preference,omitempty"`
}

type SavedBusStop struct {
	SplineID  int     `json:"spline_id"`
	WorldPosX float32 `json:"world_pos_x"`
	WorldPosY float32 `json:"world_pos_y"`
}

type SavedRoute struct {
	ID             int            `json:"id"`
	StartSplineID  int            `json:"start_spline_id"`
	EndSplineID    int            `json:"end_spline_id"`
	PathIDs        []int          `json:"path_ids"`
	SpawnPerMinute float32        `json:"spawn_per_minute"`
	ColorIndex     int            `json:"color_index,omitempty"`
	VehicleKind    string         `json:"vehicle_kind,omitempty"`
	BusStops       []SavedBusStop `json:"bus_stops,omitempty"`
}

type SavedCar struct {
	RouteID              int     `json:"route_id"`
	CurrentSplineID      int     `json:"current_spline_id"`
	DestinationSplineID  int     `json:"destination_spline_id"`
	DistanceOnSpline     float32 `json:"distance_on_spline"`
	Speed                float32 `json:"speed"`
	MaxSpeed             float32 `json:"max_speed"`
	Accel                float32 `json:"accel"`
	Length               float32 `json:"length"`
	Width                float32 `json:"width"`
	CurveSpeedMultiplier float32 `json:"curve_speed_multiplier,omitempty"`
	VehicleKind          string  `json:"vehicle_kind,omitempty"`
	NextBusStopIndex     int     `json:"next_bus_stop_index,omitempty"`
	BusStopTimer         float32 `json:"bus_stop_timer,omitempty"`
	BusStopDuration      float32 `json:"bus_stop_duration,omitempty"`
}

func (w *World) Save(path string) error {
	saved := SavedSplineFile{
		Splines:       make([]SavedSpline, 0, len(w.Splines)),
		Routes:        make([]SavedRoute, 0, len(w.Routes)),
		Cars:          make([]SavedCar, 0, len(w.Cars)),
		TrafficLights: make([]SavedTrafficLight, 0, len(w.TrafficLights)),
		TrafficCycles: make([]SavedTrafficCycle, 0, len(w.TrafficCycles)),
	}
	for _, spline := range w.Splines {
		saved.Splines = append(saved.Splines, SavedSpline{
			ID:             spline.ID,
			Priority:       spline.Priority,
			BusOnly:        spline.BusOnly,
			P0:             spline.P0,
			P1:             spline.P1,
			P2:             spline.P2,
			P3:             spline.P3,
			HardCoupledIDs: append([]int(nil), spline.HardCoupledIDs...),
			SoftCoupledIDs: append([]int(nil), spline.SoftCoupledIDs...),
			SpeedLimitKmh:  spline.SpeedLimitKmh,
			LanePreference: spline.LanePreference,
		})
	}
	for _, route := range w.Routes {
		savedStops := make([]SavedBusStop, 0, len(route.BusStops))
		for _, stop := range route.BusStops {
			savedStops = append(savedStops, SavedBusStop{
				SplineID:  stop.SplineID,
				WorldPosX: stop.WorldPos.X,
				WorldPosY: stop.WorldPos.Y,
			})
		}
		saved.Routes = append(saved.Routes, SavedRoute{
			ID:             route.ID,
			StartSplineID:  route.StartSplineID,
			EndSplineID:    route.EndSplineID,
			PathIDs:        append([]int(nil), route.PathIDs...),
			SpawnPerMinute: route.SpawnPerMinute,
			ColorIndex:     route.ColorIndex,
			VehicleKind:    VehicleKindString(route.VehicleKind),
			BusStops:       savedStops,
		})
	}
	for _, car := range w.Cars {
		saved.Cars = append(saved.Cars, SavedCar{
			RouteID:              car.RouteID,
			CurrentSplineID:      car.CurrentSplineID,
			DestinationSplineID:  car.DestinationSplineID,
			DistanceOnSpline:     car.DistanceOnSpline,
			Speed:                car.Speed,
			MaxSpeed:             car.MaxSpeed,
			Accel:                car.Accel,
			Length:               car.Length,
			Width:                car.Width,
			CurveSpeedMultiplier: car.CurveSpeedMultiplier,
			VehicleKind:          VehicleKindString(car.VehicleKind),
			NextBusStopIndex:     car.NextBusStopIndex,
			BusStopTimer:         car.BusStopTimer,
			BusStopDuration:      car.BusStopDuration,
		})
	}
	for _, l := range w.TrafficLights {
		saved.TrafficLights = append(saved.TrafficLights, SavedTrafficLight{
			ID:           l.ID,
			SplineID:     l.SplineID,
			DistOnSpline: l.DistOnSpline,
			WorldPosX:    l.WorldPos.X,
			WorldPosY:    l.WorldPos.Y,
			CycleID:      l.CycleID,
		})
	}
	for _, c := range w.TrafficCycles {
		phases := make([]SavedTrafficPhase, len(c.Phases))
		for i, p := range c.Phases {
			phases[i] = SavedTrafficPhase{
				DurationSecs:          p.DurationSecs,
				ClearanceDurationSecs: p.ClearanceDurationSecs,
				GreenLightIDs:         append([]int(nil), p.GreenLightIDs...),
			}
		}
		saved.TrafficCycles = append(saved.TrafficCycles, SavedTrafficCycle{
			ID:      c.ID,
			Enabled: c.Enabled,
			Phases:  phases,
		})
	}
	data, err := json.MarshalIndent(saved, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}

func LoadWorld(path string) (*World, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}

	var saved SavedSplineFile
	if err := json.Unmarshal(data, &saved); err != nil {
		return nil, err
	}

	world := NewWorld()
	maxSplineID, maxRouteID, maxLightID, maxCycleID := 0, 0, 0, 0

	world.Splines = make([]Spline, 0, len(saved.Splines))
	for _, entry := range saved.Splines {
		spline := NewSpline(entry.ID, entry.P0, entry.P1, entry.P2, entry.P3)
		spline.Priority = entry.Priority
		spline.BusOnly = entry.BusOnly
		spline.HardCoupledIDs = append([]int(nil), entry.HardCoupledIDs...)
		spline.SoftCoupledIDs = append([]int(nil), entry.SoftCoupledIDs...)
		spline.SpeedLimitKmh = entry.SpeedLimitKmh
		spline.LanePreference = entry.LanePreference
		spline.CurveSpeedMPS = buildCurveSpeedProfile(&spline)
		spline.TravelTimeS = buildSplineTravelTime(&spline)
		world.Splines = append(world.Splines, spline)
		if entry.ID > maxSplineID {
			maxSplineID = entry.ID
		}
	}

	world.Routes = make([]Route, 0, len(saved.Routes))
	for _, entry := range saved.Routes {
		stops := make([]BusStop, 0, len(entry.BusStops))
		for _, savedStop := range entry.BusStops {
			stops = append(stops, BusStop{
				SplineID: savedStop.SplineID,
				WorldPos: NewVec2(savedStop.WorldPosX, savedStop.WorldPosY),
			})
		}
		vehicleKind := VehicleKindFromString(entry.VehicleKind)
		route := Route{
			ID:             entry.ID,
			StartSplineID:  entry.StartSplineID,
			EndSplineID:    entry.EndSplineID,
			PathIDs:        append([]int(nil), entry.PathIDs...),
			SpawnPerMinute: entry.SpawnPerMinute,
			ColorIndex:     entry.ColorIndex,
			Color:          RoutePaletteColor(entry.ColorIndex),
			VehicleKind:    vehicleKind,
			BusStops:       stops,
		}
		if entry.ID > maxRouteID {
			maxRouteID = entry.ID
		}
		world.Routes = append(world.Routes, route)
	}

	world.Cars = make([]Car, 0, len(saved.Cars))
	routeColorByID := make(map[int]Color, len(world.Routes))
	for _, r := range world.Routes {
		routeColorByID[r.ID] = r.Color
	}
	for _, entry := range saved.Cars {
		vehicleKind := VehicleKindFromString(entry.VehicleKind)
		car := Car{
			RouteID:              entry.RouteID,
			CurrentSplineID:      entry.CurrentSplineID,
			DestinationSplineID:  entry.DestinationSplineID,
			PrevSplineIDs:        [2]int{-1, -1},
			DistanceOnSpline:     entry.DistanceOnSpline,
			Speed:                entry.Speed,
			MaxSpeed:             entry.MaxSpeed,
			Accel:                entry.Accel,
			Length:               entry.Length,
			Width:                entry.Width,
			CurveSpeedMultiplier: entry.CurveSpeedMultiplier,
			Color:                routeColorByID[entry.RouteID],
			Braking:              false,
			LaneChangeSplineID:   -1,
			AfterSplineID:        -1,
			DesiredLaneSplineID:  -1,
			PreferenceCooldown:   rand.Float32() * preferenceChangeCooldownS,
			OvertakeCooldown:     rand.Float32() * overtakeCooldownS,
			VehicleKind:          vehicleKind,
			NextBusStopIndex:     entry.NextBusStopIndex,
			BusStopTimer:         entry.BusStopTimer,
			BusStopDuration:      entry.BusStopDuration,
		}
		if car.CurveSpeedMultiplier == 0 {
			car.CurveSpeedMultiplier = randRange(0.8, 1.2)
		}
		if spline, ok := FindSplineByID(world.Splines, car.CurrentSplineID); ok {
			frontPos, tangent := SampleSplineAtDistance(spline, car.DistanceOnSpline)
			car.RearPosition = Vec2{
				X: frontPos.X - tangent.X*car.Length*wheelbaseFrac,
				Y: frontPos.Y - tangent.Y*car.Length*wheelbaseFrac,
			}
		}
		if car.VehicleKind == VehicleBus {
			for _, route := range world.Routes {
				if route.ID == car.RouteID {
					car.DestinationSplineID = CurrentRouteTarget(route, car.NextBusStopIndex)
					break
				}
			}
		}
		world.Cars = append(world.Cars, car)
	}

	world.TrafficLights = make([]TrafficLight, 0, len(saved.TrafficLights))
	for _, entry := range saved.TrafficLights {
		world.TrafficLights = append(world.TrafficLights, TrafficLight{
			ID:           entry.ID,
			SplineID:     entry.SplineID,
			DistOnSpline: entry.DistOnSpline,
			WorldPos:     NewVec2(entry.WorldPosX, entry.WorldPosY),
			CycleID:      entry.CycleID,
		})
		if entry.ID > maxLightID {
			maxLightID = entry.ID
		}
	}

	world.TrafficCycles = make([]TrafficCycle, 0, len(saved.TrafficCycles))
	for _, entry := range saved.TrafficCycles {
		phases := make([]TrafficPhase, len(entry.Phases))
		for i, p := range entry.Phases {
			clrDur := p.ClearanceDurationSecs
			if clrDur <= 0 {
				if entry.YellowDurationSecs > 0 {
					clrDur = entry.YellowDurationSecs
				} else {
					clrDur = 3.0
				}
			}
			phases[i] = TrafficPhase{
				DurationSecs:          p.DurationSecs,
				ClearanceDurationSecs: clrDur,
				GreenLightIDs:         append([]int(nil), p.GreenLightIDs...),
			}
		}
		world.TrafficCycles = append(world.TrafficCycles, TrafficCycle{
			ID:         entry.ID,
			LightIDs:   nil,
			Phases:     phases,
			Timer:      0,
			PhaseIndex: 0,
			Enabled:    entry.Enabled,
		})
		if entry.ID > maxCycleID {
			maxCycleID = entry.ID
		}
	}
	cycleIdx := make(map[int]int, len(world.TrafficCycles))
	for i, c := range world.TrafficCycles {
		cycleIdx[c.ID] = i
	}
	for _, l := range world.TrafficLights {
		if l.CycleID >= 0 {
			if i, ok := cycleIdx[l.CycleID]; ok {
				world.TrafficCycles[i].LightIDs = append(world.TrafficCycles[i].LightIDs, l.ID)
			}
		}
	}

	world.NextSplineID = maxSplineID + 1
	world.NextRouteID = maxRouteID + 1
	world.NextLightID = maxLightID + 1
	world.NextCycleID = maxCycleID + 1
	return &world, nil
}

func ComputeRoutePathWithGraph(route Route, graph *RoadGraph) ([]int, float32, string, bool) {
	if graph == nil {
		return nil, 0, "", false
	}
	if route.VehicleKind == VehicleCar {
		if startSpline, ok := graph.splineByID(route.StartSplineID); ok && startSpline.BusOnly {
			return nil, 0, "Bus-only splines cannot be used for car routes.", false
		}
		if endSpline, ok := graph.splineByID(route.EndSplineID); ok && endSpline.BusOnly {
			return nil, 0, "Bus-only splines cannot be used for car routes.", false
		}
	}
	if route.VehicleKind != VehicleBus || len(route.BusStops) == 0 {
		pathIDs, pathLength, ok := FindShortestPathWeightedWithGraph(graph, route.StartSplineID, route.EndSplineID, route.VehicleKind)
		if !ok {
			return nil, 0, "No valid path between the selected start and destination.", false
		}
		return pathIDs, pathLength, "", ok
	}

	waypoints := make([]int, 0, len(route.BusStops)+2)
	waypoints = append(waypoints, route.StartSplineID)
	for _, stop := range route.BusStops {
		waypoints = append(waypoints, stop.SplineID)
	}
	waypoints = append(waypoints, route.EndSplineID)

	fullPath := make([]int, 0, 32)
	totalCost := float32(0)
	for i := 0; i < len(waypoints)-1; i++ {
		segPath, segCost, ok := FindShortestPathWeightedWithGraph(graph, waypoints[i], waypoints[i+1], route.VehicleKind)
		if !ok || len(segPath) == 0 {
			return nil, 0, fmt.Sprintf("Stop %d cannot be reached in sequence.", i+1), false
		}
		if len(fullPath) > 0 && fullPath[len(fullPath)-1] == segPath[0] {
			segPath = segPath[1:]
			if idx, ok := graph.indexByID[waypoints[i]]; ok {
				segCost -= graph.segmentCosts[idx]
			}
		}
		fullPath = append(fullPath, segPath...)
		totalCost += segCost
	}
	return fullPath, totalCost, "", len(fullPath) > 0
}

func CurrentRouteTarget(route Route, nextBusStopIndex int) int {
	if route.VehicleKind == VehicleBus && nextBusStopIndex >= 0 && nextBusStopIndex < len(route.BusStops) {
		return route.BusStops[nextBusStopIndex].SplineID
	}
	return route.EndSplineID
}

func PickBestBusStopSpline(graph *RoadGraph, nodeKey string, fromSplineID, toSplineID int) (BusStop, bool) {
	if graph == nil {
		return BusStop{}, false
	}
	indices := graph.startsByNode[nodeKey]
	if len(indices) == 0 {
		return BusStop{}, false
	}

	bestCost := float32(math.MaxFloat32)
	best := BusStop{}
	found := false
	for _, idx := range indices {
		candidate := graph.splines[idx]
		_, firstCost, okFirst := FindShortestPathWeightedWithGraph(graph, fromSplineID, candidate.ID, VehicleBus)
		_, secondCost, okSecond := FindShortestPathWeightedWithGraph(graph, candidate.ID, toSplineID, VehicleBus)
		if !okFirst || !okSecond {
			continue
		}
		total := firstCost + secondCost - graph.segmentCosts[idx]
		if !found || total < bestCost {
			bestCost = total
			best = BusStop{SplineID: candidate.ID, WorldPos: candidate.P0}
			found = true
		}
	}
	return best, found
}

func EffectivePhaseDur(c *TrafficCycle, ei int) float32 {
	n := len(c.Phases)
	if n <= 1 {
		if n == 0 {
			return 0.001
		}
		return maxf(c.Phases[0].DurationSecs, 0.001)
	}
	if ei%2 == 0 {
		dur := c.Phases[ei/2].DurationSecs
		if dur <= 0 {
			dur = 0.001
		}
		return dur
	}
	clearDur := c.Phases[ei/2].ClearanceDurationSecs
	if clearDur <= 0 {
		clearDur = 3.0
	}
	return clearDur
}

func effectivePhaseCount(c *TrafficCycle) int {
	n := len(c.Phases)
	if n <= 1 {
		return n
	}
	return 2 * n
}

func EffectivePhaseCount(c *TrafficCycle) int {
	return effectivePhaseCount(c)
}

func UpdateTrafficCycles(cycles []TrafficCycle, dt float32) []TrafficCycle {
	for i := range cycles {
		if !cycles[i].Enabled || len(cycles[i].Phases) == 0 {
			continue
		}
		total := effectivePhaseCount(&cycles[i])
		cycles[i].Timer += dt
		dur := EffectivePhaseDur(&cycles[i], cycles[i].PhaseIndex)
		if cycles[i].Timer >= dur {
			cycles[i].Timer -= dur
			cycles[i].PhaseIndex = (cycles[i].PhaseIndex + 1) % total
		}
	}
	return cycles
}

func phaseHasGreen(phase TrafficPhase, lightID int) bool {
	for _, id := range phase.GreenLightIDs {
		if id == lightID {
			return true
		}
	}
	return false
}

func TrafficLightState(lightID, cycleID int, cycles []TrafficCycle) TrafficState {
	for _, c := range cycles {
		if c.ID != cycleID {
			continue
		}
		if !c.Enabled || len(c.Phases) == 0 {
			return TrafficRed
		}
		n := len(c.Phases)
		ei := c.PhaseIndex
		if n <= 1 || ei%2 == 0 {
			userIdx := ei / 2
			if userIdx >= n {
				userIdx = n - 1
			}
			if phaseHasGreen(c.Phases[userIdx], lightID) {
				return TrafficGreen
			}
			return TrafficRed
		}
		prevIdx := (ei / 2) % n
		nextIdx := (prevIdx + 1) % n
		prevGreen := phaseHasGreen(c.Phases[prevIdx], lightID)
		nextGreen := phaseHasGreen(c.Phases[nextIdx], lightID)
		if !prevGreen {
			return TrafficRed
		}
		if nextGreen {
			return TrafficGreen
		}
		return TrafficYellow
	}
	return TrafficRed
}

func UpdateRouteVisuals(routes []Route, splines []Spline, vehicleCounts map[int]int) []Route {
	graph := NewRoadGraph(splines, vehicleCounts)
	return UpdateRouteVisualsWithGraph(routes, graph)
}

func UpdateRouteVisualsWithGraph(routes []Route, graph *RoadGraph) []Route {
	for i := range routes {
		pathIDs, pathLength, _, ok := ComputeRoutePathWithGraph(routes[i], graph)
		routes[i].PathIDs = pathIDs
		routes[i].PathLength = pathLength
		routes[i].Valid = ok && len(pathIDs) > 0
		if routes[i].SpawnPerMinute > 0 && routes[i].NextSpawnIn <= 0 {
			routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
		}
	}
	return routes
}

func updateRouteSpawning(routes []Route, cars []Car, splines []Spline, dt float32) ([]Route, []Car) {
	for i := range routes {
		if !routes[i].Valid || routes[i].SpawnPerMinute <= 0 {
			continue
		}
		routes[i].NextSpawnIn -= dt
		if routes[i].NextSpawnIn > 0 {
			continue
		}
		candidate := spawnVehicle(routes[i], splines)
		if !spawnBlocked(candidate, cars, splines) {
			cars = append(cars, candidate)
			routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
		}
	}
	return routes, cars
}

func spawnBlocked(candidate Car, cars []Car, splines []Spline) bool {
	spline, ok := findSplineByID(splines, candidate.CurrentSplineID)
	if !ok {
		return false
	}
	frontPos, _ := sampleSplineAtDistance(spline, 0)
	cCenter := vecScale(vecAdd(frontPos, candidate.RearPosition), 0.5)
	cHeading := normalize(vecSub(frontPos, candidate.RearPosition))
	rC := collisionRadius(candidate)
	candidateOffsets := collisionCircleOffsets(candidate)

	for _, other := range cars {
		if other.CurrentSplineID != candidate.CurrentSplineID {
			continue
		}
		otherSpline, ok := findSplineByID(splines, other.CurrentSplineID)
		if !ok {
			continue
		}
		otherFront, _ := sampleSplineAtDistance(otherSpline, other.DistanceOnSpline)
		oCenter := vecScale(vecAdd(otherFront, other.RearPosition), 0.5)
		oHeading := normalize(vecSub(otherFront, other.RearPosition))
		rO := collisionRadius(other)
		thresh := rC + rO
		threshSq := thresh * thresh
		otherOffsets := collisionCircleOffsets(other)

		for _, offC := range candidateOffsets {
			cCircle := vecAdd(cCenter, vecScale(cHeading, offC))
			for _, offO := range otherOffsets {
				oCircle := vecAdd(oCenter, vecScale(oHeading, offO))
				if distSq(cCircle, oCircle) <= threshSq {
					return true
				}
			}
		}
	}
	return false
}

func spawnVehicle(route Route, splines []Spline) Car {
	hasTrailer := route.VehicleKind == VehicleCar && rand.Float32() < 0.10
	length := randRange(4.0, 4.8) / metersPerUnit
	width := randRange(1.8, 2.0) / metersPerUnit
	maxSpeed := randRange(22.2, 36.1)
	accel := randRange(2.5, 4.5)
	curveSpeedMultiplier := randRange(0.8, 1.2)
	if route.VehicleKind == VehicleBus {
		length = randRange(9.5, 13.0) / metersPerUnit
		width = randRange(2.45, 2.65) / metersPerUnit
		maxSpeed = randRange(13.9, 20.0)
		accel = randRange(1.2, 2.0)
		curveSpeedMultiplier = randRange(0.9, 1.05)
	} else if hasTrailer {
		length = randRange(4.8, 6.0) / metersPerUnit
		width = randRange(2.25, 2.45) / metersPerUnit
		maxSpeed = randRange(18.0, 25.0)
		accel = randRange(1.0, 1.8)
		curveSpeedMultiplier = randRange(0.85, 1.0)
	}

	car := Car{
		RouteID:              route.ID,
		CurrentSplineID:      route.StartSplineID,
		DestinationSplineID:  CurrentRouteTarget(route, 0),
		PrevSplineIDs:        [2]int{-1, -1},
		DistanceOnSpline:     0,
		Speed:                randRange(0, 2),
		MaxSpeed:             maxSpeed,
		Accel:                accel,
		Length:               length,
		Width:                width,
		CurveSpeedMultiplier: curveSpeedMultiplier,
		Color:                route.Color,
		Braking:              false,
		LaneChangeSplineID:   -1,
		AfterSplineID:        -1,
		DesiredLaneSplineID:  -1,
		PreferenceCooldown:   rand.Float32() * preferenceChangeCooldownS,
		OvertakeCooldown:     rand.Float32() * overtakeCooldownS,
		VehicleKind:          route.VehicleKind,
	}
	if spline, ok := findSplineByID(splines, route.StartSplineID); ok {
		frontPos, tangent := sampleSplineAtDistance(spline, 0)
		car.RearPosition = vecSub(frontPos, vecScale(tangent, car.Length*wheelbaseFrac))

		if hasTrailer {
			tLen := randRange(11.0, 13.6)
			tWid := randRange(2.25, 2.45)
			trailerRear := vecSub(car.RearPosition, vecScale(tangent, tLen*wheelbaseFrac))
			r, g, b := car.Color.R, car.Color.G, car.Color.B
			car.Trailer = Trailer{
				HasTrailer:   true,
				Length:       tLen,
				Width:        tWid,
				Color:        NewColor(r/2+20, g/2+20, b/2+20, 255),
				RearPosition: trailerRear,
			}
		}
	}
	return car
}

func beginBusStopDwell(route Route, car *Car) bool {
	if route.VehicleKind != VehicleBus || car.NextBusStopIndex >= len(route.BusStops) {
		return false
	}
	stop := route.BusStops[car.NextBusStopIndex]
	car.Speed = 0
	car.BusStopDuration = randRange(busStopMinDwellSeconds, busStopMaxDwellSeconds)
	car.BusStopTimer = car.BusStopDuration
	car.NextBusStopIndex++
	if car.CurrentSplineID == stop.SplineID {
		car.DestinationSplineID = CurrentRouteTarget(route, car.NextBusStopIndex)
	} else {
		car.DestinationSplineID = stop.SplineID
	}
	return true
}

func resumeBusRouteAfterStop(route Route, car *Car) {
	if route.VehicleKind != VehicleBus || car.NextBusStopIndex <= 0 || car.NextBusStopIndex > len(route.BusStops) {
		return
	}
	prevStopSplineID := route.BusStops[car.NextBusStopIndex-1].SplineID
	if car.CurrentSplineID == prevStopSplineID {
		car.DestinationSplineID = CurrentRouteTarget(route, car.NextBusStopIndex)
	}
}

func computeBusStopSpeedCap(car Car, route Route, currentSpline Spline, graph *RoadGraph) float32 {
	if route.VehicleKind != VehicleBus || car.NextBusStopIndex >= len(route.BusStops) {
		return float32(math.MaxFloat32)
	}
	stopSplineID := route.BusStops[car.NextBusStopIndex].SplineID
	decel := car.Accel * 1.5

	checkStop := func(rawDistAhead float32) float32 {
		adj := rawDistAhead - stopFrontGapM
		if adj <= 0 {
			return 0
		}
		return float32(math.Sqrt(float64(2 * decel * adj)))
	}

	if currentSpline.ID == stopSplineID {
		if car.DistanceOnSpline <= stopFrontGapM {
			return 0
		}
		return float32(math.MaxFloat32)
	}

	nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, stopSplineID, car.VehicleKind)
	if !ok {
		forcedNext, _, forcedOK := FindForcedLaneChangePathWithGraph(graph, currentSpline.ID, stopSplineID, car.VehicleKind)
		if !forcedOK {
			return float32(math.MaxFloat32)
		}
		nextID = forcedNext
	}
	if nextID != stopSplineID {
		return float32(math.MaxFloat32)
	}
	return checkStop(currentSpline.Length - car.DistanceOnSpline)
}

func shouldBeginBusStopDwell(car Car, route Route, currentSpline Spline, graph *RoadGraph) bool {
	if route.VehicleKind != VehicleBus || car.BusStopTimer > 0 || car.NextBusStopIndex >= len(route.BusStops) {
		return false
	}
	stopSplineID := route.BusStops[car.NextBusStopIndex].SplineID
	const stopSpeedThreshold = 0.35
	if car.Speed > stopSpeedThreshold {
		return false
	}

	stopSlack := stopFrontGapM + 0.5
	if currentSpline.ID == stopSplineID {
		return car.DistanceOnSpline <= stopSlack
	}

	nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, stopSplineID, car.VehicleKind)
	if !ok {
		forcedNext, _, forcedOK := FindForcedLaneChangePathWithGraph(graph, currentSpline.ID, stopSplineID, car.VehicleKind)
		if !forcedOK {
			return false
		}
		nextID = forcedNext
	}
	if nextID != stopSplineID {
		return false
	}
	remaining := currentSpline.Length - car.DistanceOnSpline
	return remaining <= stopSlack
}

type carPose struct {
	pos     Vec2
	heading Vec2
}

type spatialGrid struct {
	cellSize    float32
	invCellSize float32
	cells       map[[2]int32][]int
}

func newSpatialGrid(cellSize float32, n int) spatialGrid {
	return spatialGrid{
		cellSize:    cellSize,
		invCellSize: 1.0 / cellSize,
		cells:       make(map[[2]int32][]int, n),
	}
}

func (g *spatialGrid) cellKey(p Vec2) [2]int32 {
	return [2]int32{
		int32(math.Floor(float64(p.X * g.invCellSize))),
		int32(math.Floor(float64(p.Y * g.invCellSize))),
	}
}

func (g *spatialGrid) insert(index int, p Vec2, radius float32) {
	if radius <= g.cellSize {
		k := g.cellKey(p)
		g.cells[k] = append(g.cells[k], index)
		return
	}
	minX := int32(math.Floor(float64((p.X - radius) * g.invCellSize)))
	maxX := int32(math.Floor(float64((p.X + radius) * g.invCellSize)))
	minY := int32(math.Floor(float64((p.Y - radius) * g.invCellSize)))
	maxY := int32(math.Floor(float64((p.Y + radius) * g.invCellSize)))
	for cx := minX; cx <= maxX; cx++ {
		for cy := minY; cy <= maxY; cy++ {
			k := [2]int32{cx, cy}
			g.cells[k] = append(g.cells[k], index)
		}
	}
}

func (g *spatialGrid) queryNeighbors(p Vec2, buf []int) []int {
	buf = buf[:0]
	c := g.cellKey(p)
	for dx := int32(-1); dx <= 1; dx++ {
		for dy := int32(-1); dy <= 1; dy++ {
			k := [2]int32{c[0] + dx, c[1] + dy}
			buf = append(buf, g.cells[k]...)
		}
	}
	return buf
}

type trajectoryAABB struct {
	minX, minY, maxX, maxY float32
}

func buildTrajectoryAABB(samples []TrajectorySample, coarseRadius float32) trajectoryAABB {
	if len(samples) == 0 {
		return trajectoryAABB{}
	}
	bb := trajectoryAABB{
		minX: samples[0].Position.X,
		minY: samples[0].Position.Y,
		maxX: samples[0].Position.X,
		maxY: samples[0].Position.Y,
	}
	for _, s := range samples[1:] {
		if s.Position.X < bb.minX {
			bb.minX = s.Position.X
		}
		if s.Position.X > bb.maxX {
			bb.maxX = s.Position.X
		}
		if s.Position.Y < bb.minY {
			bb.minY = s.Position.Y
		}
		if s.Position.Y > bb.maxY {
			bb.maxY = s.Position.Y
		}
		if s.HasTrailer {
			if s.TrailerPosition.X < bb.minX {
				bb.minX = s.TrailerPosition.X
			}
			if s.TrailerPosition.X > bb.maxX {
				bb.maxX = s.TrailerPosition.X
			}
			if s.TrailerPosition.Y < bb.minY {
				bb.minY = s.TrailerPosition.Y
			}
			if s.TrailerPosition.Y > bb.maxY {
				bb.maxY = s.TrailerPosition.Y
			}
		}
	}
	bb.minX -= coarseRadius
	bb.minY -= coarseRadius
	bb.maxX += coarseRadius
	bb.maxY += coarseRadius
	return bb
}

func aabbOverlap(a, b trajectoryAABB) bool {
	return a.minX <= b.maxX && a.maxX >= b.minX &&
		a.minY <= b.maxY && a.maxY >= b.minY
}

// closingRateScale returns a multiplier in [0.3, 1.0] for the broad-phase
// distance threshold based on how fast two cars are approaching each other.
// 1.0 when closing head-on, 0.3 when separating.
func closingRateScale(posI, posJ, headingI, headingJ Vec2, speedI, speedJ float32) float32 {
	displacement := vecSub(posJ, posI)
	dLen := float32(math.Sqrt(float64(vectorLengthSq(displacement))))
	if dLen < 1e-6 {
		return 1.0
	}
	dNorm := vecScale(displacement, 1.0/dLen)
	// positive closing rate = cars approaching
	closingRate := dot(vecScale(headingI, speedI), dNorm) - dot(vecScale(headingJ, speedJ), dNorm)
	maxClosing := speedI + speedJ
	if maxClosing < 1e-6 {
		return 1.0
	}
	// ratio: 1.0 = head-on, 0.0 = perpendicular, -1.0 = separating
	ratio := closingRate / maxClosing
	// map [-1, 1] to [0.3, 1.0]
	if ratio > 1 {
		ratio = 1
	} else if ratio < -1 {
		ratio = -1
	}
	return 0.65 + 0.35*ratio
}

func recentlyLeft(car Car, splineID int) bool {
	return splineID >= 0 && (car.PrevSplineIDs[0] == splineID || car.PrevSplineIDs[1] == splineID)
}

func medianReach(reach []float32) float32 {
	if len(reach) == 0 {
		return 1
	}
	sorted := make([]float32, len(reach))
	copy(sorted, reach)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i] < sorted[j] })
	return sorted[len(sorted)/2]
}

func computeBrakingDecisions(cars []Car, graph *RoadGraph, debugSelectedCar int, debugSelectedCarMode int) ([]bool, []bool, []DebugBlameLink, []DebugBlameLink, []DebugBlameLink, BrakingProfile) {
	flags := make([]bool, len(cars))
	holdSpeed := make([]bool, len(cars))
	initialBlame := make([]bool, len(cars))
	tentativeLinks := make([]DebugBlameLink, 0)
	holdLinks := make([]DebugBlameLink, 0)
	candidateLinks := make([]DebugBlameLink, 0)
	profile := BrakingProfile{Cars: len(cars)}
	if len(cars) < 2 {
		return flags, holdSpeed, tentativeLinks, holdLinks, candidateLinks, profile
	}

	basePredictStart := time.Now()
	baseResults := computeBasePredictionResults(cars, graph)
	predictions := make([][]TrajectorySample, len(cars))
	stationaryPredictions := make([][]TrajectorySample, len(cars))
	geometries := make([]collisionGeometry, len(cars))
	poses := make([]carPose, len(cars))
	reach := make([]float32, len(cars))
	for i, result := range baseResults {
		predictions[i] = result.prediction
		geometries[i] = result.geometry
		poses[i] = result.pose
		reach[i] = result.reach
		profile.BasePredictions++
		profile.TotalPredictions++
		profile.TotalPredictionSamples += result.totalSamples
	}
	profile.BasePredictMS = sinceMS(basePredictStart)

	trajAABBs := make([]trajectoryAABB, len(cars))
	for i := range cars {
		trajAABBs[i] = buildTrajectoryAABB(predictions[i], geometries[i].coarseRadius)
	}

	gridCellSize := medianReach(reach)
	if gridCellSize < 1 {
		gridCellSize = 1
	}
	grid := newSpatialGrid(gridCellSize, len(cars))
	for i := range cars {
		if len(predictions[i]) > 0 {
			grid.insert(i, poses[i].pos, reach[i])
		}
	}

	conflictScanStart := time.Now()
	seen := make(map[[2]int]bool, len(cars)*4)
	neighborBuf := make([]int, 0, 32)
	for i := 0; i < len(cars); i++ {
		if len(predictions[i]) == 0 {
			continue
		}
		neighborBuf = grid.queryNeighbors(poses[i].pos, neighborBuf)
		for _, j := range neighborBuf {
			if j <= i {
				continue
			}
			if seen[[2]int{i, j}] {
				continue
			}
			seen[[2]int{i, j}] = true
			profile.PrimaryPairCandidates++
			scale := closingRateScale(poses[i].pos, poses[j].pos, poses[i].heading, poses[j].heading, cars[i].Speed, cars[j].Speed)
			broadPhaseDist := (reach[i] + reach[j]) * scale
			if distSq(poses[i].pos, poses[j].pos) > broadPhaseDist*broadPhaseDist {
				continue
			}
			profile.PrimaryBroadPhasePairs++
			if !aabbOverlap(trajAABBs[i], trajAABBs[j]) {
				continue
			}
			if debugSelectedCar >= 0 && debugSelectedCarMode == 0 && (i == debugSelectedCar || j == debugSelectedCar) {
				candidateLinks = append(candidateLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
			}
			profile.PrimaryCollisionChecks++
			collision, ok := predictCollision(predictions[i], predictions[j], geometries[i], geometries[j])
			if !ok {
				continue
			}
			profile.PrimaryCollisionHits++
			blameI, blameJ := determineBlame(collision, cars[i], cars[j], graph.splines)
			alreadyCollided := collision.AlreadyCollided
			if blameI && recentlyLeft(cars[i], cars[j].CurrentSplineID) {
				blameI = false
			}
			if blameJ && recentlyLeft(cars[j], cars[i].CurrentSplineID) {
				blameJ = false
			}
			if blameI && !alreadyCollided {
				if stationaryPredictions[i] == nil {
					sc := cars[i]
					sc.Speed = 0
					stationaryPredictions[i] = predictCarTrajectory(sc, graph, predictionHorizonSeconds, predictionStepSeconds)
					profile.StationaryPredictions++
					profile.TotalPredictions++
					profile.TotalPredictionSamples += len(stationaryPredictions[i])
				}
				profile.StationaryCollisionChecks++
				if _, still := predictCollision(stationaryPredictions[i], predictions[j], geometries[i], geometries[j]); still {
					profile.StationaryCollisionHits++
					blameI = false
				}
			}
			if blameJ && !alreadyCollided {
				if stationaryPredictions[j] == nil {
					sc := cars[j]
					sc.Speed = 0
					stationaryPredictions[j] = predictCarTrajectory(sc, graph, predictionHorizonSeconds, predictionStepSeconds)
					profile.StationaryPredictions++
					profile.TotalPredictions++
					profile.TotalPredictionSamples += len(stationaryPredictions[j])
				}
				profile.StationaryCollisionChecks++
				if _, still := predictCollision(stationaryPredictions[j], predictions[i], geometries[j], geometries[i]); still {
					profile.StationaryCollisionHits++
					blameJ = false
				}
			}
			if blameI {
				if !initialBlame[i] {
					profile.InitiallyBlamedCars++
				}
				initialBlame[i] = true
				tentativeLinks = append(tentativeLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
			}
			if blameJ {
				if !initialBlame[j] {
					profile.InitiallyBlamedCars++
				}
				initialBlame[j] = true
				tentativeLinks = append(tentativeLinks, DebugBlameLink{FromCarIndex: j, ToCarIndex: i})
			}
		}
	}
	profile.ConflictScanMS = sinceMS(conflictScanStart)

	brakeProbeStart := time.Now()
	for i, result := range computeBrakeProbeResults(cars, graph, initialBlame, predictions, geometries, poses, reach, trajAABBs, &grid) {
		if result.shouldBrake {
			flags[i] = true
		}
		profile.EscapePredictions += result.escapePredictions
		profile.TotalPredictions += result.escapePredictions
		profile.TotalPredictionSamples += result.totalPredictionSamples
		profile.EscapeCollisionChecks += result.escapeCollisionChecks
		profile.EscapeCollisionHits += result.escapeCollisionHits
	}
	profile.BrakeProbeMS = sinceMS(brakeProbeStart)

	holdProbeStart := time.Now()
	for i, result := range computeHoldProbeResults(cars, graph, flags, debugSelectedCar, debugSelectedCarMode, predictions, stationaryPredictions, geometries, poses, reach, trajAABBs, &grid) {
		if result.shouldHold {
			holdSpeed[i] = true
		}
		if result.hasHoldLink {
			holdLinks = append(holdLinks, result.holdLink)
		}
		candidateLinks = append(candidateLinks, result.candidateLinks...)
		profile.FasterPredictions += result.fasterPredictions
		profile.StationaryPredictions += result.stationaryPredictions
		profile.TotalPredictions += result.fasterPredictions + result.stationaryPredictions
		profile.TotalPredictionSamples += result.totalPredictionSamples
		profile.HoldCollisionChecks += result.holdCollisionChecks
		profile.HoldCollisionHits += result.holdCollisionHits
		profile.StationaryCollisionChecks += result.stationaryCollisionChecks
		profile.StationaryCollisionHits += result.stationaryCollisionHits
	}
	profile.HoldProbeMS = sinceMS(holdProbeStart)

	finalizeStart := time.Now()
	released := detectDeadlockReleases(len(cars), tentativeLinks, holdLinks, flags, holdSpeed)
	for i := range released {
		if !released[i] {
			continue
		}
		flags[i] = false
		holdSpeed[i] = false
	}

	debugLinks := make([]DebugBlameLink, 0, len(tentativeLinks))
	for _, link := range tentativeLinks {
		if link.FromCarIndex >= 0 && link.FromCarIndex < len(flags) && flags[link.FromCarIndex] {
			debugLinks = append(debugLinks, link)
		}
	}
	activeHoldLinks := make([]DebugBlameLink, 0, len(holdLinks))
	for _, link := range holdLinks {
		if link.FromCarIndex >= 0 && link.FromCarIndex < len(holdSpeed) && holdSpeed[link.FromCarIndex] {
			activeHoldLinks = append(activeHoldLinks, link)
		}
	}
	for _, flagged := range flags {
		if flagged {
			profile.BrakingCars++
		}
	}
	for _, held := range holdSpeed {
		if held {
			profile.HoldCars++
		}
	}
	profile.FinalizeMS = sinceMS(finalizeStart)

	return flags, holdSpeed, debugLinks, activeHoldLinks, candidateLinks, profile
}

func computeBasePredictionResults(cars []Car, graph *RoadGraph) []basePredictionCarResult {
	results := make([]basePredictionCarResult, len(cars))
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			car := cars[i]
			result := &results[i]
			if spline, ok := graph.splineByID(car.CurrentSplineID); ok {
				pos, heading := sampleSplineAtDistance(spline, car.DistanceOnSpline)
				result.pose = carPose{pos: pos, heading: heading}
			}
			physicalSize := maxf(car.Length, car.Width)
			if car.Trailer.HasTrailer {
				physicalSize = car.Length + car.Trailer.Length
			}
			result.reach = maxf(car.Speed, 0)*predictionHorizonSeconds + 0.5*car.Accel*predictionHorizonSeconds*predictionHorizonSeconds +
				physicalSize + collisionBroadPhaseSlackM
			result.geometry = buildCollisionGeometry(car)
			result.prediction = predictCarTrajectory(car, graph, predictionHorizonSeconds, predictionStepSeconds)
			result.totalSamples = len(result.prediction)
		}
	})
	return results
}

func computeBrakeProbeResults(cars []Car, graph *RoadGraph, initialBlame []bool, predictions [][]TrajectorySample, geometries []collisionGeometry, poses []carPose, reach []float32, trajAABBs []trajectoryAABB, grid *spatialGrid) []brakeProbeCarResult {
	results := make([]brakeProbeCarResult, len(cars))
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			if i >= len(initialBlame) || !initialBlame[i] {
				continue
			}
			var localProfile BrakingProfile
			results[i] = brakeProbeCarResult{
				shouldBrake:            shouldBrakeForBlamedConflicts(i, cars, graph, predictions, geometries, poses, reach, trajAABBs, grid, &localProfile),
				escapePredictions:      localProfile.EscapePredictions,
				totalPredictionSamples: localProfile.TotalPredictionSamples,
				escapeCollisionChecks:  localProfile.EscapeCollisionChecks,
				escapeCollisionHits:    localProfile.EscapeCollisionHits,
			}
		}
	})
	return results
}

func computeHoldProbeResults(cars []Car, graph *RoadGraph, flags []bool, debugSelectedCar int, debugSelectedCarMode int, predictions [][]TrajectorySample, stationaryPredictions [][]TrajectorySample, geometries []collisionGeometry, poses []carPose, reach []float32, trajAABBs []trajectoryAABB, grid *spatialGrid) []holdProbeCarResult {
	results := make([]holdProbeCarResult, len(cars))
	if len(cars) == 0 {
		return results
	}

	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			results[i] = computeHoldProbeResultForCar(i, cars, graph, flags, debugSelectedCar, debugSelectedCarMode, predictions, stationaryPredictions, geometries, poses, reach, trajAABBs, grid)
		}
	})
	return results
}

func computeHoldProbeResultForCar(i int, cars []Car, graph *RoadGraph, flags []bool, debugSelectedCar int, debugSelectedCarMode int, predictions [][]TrajectorySample, stationaryPredictions [][]TrajectorySample, geometries []collisionGeometry, poses []carPose, reach []float32, trajAABBs []trajectoryAABB, grid *spatialGrid) holdProbeCarResult {
	var result holdProbeCarResult
	if i < 0 || i >= len(cars) || i >= len(predictions) || i >= len(flags) || flags[i] || len(predictions[i]) == 0 {
		return result
	}

	car := cars[i]
	fasterCar := car
	fasterCar.Speed += 0.25 * fasterCar.Accel
	if fasterCar.Speed <= car.Speed+1e-4 {
		return result
	}

	fasterPred := predictCarTrajectory(fasterCar, graph, predictionHorizonSeconds, predictionStepSeconds)
	result.fasterPredictions = 1
	result.totalPredictionSamples += len(fasterPred)
	if len(fasterPred) == 0 {
		return result
	}

	fasterAABB := buildTrajectoryAABB(fasterPred, geometries[i].coarseRadius)
	neighbors := grid.queryNeighbors(poses[i].pos, nil)
	stationaryPred := stationaryPredictions[i]
	for _, j := range neighbors {
		if j == i || j < 0 || j >= len(predictions) || len(predictions[j]) == 0 {
			continue
		}
		scale := closingRateScale(poses[i].pos, poses[j].pos, poses[i].heading, poses[j].heading, cars[i].Speed, cars[j].Speed)
		broadPhaseDist := (reach[i] + reach[j]) * scale * scale
		if distSq(poses[i].pos, poses[j].pos) > broadPhaseDist*broadPhaseDist {
			continue
		}
		if !aabbOverlap(fasterAABB, trajAABBs[j]) {
			continue
		}
		if debugSelectedCar >= 0 && debugSelectedCarMode == 1 && i == debugSelectedCar {
			result.candidateLinks = append(result.candidateLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
		}
		result.holdCollisionChecks++
		collision, ok := predictCollision(fasterPred, predictions[j], geometries[i], geometries[j])
		if !ok {
			continue
		}
		result.holdCollisionHits++
		blameI, _ := determineBlame(collision, fasterCar, cars[j], graph.splines)
		if !blameI || recentlyLeft(fasterCar, cars[j].CurrentSplineID) {
			continue
		}
		if collision.AlreadyCollided {
			result.shouldHold = true
			result.holdLink = DebugBlameLink{FromCarIndex: i, ToCarIndex: j}
			result.hasHoldLink = true
			break
		}
		if stationaryPred == nil {
			sc := cars[i]
			sc.Speed = 0
			stationaryPred = predictCarTrajectory(sc, graph, predictionHorizonSeconds, predictionStepSeconds)
			result.stationaryPredictions++
			result.totalPredictionSamples += len(stationaryPred)
		}
		result.stationaryCollisionChecks++
		if _, still := predictCollision(stationaryPred, predictions[j], geometries[i], geometries[j]); !still {
			result.shouldHold = true
			result.holdLink = DebugBlameLink{FromCarIndex: i, ToCarIndex: j}
			result.hasHoldLink = true
			break
		}
		result.stationaryCollisionHits++
	}

	return result
}

func remapBlameLinks(links []DebugBlameLink, remap []int) []DebugBlameLink {
	if len(remap) == 0 {
		return links
	}
	out := make([]DebugBlameLink, 0, len(links))
	for _, link := range links {
		if link.FromCarIndex < 0 || link.FromCarIndex >= len(remap) || link.ToCarIndex < 0 || link.ToCarIndex >= len(remap) {
			continue
		}
		newFrom := remap[link.FromCarIndex]
		newTo := remap[link.ToCarIndex]
		if newFrom < 0 || newTo < 0 {
			continue
		}
		out = append(out, DebugBlameLink{FromCarIndex: newFrom, ToCarIndex: newTo})
	}
	return out
}

func shouldBrakeForBlamedConflicts(carIndex int, cars []Car, graph *RoadGraph, predictions [][]TrajectorySample, geometries []collisionGeometry, poses []carPose, reach []float32, trajAABBs []trajectoryAABB, grid *spatialGrid, profile *BrakingProfile) bool {
	if carIndex < 0 || carIndex >= len(cars) {
		return false
	}

	car := cars[carIndex]
	currentSpline, ok := graph.splineByID(car.CurrentSplineID)
	if !ok {
		return true
	}

	targetSpeed := car.MaxSpeed * currentSpline.SpeedFactor
	if car.Speed >= targetSpeed-accelEscapeLookaheadSecs*car.Accel {
		return true
	}

	for _, seconds := range []float32{
		accelEscapeTestShortSecs,
		accelEscapeTestMediumSecs,
	} {
		testCar := car
		testCar.Speed = minf(targetSpeed, car.Speed+car.Accel*seconds)
		if testCar.Speed <= car.Speed+1e-4 {
			return true
		}

		testPrediction := predictCarTrajectory(testCar, graph, predictionHorizonSeconds, predictionStepSeconds)
		if profile != nil {
			profile.EscapePredictions++
			profile.TotalPredictions++
			profile.TotalPredictionSamples += len(testPrediction)
		}
		if len(testPrediction) == 0 {
			return true
		}
		testAABB := buildTrajectoryAABB(testPrediction, geometries[carIndex].coarseRadius)
		if hasBlamedConflictWithPrediction(carIndex, testCar, testPrediction, testAABB, cars, graph, predictions, geometries, poses, reach, trajAABBs, grid, profile) {
			return true
		}
	}

	return false
}

func hasBlamedConflictWithPrediction(carIndex int, testCar Car, testPrediction []TrajectorySample, testAABB trajectoryAABB, cars []Car, graph *RoadGraph, predictions [][]TrajectorySample, geometries []collisionGeometry, poses []carPose, reach []float32, trajAABBs []trajectoryAABB, grid *spatialGrid, profile *BrakingProfile) bool {
	neighbors := grid.queryNeighbors(poses[carIndex].pos, nil)
	for _, otherIndex := range neighbors {
		if otherIndex == carIndex || otherIndex >= len(predictions) || len(predictions[otherIndex]) == 0 {
			continue
		}
		scale := closingRateScale(poses[carIndex].pos, poses[otherIndex].pos, poses[carIndex].heading, poses[otherIndex].heading, cars[carIndex].Speed, cars[otherIndex].Speed)
		broadPhaseDist := (reach[carIndex] + reach[otherIndex]) * scale
		if distSq(poses[carIndex].pos, poses[otherIndex].pos) > broadPhaseDist*broadPhaseDist {
			continue
		}
		if !aabbOverlap(testAABB, trajAABBs[otherIndex]) {
			continue
		}
		otherCar := cars[otherIndex]
		if recentlyLeft(testCar, otherCar.CurrentSplineID) {
			continue
		}

		if profile != nil {
			profile.EscapeCollisionChecks++
		}
		collision, ok := predictCollision(testPrediction, predictions[otherIndex], geometries[carIndex], geometries[otherIndex])
		if !ok {
			continue
		}
		if profile != nil {
			profile.EscapeCollisionHits++
		}

		blameTestCar, _ := determineBlame(collision, testCar, otherCar, graph.splines)
		if blameTestCar {
			return true
		}
	}
	return false
}

func detectDeadlockReleases(numCars int, brakeLinks, holdLinks []DebugBlameLink, braking []bool, holdSpeed []bool) []bool {
	if numCars <= 1 {
		return nil
	}

	outgoing := make([][]int, numCars)
	addActiveLinks := func(links []DebugBlameLink, active []bool) {
		for _, link := range links {
			if link.FromCarIndex < 0 || link.FromCarIndex >= numCars || link.ToCarIndex < 0 || link.ToCarIndex >= numCars {
				continue
			}
			if link.FromCarIndex >= len(active) || !active[link.FromCarIndex] {
				continue
			}
			outgoing[link.FromCarIndex] = append(outgoing[link.FromCarIndex], link.ToCarIndex)
		}
	}
	addActiveLinks(brakeLinks, braking)
	addActiveLinks(holdLinks, holdSpeed)

	released := make([]bool, numCars)
	path := make([]int, 4)
	for start := 0; start < numCars; start++ {
		if len(outgoing[start]) == 0 {
			continue
		}
		path[0] = start
		var search func(pathLen int)
		search = func(pathLen int) {
			cur := path[pathLen-1]
			for _, next := range outgoing[cur] {
				if next == start {
					minIdx := path[0]
					for i := 1; i < pathLen; i++ {
						if path[i] < minIdx {
							minIdx = path[i]
						}
					}
					released[minIdx] = true
					continue
				}
				if pathLen >= len(path) {
					continue
				}
				repeated := false
				for i := 0; i < pathLen; i++ {
					if path[i] == next {
						repeated = true
						break
					}
				}
				if repeated {
					continue
				}
				path[pathLen] = next
				search(pathLen + 1)
			}
		}
		search(1)
	}

	return released
}

func predictCarTrajectory(car Car, graph *RoadGraph, horizon, step float32) []TrajectorySample {
	if len(graph.splines) == 0 || car.CurrentSplineID < 0 {
		return nil
	}

	steps := int(math.Ceil(float64(horizon / step)))
	samples := make([]TrajectorySample, 0, steps+1)
	simCar := car
	simRearPos := car.RearPosition
	simTrailerRearPos := car.Trailer.RearPosition
	speed := maxf(car.Speed, 0)
	active := true

	for stepIndex := 0; stepIndex <= steps; stepIndex++ {
		spline, ok := graph.splineByID(simCar.CurrentSplineID)
		if !ok {
			break
		}
		splinePos, splineTangent := sampleSplineAtDistance(spline, simCar.DistanceOnSpline)
		κ := signedCurvatureAtArcLen(&spline, simCar.DistanceOnSpline)
		wb := simCar.Length * wheelbaseFrac
		targetOffset := κ * wb * wb / 6
		if simCar.Trailer.HasTrailer {
			trailerWb := simCar.Trailer.Length * wheelbaseFrac
			targetOffset = κ * (wb*wb + trailerWb*trailerWb) / 6
		}
		lerpRate := speed / (2 * (wb + 0.01))
		simCar.LateralOffset += (targetOffset - simCar.LateralOffset) * clampf(lerpRate*step, 0, 1)
		rightNormal := Vec2{X: splineTangent.Y, Y: -splineTangent.X}
		frontPos := vecAdd(splinePos, vecScale(rightNormal, simCar.LateralOffset))
		rearToFront := vecSub(frontPos, simRearPos)
		if vectorLengthSq(rearToFront) > 1e-9 {
			simRearPos = vecSub(frontPos, vecScale(normalize(rearToFront), simCar.Length*wheelbaseFrac))
		} else {
			simRearPos = vecSub(frontPos, vecScale(splineTangent, simCar.Length*wheelbaseFrac))
		}
		center := vecScale(vecAdd(frontPos, simRearPos), 0.5)
		bodyHeading := normalize(vecSub(frontPos, simRearPos))

		sample := TrajectorySample{
			Time:     float32(stepIndex) * step,
			Position: center,
			Heading:  bodyHeading,
			Priority: spline.Priority,
			SplineID: spline.ID,
		}

		if simCar.Trailer.HasTrailer {
			hitchPos := simRearPos
			trailerRearToHitch := vecSub(hitchPos, simTrailerRearPos)
			if vectorLengthSq(trailerRearToHitch) > 1e-9 {
				simTrailerRearPos = vecSub(hitchPos, vecScale(normalize(trailerRearToHitch), simCar.Trailer.Length*wheelbaseFrac))
			} else {
				simTrailerRearPos = vecSub(hitchPos, vecScale(splineTangent, simCar.Trailer.Length*wheelbaseFrac))
			}
			trailerCenter := vecScale(vecAdd(hitchPos, simTrailerRearPos), 0.5)
			trailerHeading := normalize(vecSub(hitchPos, simTrailerRearPos))
			sample.HasTrailer = true
			sample.TrailerPosition = trailerCenter
			sample.TrailerHeading = trailerHeading
		}

		samples = append(samples, sample)

		if !active {
			break
		}
		if stepIndex == steps || speed <= 0.01 {
			continue
		}

		move := speed * step
		for move > 0 && active {
			currentSpline, ok := graph.splineByID(simCar.CurrentSplineID)
			if !ok {
				active = false
				break
			}
			remaining := currentSpline.Length - simCar.DistanceOnSpline
			if move <= remaining {
				simCar.DistanceOnSpline += move
				move = 0
				break
			}

			overshoot := move - remaining
			move = overshoot

			if simCar.CurrentSplineID == simCar.DestinationSplineID {
				simCar.DistanceOnSpline = currentSpline.Length
				active = false
				break
			}

			simCar.DistanceOnSpline = 0

			if simCar.LaneChanging && simCar.CurrentSplineID == simCar.LaneChangeSplineID {
				simCar.PrevSplineIDs[1] = simCar.PrevSplineIDs[0]
				simCar.PrevSplineIDs[0] = simCar.CurrentSplineID
				simCar.CurrentSplineID = simCar.AfterSplineID
				simCar.DistanceOnSpline = simCar.AfterSplineDist
				simCar.LaneChanging = false
				simCar.LaneChangeSplineID = -1
				continue
			}

			nextSplineID, ok := ChooseNextSplineOnBestPathWithGraph(graph, simCar.CurrentSplineID, simCar.DestinationSplineID, simCar.VehicleKind)
			if !ok {
				forcedNext, _, forcedOk := FindForcedLaneChangePathWithGraph(graph, simCar.CurrentSplineID, simCar.DestinationSplineID, simCar.VehicleKind)
				if !forcedOk {
					active = false
					break
				}
				nextSplineID = forcedNext
			}
			simCar.CurrentSplineID = nextSplineID
		}
	}

	return samples
}

func predictCollision(aSamples, bSamples []TrajectorySample, geomA, geomB collisionGeometry) (CollisionPrediction, bool) {
	count := len(aSamples)
	if len(bSamples) < count {
		count = len(bSamples)
	}
	if count == 0 {
		return CollisionPrediction{}, false
	}

	checkCircleGroups := func(cenA, hA Vec2, offsA []float32, rA float32, cenB, hB Vec2, offsB []float32, rB float32) bool {
		threshSq := (rA + rB) * (rA + rB)
		for _, offA := range offsA {
			cA := vecAdd(cenA, vecScale(hA, offA))
			for _, offB := range offsB {
				if distSq(cA, vecAdd(cenB, vecScale(hB, offB))) <= threshSq {
					return true
				}
			}
		}
		return false
	}

	coarseDistSq := (geomA.coarseRadius + geomB.coarseRadius) * (geomA.coarseRadius + geomB.coarseRadius)

	for i := 0; i < count; i++ {
		pA, hA := aSamples[i].Position, aSamples[i].Heading
		pB, hB := bSamples[i].Position, bSamples[i].Heading
		if distSq(pA, pB) > coarseDistSq {
			continue
		}

		collides := checkCircleGroups(pA, hA, geomA.bodyOffsets, geomA.bodyRadius, pB, hB, geomB.bodyOffsets, geomB.bodyRadius)
		if !collides && aSamples[i].HasTrailer {
			collides = checkCircleGroups(aSamples[i].TrailerPosition, aSamples[i].TrailerHeading, geomA.trailerOffsets, geomA.trailerRadius, pB, hB, geomB.bodyOffsets, geomB.bodyRadius)
		}
		if !collides && bSamples[i].HasTrailer {
			collides = checkCircleGroups(pA, hA, geomA.bodyOffsets, geomA.bodyRadius, bSamples[i].TrailerPosition, bSamples[i].TrailerHeading, geomB.trailerOffsets, geomB.trailerRadius)
		}
		if !collides && aSamples[i].HasTrailer && bSamples[i].HasTrailer {
			collides = checkCircleGroups(aSamples[i].TrailerPosition, aSamples[i].TrailerHeading, geomA.trailerOffsets, geomA.trailerRadius, bSamples[i].TrailerPosition, bSamples[i].TrailerHeading, geomB.trailerOffsets, geomB.trailerRadius)
		}
		if !collides {
			continue
		}
		prevIndex := i - 1
		if prevIndex < 0 {
			prevIndex = 0
		}
		return CollisionPrediction{
			Time:            aSamples[i].Time,
			PosA:            pA,
			PosB:            pB,
			PrevPosA:        aSamples[prevIndex].Position,
			PrevPosB:        bSamples[prevIndex].Position,
			HeadingA:        hA,
			HeadingB:        hB,
			AlreadyCollided: i == 0,
			PriorityA:       aSamples[i].Priority,
			PriorityB:       bSamples[i].Priority,
			SplineAID:       aSamples[i].SplineID,
			SplineBID:       bSamples[i].SplineID,
		}, true
	}

	return CollisionPrediction{}, false
}

func determineBlame(collision CollisionPrediction, carA, carB Car, splines []Spline) (bool, bool) {
	if collision.AlreadyCollided {
		if headingAngleDegrees(collision.HeadingA, collision.HeadingB) >= 60 {
			return false, false
		}
		return blameRearCar(collision, carA, carB)
	}
	if collision.PriorityA != collision.PriorityB {
		if collision.PriorityA {
			if normalCarOccupiesPriorityCollisionSpline(carB, collision.SplineAID, splines) {
				return true, false
			}
			return false, true
		}
		if normalCarOccupiesPriorityCollisionSpline(carA, collision.SplineBID, splines) {
			return false, true
		}
		return true, false
	}

	angleDeg := headingAngleDegrees(collision.HeadingA, collision.HeadingB)
	if angleDeg < blameAngleThresholdDeg {
		return blameRearCar(collision, carA, carB)
	}
	return blameLeftCar(collision, carA, carB)
}

func blameRearCar(collision CollisionPrediction, carA, carB Car) (bool, bool) {
	referenceHeading := normalize(vecAdd(collision.HeadingA, collision.HeadingB))
	if vectorLengthSq(referenceHeading) <= 1e-6 {
		referenceHeading = normalize(collision.HeadingA)
	}

	relPrev := vecSub(collision.PrevPosA, collision.PrevPosB)
	projection := dot(relPrev, referenceHeading)
	if absf(projection) < 1e-4 {
		projection = dot(vecSub(collision.PosA, collision.PosB), referenceHeading)
	}
	if absf(projection) < 1e-4 {
		if carA.Speed > carB.Speed {
			return true, false
		}
		if carB.Speed > carA.Speed {
			return false, true
		}
		if carA.RouteID <= carB.RouteID {
			return true, false
		}
		return false, true
	}

	if projection < 0 {
		return true, false
	}
	return false, true
}

func blameLeftCar(collision CollisionPrediction, carA, carB Car) (bool, bool) {
	relToBFromA := vecSub(collision.PrevPosB, collision.PrevPosA)
	relToAFromB := vecSub(collision.PrevPosA, collision.PrevPosB)
	if vectorLengthSq(relToBFromA) <= 1e-6 || vectorLengthSq(relToAFromB) <= 1e-6 {
		relToBFromA = vecSub(collision.PosB, collision.PosA)
		relToAFromB = vecSub(collision.PosA, collision.PosB)
	}

	bearingToBFromA := signedAngleDegrees(collision.HeadingA, relToBFromA)
	bearingToAFromB := signedAngleDegrees(collision.HeadingB, relToAFromB)
	const sideEps float32 = 1.0

	if bearingToBFromA < -sideEps && bearingToAFromB > sideEps {
		return false, true
	}
	if bearingToAFromB < -sideEps && bearingToBFromA > sideEps {
		return true, false
	}

	crossHeading := cross2D(collision.HeadingA, collision.HeadingB)
	if crossHeading < 0 {
		return true, false
	}
	if crossHeading > 0 {
		return false, true
	}

	if carA.RouteID <= carB.RouteID {
		return false, true
	}
	return true, false
}

func normalCarOccupiesPriorityCollisionSpline(car Car, prioritySplineID int, splines []Spline) bool {
	if prioritySplineID < 0 {
		return false
	}
	prioritySpline, ok := findSplineByID(splines, prioritySplineID)
	if !ok || !prioritySpline.Priority {
		return false
	}
	return carHitboxTouchesSpline(car, prioritySpline, splines)
}

func carHitboxTouchesSpline(car Car, targetSpline Spline, splines []Spline) bool {
	currentSpline, ok := findSplineByID(splines, car.CurrentSplineID)
	if !ok {
		return false
	}
	splinePos3, splineTangent := sampleSplineAtDistance(currentSpline, car.DistanceOnSpline)
	rightNormal3 := Vec2{X: splineTangent.Y, Y: -splineTangent.X}
	frontPos := vecAdd(splinePos3, vecScale(rightNormal3, car.LateralOffset))
	bodyHeading := normalize(vecSub(frontPos, car.RearPosition))
	if vectorLengthSq(vecSub(frontPos, car.RearPosition)) <= 1e-9 {
		bodyHeading = splineTangent
	}
	center := vecScale(vecAdd(frontPos, car.RearPosition), 0.5)
	radius := collisionRadius(car)
	radiusSq := radius * radius
	for _, offset := range collisionCircleOffsets(car) {
		circle := vecAdd(center, vecScale(bodyHeading, offset))
		nearest := nearestSampleOnSpline(targetSpline, circle)
		if distSq(circle, nearest) <= radiusSq {
			return true
		}
	}
	return false
}

func buildLookaheadSplineSets(cars []Car, graph *RoadGraph, lookahead float32) []map[int]bool {
	pathSets := make([]map[int]bool, len(cars))
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			car := cars[i]
			set := map[int]bool{car.CurrentSplineID: true}
			currentSpline, ok := graph.splineByID(car.CurrentSplineID)
			if !ok {
				pathSets[i] = set
				continue
			}
			covered := currentSpline.Length - car.DistanceOnSpline
			curID := car.CurrentSplineID
			for steps := 0; covered < lookahead && steps < len(graph.splines); steps++ {
				nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, curID, car.DestinationSplineID, car.VehicleKind)
				if !ok {
					break
				}
				set[nextID] = true
				nextSpline, ok := graph.splineByID(nextID)
				if !ok {
					break
				}
				covered += nextSpline.Length
				curID = nextID
			}
			pathSets[i] = set
		}
	})
	return pathSets
}

func computeFollowingSpeedCaps(cars []Car, graph *RoadGraph) []float32 {
	caps := make([]float32, len(cars))
	for i := range caps {
		caps[i] = math.MaxFloat32
	}

	poses := make([]carPose, len(cars))
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			car := cars[i]
			splineIdx, ok := graph.indexByID[car.CurrentSplineID]
			if !ok {
				continue
			}
			p, h := sampleSplineAtDistance(graph.splines[splineIdx], car.DistanceOnSpline)
			poses[i] = carPose{p, h}
		}
	})

	pathSets := buildLookaheadSplineSets(cars, graph, followLookaheadM)

	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			car := cars[i]
			hI := poses[i].heading
			pI := poses[i].pos
			desiredGap := followMinGapM + car.Speed*followTimeHeadwaySecs
			bestDist := float32(math.MaxFloat32)
			bestSpeed := float32(math.MaxFloat32)

			for j, other := range cars {
				if i == j {
					continue
				}
				if !pathSets[i][other.CurrentSplineID] {
					continue
				}
				hJ := poses[j].heading
				d := hI.X*hJ.X + hI.Y*hJ.Y
				if d < followHeadingCos {
					continue
				}
				diff := Vec2{X: poses[j].pos.X - pI.X, Y: poses[j].pos.Y - pI.Y}
				proj := diff.X*hI.X + diff.Y*hI.Y
				if proj <= 0 {
					continue
				}
				euclidean := float32(math.Sqrt(float64(diff.X*diff.X + diff.Y*diff.Y)))
				if euclidean > followLookaheadM {
					continue
				}
				leaderRear := other.RearPosition
				if other.Trailer.HasTrailer {
					leaderRear = other.Trailer.RearPosition
				}
				rearDiff := vecSub(leaderRear, pI)
				gap := float32(math.Sqrt(float64(rearDiff.X*rearDiff.X + rearDiff.Y*rearDiff.Y)))
				if gap > desiredGap {
					continue
				}
				if euclidean < bestDist {
					bestDist = euclidean
					bestSpeed = other.Speed
				}
			}
			caps[i] = bestSpeed
		}
	})
	return caps
}

func updateCars(cars []Car, routes []Route, graph *RoadGraph, brakingDecisions []bool, holdSpeedDecisions []bool, followCaps []float32, lights []TrafficLight, cycles []TrafficCycle, dt float32) ([]Car, []int) {
	if len(cars) == 0 {
		return cars, nil
	}

	routeIndexByID := map[int]int{}
	for i, route := range routes {
		routeIndexByID[route.ID] = i
	}

	indexRemap := make([]int, len(cars))
	for i := range indexRemap {
		indexRemap[i] = -1
	}
	alive := cars[:0]
	for i, car := range cars {
		routeIdx, ok := routeIndexByID[car.RouteID]
		if !ok || !routes[routeIdx].Valid {
			continue
		}
		route := routes[routeIdx]
		if route.VehicleKind == VehicleBus && car.DestinationSplineID <= 0 {
			car.DestinationSplineID = CurrentRouteTarget(route, car.NextBusStopIndex)
		}

		if route.VehicleKind == VehicleBus && car.BusStopTimer > 0 {
			car.BusStopTimer -= dt
			if car.BusStopTimer < 0 {
				car.BusStopTimer = 0
			}
			car.Speed = 0
			car.Braking = false
			car.SoftSlowing = false
			indexRemap[i] = len(alive)
			alive = append(alive, car)
			continue
		}

		shouldBrake := i < len(brakingDecisions) && brakingDecisions[i]
		shouldHoldSpeed := !shouldBrake && i < len(holdSpeedDecisions) && holdSpeedDecisions[i]
		car.Braking = shouldBrake

		followCap := float32(math.MaxFloat32)
		if i < len(followCaps) {
			followCap = followCaps[i]
		}
		car.SoftSlowing = !shouldBrake && followCap < float32(math.MaxFloat32)

		const frustrateThreshMPS = 10.0 / 3.6
		if !shouldBrake && followCap < float32(math.MaxFloat32) {
			frustrated := false
			if spline, ok := graph.splineByID(car.CurrentSplineID); ok {
				preferredSpeed := car.MaxSpeed * spline.SpeedFactor
				if spline.SpeedLimitKmh > 0 {
					if limitMPS := spline.SpeedLimitKmh / 3.6; limitMPS < preferredSpeed {
						preferredSpeed = limitMPS
					}
				}
				if cs := lookupCurveSpeed(spline, car.DistanceOnSpline) * car.CurveSpeedMultiplier; cs < preferredSpeed {
					preferredSpeed = cs
				}
				frustrated = preferredSpeed-car.Speed >= frustrateThreshMPS
			}
			if frustrated {
				car.SlowedTimer += dt
			} else {
				car.SlowedTimer = 0
			}
		} else {
			car.SlowedTimer = 0
		}

		for {
			currentSpline, ok := graph.splineByID(car.CurrentSplineID)
			if !ok {
				break
			}
			if shouldBeginBusStopDwell(car, route, currentSpline, graph) {
				beginBusStopDwell(route, &car)
				indexRemap[i] = len(alive)
				alive = append(alive, car)
				break
			}

			targetSpeed := car.MaxSpeed * currentSpline.SpeedFactor
			if currentSpline.SpeedLimitKmh > 0 {
				if limitMPS := currentSpline.SpeedLimitKmh / 3.6; limitMPS < targetSpeed {
					targetSpeed = limitMPS
				}
			}
			if cs := lookupCurveSpeed(currentSpline, car.DistanceOnSpline) * car.CurveSpeedMultiplier; cs < targetSpeed {
				targetSpeed = cs
			}
			if at := computeAnticipatoryTargetSpeed(car, currentSpline, graph); at < targetSpeed {
				targetSpeed = at
			}
			if tl := computeTrafficLightSpeedCap(car, currentSpline, graph, lights, cycles); tl < targetSpeed {
				targetSpeed = tl
			}
			if bs := computeBusStopSpeedCap(car, route, currentSpline, graph); bs < targetSpeed {
				targetSpeed = bs
			}
			if car.DesiredLaneSplineID >= 0 {
				remaining := car.DesiredLaneDeadline - car.DistanceOnSpline
				if remaining >= 0 && car.Speed > laneChangeForcedSpeedMPS {
					normalDecel := car.Accel * 1.5 * 0.9
					brakingDist := (car.Speed*car.Speed - laneChangeForcedSpeedMPS*laneChangeForcedSpeedMPS) / (2 * normalDecel)
					if remaining <= brakingDist && targetSpeed > laneChangeForcedSpeedMPS {
						targetSpeed = laneChangeForcedSpeedMPS
					}
				}
			}
			if car.Braking {
				targetSpeed = 0
			} else if followCap < targetSpeed {
				targetSpeed = followCap
			}
			if shouldHoldSpeed && targetSpeed > car.Speed {
				targetSpeed = car.Speed
			}

			if car.Speed < targetSpeed {
				car.Speed += car.Accel * dt
				if car.Speed > targetSpeed {
					car.Speed = targetSpeed
				}
			} else {
				decel := car.Accel * 1.5
				if car.Braking {
					decel = car.Accel * brakeDecelMultiplier
				}
				car.Speed -= decel * dt
				if car.Speed < targetSpeed {
					car.Speed = targetSpeed
				}
				if car.Speed < 0 {
					car.Speed = 0
				}
			}

			car.DistanceOnSpline += car.Speed * dt
			if car.DistanceOnSpline <= currentSpline.Length {
				κ := signedCurvatureAtArcLen(&currentSpline, car.DistanceOnSpline)
				wb := car.Length * wheelbaseFrac
				targetOffset := κ * wb * wb / 6
				if car.Trailer.HasTrailer {
					trailerWb := car.Trailer.Length * wheelbaseFrac
					targetOffset = κ * (wb*wb + trailerWb*trailerWb) / 6
				}
				lerpRate := car.Speed / (2 * (wb + 0.01))
				car.LateralOffset += (targetOffset - car.LateralOffset) * clampf(lerpRate*dt, 0, 1)

				splinePos, tangent := sampleSplineAtDistance(currentSpline, car.DistanceOnSpline)
				rightNormal := Vec2{X: tangent.Y, Y: -tangent.X}
				frontPos := vecAdd(splinePos, vecScale(rightNormal, car.LateralOffset))
				rearToFront := vecSub(frontPos, car.RearPosition)
				if vectorLengthSq(rearToFront) > 1e-9 {
					car.RearPosition = vecSub(frontPos, vecScale(normalize(rearToFront), car.Length*wheelbaseFrac))
				}
				if car.Trailer.HasTrailer {
					hitchPos := car.RearPosition
					trailerRearToHitch := vecSub(hitchPos, car.Trailer.RearPosition)
					if vectorLengthSq(trailerRearToHitch) > 1e-9 {
						car.Trailer.RearPosition = vecSub(hitchPos, vecScale(normalize(trailerRearToHitch), car.Trailer.Length*wheelbaseFrac))
					}
				}
				if shouldBeginBusStopDwell(car, route, currentSpline, graph) {
					beginBusStopDwell(route, &car)
				}
				indexRemap[i] = len(alive)
				alive = append(alive, car)
				break
			}

			overshoot := car.DistanceOnSpline - currentSpline.Length
			car.DistanceOnSpline = overshoot
			if car.CurrentSplineID == car.DestinationSplineID {
				break
			}

			if car.LaneChanging && car.CurrentSplineID == car.LaneChangeSplineID {
				car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
				car.PrevSplineIDs[0] = car.CurrentSplineID
				car.CurrentSplineID = car.AfterSplineID
				car.DistanceOnSpline = car.AfterSplineDist + overshoot
				car.LaneChanging = false
				car.LaneChangeSplineID = -1
				if car.CurrentSplineID == car.DesiredLaneSplineID {
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
				continue
			}

			nextSplineID, ok := ChooseNextSplineOnBestPathWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			if ok {
				if car.DesiredLaneSplineID >= 0 {
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
			} else {
				forcedNext, desiredLane, forcedOk := FindForcedLaneChangePathWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
				if !forcedOk {
					break
				}
				nextSplineID = forcedNext
				car.DesiredLaneSplineID = desiredLane
				if src, srcOk := graph.splineByID(forcedNext); srcOk {
					car.DesiredLaneDeadline = maxf(src.Length-laneChangeForcedDistEnd, 0)
				}
			}
			car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
			car.PrevSplineIDs[0] = car.CurrentSplineID
			car.CurrentSplineID = nextSplineID
			resumeBusRouteAfterStop(route, &car)

			if car.DesiredLaneSplineID < 0 && car.CurrentSplineID != car.DestinationSplineID {
				if newSpline, scOk := graph.splineByID(car.CurrentSplineID); scOk && len(newSpline.HardCoupledIDs) > 0 {
					_, curTime, curOk := FindShortestPathWeightedWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
					if curOk {
						for _, cid := range newSpline.HardCoupledIDs {
							_, coupledTime, coupledOk := FindShortestPathWeightedWithGraph(graph, cid, car.DestinationSplineID, car.VehicleKind)
							if coupledOk && curTime-coupledTime >= 20.0 {
								car.DesiredLaneSplineID = cid
								car.DesiredLaneDeadline = maxf(newSpline.Length-laneChangeForcedDistEnd, 0)
								break
							}
						}
					}
				}
			}
		}
	}
	return alive, indexRemap
}

func laneChangeFeasibleAt(src, dst Spline, distance, speed float32) bool {
	carPos, carHeading := sampleSplineAtDistance(src, distance)
	halfDist := speed * laneChangeHalfSecs
	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(dst, p1)
	if crossDist == 0 || dst.Length-crossDist < halfDist {
		return false
	}
	_, destHeading := sampleSplineAtDistance(dst, crossDist+halfDist)
	return carHeading.X*destHeading.X+carHeading.Y*destHeading.Y >= laneChangeDirCos
}

func EffectiveMaxSpeedMPS(spline Spline) float32 {
	if spline.SpeedLimitKmh > 0 {
		return minf(maxCarSpeed*spline.SpeedFactor, spline.SpeedLimitKmh/3.6)
	}
	return maxCarSpeed * spline.SpeedFactor
}

func FindForcedLaneChangePathWithGraph(graph *RoadGraph, currentSplineID, destSplineID int, vehicleKind VehicleKind) (nextSplineID, desiredSplineID int, ok bool) {
	currentSpline, found := graph.splineByID(currentSplineID)
	if !found {
		return 0, 0, false
	}
	for _, nextIdx := range graph.startsByNode[pointKey(currentSpline.P3)] {
		nextSpline := graph.splines[nextIdx]
		if !isSplineUsableForVehicle(nextSpline, vehicleKind) {
			continue
		}
		for _, coupledID := range nextSpline.HardCoupledIDs {
			if coupledSpline, ok := graph.splineByID(coupledID); ok && !isSplineUsableForVehicle(coupledSpline, vehicleKind) {
				continue
			}
			if _, _, pathOk := FindShortestPathWeightedWithGraph(graph, coupledID, destSplineID, vehicleKind); pathOk {
				return nextSpline.ID, coupledID, true
			}
		}
	}
	return 0, 0, false
}

func laneChangeLandingDist(car Car, srcSpline, destSpline Spline) (float32, bool) {
	if car.Speed < laneChangeMinSpeed {
		return 0, false
	}
	carPos, carHeading := sampleSplineAtDistance(srcSpline, car.DistanceOnSpline)
	halfDist := car.Speed * laneChangeHalfSecs
	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(destSpline, p1)
	if crossDist == 0 || destSpline.Length-crossDist < halfDist {
		return 0, false
	}
	p3Dist := crossDist + halfDist
	_, destHeading := sampleSplineAtDistance(destSpline, p3Dist)
	if carHeading.X*destHeading.X+carHeading.Y*destHeading.Y < laneChangeDirCos {
		return 0, false
	}
	return p3Dist, true
}

func isLaneChangeLandingSafe(p3Dist float32, destSplineID int, switchingCar Car, cars []Car) bool {
	T := 2 * laneChangeHalfSecs
	const safetyMargin = 2.0
	for _, other := range cars {
		if other.CurrentSplineID != destSplineID {
			continue
		}
		otherFrontAtLanding := other.DistanceOnSpline + other.Speed*T
		gapAtLanding := otherFrontAtLanding - p3Dist
		if gapAtLanding >= 0 {
			if gapAtLanding < other.Length+safetyMargin {
				return false
			}
		} else {
			if -gapAtLanding < switchingCar.Length+safetyMargin {
				return false
			}
		}
	}
	return true
}

func buildLaneChangeBridge(car *Car, destSplineID int, splines []Spline, splineIndexByID map[int]int, lcs []Spline, nextID *int, forced bool) ([]Spline, bool) {
	srcIdx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return lcs, false
	}
	destIdx, ok := splineIndexByID[destSplineID]
	if !ok {
		return lcs, false
	}
	if !forced && car.Speed < laneChangeMinSpeed {
		return lcs, false
	}

	srcSpline := splines[srcIdx]
	destSpline := splines[destIdx]
	carPos, carHeading := sampleSplineAtDistance(srcSpline, car.DistanceOnSpline)
	effectiveSpeed := car.Speed
	if forced && effectiveSpeed < laneChangeMinSpeed {
		effectiveSpeed = laneChangeMinSpeed
	}
	halfDist := effectiveSpeed * laneChangeHalfSecs

	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(destSpline, p1)
	if crossDist == 0 || destSpline.Length-crossDist < halfDist {
		return lcs, false
	}
	p3Dist := crossDist + halfDist
	p3, destHeading := sampleSplineAtDistance(destSpline, p3Dist)
	if carHeading.X*destHeading.X+carHeading.Y*destHeading.Y < laneChangeDirCos {
		return lcs, false
	}

	p2 := vecSub(p3, vecScale(destHeading, halfDist))
	id := *nextID
	*nextID++
	lcs = append(lcs, NewSpline(id, carPos, p1, p2, p3))

	car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
	car.PrevSplineIDs[0] = car.CurrentSplineID
	car.CurrentSplineID = id
	car.DistanceOnSpline = 0
	car.LaneChanging = true
	car.LaneChangeSplineID = id
	car.AfterSplineID = destSplineID
	car.AfterSplineDist = p3Dist
	return lcs, true
}

func computeLaneChanges(cars []Car, splines []Spline, lcs []Spline, nextID *int, graph *RoadGraph, dt float32) ([]Spline, []Car) {
	splineIndexByID := graph.indexByID
	poses := make([]carPose, len(cars))
	carsBySpline := make(map[int][]int, len(cars))
	for i, car := range cars {
		if splineIdx, ok := splineIndexByID[car.CurrentSplineID]; ok {
			pos, heading := sampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
			poses[i] = carPose{pos: pos, heading: heading}
		}
		carsBySpline[car.CurrentSplineID] = append(carsBySpline[car.CurrentSplineID], i)
	}

	for i := range cars {
		car := &cars[i]
		if car.LaneChanging {
			continue
		}

		if car.DesiredLaneSplineID >= 0 {
			if car.DistanceOnSpline >= car.DesiredLaneDeadline {
				if newLcs, ok := buildLaneChangeBridge(car, car.DesiredLaneSplineID, splines, splineIndexByID, lcs, nextID, true); ok {
					lcs = newLcs
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
			} else {
				srcIdx, srcOk := splineIndexByID[car.CurrentSplineID]
				destIdx, destOk := splineIndexByID[car.DesiredLaneSplineID]
				if srcOk && destOk {
					if p3Dist, feasible := laneChangeLandingDist(*car, splines[srcIdx], splines[destIdx]); feasible {
						if isLaneChangeLandingSafe(p3Dist, car.DesiredLaneSplineID, *car, cars) {
							if newLcs, ok := buildLaneChangeBridge(car, car.DesiredLaneSplineID, splines, splineIndexByID, lcs, nextID, false); ok {
								lcs = newLcs
								car.DesiredLaneSplineID = -1
								car.DesiredLaneDeadline = 0
							}
						}
					}
				}
			}
			continue
		}

		car.PreferenceCooldown -= dt
		if car.PreferenceCooldown <= 0 {
			car.PreferenceCooldown = preferenceChangeCooldownS
			_, curPrefTime, curPrefTimeOk := FindShortestPathWeightedWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			for _, destID := range findBetterPreferenceLaneCandidates(*car, splines, splineIndexByID) {
				_, destTime, pathOk := FindShortestPathWeightedWithGraph(graph, destID, car.DestinationSplineID, car.VehicleKind)
				if !pathOk {
					continue
				}
				if curPrefTimeOk && destTime-curPrefTime > 20.0 {
					continue
				}
				srcIdx, srcOk := splineIndexByID[car.CurrentSplineID]
				destIdx, destOk := splineIndexByID[destID]
				if !srcOk || !destOk {
					continue
				}
				p3Dist, feasible := laneChangeLandingDist(*car, splines[srcIdx], splines[destIdx])
				if !feasible || !isLaneChangeLandingSafe(p3Dist, destID, *car, cars) {
					continue
				}
				if newLcs, ok := buildLaneChangeBridge(car, destID, splines, splineIndexByID, lcs, nextID, false); ok {
					lcs = newLcs
					break
				}
			}
		}

		car.OvertakeCooldown -= dt
		leaderSpeed, leaderFound := nearestLeaderSpeed(i, cars, carsBySpline, poses)
		if car.SlowedTimer > overtakeSlowThresholdS && car.OvertakeCooldown <= 0 && (!leaderFound || leaderSpeed <= car.Speed) {
			car.OvertakeCooldown = overtakeCooldownS
			_, curOvertakeTime, curOvertakeTimeOk := FindShortestPathWeightedWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			for _, destID := range findOvertakeLaneCandidates(*car, splines, splineIndexByID) {
				_, destTime, pathOk := FindShortestPathWeightedWithGraph(graph, destID, car.DestinationSplineID, car.VehicleKind)
				if !pathOk {
					continue
				}
				if curOvertakeTimeOk && destTime-curOvertakeTime > 20.0 {
					continue
				}
				srcIdx, srcOk := splineIndexByID[car.CurrentSplineID]
				destIdx, destOk := splineIndexByID[destID]
				if !srcOk || !destOk {
					continue
				}
				p3Dist, feasible := laneChangeLandingDist(*car, splines[srcIdx], splines[destIdx])
				if !feasible || !isLaneChangeLandingSafe(p3Dist, destID, *car, cars) {
					continue
				}
				if newLcs, ok := buildLaneChangeBridge(car, destID, splines, splineIndexByID, lcs, nextID, false); ok {
					lcs = newLcs
					car.PreferenceCooldown = preferenceChangeCooldownS
					car.SlowedTimer = 0
					break
				}
			}
		}
	}

	return lcs, cars
}

func findBetterPreferenceLaneCandidates(car Car, splines []Spline, splineIndexByID map[int]int) []int {
	idx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return nil
	}
	currentSpline := splines[idx]
	currentPref := currentSpline.LanePreference

	allCoupled := append(append([]int(nil), currentSpline.HardCoupledIDs...), currentSpline.SoftCoupledIDs...)
	type candidate struct {
		id   int
		pref int
	}
	candidates := make([]candidate, 0, len(allCoupled))
	for _, coupledID := range allCoupled {
		cIdx, cOk := splineIndexByID[coupledID]
		if !cOk {
			continue
		}
		coupled := splines[cIdx]
		if coupled.LanePreference <= 0 {
			continue
		}
		if currentPref != 0 && coupled.LanePreference >= currentPref {
			continue
		}
		candidates = append(candidates, candidate{id: coupledID, pref: coupled.LanePreference})
	}
	sort.SliceStable(candidates, func(i, j int) bool {
		return candidates[i].pref < candidates[j].pref
	})
	result := make([]int, 0, len(candidates))
	for _, candidate := range candidates {
		result = append(result, candidate.id)
	}
	return result
}

func nearestLeaderSpeed(carIdx int, cars []Car, carsBySpline map[int][]int, poses []carPose) (float32, bool) {
	car := cars[carIdx]
	if carIdx < 0 || carIdx >= len(poses) {
		return 0, false
	}
	carPos, carHeading := poses[carIdx].pos, poses[carIdx].heading

	bestDist := float32(math.MaxFloat32)
	bestSpeed := float32(0)
	found := false
	for _, j := range carsBySpline[car.CurrentSplineID] {
		if j == carIdx {
			continue
		}
		other := cars[j]
		otherPos := poses[j].pos
		diff := Vec2{X: otherPos.X - carPos.X, Y: otherPos.Y - carPos.Y}
		proj := diff.X*carHeading.X + diff.Y*carHeading.Y
		if proj <= 0 {
			continue
		}
		euclidean := float32(math.Sqrt(float64(diff.X*diff.X + diff.Y*diff.Y)))
		if euclidean > followLookaheadM {
			continue
		}
		if euclidean < bestDist {
			bestDist = euclidean
			bestSpeed = other.Speed
			found = true
		}
	}
	return bestSpeed, found
}

func findOvertakeLaneCandidates(car Car, splines []Spline, splineIndexByID map[int]int) []int {
	idx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return nil
	}
	currentSpline := splines[idx]
	currentPref := currentSpline.LanePreference
	if currentPref <= 0 {
		return nil
	}
	targetPref := currentPref + 1
	allCoupled := append(append([]int(nil), currentSpline.HardCoupledIDs...), currentSpline.SoftCoupledIDs...)
	candidates := make([]int, 0, len(allCoupled))
	for _, coupledID := range allCoupled {
		cIdx, cOk := splineIndexByID[coupledID]
		if !cOk {
			continue
		}
		if splines[cIdx].LanePreference == targetPref {
			candidates = append(candidates, coupledID)
		}
	}
	return candidates
}

func cacheSpline(s *Spline) {
	s.Samples[0] = s.P0
	s.CumLen[0] = 0
	total := float32(0)
	prev := s.P0
	for i := 1; i <= simSamples; i++ {
		t := float32(i) / float32(simSamples)
		pt := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)
		total += float32(math.Sqrt(float64(distSq(prev, pt))))
		s.Samples[i] = pt
		s.CumLen[i] = total
		prev = pt
	}
	s.Length = total
	s.SpeedFactor = 1.0
	s.CurveSpeedMPS = buildCurveSpeedProfile(s)
	s.TravelTimeS = buildSplineTravelTime(s)
}

func buildCurveSpeedProfile(s *Spline) []float32 {
	if s.Length <= 0 {
		return nil
	}
	limitMPS := float32(math.MaxFloat32)
	if s.SpeedLimitKmh > 0 {
		limitMPS = s.SpeedLimitKmh / 3.6
	}
	count := int(s.Length/curveSpeedIntervalM) + 1
	profile := make([]float32, count)
	for i := range profile {
		d := float32(i) * curveSpeedIntervalM
		v := curveSpeedAtArcLen(s, d)
		if v > limitMPS {
			v = limitMPS
		}
		profile[i] = v
	}
	return profile
}

func buildSplineTravelTime(s *Spline) float32 {
	if s.Length <= 0 {
		return 0
	}
	if len(s.CurveSpeedMPS) == 0 {
		speed := maxCarSpeed * s.SpeedFactor
		if speed <= 1e-3 {
			return float32(math.MaxFloat32)
		}
		return s.Length / speed
	}

	totalTime := float32(0)
	remaining := s.Length
	for _, fragSpeed := range s.CurveSpeedMPS {
		if remaining <= 0 {
			break
		}
		fragLen := minf(curveSpeedIntervalM, remaining)
		speed := fragSpeed * s.SpeedFactor
		if speed <= 1e-3 {
			return float32(math.MaxFloat32)
		}
		totalTime += fragLen / speed
		remaining -= fragLen
	}
	if remaining > 0 {
		speed := s.CurveSpeedMPS[len(s.CurveSpeedMPS)-1] * s.SpeedFactor
		if speed <= 1e-3 {
			return float32(math.MaxFloat32)
		}
		totalTime += remaining / speed
	}
	return totalTime
}

func curveSpeedAtArcLen(s *Spline, d float32) float32 {
	idx := 0
	for idx < simSamples && s.CumLen[idx+1] < d {
		idx++
	}
	i0, i1, i2 := idx-1, idx, idx+1
	if i0 < 0 {
		i0, i1, i2 = 0, 1, 2
	}
	if i2 > simSamples {
		i0, i1, i2 = simSamples-2, simSamples-1, simSamples
	}
	A, B, C := s.Samples[i0], s.Samples[i1], s.Samples[i2]
	ab := float32(math.Sqrt(float64(distSq(A, B))))
	bc := float32(math.Sqrt(float64(distSq(B, C))))
	ca := float32(math.Sqrt(float64(distSq(C, A))))
	cross := absf((B.X-A.X)*(C.Y-A.Y) - (B.Y-A.Y)*(C.X-A.X))
	if cross < 1e-6 {
		return maxCarSpeed
	}
	r := ab * bc * ca / (2 * cross)
	v := float32(math.Sqrt(float64(maxLateralAccelMPS2 * r)))
	if v > maxCarSpeed {
		return maxCarSpeed
	}
	return v
}

func signedCurvatureAtArcLen(s *Spline, d float32) float32 {
	idx := 0
	for idx < simSamples && s.CumLen[idx+1] < d {
		idx++
	}
	i0, i1, i2 := idx-1, idx, idx+1
	if i0 < 0 {
		i0, i1, i2 = 0, 1, 2
	}
	if i2 > simSamples {
		i0, i1, i2 = simSamples-2, simSamples-1, simSamples
	}
	A, B, C := s.Samples[i0], s.Samples[i1], s.Samples[i2]
	ab := float32(math.Sqrt(float64(distSq(A, B))))
	bc := float32(math.Sqrt(float64(distSq(B, C))))
	ca := float32(math.Sqrt(float64(distSq(C, A))))
	denom := ab * bc * ca
	if denom < 1e-6 {
		return 0
	}
	cross := (B.X-A.X)*(C.Y-A.Y) - (B.Y-A.Y)*(C.X-A.X)
	return 2 * cross / denom
}

func lookupCurveSpeed(spline Spline, dist float32) float32 {
	if len(spline.CurveSpeedMPS) == 0 {
		return maxCarSpeed
	}
	idx := int(dist / curveSpeedIntervalM)
	if idx >= len(spline.CurveSpeedMPS) {
		idx = len(spline.CurveSpeedMPS) - 1
	}
	return spline.CurveSpeedMPS[idx]
}

func trafficLightShouldStop(l TrafficLight, cycles []TrafficCycle) bool {
	if l.CycleID < 0 {
		return false
	}
	for _, c := range cycles {
		if c.ID != l.CycleID {
			continue
		}
		if !c.Enabled {
			return false
		}
		state := TrafficLightState(l.ID, l.CycleID, cycles)
		return state == TrafficRed || state == TrafficYellow
	}
	return false
}

func computeTrafficLightSpeedCap(car Car, currentSpline Spline, graph *RoadGraph, lights []TrafficLight, cycles []TrafficCycle) float32 {
	decel := car.Accel * 1.5
	result := float32(math.MaxFloat32)
	remaining := currentSpline.Length - car.DistanceOnSpline
	lookahead := car.Speed*car.Speed/(2*decel) + 20
	if lookahead > 200 {
		lookahead = 200
	}

	checkLight := func(rawDistAhead float32) {
		adj := rawDistAhead - stopFrontGapM
		if adj <= 0 {
			result = 0
			return
		}
		allowed := float32(math.Sqrt(float64(2 * decel * adj)))
		if allowed < result {
			result = allowed
		}
	}

	for _, l := range lights {
		if l.SplineID != currentSpline.ID || l.DistOnSpline <= car.DistanceOnSpline || !trafficLightShouldStop(l, cycles) {
			continue
		}
		checkLight(l.DistOnSpline - car.DistanceOnSpline)
	}

	if remaining < lookahead {
		nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, car.DestinationSplineID, car.VehicleKind)
		if ok {
			if _, ok2 := graph.splineByID(nextID); ok2 {
				for _, l := range lights {
					if l.SplineID != nextID || !trafficLightShouldStop(l, cycles) {
						continue
					}
					checkLight(remaining + l.DistOnSpline)
				}
			}
		}
	}

	return result
}

func computeAnticipatoryTargetSpeed(car Car, currentSpline Spline, graph *RoadGraph) float32 {
	decel := car.Accel * 1.5
	result := float32(math.MaxFloat32)

	checkSpline := func(spline Spline, startDist, offset float32) {
		for d := startDist + curveSpeedIntervalM; d <= spline.Length; d += curveSpeedIntervalM {
			reqSpeed := lookupCurveSpeed(spline, d) * car.CurveSpeedMultiplier
			rawDistAhead := offset + (d - startDist)
			allowedNow := float32(math.Sqrt(float64(reqSpeed*reqSpeed + 2*decel*rawDistAhead)))
			if allowedNow < result {
				result = allowedNow
			}
		}
	}

	checkSpline(currentSpline, car.DistanceOnSpline, 0)

	remaining := currentSpline.Length - car.DistanceOnSpline
	if nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, car.DestinationSplineID, car.VehicleKind); ok {
		if nextSpline, ok2 := graph.splineByID(nextID); ok2 {
			checkSpline(nextSpline, 0, remaining)
		}
	}

	return result
}

func BuildVehicleCounts(cars []Car) map[int]int {
	counts := make(map[int]int, len(cars))
	for _, car := range cars {
		counts[car.CurrentSplineID]++
	}
	return counts
}

func isSplineUsableForVehicle(s Spline, vehicleKind VehicleKind) bool {
	return vehicleKind == VehicleBus || !s.BusOnly
}

func FindShortestPathWeightedWithGraph(graph *RoadGraph, startSplineID, endSplineID int, vehicleKind VehicleKind) ([]int, float32, bool) {
	if graph == nil || len(graph.splines) == 0 {
		return nil, 0, false
	}
	key := pathCacheKey{StartID: startSplineID, EndID: endSplineID, VehicleKind: vehicleKind}
	graph.pathCacheMu.RLock()
	cached, ok := graph.pathCache[key]
	graph.pathCacheMu.RUnlock()
	if ok {
		graph.pathCacheMu.Lock()
		graph.pathCacheHits++
		graph.pathCacheMu.Unlock()
		return append([]int(nil), cached.PathIDs...), cached.Cost, cached.OK
	}
	graph.pathCacheMu.Lock()
	graph.pathCacheMisses++
	graph.pathCacheMu.Unlock()
	startIdx, okStart := graph.indexByID[startSplineID]
	endIdx, okEnd := graph.indexByID[endSplineID]
	if !okStart || !okEnd {
		graph.pathCacheMu.Lock()
		graph.pathCache[key] = pathCacheEntry{OK: false}
		graph.pathCacheMu.Unlock()
		return nil, 0, false
	}
	if vehicleKind == VehicleCar && startIdx != endIdx && graph.splines[endIdx].BusOnly {
		graph.pathCacheMu.Lock()
		graph.pathCache[key] = pathCacheEntry{OK: false}
		graph.pathCacheMu.Unlock()
		return nil, 0, false
	}

	const inf = float32(1e30)
	dist := make([]float32, len(graph.splines))
	prev := make([]int, len(graph.splines))
	for i := range dist {
		dist[i] = inf
		prev[i] = -1
	}
	dist[startIdx] = graph.segmentCosts[startIdx]
	pq := dijkstraHeap{{idx: startIdx, dist: dist[startIdx]}}
	heap.Init(&pq)

	for pq.Len() > 0 {
		item := heap.Pop(&pq).(dijkstraItem)
		if item.dist != dist[item.idx] {
			continue
		}
		if item.idx == endIdx {
			break
		}
		for _, nextIdx := range graph.routeNeighbors[item.idx] {
			if vehicleKind == VehicleCar && graph.splines[nextIdx].BusOnly {
				continue
			}
			alt := item.dist + graph.segmentCosts[nextIdx]
			if alt < dist[nextIdx] {
				dist[nextIdx] = alt
				prev[nextIdx] = item.idx
				heap.Push(&pq, dijkstraItem{idx: nextIdx, dist: alt})
			}
		}
	}

	if dist[endIdx] >= inf/2 {
		graph.pathCacheMu.Lock()
		graph.pathCache[key] = pathCacheEntry{OK: false}
		graph.pathCacheMu.Unlock()
		return nil, 0, false
	}

	pathIndices := make([]int, 0, 16)
	for at := endIdx; at >= 0; at = prev[at] {
		pathIndices = append(pathIndices, at)
	}
	for i, j := 0, len(pathIndices)-1; i < j; i, j = i+1, j-1 {
		pathIndices[i], pathIndices[j] = pathIndices[j], pathIndices[i]
	}
	pathIDs := make([]int, 0, len(pathIndices))
	for _, idx := range pathIndices {
		pathIDs = append(pathIDs, graph.splines[idx].ID)
	}
	graph.pathCacheMu.Lock()
	graph.pathCache[key] = pathCacheEntry{
		PathIDs: append([]int(nil), pathIDs...),
		Cost:    dist[endIdx],
		OK:      true,
	}
	graph.pathCacheMu.Unlock()
	return pathIDs, dist[endIdx], true
}

func ChooseNextSplineOnBestPathWithGraph(graph *RoadGraph, currentSplineID, destinationSplineID int, vehicleKind VehicleKind) (int, bool) {
	currentSpline, ok := graph.splineByID(currentSplineID)
	if !ok {
		return 0, false
	}
	candidateIndices := graph.startsByNode[pointKey(currentSpline.P3)]
	if len(candidateIndices) == 0 {
		return 0, false
	}

	bestSplineID := 0
	bestCost := float32(1e30)
	found := false
	for _, idx := range candidateIndices {
		candidate := graph.splines[idx]
		if !isSplineUsableForVehicle(candidate, vehicleKind) {
			continue
		}
		_, cost, ok := FindShortestPathWeightedWithGraph(graph, candidate.ID, destinationSplineID, vehicleKind)
		if !ok {
			continue
		}
		if !found || cost < bestCost {
			bestCost = cost
			bestSplineID = candidate.ID
			found = true
		}
	}
	return bestSplineID, found
}

func expandWithCoupledNeighbors(indices []int, splines []Spline, indexByID map[int]int) []int {
	seen := make(map[int]bool, len(indices)*2)
	result := make([]int, 0, len(indices)*2)
	for _, idx := range indices {
		if !seen[idx] {
			seen[idx] = true
			result = append(result, idx)
		}
		for _, coupledID := range splines[idx].HardCoupledIDs {
			if coupledIdx, ok := indexByID[coupledID]; ok && !seen[coupledIdx] {
				seen[coupledIdx] = true
				result = append(result, coupledIdx)
			}
		}
	}
	return result
}

func buildStartsByNode(splines []Spline) map[string][]int {
	startsByNode := map[string][]int{}
	for i, spline := range splines {
		key := pointKey(spline.P0)
		startsByNode[key] = append(startsByNode[key], i)
	}
	return startsByNode
}

func segmentTravelCost(s Spline, vehicleCounts map[int]int) float32 {
	return s.TravelTimeS + float32(vehicleCounts[s.ID])*2.0
}

func sampleSplineAtDistance(s Spline, distance float32) (Vec2, Vec2) {
	if s.Length <= 0 {
		return s.P0, NewVec2(1, 0)
	}
	distance = clampf(distance, 0, s.Length)
	for i := 1; i <= simSamples; i++ {
		if distance <= s.CumLen[i] {
			span := s.CumLen[i] - s.CumLen[i-1]
			alpha := float32(0)
			if span > 0 {
				alpha = (distance - s.CumLen[i-1]) / span
			}
			t0 := float32(i-1) / float32(simSamples)
			t1 := float32(i) / float32(simSamples)
			t := t0 + (t1-t0)*alpha
			pos := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)
			tangent := bezierDerivative(s, t)
			return pos, normalize(tangent)
		}
	}
	return s.P3, normalize(vecSub(s.P3, s.P2))
}

func SampleSplineAtDistance(s Spline, distance float32) (Vec2, Vec2) {
	return sampleSplineAtDistance(s, distance)
}

func bezierDerivative(s Spline, t float32) Vec2 {
	u := 1 - t
	term0 := vecScale(vecSub(s.P1, s.P0), 3*u*u)
	term1 := vecScale(vecSub(s.P2, s.P1), 6*u*t)
	term2 := vecScale(vecSub(s.P3, s.P2), 3*t*t)
	return vecAdd(vecAdd(term0, term1), term2)
}

func bezierPoint(p0, p1, p2, p3 Vec2, t float32) Vec2 {
	u := 1 - t
	uu := u * u
	tt := t * t
	uuu := uu * u
	ttt := tt * t
	return Vec2{
		X: uuu*p0.X + 3*uu*t*p1.X + 3*u*tt*p2.X + ttt*p3.X,
		Y: uuu*p0.Y + 3*uu*t*p1.Y + 3*u*tt*p2.Y + ttt*p3.Y,
	}
}

func buildSplineIndexByID(splines []Spline) map[int]int {
	m := make(map[int]int, len(splines))
	for i, s := range splines {
		m[s.ID] = i
	}
	return m
}

func BuildSplineIndexByID(splines []Spline) map[int]int {
	return buildSplineIndexByID(splines)
}

func findSplineByID(splines []Spline, id int) (Spline, bool) {
	for _, s := range splines {
		if s.ID == id {
			return s, true
		}
	}
	return Spline{}, false
}

func FindSplineByID(splines []Spline, id int) (Spline, bool) {
	return findSplineByID(splines, id)
}

func FindRouteID(routes []Route, startSplineID, endSplineID int, vehicleKind VehicleKind) int {
	for _, route := range routes {
		if route.StartSplineID == startSplineID && route.EndSplineID == endSplineID && route.VehicleKind == vehicleKind {
			return route.ID
		}
	}
	return -1
}

func FindRouteIndexByID(routes []Route, id int) int {
	for i, route := range routes {
		if route.ID == id {
			return i
		}
	}
	return -1
}

func RemoveRouteByID(routes []Route, id int) []Route {
	for i, route := range routes {
		if route.ID == id {
			return append(routes[:i], routes[i+1:]...)
		}
	}
	return routes
}

func RemoveCarsForRoute(cars []Car, routeID int) []Car {
	kept := cars[:0]
	for _, car := range cars {
		if car.RouteID != routeID {
			kept = append(kept, car)
		}
	}
	return kept
}

func nearestSampleOnSpline(spline Spline, pos Vec2) Vec2 {
	best := spline.Samples[0]
	bestDSq := distSq(best, pos)
	for i := 1; i <= simSamples; i++ {
		d := distSq(spline.Samples[i], pos)
		if d < bestDSq {
			bestDSq = d
			best = spline.Samples[i]
		}
	}
	return best
}

func nearestSampleWithDist(spline Spline, pos Vec2) (point Vec2, dist float32) {
	point = spline.Samples[0]
	dist = spline.CumLen[0]
	bestDSq := distSq(point, pos)
	for i := 1; i <= simSamples; i++ {
		d := distSq(spline.Samples[i], pos)
		if d < bestDSq {
			bestDSq = d
			point = spline.Samples[i]
			dist = spline.CumLen[i]
		}
	}
	return
}

func mergedSplines(permanent, temporary []Spline) []Spline {
	if len(temporary) == 0 {
		return permanent
	}
	all := make([]Spline, 0, len(permanent)+len(temporary))
	all = append(all, permanent...)
	all = append(all, temporary...)
	return all
}

func MergedSplines(permanent, temporary []Spline) []Spline {
	return mergedSplines(permanent, temporary)
}

func gcLaneChangeSplines(lcs []Spline, cars []Car) []Spline {
	active := make(map[int]struct{}, len(cars))
	for _, car := range cars {
		active[car.LaneChangeSplineID] = struct{}{}
	}
	out := lcs[:0]
	for _, lc := range lcs {
		if _, ok := active[lc.ID]; ok {
			out = append(out, lc)
		}
	}
	return out
}

func VehicleKindString(kind VehicleKind) string {
	if kind == VehicleBus {
		return "bus"
	}
	return "car"
}

func VehicleKindFromString(s string) VehicleKind {
	if strings.EqualFold(s, "bus") {
		return VehicleBus
	}
	return VehicleCar
}

var routePalette = []Color{
	NewColor(224, 94, 94, 255),
	NewColor(76, 150, 230, 255),
	NewColor(99, 190, 123, 255),
	NewColor(225, 169, 76, 255),
	NewColor(154, 108, 224, 255),
	NewColor(76, 191, 188, 255),
	NewColor(213, 104, 171, 255),
	NewColor(218, 205, 60, 255),
	NewColor(140, 205, 70, 255),
	NewColor(98, 118, 228, 255),
	NewColor(230, 112, 82, 255),
	NewColor(62, 178, 152, 255),
	NewColor(178, 132, 228, 255),
	NewColor(225, 82, 128, 255),
	NewColor(228, 158, 48, 255),
}

func RoutePaletteColor(idx int) Color {
	n := len(routePalette)
	if n == 0 {
		return NewColor(90, 90, 90, 255)
	}
	return routePalette[((idx%n)+n)%n]
}

func PickNextColorIndex(routes []Route) int {
	counts := make([]int, len(routePalette))
	for _, r := range routes {
		if r.ColorIndex >= 0 && r.ColorIndex < len(routePalette) {
			counts[r.ColorIndex]++
		}
	}
	minIdx := 0
	for i, c := range counts {
		if c < counts[minIdx] {
			minIdx = i
		}
	}
	return minIdx
}

func hitboxRadius(width float32) float32 {
	return width/2 + 0.5
}

func hitboxCircleOffsets(length, width float32) []float32 {
	r := hitboxRadius(width)
	span := 2 * (length/2 + 1.0 - r)
	if span < 0 {
		span = 0
	}
	maxSpacing := 2*r + 1.0
	count := int(math.Ceil(float64(span/maxSpacing))) + 1
	if count < 2 {
		count = 2
	}
	offsets := make([]float32, count)
	start := -span / 2
	step := float32(0)
	if count > 1 {
		step = span / float32(count-1)
	}
	for i := range offsets {
		offsets[i] = start + float32(i)*step
	}
	return offsets
}

func buildCollisionGeometry(car Car) collisionGeometry {
	geom := collisionGeometry{
		bodyRadius:   collisionRadius(car),
		bodyOffsets:  collisionCircleOffsets(car),
		coarseRadius: car.Length/2 + 1.0,
	}
	if car.Trailer.HasTrailer {
		geom.trailerRadius = hitboxRadius(car.Trailer.Width)
		geom.trailerOffsets = hitboxCircleOffsets(car.Trailer.Length, car.Trailer.Width)
		geom.coarseRadius += car.Trailer.Length + 1.0
	}
	return geom
}

func buildCollisionGeometries(cars []Car) []collisionGeometry {
	geometries := make([]collisionGeometry, len(cars))
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			geometries[i] = buildCollisionGeometry(cars[i])
		}
	})
	return geometries
}

func collisionRadius(car Car) float32          { return hitboxRadius(car.Width) }
func collisionCircleOffsets(car Car) []float32 { return hitboxCircleOffsets(car.Length, car.Width) }

func headingAngleDegrees(a, b Vec2) float32 {
	da := normalize(a)
	db := normalize(b)
	d := clampf(dot(da, db), -1, 1)
	return float32(math.Acos(float64(d)) * 180 / math.Pi)
}

func signedAngleDegrees(from, to Vec2) float32 {
	nFrom := normalize(from)
	nTo := normalize(to)
	return float32(math.Atan2(float64(cross2D(nFrom, nTo)), float64(dot(nFrom, nTo))) * 180 / math.Pi)
}

func cross2D(a, b Vec2) float32 {
	return a.X*b.Y - a.Y*b.X
}

func vectorLengthSq(v Vec2) float32 {
	return v.X*v.X + v.Y*v.Y
}

func vecAdd(a, b Vec2) Vec2 {
	return NewVec2(a.X+b.X, a.Y+b.Y)
}

func vecSub(a, b Vec2) Vec2 {
	return NewVec2(a.X-b.X, a.Y-b.Y)
}

func vecScale(v Vec2, s float32) Vec2 {
	return NewVec2(v.X*s, v.Y*s)
}

func dot(a, b Vec2) float32 {
	return a.X*b.X + a.Y*b.Y
}

func distSq(a, b Vec2) float32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}

func normalize(v Vec2) Vec2 {
	lenSq := v.X*v.X + v.Y*v.Y
	if lenSq <= 1e-9 {
		return NewVec2(1, 0)
	}
	inv := 1 / float32(math.Sqrt(float64(lenSq)))
	return NewVec2(v.X*inv, v.Y*inv)
}

func clampf(v, lo, hi float32) float32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func randRange(lo, hi float32) float32 {
	return lo + rand.Float32()*(hi-lo)
}

func minf(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func maxf(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

func absf(v float32) float32 {
	if v < 0 {
		return -v
	}
	return v
}

func pointKey(v Vec2) string {
	qx := int(math.Round(float64(v.X * 100)))
	qy := int(math.Round(float64(v.Y * 100)))
	return fmt.Sprintf("%d:%d", qx, qy)
}

func randomizedSpawnDelay(spawnPerMinute float32) float32 {
	if spawnPerMinute <= 0 {
		return float32(math.MaxFloat32)
	}
	lambda := float64(spawnPerMinute) / 60.0
	u := math.Max(rand.Float64(), 1e-5)
	return float32(-math.Log(u) / lambda)
}
