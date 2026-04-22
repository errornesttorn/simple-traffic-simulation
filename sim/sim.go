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
	"sync/atomic"
	"time"
	"unsafe"
)

type VehicleKind int

const (
	VehicleCar VehicleKind = iota
	VehicleBus
)

type CarControlMode uint8

const (
	CarControlAI CarControlMode = iota
	CarControlExternal
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
	// The car is modelled as a front pivot that rides exactly on the spline
	// offset by LateralOffset, and a rear point dragged behind it at
	// Length * wheelbaseFrac. Trailers hitch to the rear point using the
	// same scheme. predictCarTrajectory carries simRearPos as mutable state
	// because its lag produces the body heading (not just the spline tangent).
	frontPivotFrac float32 = 0.20
	rearPivotFrac  float32 = 0.80
	wheelbaseFrac  float32 = rearPivotFrac - frontPivotFrac
)

const (
	pedestrianSpawnIntervalS          float32 = 32.0
	pedestrianSpawnJitterFrac         float32 = 0.25
	pedestrianSpawnRetryDelayS        float32 = 0.75
	pedestrianSpawnInsetM             float32 = 0.35
	pedestrianSpeedMinMPS             float32 = 1.1
	pedestrianSpeedMaxMPS             float32 = 1.7
	pedestrianMinSpeedFactor          float32 = 0.35
	pedestrianRadiusM                 float32 = 0.28
	pedestrianPreferredOffsetM        float32 = 0.75
	pedestrianSideBiasM               float32 = 0.22
	pedestrianHeadOnLookaheadM        float32 = 1.8
	pedestrianSoftFollowDistM         float32 = 1.25
	pedestrianSpawnClearanceM         float32 = 0.9
	pedestrianLateralResponseRate     float32 = 3.5
	pedestrianSpeedResponseRate       float32 = 4.0
	pedestrianHeadOnSlowdownStrength  float32 = 0.25
	pedestrianHeadOnExtraOffsetM      float32 = 0.35
	pedestrianSameDirectionSlowFactor float32 = 0.65
	pedestrianTurnLengthFloorM        float32 = 0.4
	pedestrianTurnSamples                     = 12
	// pedestrianCrossingBlockRadiusM is how close a pedestrian must be (along
	// their path, either side of the crossing) for the crossing to act like a
	// red traffic light to cars on the intersecting spline.
	pedestrianCrossingBlockRadiusM float32 = 5.0
	// pedestrianCrossingCarRadiusM is how close a car must be to the crossing
	// (along its spline) to count as occupying it from a pedestrian's POV.
	pedestrianCrossingCarRadiusM float32 = 3.0
	// pedestrianCrossingStopBufferM is how far (along the pedestrian path) a
	// pedestrian stops in front of the crossing when waiting. Kept smaller than
	// stopFrontGapM so cars stopping at the crossing stay behind the pedestrian.
	pedestrianCrossingStopBufferM float32 = 2.0
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
	SampleTangents []Vec2
	SampleCurv     []float32
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
	ID          int
	RouteID     int
	ControlMode CarControlMode

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

type carControlSplit struct {
	simCars               []Car
	simSourceIndices      []int
	externalCars          []Car
	externalSourceIndices []int
}

func splitCarsByControlMode(cars []Car) carControlSplit {
	if len(cars) == 0 {
		return carControlSplit{}
	}
	split := carControlSplit{
		simCars:               make([]Car, 0, len(cars)),
		simSourceIndices:      make([]int, 0, len(cars)),
		externalCars:          make([]Car, 0, 1),
		externalSourceIndices: make([]int, 0, 1),
	}
	for i, car := range cars {
		if car.ControlMode == CarControlExternal {
			split.externalCars = append(split.externalCars, car)
			split.externalSourceIndices = append(split.externalSourceIndices, i)
			continue
		}
		split.simCars = append(split.simCars, car)
		split.simSourceIndices = append(split.simSourceIndices, i)
	}
	return split
}

func projectBoolSlice(values []bool, indices []int) []bool {
	if len(indices) == 0 {
		return nil
	}
	out := make([]bool, len(indices))
	for i, idx := range indices {
		if idx >= 0 && idx < len(values) {
			out[i] = values[idx]
		}
	}
	return out
}

func projectFloat32Slice(values []float32, indices []int) []float32 {
	if len(indices) == 0 {
		return nil
	}
	out := make([]float32, len(indices))
	for i, idx := range indices {
		if idx >= 0 && idx < len(values) {
			out[i] = values[idx]
		}
	}
	return out
}

func buildFullCarRemap(total int, simSourceIndices []int, simIndexRemap []int, externalSourceIndices []int, simCount int) []int {
	if total == 0 {
		return nil
	}
	remap := make([]int, total)
	for i := range remap {
		remap[i] = -1
	}
	for simIdx, sourceIdx := range simSourceIndices {
		if sourceIdx < 0 || sourceIdx >= len(remap) || simIdx < 0 || simIdx >= len(simIndexRemap) {
			continue
		}
		remap[sourceIdx] = simIndexRemap[simIdx]
	}
	for extIdx, sourceIdx := range externalSourceIndices {
		if sourceIdx < 0 || sourceIdx >= len(remap) {
			continue
		}
		remap[sourceIdx] = simCount + extIdx
	}
	return remap
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

	MarshalMS      float64
	RouteTreeMS    float64
	MarshalSetupMS float64
	BasePredictMS  float64
	ConflictScanMS float64
	BrakeProbeMS   float64
	HoldProbeMS    float64
	FinalizeMS     float64
	KernelMS       float64
	UnmarshalMS    float64

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

type UpdateCarsProfile struct {
	Cars int

	SetupMS      float64
	FastPathMS   float64
	TransitionMS float64

	DwellCars      int
	FastPathCars   int
	TransitionCars int
	RemovedCars    int
}

type FollowingProfile struct {
	Cars int

	PoseMS      float64
	IndexMS     float64
	CandidateMS float64
	ScanMS      float64

	CandidateRefs int
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

type routeTreeKey struct {
	DestinationID int
	VehicleKind   VehicleKind
}

type routeTreeEntry struct {
	costs   []float32
	nextHop []int
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
	splines          []*Spline
	indexByID        map[int]int
	startsByNode     map[NodeKey][]int
	routeNeighbors   [][]int
	reverseNeighbors [][]int
	segmentCosts     []float32
	routeCacheMu     sync.RWMutex
	routeCache       map[routeTreeKey]routeTreeEntry
	pathCacheHits    atomic.Int64
	pathCacheMisses  atomic.Int64
}

type roadGraphTopology struct {
	splines          []*Spline
	indexByID        map[int]int
	startsByNode     map[NodeKey][]int
	endsByNode       map[NodeKey][]int
	routeNeighbors   [][]int
	reverseNeighbors [][]int
}

func NewRoadGraph(splines []Spline, vehicleCounts map[int]int) *RoadGraph {
	return newRoadGraphFromSlices(splines, nil, vehicleCounts)
}

func newRoadGraphFromSlices(permanent, temporary []Spline, vehicleCounts map[int]int) *RoadGraph {
	splineRefs := make([]*Spline, 0, len(permanent)+len(temporary))
	for i := range permanent {
		splineRefs = append(splineRefs, &permanent[i])
	}
	for i := range temporary {
		splineRefs = append(splineRefs, &temporary[i])
	}
	return newRoadGraphFromTopology(buildRoadGraphTopologyFromSplineRefs(splineRefs), vehicleCounts)
}

func buildRoadGraphTopologyFromSplineRefs(splineRefs []*Spline) *roadGraphTopology {
	indexByID := buildSplinePtrIndexByID(splineRefs)
	startsByNode := buildStartsByNodeRefs(splineRefs)
	endsByNode := buildEndsByNodeRefs(splineRefs)
	routeNeighbors := make([][]int, len(splineRefs))
	reverseNeighbors := make([][]int, len(splineRefs))
	for i := range splineRefs {
		next := startsByNode[nodeKeyFromVec2(splineRefs[i].P3)]
		if len(next) > 0 {
			routeNeighbors[i] = append([]int(nil), expandWithCoupledNeighborRefs(next, splineRefs, indexByID)...)
			for _, nextIdx := range routeNeighbors[i] {
				reverseNeighbors[nextIdx] = append(reverseNeighbors[nextIdx], i)
			}
		}
	}
	return &roadGraphTopology{
		splines:          splineRefs,
		indexByID:        indexByID,
		startsByNode:     startsByNode,
		endsByNode:       endsByNode,
		routeNeighbors:   routeNeighbors,
		reverseNeighbors: reverseNeighbors,
	}
}

func newRoadGraphFromTopology(topology *roadGraphTopology, vehicleCounts map[int]int) *RoadGraph {
	segmentCosts := make([]float32, len(topology.splines))
	for i := range topology.splines {
		segmentCosts[i] = segmentTravelCost(topology.splines[i], vehicleCounts)
	}
	return &RoadGraph{
		splines:          topology.splines,
		indexByID:        topology.indexByID,
		startsByNode:     topology.startsByNode,
		routeNeighbors:   topology.routeNeighbors,
		reverseNeighbors: topology.reverseNeighbors,
		segmentCosts:     segmentCosts,
		routeCache:       make(map[routeTreeKey]routeTreeEntry),
	}
}

func (g *RoadGraph) splinePtrByID(id int) (*Spline, bool) {
	idx, ok := g.indexByID[id]
	if !ok {
		return nil, false
	}
	return g.splines[idx], true
}

func (g *RoadGraph) splineByID(id int) (Spline, bool) {
	spline, ok := g.splinePtrByID(id)
	if !ok {
		return Spline{}, false
	}
	return *spline, true
}

func (g *RoadGraph) SplineByID(id int) (Spline, bool) {
	return g.splineByID(id)
}

func (g *RoadGraph) nextPhysicalIndices(currentSplineID int) []int {
	currentSpline, ok := g.splinePtrByID(currentSplineID)
	if !ok {
		return nil
	}
	return g.startsByNode[nodeKeyFromVec2(currentSpline.P3)]
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

type TopologyState struct {
	Splines         []Spline
	Routes          []Route
	TrafficLights   []TrafficLight
	TrafficCycles   []TrafficCycle
	PedestrianPaths []PedestrianPath

	NextSplineID int
	NextRouteID  int
	NextLightID  int
	NextCycleID  int
}

// PedestrianPath is a straight walking segment between two endpoints.
// Width is fixed (see PedestrianPathWidthM); no per-path properties yet.
type PedestrianPath struct {
	P0 Vec2
	P1 Vec2
}

const PedestrianPathWidthM float32 = 4.0

// pedestrianCrossing is a point where a pedestrian path meets a car spline.
// Distances are arc-length: along the pedestrian path from P0, and along the
// spline from its start.
type pedestrianCrossing struct {
	SplineID     int
	DistOnSpline float32
	PathIndex    int
	DistOnPath   float32
}

type Pedestrian struct {
	ID            int
	PathIndex     int
	Distance      float32
	Forward       bool
	Speed         float32
	BaseSpeed     float32
	Radius        float32
	LateralOffset float32
	SideBias      float32

	TransitionActive      bool
	TransitionDistance    float32
	TransitionLength      float32
	TransitionP0          Vec2
	TransitionP1          Vec2
	TransitionP2          Vec2
	TransitionNextPath    int
	TransitionNextForward bool
	TransitionEndOffset   float32
}

type pedestrianSpawnKey struct {
	PathIndex int
	Forward   bool
}

type RuntimeState struct {
	LaneChangeSplines []Spline
	Cars              []Car
	NextCarID         int
	Pedestrians       []Pedestrian
	NextPedestrianID  int
	HasPlayerProxy    bool
	PlayerProxyCar    Car
	PlayerProxyAttach PlayerProxyAttachment

	RouteVisualsTimer float32

	DebugBlameLinks      []DebugBlameLink
	HoldBlameLinks       []DebugBlameLink
	DebugSelectedCarID   int
	DebugSelectedCarMode int // 0 = primary candidates, 1 = hold candidates
	DebugCandidateLinks  []DebugBlameLink
	BasePathHits         int
	BasePathMisses       int
	AllPathHits          int
	AllPathMisses        int
	BrakingProfile       BrakingProfile
	FollowingProfile     FollowingProfile
	UpdateCarsProfile    UpdateCarsProfile

	RouteVisualsMS float64
	LaneChangesMS  float64
	GraphBuildMS   float64
	BrakingMS      float64
	FollowMS       float64
	UpdateCarsMS   float64
	StepMS         float64

	PedestrianSpawnTimers map[pedestrianSpawnKey]float32

	permanentGraphTopology    *roadGraphTopology
	permanentGraphTopologyKey uint64
	permanentGraphSplineAddr  uintptr
	permanentGraphSplineCount int
}

type World struct {
	TopologyState
	RuntimeState
}

func NewWorld() World {
	return World{
		TopologyState: TopologyState{
			Splines:         make([]Spline, 0, 128),
			Routes:          make([]Route, 0, 32),
			TrafficLights:   make([]TrafficLight, 0),
			TrafficCycles:   make([]TrafficCycle, 0),
			PedestrianPaths: make([]PedestrianPath, 0, 64),
			NextSplineID:    1,
			NextRouteID:     1,
			NextLightID:     1,
			NextCycleID:     1,
		},
		RuntimeState: RuntimeState{
			LaneChangeSplines:     make([]Spline, 0, 32),
			Cars:                  make([]Car, 0, 256),
			NextCarID:             1,
			Pedestrians:           make([]Pedestrian, 0, 64),
			NextPedestrianID:      1,
			DebugSelectedCarID:    -1,
			PedestrianSpawnTimers: make(map[pedestrianSpawnKey]float32),
		},
	}
}

func cloneIntSlice(src []int) []int {
	if len(src) == 0 {
		return nil
	}
	return append([]int(nil), src...)
}

func cloneFloat32Slice(src []float32) []float32 {
	if len(src) == 0 {
		return nil
	}
	return append([]float32(nil), src...)
}

func cloneVec2Slice(src []Vec2) []Vec2 {
	if len(src) == 0 {
		return nil
	}
	return append([]Vec2(nil), src...)
}

func cloneSplines(src []Spline) []Spline {
	if len(src) == 0 {
		return nil
	}
	dst := append([]Spline(nil), src...)
	for i := range dst {
		dst[i].HardCoupledIDs = cloneIntSlice(src[i].HardCoupledIDs)
		dst[i].SoftCoupledIDs = cloneIntSlice(src[i].SoftCoupledIDs)
		dst[i].SampleTangents = cloneVec2Slice(src[i].SampleTangents)
		dst[i].SampleCurv = cloneFloat32Slice(src[i].SampleCurv)
		dst[i].CurveSpeedMPS = cloneFloat32Slice(src[i].CurveSpeedMPS)
	}
	return dst
}

func cloneRoutes(src []Route) []Route {
	if len(src) == 0 {
		return nil
	}
	dst := append([]Route(nil), src...)
	for i := range dst {
		dst[i].PathIDs = cloneIntSlice(src[i].PathIDs)
		if len(src[i].BusStops) > 0 {
			dst[i].BusStops = append([]BusStop(nil), src[i].BusStops...)
		} else {
			dst[i].BusStops = nil
		}
	}
	return dst
}

func cloneTrafficCycles(src []TrafficCycle) []TrafficCycle {
	if len(src) == 0 {
		return nil
	}
	dst := append([]TrafficCycle(nil), src...)
	for i := range dst {
		dst[i].LightIDs = cloneIntSlice(src[i].LightIDs)
		if len(src[i].Phases) > 0 {
			dst[i].Phases = append([]TrafficPhase(nil), src[i].Phases...)
			for j := range dst[i].Phases {
				dst[i].Phases[j].GreenLightIDs = cloneIntSlice(src[i].Phases[j].GreenLightIDs)
			}
		} else {
			dst[i].Phases = nil
		}
	}
	return dst
}

func cloneDebugBlameLinks(src []DebugBlameLink) []DebugBlameLink {
	if len(src) == 0 {
		return nil
	}
	return append([]DebugBlameLink(nil), src...)
}

func clonePedestrianSpawnTimers(src map[pedestrianSpawnKey]float32) map[pedestrianSpawnKey]float32 {
	if len(src) == 0 {
		return nil
	}
	dst := make(map[pedestrianSpawnKey]float32, len(src))
	for k, v := range src {
		dst[k] = v
	}
	return dst
}

func (w World) Clone() World {
	clone := w
	clone.TopologyState = w.TopologyState
	clone.RuntimeState = w.RuntimeState
	clone.Splines = cloneSplines(w.Splines)
	clone.LaneChangeSplines = cloneSplines(w.LaneChangeSplines)
	clone.Routes = cloneRoutes(w.Routes)
	if len(w.Cars) > 0 {
		clone.Cars = append([]Car(nil), w.Cars...)
	} else {
		clone.Cars = nil
	}
	if len(w.TrafficLights) > 0 {
		clone.TrafficLights = append([]TrafficLight(nil), w.TrafficLights...)
	} else {
		clone.TrafficLights = nil
	}
	clone.TrafficCycles = cloneTrafficCycles(w.TrafficCycles)
	if len(w.PedestrianPaths) > 0 {
		clone.PedestrianPaths = append([]PedestrianPath(nil), w.PedestrianPaths...)
	} else {
		clone.PedestrianPaths = nil
	}
	if len(w.Pedestrians) > 0 {
		clone.Pedestrians = append([]Pedestrian(nil), w.Pedestrians...)
	} else {
		clone.Pedestrians = nil
	}
	clone.DebugBlameLinks = cloneDebugBlameLinks(w.DebugBlameLinks)
	clone.HoldBlameLinks = cloneDebugBlameLinks(w.HoldBlameLinks)
	clone.DebugCandidateLinks = cloneDebugBlameLinks(w.DebugCandidateLinks)
	clone.PedestrianSpawnTimers = clonePedestrianSpawnTimers(w.PedestrianSpawnTimers)
	clone.permanentGraphTopology = nil
	clone.permanentGraphTopologyKey = 0
	clone.permanentGraphSplineAddr = 0
	clone.permanentGraphSplineCount = 0
	return clone
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
	w.ResetPedestrianRuntime()
	w.HasPlayerProxy = false
	w.PlayerProxyCar = Car{}
	w.PlayerProxyAttach = PlayerProxyAttachment{}
	w.DebugBlameLinks = nil
	w.HoldBlameLinks = nil
	w.DebugCandidateLinks = nil
	w.DebugSelectedCarID = -1
	w.DebugSelectedCarMode = 0
}

func (w *World) ResetPedestrianRuntime() {
	w.Pedestrians = w.Pedestrians[:0]
	w.NextPedestrianID = 1
	if w.PedestrianSpawnTimers == nil {
		w.PedestrianSpawnTimers = make(map[pedestrianSpawnKey]float32)
	} else {
		clear(w.PedestrianSpawnTimers)
	}
}

func findCarIndexByID(cars []Car, id int) int {
	if id < 0 {
		return -1
	}
	for i := range cars {
		if cars[i].ID == id {
			return i
		}
	}
	return -1
}

func FindCarIndexByID(cars []Car, id int) int {
	return findCarIndexByID(cars, id)
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

func splineSliceAddr(splines []Spline) uintptr {
	if len(splines) == 0 {
		return 0
	}
	return uintptr(unsafe.Pointer(&splines[0]))
}

func hashFloat32Bits(h uint64, v float32) uint64 {
	const prime = 1099511628211
	h ^= uint64(math.Float32bits(v))
	return h * prime
}

func hashIntBits(h uint64, v int) uint64 {
	const prime = 1099511628211
	h ^= uint64(uint32(v))
	return h * prime
}

func splineTopologyKey(splines []Spline) uint64 {
	const offset = 1469598103934665603
	h := uint64(offset)
	h = hashIntBits(h, len(splines))
	for i := range splines {
		s := &splines[i]
		h = hashIntBits(h, s.ID)
		h = hashFloat32Bits(h, s.P0.X)
		h = hashFloat32Bits(h, s.P0.Y)
		h = hashFloat32Bits(h, s.P3.X)
		h = hashFloat32Bits(h, s.P3.Y)
		h = hashIntBits(h, len(s.HardCoupledIDs))
		for _, coupledID := range s.HardCoupledIDs {
			h = hashIntBits(h, coupledID)
		}
	}
	return h
}

func cloneIndexMap(src map[NodeKey][]int) map[NodeKey][]int {
	if len(src) == 0 {
		return make(map[NodeKey][]int)
	}
	dst := make(map[NodeKey][]int, len(src))
	for key, indices := range src {
		dst[key] = append([]int(nil), indices...)
	}
	return dst
}

func buildMergedRoadGraphFromTopology(base *roadGraphTopology, temporary []Spline, vehicleCounts map[int]int) *RoadGraph {
	if len(temporary) == 0 {
		return newRoadGraphFromTopology(base, vehicleCounts)
	}

	baseCount := len(base.splines)
	total := baseCount + len(temporary)
	splineRefs := make([]*Spline, 0, total)
	splineRefs = append(splineRefs, base.splines...)
	for i := range temporary {
		splineRefs = append(splineRefs, &temporary[i])
	}

	indexByID := make(map[int]int, total)
	for id, idx := range base.indexByID {
		indexByID[id] = idx
	}
	startsByNode := cloneIndexMap(base.startsByNode)
	endsByNode := cloneIndexMap(base.endsByNode)
	for i := range temporary {
		idx := baseCount + i
		spline := &temporary[i]
		indexByID[spline.ID] = idx
		startKey := nodeKeyFromVec2(spline.P0)
		endKey := nodeKeyFromVec2(spline.P3)
		startsByNode[startKey] = append(startsByNode[startKey], idx)
		endsByNode[endKey] = append(endsByNode[endKey], idx)
	}

	routeNeighbors := make([][]int, total)
	copy(routeNeighbors, base.routeNeighbors)
	recompute := make([]bool, total)
	for i := range temporary {
		idx := baseCount + i
		recompute[idx] = true
		startKey := nodeKeyFromVec2(temporary[i].P0)
		for _, srcIdx := range endsByNode[startKey] {
			recompute[srcIdx] = true
		}
	}
	for idx, needs := range recompute {
		if !needs {
			continue
		}
		next := startsByNode[nodeKeyFromVec2(splineRefs[idx].P3)]
		if len(next) == 0 {
			routeNeighbors[idx] = nil
			continue
		}
		routeNeighbors[idx] = append([]int(nil), expandWithCoupledNeighborRefs(next, splineRefs, indexByID)...)
	}

	reverseNeighbors := make([][]int, total)
	for fromIdx, next := range routeNeighbors {
		for _, toIdx := range next {
			reverseNeighbors[toIdx] = append(reverseNeighbors[toIdx], fromIdx)
		}
	}

	return newRoadGraphFromTopology(&roadGraphTopology{
		splines:          splineRefs,
		indexByID:        indexByID,
		startsByNode:     startsByNode,
		endsByNode:       endsByNode,
		routeNeighbors:   routeNeighbors,
		reverseNeighbors: reverseNeighbors,
	}, vehicleCounts)
}

// permanentRoadGraphTopology returns the cached adjacency built from
// w.Splines, rebuilding it only when the topology actually changed. The
// cache key intentionally includes the slice backing address as well as
// a content hash: replacing w.Splines with a fresh slice must invalidate
// the cache even if the splines happen to hash the same, because the
// cached topology holds *Spline pointers into the old backing array.
func (w *World) permanentRoadGraphTopology() *roadGraphTopology {
	key := splineTopologyKey(w.Splines)
	addr := splineSliceAddr(w.Splines)
	if w.permanentGraphTopology != nil &&
		w.permanentGraphTopologyKey == key &&
		w.permanentGraphSplineAddr == addr &&
		w.permanentGraphSplineCount == len(w.Splines) {
		return w.permanentGraphTopology
	}

	splineRefs := make([]*Spline, len(w.Splines))
	for i := range w.Splines {
		splineRefs[i] = &w.Splines[i]
	}
	w.permanentGraphTopology = buildRoadGraphTopologyFromSplineRefs(splineRefs)
	w.permanentGraphTopologyKey = key
	w.permanentGraphSplineAddr = addr
	w.permanentGraphSplineCount = len(w.Splines)
	return w.permanentGraphTopology
}

// Step advances the world by dt seconds. Order here is load-bearing:
// lane-change bridges are generated before braking so the braking kernel
// sees the merged graph (permanent topology + transient bridges); braking
// and following caps run in parallel because they read the same snapshot
// and write to disjoint outputs; updateCars is last because it integrates
// positions using those outputs. See AGENTS.md for the full pipeline.
func (w *World) Step(dt float32) {
	stepStart := time.Now()
	w.RouteVisualsMS = 0
	w.LaneChangesMS = 0
	w.GraphBuildMS = 0
	w.BrakingMS = 0
	w.FollowMS = 0
	w.UpdateCarsMS = 0
	w.StepMS = 0
	w.BrakingProfile = BrakingProfile{}
	w.FollowingProfile = FollowingProfile{}
	w.UpdateCarsProfile = UpdateCarsProfile{}
	reactionCars := append([]Car(nil), w.Cars...)

	graphStart := time.Now()
	vehicleCounts := BuildVehicleCounts(reactionCars)
	permanentTopology := w.permanentRoadGraphTopology()
	baseGraph := newRoadGraphFromTopology(permanentTopology, vehicleCounts)
	w.GraphBuildMS += sinceMS(graphStart)

	w.RouteVisualsTimer -= dt
	if w.RouteVisualsTimer <= 0 {
		routeVisualsStart := time.Now()
		w.Routes = UpdateRouteVisualsWithGraph(w.Routes, baseGraph)
		w.RouteVisualsMS = sinceMS(routeVisualsStart)
		w.RouteVisualsTimer = 0.5
	}

	laneChangesStart := time.Now()
	w.LaneChangeSplines, reactionCars = computeLaneChanges(reactionCars, w.Splines, w.LaneChangeSplines, &w.NextSplineID, baseGraph, dt)
	w.LaneChangesMS = sinceMS(laneChangesStart)

	graphStart = time.Now()
	allGraph := buildMergedRoadGraphFromTopology(permanentTopology, w.LaneChangeSplines, vehicleCounts)
	w.GraphBuildMS += sinceMS(graphStart)

	debugSelectedCar := findCarIndexByID(reactionCars, w.DebugSelectedCarID)
	var brakingDecisions, holdSpeedDecisions []bool
	var debugBlameLinks, holdBlameLinks, candidateLinks []DebugBlameLink
	var brakingProfile BrakingProfile
	var followCaps []float32
	var followingProfile FollowingProfile
	var brakingMS, followMS float64

	var wgBrakeFollow sync.WaitGroup
	wgBrakeFollow.Add(2)
	go func() {
		defer wgBrakeFollow.Done()
		brakingStart := time.Now()
		brakingDecisions, holdSpeedDecisions, debugBlameLinks, holdBlameLinks, candidateLinks, brakingProfile = computeBrakingDecisionsC(reactionCars, allGraph, debugSelectedCar, w.DebugSelectedCarMode)
		brakingMS = sinceMS(brakingStart)
	}()
	go func() {
		defer wgBrakeFollow.Done()
		followStart := time.Now()
		followCaps, followingProfile = computeFollowingSpeedCaps(reactionCars, allGraph)
		followMS = sinceMS(followStart)
	}()
	wgBrakeFollow.Wait()
	w.BrakingMS = brakingMS
	w.BrakingProfile = brakingProfile
	w.FollowMS = followMS
	w.FollowingProfile = followingProfile

	split := splitCarsByControlMode(reactionCars)
	simBrakingDecisions := projectBoolSlice(brakingDecisions, split.simSourceIndices)
	simHoldSpeedDecisions := projectBoolSlice(holdSpeedDecisions, split.simSourceIndices)
	simFollowCaps := projectFloat32Slice(followCaps, split.simSourceIndices)

	pedestrianCrossings := computePedestrianCrossings(w.PedestrianPaths, w.Splines)
	pedestrianBlockedBySpline := buildPedestrianBlockedSplineDists(pedestrianCrossings, w.Pedestrians)

	updateCarsStart := time.Now()
	simCars := split.simCars
	var indexRemap []int
	simCars, indexRemap, w.UpdateCarsProfile = updateCars(split.simCars, w.Routes, allGraph, simBrakingDecisions, simHoldSpeedDecisions, simFollowCaps, w.TrafficLights, w.TrafficCycles, pedestrianBlockedBySpline, dt)
	w.LaneChangeSplines = gcLaneChangeSplines(w.LaneChangeSplines, simCars)
	carsForSpawn := make([]Car, 0, len(simCars)+len(split.externalCars))
	carsForSpawn = append(carsForSpawn, simCars...)
	carsForSpawn = append(carsForSpawn, split.externalCars...)
	w.Routes, carsForSpawn = updateRouteSpawning(w.Routes, carsForSpawn, w.Splines, &w.NextCarID, dt)
	postSpawnSplit := splitCarsByControlMode(carsForSpawn)
	simCars = postSpawnSplit.simCars
	externalCars := postSpawnSplit.externalCars
	w.TrafficCycles = UpdateTrafficCycles(w.TrafficCycles, dt)
	pedestrianCars := make([]Car, 0, len(simCars)+len(externalCars))
	pedestrianCars = append(pedestrianCars, simCars...)
	pedestrianCars = append(pedestrianCars, externalCars...)
	w.Pedestrians, w.PedestrianSpawnTimers = updatePedestrians(w.Pedestrians, w.PedestrianPaths, &w.NextPedestrianID, w.PedestrianSpawnTimers, dt, pedestrianCrossings, pedestrianCars)
	w.UpdateCarsMS = sinceMS(updateCarsStart)
	w.Cars = append(simCars, externalCars...)

	fullIndexRemap := buildFullCarRemap(len(reactionCars), split.simSourceIndices, indexRemap, split.externalSourceIndices, len(simCars))
	w.DebugBlameLinks = remapBlameLinks(debugBlameLinks, fullIndexRemap)
	w.HoldBlameLinks = remapBlameLinks(holdBlameLinks, fullIndexRemap)
	w.DebugCandidateLinks = remapBlameLinks(candidateLinks, fullIndexRemap)
	if w.DebugSelectedCarID >= 0 && findCarIndexByID(w.Cars, w.DebugSelectedCarID) < 0 {
		w.DebugSelectedCarID = -1
		w.DebugSelectedCarMode = 0
	}
	w.BasePathHits = int(baseGraph.pathCacheHits.Load())
	w.BasePathMisses = int(baseGraph.pathCacheMisses.Load())
	w.AllPathHits = int(allGraph.pathCacheHits.Load())
	w.AllPathMisses = int(allGraph.pathCacheMisses.Load())
	w.StepMS = sinceMS(stepStart)
}

type pedestrianRuntimePath struct {
	Valid  bool
	Index  int
	P0     Vec2
	P1     Vec2
	Node0  NodeKey
	Node1  NodeKey
	Length float32
	Dir    Vec2
	Normal Vec2
}

type pedestrianTopology struct {
	paths          []pedestrianRuntimePath
	neighborsByKey map[NodeKey][]int
	deadEndSources []pedestrianSpawnSource
}

type pedestrianSpawnSource struct {
	key       pedestrianSpawnKey
	pathIndex int
	forward   bool
}

func buildPedestrianTopology(paths []PedestrianPath) pedestrianTopology {
	topology := pedestrianTopology{
		paths:          make([]pedestrianRuntimePath, len(paths)),
		neighborsByKey: make(map[NodeKey][]int),
	}
	for i, path := range paths {
		diff := vecSub(path.P1, path.P0)
		lengthSq := vectorLengthSq(diff)
		if lengthSq <= 1e-6 {
			continue
		}
		length := sqrtf(lengthSq)
		dir := vecScale(diff, 1/length)
		node0 := nodeKeyFromVec2(path.P0)
		node1 := nodeKeyFromVec2(path.P1)
		topology.paths[i] = pedestrianRuntimePath{
			Valid:  true,
			Index:  i,
			P0:     path.P0,
			P1:     path.P1,
			Node0:  node0,
			Node1:  node1,
			Length: length,
			Dir:    dir,
			Normal: NewVec2(-dir.Y, dir.X),
		}
		topology.neighborsByKey[node0] = append(topology.neighborsByKey[node0], i)
		topology.neighborsByKey[node1] = append(topology.neighborsByKey[node1], i)
	}
	for key, edges := range topology.neighborsByKey {
		if len(edges) != 1 {
			continue
		}
		path := topology.paths[edges[0]]
		if !path.Valid {
			continue
		}
		forward := path.Node0 == key
		topology.deadEndSources = append(topology.deadEndSources, pedestrianSpawnSource{
			key: pedestrianSpawnKey{
				PathIndex: path.Index,
				Forward:   forward,
			},
			pathIndex: path.Index,
			forward:   forward,
		})
	}
	return topology
}

func preferredPedestrianOffset(forward bool, sideBias float32) float32 {
	base := pedestrianPreferredOffsetM
	if !forward {
		base = -base
	}
	return base + sideBias
}

func pedestrianCanonicalDistance(path pedestrianRuntimePath, ped Pedestrian) float32 {
	if ped.Forward {
		return ped.Distance
	}
	return path.Length - ped.Distance
}

func pedestrianPoseOnRuntimePath(path pedestrianRuntimePath, ped Pedestrian) (Vec2, Vec2) {
	dist := clampf(ped.Distance, 0, path.Length)
	var pos, heading Vec2
	if ped.Forward {
		pos = vecAdd(path.P0, vecScale(path.Dir, dist))
		heading = path.Dir
	} else {
		pos = vecSub(path.P1, vecScale(path.Dir, dist))
		heading = vecScale(path.Dir, -1)
	}
	return vecAdd(pos, vecScale(path.Normal, ped.LateralOffset)), heading
}

func quadraticBezierPoint(p0, p1, p2 Vec2, t float32) Vec2 {
	u := 1 - t
	return Vec2{
		X: u*u*p0.X + 2*u*t*p1.X + t*t*p2.X,
		Y: u*u*p0.Y + 2*u*t*p1.Y + t*t*p2.Y,
	}
}

func quadraticBezierTangent(p0, p1, p2 Vec2, t float32) Vec2 {
	u := 1 - t
	return normalize(Vec2{
		X: 2*u*(p1.X-p0.X) + 2*t*(p2.X-p1.X),
		Y: 2*u*(p1.Y-p0.Y) + 2*t*(p2.Y-p1.Y),
	})
}

func quadraticBezierLength(p0, p1, p2 Vec2) float32 {
	total := float32(0)
	prev := p0
	for i := 1; i <= pedestrianTurnSamples; i++ {
		t := float32(i) / float32(pedestrianTurnSamples)
		pt := quadraticBezierPoint(p0, p1, p2, t)
		total += sqrtf(distSq(prev, pt))
		prev = pt
	}
	return maxf(total, pedestrianTurnLengthFloorM)
}

func pedestrianTransitionPose(ped Pedestrian) (Vec2, Vec2) {
	if !ped.TransitionActive || ped.TransitionLength <= 1e-6 {
		return ped.TransitionP2, quadraticBezierTangent(ped.TransitionP0, ped.TransitionP1, ped.TransitionP2, 1)
	}
	t := clampf(ped.TransitionDistance/ped.TransitionLength, 0, 1)
	return quadraticBezierPoint(ped.TransitionP0, ped.TransitionP1, ped.TransitionP2, t),
		quadraticBezierTangent(ped.TransitionP0, ped.TransitionP1, ped.TransitionP2, t)
}

func pedestrianCurrentPose(path pedestrianRuntimePath, ped Pedestrian) (Vec2, Vec2) {
	if ped.TransitionActive {
		return pedestrianTransitionPose(ped)
	}
	return pedestrianPoseOnRuntimePath(path, ped)
}

func buildPedestrianTurn(path pedestrianRuntimePath, ped Pedestrian, nextPath pedestrianRuntimePath, nextForward bool) Pedestrian {
	nodeCenter := path.P1
	if !ped.Forward {
		nodeCenter = path.P0
	}
	exitPos, _ := pedestrianPoseOnRuntimePath(path, Pedestrian{
		Distance:      path.Length,
		Forward:       ped.Forward,
		LateralOffset: ped.LateralOffset,
	})
	entryOffset := preferredPedestrianOffset(nextForward, ped.SideBias)
	entryPos, _ := pedestrianPoseOnRuntimePath(nextPath, Pedestrian{
		Distance:      0,
		Forward:       nextForward,
		LateralOffset: entryOffset,
	})
	ped.TransitionActive = true
	ped.TransitionDistance = 0
	ped.TransitionP0 = exitPos
	ped.TransitionP1 = nodeCenter
	ped.TransitionP2 = entryPos
	ped.TransitionLength = quadraticBezierLength(ped.TransitionP0, ped.TransitionP1, ped.TransitionP2)
	ped.TransitionNextPath = nextPath.Index
	ped.TransitionNextForward = nextForward
	ped.TransitionEndOffset = entryOffset
	return ped
}

func clearPedestrianTransition(ped *Pedestrian) {
	if ped == nil {
		return
	}
	ped.TransitionActive = false
	ped.TransitionDistance = 0
	ped.TransitionLength = 0
	ped.TransitionP0 = Vec2{}
	ped.TransitionP1 = Vec2{}
	ped.TransitionP2 = Vec2{}
	ped.TransitionNextPath = 0
	ped.TransitionNextForward = false
	ped.TransitionEndOffset = 0
}

func PedestrianPose(paths []PedestrianPath, ped Pedestrian) (Vec2, Vec2, bool) {
	if ped.PathIndex < 0 || ped.PathIndex >= len(paths) {
		return Vec2{}, Vec2{}, false
	}
	raw := paths[ped.PathIndex]
	diff := vecSub(raw.P1, raw.P0)
	lengthSq := vectorLengthSq(diff)
	if lengthSq <= 1e-6 {
		return Vec2{}, Vec2{}, false
	}
	length := sqrtf(lengthSq)
	path := pedestrianRuntimePath{
		Valid:  true,
		P0:     raw.P0,
		P1:     raw.P1,
		Length: length,
		Dir:    vecScale(diff, 1/length),
	}
	path.Normal = NewVec2(-path.Dir.Y, path.Dir.X)
	pos, heading := pedestrianCurrentPose(path, ped)
	return pos, heading, true
}

func computePedestrianTargets(pedestrians []Pedestrian, topology pedestrianTopology) ([]float32, []float32) {
	targetSpeeds := make([]float32, len(pedestrians))
	targetOffsets := make([]float32, len(pedestrians))
	maxOffset := PedestrianPathWidthM*0.5 - pedestrianRadiusM*1.1
	for i, ped := range pedestrians {
		path := topology.paths[ped.PathIndex]
		targetOffset := preferredPedestrianOffset(ped.Forward, ped.SideBias)
		targetSpeed := ped.BaseSpeed
		selfCanonical := pedestrianCanonicalDistance(path, ped)
		dirSign := float32(1)
		if !ped.Forward {
			dirSign = -1
		}
		for j, other := range pedestrians {
			if i == j || other.PathIndex != ped.PathIndex {
				continue
			}
			otherCanonical := pedestrianCanonicalDistance(path, other)
			if ped.Forward == other.Forward {
				relAhead := (otherCanonical - selfCanonical) * dirSign
				if relAhead <= 0 || relAhead >= pedestrianSoftFollowDistM {
					continue
				}
				slow := 1 - pedestrianSameDirectionSlowFactor*(1-relAhead/pedestrianSoftFollowDistM)
				minSpeed := ped.BaseSpeed * pedestrianMinSpeedFactor
				targetSpeed = minf(targetSpeed, maxf(minSpeed, ped.BaseSpeed*slow))
				continue
			}
			separation := absf(otherCanonical - selfCanonical)
			if separation >= pedestrianHeadOnLookaheadM {
				continue
			}
			t := 1 - separation/pedestrianHeadOnLookaheadM
			targetOffset += preferredPedestrianOffset(ped.Forward, 0) / pedestrianPreferredOffsetM * pedestrianHeadOnExtraOffsetM * t
			slow := 1 - pedestrianHeadOnSlowdownStrength*t
			minSpeed := ped.BaseSpeed * pedestrianMinSpeedFactor
			targetSpeed = maxf(minSpeed, minf(targetSpeed, ped.BaseSpeed*slow))
		}
		targetSpeeds[i] = maxf(ped.BaseSpeed*pedestrianMinSpeedFactor, targetSpeed)
		targetOffsets[i] = clampf(targetOffset, -maxOffset, maxOffset)
	}
	return targetSpeeds, targetOffsets
}

func chooseNextPedestrianEdge(topology pedestrianTopology, nodeKey NodeKey, currentPathIndex int) (int, bool, bool) {
	neighbors := topology.neighborsByKey[nodeKey]
	if len(neighbors) <= 1 {
		return 0, false, false
	}
	candidates := make([]int, 0, len(neighbors)-1)
	for _, idx := range neighbors {
		if idx != currentPathIndex {
			candidates = append(candidates, idx)
		}
	}
	if len(candidates) == 0 {
		return 0, false, false
	}
	nextPathIndex := candidates[rand.Intn(len(candidates))]
	nextPath := topology.paths[nextPathIndex]
	return nextPathIndex, nextPath.Node0 == nodeKey, true
}

func canSpawnPedestrianAtSource(source pedestrianSpawnSource, topology pedestrianTopology, pedestrians []Pedestrian) bool {
	for _, ped := range pedestrians {
		if ped.PathIndex != source.pathIndex || ped.Forward != source.forward {
			continue
		}
		if ped.Distance < pedestrianSpawnClearanceM {
			return false
		}
	}
	return true
}

func pedestrianSpawnDelay() float32 {
	return pedestrianSpawnIntervalS * randRange(1-pedestrianSpawnJitterFrac, 1+pedestrianSpawnJitterFrac)
}

// segmentIntersectionParams returns the parameters (t along a0->a1, u along
// b0->b1) at which two line segments meet. Both parameters lie in [0,1] when
// the segments actually cross. ok is false for parallel or degenerate segments.
func segmentIntersectionParams(a0, a1, b0, b1 Vec2) (t, u float32, ok bool) {
	r := vecSub(a1, a0)
	s := vecSub(b1, b0)
	denom := cross2D(r, s)
	if absf(denom) < 1e-7 {
		return 0, 0, false
	}
	qp := vecSub(b0, a0)
	t = cross2D(qp, s) / denom
	u = cross2D(qp, r) / denom
	if t < 0 || t > 1 || u < 0 || u > 1 {
		return 0, 0, false
	}
	return t, u, true
}

// computePedestrianCrossings finds every point where a pedestrian path meets a
// car spline, using each spline's sampled polyline as an approximation. A path
// may appear multiple times for a single spline if they cross more than once.
func computePedestrianCrossings(paths []PedestrianPath, splines []Spline) []pedestrianCrossing {
	if len(paths) == 0 || len(splines) == 0 {
		return nil
	}
	var out []pedestrianCrossing
	for pathIdx, p := range paths {
		diff := vecSub(p.P1, p.P0)
		pathLen := sqrtf(vectorLengthSq(diff))
		if pathLen <= 1e-4 {
			continue
		}
		for si := range splines {
			s := &splines[si]
			if s.Length <= 0 {
				continue
			}
			for i := 1; i <= simSamples; i++ {
				a0 := s.Samples[i-1]
				a1 := s.Samples[i]
				splineT, pathT, ok := segmentIntersectionParams(a0, a1, p.P0, p.P1)
				if !ok {
					continue
				}
				// Treat each spline segment as half-open at its end so an
				// intersection exactly on a shared sample point is counted
				// once. The final segment keeps its endpoint.
				if splineT >= 1 && i < simSamples {
					continue
				}
				segLen := s.CumLen[i] - s.CumLen[i-1]
				splineDist := s.CumLen[i-1] + splineT*segLen
				out = append(out, pedestrianCrossing{
					SplineID:     s.ID,
					DistOnSpline: splineDist,
					PathIndex:    pathIdx,
					DistOnPath:   pathT * pathLen,
				})
			}
		}
	}
	return out
}

// buildPedestrianBlockedSplineDists returns, per spline ID, the distances at
// which a pedestrian is near enough to a crossing to block cars as if it were
// a red light. Duplicates may appear and are fine: the speed-cap function just
// picks the most restrictive one.
func buildPedestrianBlockedSplineDists(crossings []pedestrianCrossing, pedestrians []Pedestrian) map[int][]float32 {
	if len(crossings) == 0 || len(pedestrians) == 0 {
		return nil
	}
	byPath := make(map[int][]pedestrianCrossing, len(crossings))
	for _, c := range crossings {
		byPath[c.PathIndex] = append(byPath[c.PathIndex], c)
	}
	result := make(map[int][]float32)
	for _, ped := range pedestrians {
		pathCrossings := byPath[ped.PathIndex]
		if len(pathCrossings) == 0 {
			continue
		}
		for _, c := range pathCrossings {
			if absf(ped.Distance-c.DistOnPath) > pedestrianCrossingBlockRadiusM {
				continue
			}
			result[c.SplineID] = append(result[c.SplineID], c.DistOnSpline)
		}
	}
	return result
}

// computePedestrianCrossingSpeedCap mirrors computeTrafficLightSpeedCap for
// crossings blocked by pedestrians: a blocked crossing behaves like a red
// light. Cars with enough distance will brake to stop before it; cars too
// close simply miss the cap and keep going, matching red-light behaviour.
func computePedestrianCrossingSpeedCap(car Car, currentSpline *Spline, graph *RoadGraph, blockedBySpline map[int][]float32) float32 {
	if len(blockedBySpline) == 0 {
		return float32(math.MaxFloat32)
	}
	decel := car.Accel * 1.5
	result := float32(math.MaxFloat32)
	remaining := currentSpline.Length - car.DistanceOnSpline
	lookahead := car.Speed*car.Speed/(2*decel) + 20
	if lookahead > 200 {
		lookahead = 200
	}

	checkDist := func(rawDistAhead float32) {
		adj := rawDistAhead - stopFrontGapM
		if adj <= 0 {
			result = 0
			return
		}
		allowed := sqrtf(2 * decel * adj)
		if allowed < result {
			result = allowed
		}
	}

	for _, d := range blockedBySpline[currentSpline.ID] {
		if d <= car.DistanceOnSpline {
			continue
		}
		checkDist(d - car.DistanceOnSpline)
	}

	if remaining < lookahead {
		nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, car.DestinationSplineID, car.VehicleKind)
		if ok {
			if _, ok2 := graph.splinePtrByID(nextID); ok2 {
				for _, d := range blockedBySpline[nextID] {
					checkDist(remaining + d)
				}
			}
		}
	}

	return result
}

// anyCarOccupiesCrossing returns true if any car on the given spline is within
// pedestrianCrossingCarRadiusM of the crossing point. The car's body is
// [front - Length, front] where front is DistanceOnSpline. Stopped cars are
// treated as yielding so pedestrians don't deadlock against one that already
// came to rest on the crossing.
func anyCarOccupiesCrossing(cars []Car, splineID int, distOnSpline float32) bool {
	const stoppedCarSpeedMPS float32 = 0.5
	lo := distOnSpline - pedestrianCrossingCarRadiusM
	hi := distOnSpline + pedestrianCrossingCarRadiusM
	for _, car := range cars {
		if car.CurrentSplineID != splineID {
			continue
		}
		if car.Speed < stoppedCarSpeedMPS {
			continue
		}
		front := car.DistanceOnSpline
		rear := front - car.Length
		if front < lo || rear > hi {
			continue
		}
		return true
	}
	return false
}

// computePedestrianMovementCaps returns, for each pedestrian, the maximum
// canonical distance they are allowed to advance along their current path this
// step. MaxFloat32 means no cap. The cap is placed pedestrianCrossingStopBufferM
// before any crossing whose spline currently has a car in it. Pedestrians on a
// junction transition are skipped; they'll pick up a cap once they settle on
// the next path.
func computePedestrianMovementCaps(pedestrians []Pedestrian, topology pedestrianTopology, crossings []pedestrianCrossing, cars []Car) []float32 {
	caps := make([]float32, len(pedestrians))
	for i := range caps {
		caps[i] = float32(math.MaxFloat32)
	}
	if len(crossings) == 0 || len(cars) == 0 {
		return caps
	}
	byPath := make(map[int][]pedestrianCrossing, len(crossings))
	for _, c := range crossings {
		byPath[c.PathIndex] = append(byPath[c.PathIndex], c)
	}
	for i, ped := range pedestrians {
		if ped.TransitionActive {
			continue
		}
		if ped.PathIndex < 0 || ped.PathIndex >= len(topology.paths) {
			continue
		}
		path := topology.paths[ped.PathIndex]
		if !path.Valid {
			continue
		}
		pathCrossings := byPath[ped.PathIndex]
		if len(pathCrossings) == 0 {
			continue
		}
		pedCanonical := pedestrianCanonicalDistance(path, ped)
		for _, c := range pathCrossings {
			var canonicalStop float32
			if ped.Forward {
				if c.DistOnPath <= ped.Distance {
					continue
				}
				canonicalStop = c.DistOnPath - pedestrianCrossingStopBufferM
			} else {
				if c.DistOnPath >= ped.Distance {
					continue
				}
				canonicalStop = path.Length - (c.DistOnPath + pedestrianCrossingStopBufferM)
			}
			if canonicalStop < 0 {
				canonicalStop = 0
			}
			if !anyCarOccupiesCrossing(cars, c.SplineID, c.DistOnSpline) {
				continue
			}
			if canonicalStop < caps[i] {
				caps[i] = canonicalStop
			}
		}
		// Pedestrian already at/past the cap: keep them from sliding forward.
		if caps[i] < pedCanonical {
			caps[i] = pedCanonical
		}
	}
	return caps
}

func spawnPedestrianAtSource(source pedestrianSpawnSource) Pedestrian {
	sideBias := randRange(-pedestrianSideBiasM, pedestrianSideBiasM)
	baseSpeed := randRange(pedestrianSpeedMinMPS, pedestrianSpeedMaxMPS)
	return Pedestrian{
		PathIndex:     source.pathIndex,
		Forward:       source.forward,
		Distance:      pedestrianSpawnInsetM,
		Speed:         baseSpeed,
		BaseSpeed:     baseSpeed,
		Radius:        pedestrianRadiusM,
		LateralOffset: preferredPedestrianOffset(source.forward, sideBias),
		SideBias:      sideBias,
	}
}

func updatePedestrians(pedestrians []Pedestrian, paths []PedestrianPath, nextID *int, existingTimers map[pedestrianSpawnKey]float32, dt float32, crossings []pedestrianCrossing, cars []Car) ([]Pedestrian, map[pedestrianSpawnKey]float32) {
	if len(paths) == 0 {
		if existingTimers != nil {
			clear(existingTimers)
		}
		return pedestrians[:0], existingTimers
	}

	topology := buildPedestrianTopology(paths)
	active := make([]Pedestrian, 0, len(pedestrians))
	for _, ped := range pedestrians {
		if ped.PathIndex < 0 || ped.PathIndex >= len(topology.paths) {
			continue
		}
		path := topology.paths[ped.PathIndex]
		if !path.Valid {
			continue
		}
		if ped.BaseSpeed <= 0 {
			ped.BaseSpeed = randRange(pedestrianSpeedMinMPS, pedestrianSpeedMaxMPS)
		}
		if ped.Speed <= 0 {
			ped.Speed = ped.BaseSpeed
		}
		if ped.Radius <= 0 {
			ped.Radius = pedestrianRadiusM
		}
		ped.Distance = clampf(ped.Distance, 0, path.Length)
		active = append(active, ped)
	}

	if dt > 0 && len(active) > 0 {
		targetSpeeds, targetOffsets := computePedestrianTargets(active, topology)
		movementCaps := computePedestrianMovementCaps(active, topology, crossings, cars)
		for i := range active {
			ped := active[i]
			path := topology.paths[ped.PathIndex]
			blocked := false
			var canonicalCap float32
			if i < len(movementCaps) && movementCaps[i] < float32(math.MaxFloat32) {
				canonicalCap = movementCaps[i]
				headroom := canonicalCap - pedestrianCanonicalDistance(path, ped)
				if headroom <= 0.05 {
					blocked = true
					targetSpeeds[i] = 0
				}
			}
			ped.Speed += (targetSpeeds[i] - ped.Speed) * clampf(dt*pedestrianSpeedResponseRate, 0, 1)
			if !ped.TransitionActive {
				ped.LateralOffset += (targetOffsets[i] - ped.LateralOffset) * clampf(dt*pedestrianLateralResponseRate, 0, 1)
			}
			var moveSpeed float32
			if blocked {
				moveSpeed = 0
				ped.Speed = 0
			} else {
				moveSpeed = maxf(ped.Speed, ped.BaseSpeed*pedestrianMinSpeedFactor)
			}
			remaining := moveSpeed * dt
			if i < len(movementCaps) && movementCaps[i] < float32(math.MaxFloat32) && !ped.TransitionActive {
				headroom := movementCaps[i] - pedestrianCanonicalDistance(path, ped)
				if headroom < 0 {
					headroom = 0
				}
				if remaining > headroom {
					remaining = headroom
				}
			}
			alive := true
			for remaining > 0 && alive {
				if ped.TransitionActive {
					transitionRemaining := ped.TransitionLength - ped.TransitionDistance
					if remaining < transitionRemaining {
						ped.TransitionDistance += remaining
						remaining = 0
						break
					}
					remaining -= transitionRemaining
					ped.PathIndex = ped.TransitionNextPath
					ped.Forward = ped.TransitionNextForward
					ped.Distance = 0
					ped.LateralOffset = ped.TransitionEndOffset
					clearPedestrianTransition(&ped)
					continue
				}
				path = topology.paths[ped.PathIndex]
				distToEnd := path.Length - ped.Distance
				if remaining < distToEnd {
					ped.Distance += remaining
					remaining = 0
					break
				}
				remaining -= distToEnd
				nodeKey := path.Node1
				if !ped.Forward {
					nodeKey = path.Node0
				}
				nextPathIndex, nextForward, ok := chooseNextPedestrianEdge(topology, nodeKey, ped.PathIndex)
				if !ok {
					alive = false
					break
				}
				ped = buildPedestrianTurn(path, ped, topology.paths[nextPathIndex], nextForward)
			}
			if alive {
				active[i] = ped
			} else {
				active[i].PathIndex = -1
			}
		}
		write := active[:0]
		for _, ped := range active {
			if ped.PathIndex >= 0 {
				write = append(write, ped)
			}
		}
		active = write
	}

	nextTimers := make(map[pedestrianSpawnKey]float32, len(topology.deadEndSources))
	for _, source := range topology.deadEndSources {
		timer, ok := existingTimers[source.key]
		if !ok || timer <= 0 {
			timer = pedestrianSpawnDelay()
		}
		if dt > 0 {
			timer -= dt
		}
		for timer <= 0 {
			if canSpawnPedestrianAtSource(source, topology, active) {
				ped := spawnPedestrianAtSource(source)
				if nextID != nil {
					ped.ID = *nextID
					*nextID = *nextID + 1
				}
				active = append(active, ped)
				timer += pedestrianSpawnDelay()
			} else {
				timer += pedestrianSpawnRetryDelayS
				break
			}
		}
		nextTimers[source.key] = timer
	}
	return active, nextTimers
}

type SavedSplineFile struct {
	Splines         []SavedSpline         `json:"splines"`
	Routes          []SavedRoute          `json:"routes,omitempty"`
	Cars            []SavedCar            `json:"cars,omitempty"`
	TrafficLights   []SavedTrafficLight   `json:"traffic_lights,omitempty"`
	TrafficCycles   []SavedTrafficCycle   `json:"traffic_cycles,omitempty"`
	PedestrianPaths []SavedPedestrianPath `json:"pedestrian_paths,omitempty"`
}

type SavedPedestrianPath struct {
	P0 Vec2 `json:"p0"`
	P1 Vec2 `json:"p1"`
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
	ID                   int     `json:"id,omitempty"`
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
		Splines:         make([]SavedSpline, 0, len(w.Splines)),
		Routes:          make([]SavedRoute, 0, len(w.Routes)),
		Cars:            make([]SavedCar, 0, len(w.Cars)),
		TrafficLights:   make([]SavedTrafficLight, 0, len(w.TrafficLights)),
		TrafficCycles:   make([]SavedTrafficCycle, 0, len(w.TrafficCycles)),
		PedestrianPaths: make([]SavedPedestrianPath, 0, len(w.PedestrianPaths)),
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
		if car.ControlMode == CarControlExternal {
			continue
		}
		saved.Cars = append(saved.Cars, SavedCar{
			ID:                   car.ID,
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
	for _, p := range w.PedestrianPaths {
		saved.PedestrianPaths = append(saved.PedestrianPaths, SavedPedestrianPath{
			P0: p.P0,
			P1: p.P1,
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
	maxSplineID, maxRouteID, maxLightID, maxCycleID, maxCarID := 0, 0, 0, 0, 0

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
		carID := entry.ID
		if carID <= 0 {
			carID = maxCarID + 1
		}
		car := Car{
			ID:                   carID,
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
		if carID > maxCarID {
			maxCarID = carID
		}
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

	if len(saved.PedestrianPaths) > 0 {
		world.PedestrianPaths = make([]PedestrianPath, 0, len(saved.PedestrianPaths))
		for _, entry := range saved.PedestrianPaths {
			world.PedestrianPaths = append(world.PedestrianPaths, PedestrianPath{
				P0: entry.P0,
				P1: entry.P1,
			})
		}
	}

	world.NextSplineID = maxSplineID + 1
	world.NextRouteID = maxRouteID + 1
	world.NextLightID = maxLightID + 1
	world.NextCycleID = maxCycleID + 1
	world.NextCarID = maxCarID + 1
	return &world, nil
}

func ComputeRoutePathWithGraph(route Route, graph *RoadGraph) ([]int, float32, string, bool) {
	if graph == nil {
		return nil, 0, "", false
	}
	if route.VehicleKind == VehicleCar {
		if startSpline, ok := graph.splinePtrByID(route.StartSplineID); ok && startSpline.BusOnly {
			return nil, 0, "Bus-only splines cannot be used for car routes.", false
		}
		if endSpline, ok := graph.splinePtrByID(route.EndSplineID); ok && endSpline.BusOnly {
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

func PickBestBusStopSpline(graph *RoadGraph, nodeKey NodeKey, fromSplineID, toSplineID int) (BusStop, bool) {
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
		firstCost, okFirst := PathCostToDestinationWithGraph(graph, fromSplineID, candidate.ID, VehicleBus)
		secondCost, okSecond := PathCostToDestinationWithGraph(graph, candidate.ID, toSplineID, VehicleBus)
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

func updateRouteSpawning(routes []Route, cars []Car, splines []Spline, nextCarID *int, dt float32) ([]Route, []Car) {
	for i := range routes {
		if !routes[i].Valid || routes[i].SpawnPerMinute <= 0 {
			continue
		}
		routes[i].NextSpawnIn -= dt
		if routes[i].NextSpawnIn > 0 {
			continue
		}
		candidate := spawnVehicle(*nextCarID, routes[i], splines)
		if !spawnBlocked(candidate, cars, splines) {
			cars = append(cars, candidate)
			*nextCarID = *nextCarID + 1
			routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
		}
	}
	return routes, cars
}

func spawnBlocked(candidate Car, cars []Car, splines []Spline) bool {
	spline, ok := findSplinePtrByID(splines, candidate.CurrentSplineID)
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
		otherSpline, ok := findSplinePtrByID(splines, other.CurrentSplineID)
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

func spawnVehicle(carID int, route Route, splines []Spline) Car {
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
		ID:                   carID,
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
	if spline, ok := findSplinePtrByID(splines, route.StartSplineID); ok {
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

func computeBusStopSpeedCap(car Car, route Route, currentSpline *Spline, graph *RoadGraph) float32 {
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
		return sqrtf(2 * decel * adj)
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

func shouldBeginBusStopDwell(car Car, route Route, currentSpline *Spline, graph *RoadGraph) bool {
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
		floorf32(p.X * g.invCellSize),
		floorf32(p.Y * g.invCellSize),
	}
}

func (g *spatialGrid) insert(index int, p Vec2, radius float32) {
	if radius <= g.cellSize {
		k := g.cellKey(p)
		g.cells[k] = append(g.cells[k], index)
		return
	}
	minX := floorf32((p.X - radius) * g.invCellSize)
	maxX := floorf32((p.X + radius) * g.invCellSize)
	minY := floorf32((p.Y - radius) * g.invCellSize)
	maxY := floorf32((p.Y + radius) * g.invCellSize)
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

type splineSampleCursor struct {
	splineID int
	idx      int
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
// closingRateScale returns a multiplier in [0.3, 1.0] for the broad-phase
// distance threshold based on how fast two cars are approaching each other.
// Takes precomputed displacement vector and its squared length to avoid
// redundant computation at call sites.
func closingRateScale(displacement Vec2, dLenSq float32, headingI, headingJ Vec2, speedI, speedJ float32) float32 {
	if dLenSq < 1e-12 {
		return 1.0
	}
	maxClosing := speedI + speedJ
	if maxClosing < 1e-6 {
		return 1.0
	}
	// ratio = dot(velI - velJ, displacement) / (|displacement| * maxClosing)
	numerator := dot(vecScale(headingI, speedI), displacement) - dot(vecScale(headingJ, speedJ), displacement)
	invDLenMaxClosing := 1.0 / (sqrtf(dLenSq) * maxClosing)
	ratio := numerator * invDLenMaxClosing
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
	type conflictScanResult struct {
		initialBlame           []bool
		stationaryPredictions  [][]TrajectorySample
		tentativeLinks         []DebugBlameLink
		candidateLinks         []DebugBlameLink
		primaryPairCandidates  int
		primaryBroadPhasePairs int
		primaryCollisionChecks int
		primaryCollisionHits   int
		stationaryPredictions_ int
		stationaryCollChecks   int
		stationaryCollHits     int
		totalPredictions       int
		totalPredSamples       int
	}
	workers := parallelWorkerCount(len(cars))
	scanResults := make([]conflictScanResult, workers)
	chunkSize := (len(cars) + workers - 1) / workers
	var scanWg sync.WaitGroup
	for w := 0; w < workers; w++ {
		wStart := w * chunkSize
		if wStart >= len(cars) {
			break
		}
		wEnd := wStart + chunkSize
		if wEnd > len(cars) {
			wEnd = len(cars)
		}
		scanWg.Add(1)
		go func(workerIdx, iStart, iEnd int) {
			defer scanWg.Done()
			r := &scanResults[workerIdx]
			r.initialBlame = make([]bool, len(cars))
			r.stationaryPredictions = make([][]TrajectorySample, len(cars))
			seenJ := make([]bool, len(cars))
			var seenJReset []int
			neighborBuf := make([]int, 0, 32)
			for i := iStart; i < iEnd; i++ {
				if len(predictions[i]) == 0 {
					continue
				}
				for _, idx := range seenJReset {
					seenJ[idx] = false
				}
				seenJReset = seenJReset[:0]
				neighborBuf = grid.queryNeighbors(poses[i].pos, neighborBuf)
				for _, j := range neighborBuf {
					if j <= i {
						continue
					}
					if seenJ[j] {
						continue
					}
					seenJ[j] = true
					seenJReset = append(seenJReset, j)
					r.primaryPairCandidates++
					disp := vecSub(poses[j].pos, poses[i].pos)
					dSq := vectorLengthSq(disp)
					scale := closingRateScale(disp, dSq, poses[i].heading, poses[j].heading, cars[i].Speed, cars[j].Speed)
					broadPhaseDist := (reach[i] + reach[j]) * scale
					if dSq > broadPhaseDist*broadPhaseDist {
						continue
					}
					r.primaryBroadPhasePairs++
					if !aabbOverlap(trajAABBs[i], trajAABBs[j]) {
						continue
					}
					if debugSelectedCar >= 0 && debugSelectedCarMode == 0 && (i == debugSelectedCar || j == debugSelectedCar) {
						r.candidateLinks = append(r.candidateLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
					}
					r.primaryCollisionChecks++
					collision, ok := predictCollision(predictions[i], predictions[j], geometries[i], geometries[j])
					if !ok {
						continue
					}
					r.primaryCollisionHits++
					blameI, blameJ := determineBlame(collision, cars[i], cars[j], graph)
					alreadyCollided := collision.AlreadyCollided
					if blameI && recentlyLeft(cars[i], cars[j].CurrentSplineID) {
						blameI = false
					}
					if blameJ && recentlyLeft(cars[j], cars[i].CurrentSplineID) {
						blameJ = false
					}
					// Stationary exoneration: if car i would still collide with j
					// even by stopping dead, then i is not actually the cause — it's
					// already where it should be and j is driving into it. Clearing
					// blameI here is what stops long queues from cascading into
					// "everyone brakes for everyone ahead".
					if blameI && !alreadyCollided {
						if r.stationaryPredictions[i] == nil {
							sc := cars[i]
							sc.Speed = 0
							r.stationaryPredictions[i] = predictCarTrajectory(sc, graph, predictionHorizonSeconds, predictionStepSeconds)
							r.stationaryPredictions_++
							r.totalPredictions++
							r.totalPredSamples += len(r.stationaryPredictions[i])
						}
						r.stationaryCollChecks++
						if _, still := predictCollision(r.stationaryPredictions[i], predictions[j], geometries[i], geometries[j]); still {
							r.stationaryCollHits++
							blameI = false
						}
					}
					if blameJ && !alreadyCollided {
						if r.stationaryPredictions[j] == nil {
							sc := cars[j]
							sc.Speed = 0
							r.stationaryPredictions[j] = predictCarTrajectory(sc, graph, predictionHorizonSeconds, predictionStepSeconds)
							r.stationaryPredictions_++
							r.totalPredictions++
							r.totalPredSamples += len(r.stationaryPredictions[j])
						}
						r.stationaryCollChecks++
						if _, still := predictCollision(r.stationaryPredictions[j], predictions[i], geometries[j], geometries[i]); still {
							r.stationaryCollHits++
							blameJ = false
						}
					}
					if blameI {
						r.initialBlame[i] = true
						r.tentativeLinks = append(r.tentativeLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
					}
					if blameJ {
						r.initialBlame[j] = true
						r.tentativeLinks = append(r.tentativeLinks, DebugBlameLink{FromCarIndex: j, ToCarIndex: i})
					}
				}
			}
		}(w, wStart, wEnd)
	}
	scanWg.Wait()

	// Merge per-worker results.
	for _, r := range scanResults {
		for i, blamed := range r.initialBlame {
			if blamed {
				initialBlame[i] = true
			}
		}
		for i, sp := range r.stationaryPredictions {
			if sp != nil && stationaryPredictions[i] == nil {
				stationaryPredictions[i] = sp
			}
		}
		tentativeLinks = append(tentativeLinks, r.tentativeLinks...)
		candidateLinks = append(candidateLinks, r.candidateLinks...)
		profile.PrimaryPairCandidates += r.primaryPairCandidates
		profile.PrimaryBroadPhasePairs += r.primaryBroadPhasePairs
		profile.PrimaryCollisionChecks += r.primaryCollisionChecks
		profile.PrimaryCollisionHits += r.primaryCollisionHits
		profile.StationaryPredictions += r.stationaryPredictions_
		profile.StationaryCollisionChecks += r.stationaryCollChecks
		profile.StationaryCollisionHits += r.stationaryCollHits
		profile.TotalPredictions += r.totalPredictions
		profile.TotalPredictionSamples += r.totalPredSamples
	}
	for _, blamed := range initialBlame {
		if blamed {
			profile.InitiallyBlamedCars++
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
			if spline, ok := graph.splinePtrByID(car.CurrentSplineID); ok {
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
		disp := vecSub(poses[j].pos, poses[i].pos)
		dSq := vectorLengthSq(disp)
		scale := closingRateScale(disp, dSq, poses[i].heading, poses[j].heading, cars[i].Speed, cars[j].Speed)
		broadPhaseDist := (reach[i] + reach[j]) * scale * scale
		if dSq > broadPhaseDist*broadPhaseDist {
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
		blameI, _ := determineBlame(collision, fasterCar, cars[j], graph)
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
	currentSpline, ok := graph.splinePtrByID(car.CurrentSplineID)
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
		disp := vecSub(poses[otherIndex].pos, poses[carIndex].pos)
		dSq := vectorLengthSq(disp)
		scale := closingRateScale(disp, dSq, poses[carIndex].heading, poses[otherIndex].heading, cars[carIndex].Speed, cars[otherIndex].Speed)
		broadPhaseDist := (reach[carIndex] + reach[otherIndex]) * scale
		if dSq > broadPhaseDist*broadPhaseDist {
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

		blameTestCar, _ := determineBlame(collision, testCar, otherCar, graph)
		if blameTestCar {
			return true
		}
	}
	return false
}

// detectDeadlockReleases finds short blame cycles (length ≤ 4) in the
// "A is blamed for blocking B" graph and picks one car per cycle to
// release (clear its brake AND hold flags). The victim is the cycle
// member with the lowest car index — arbitrary but deterministic, which
// matters for reproducibility. Without this, a symmetric four-way yield
// at an intersection would gridlock indefinitely.
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
	var sampleCursor splineSampleCursor
	sampleCursor.splineID = -1

	if speed <= 0.01 {
		spline, ok := graph.splinePtrByID(simCar.CurrentSplineID)
		if !ok {
			return nil
		}
		splinePos, splineTangent, _ := sampleSplineStateAtDistance(spline, simCar.DistanceOnSpline, &sampleCursor)
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

		baseSample := TrajectorySample{
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
			baseSample.HasTrailer = true
			baseSample.TrailerPosition = vecScale(vecAdd(hitchPos, simTrailerRearPos), 0.5)
			baseSample.TrailerHeading = normalize(vecSub(hitchPos, simTrailerRearPos))
		}

		for stepIndex := 0; stepIndex <= steps; stepIndex++ {
			sample := baseSample
			sample.Time = float32(stepIndex) * step
			samples = append(samples, sample)
		}
		return samples
	}

	for stepIndex := 0; stepIndex <= steps; stepIndex++ {
		spline, ok := graph.splinePtrByID(simCar.CurrentSplineID)
		if !ok {
			break
		}
		splinePos, splineTangent, κ := sampleSplineStateAtDistance(spline, simCar.DistanceOnSpline, &sampleCursor)
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
			currentSpline, ok := graph.splinePtrByID(simCar.CurrentSplineID)
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

func determineBlame(collision CollisionPrediction, carA, carB Car, graph *RoadGraph) (bool, bool) {
	if collision.AlreadyCollided {
		if headingAngleDegrees(collision.HeadingA, collision.HeadingB) >= 60 {
			return false, false
		}
		return blameRearCar(collision, carA, carB)
	}
	if collision.PriorityA != collision.PriorityB {
		if collision.PriorityA {
			if normalCarOccupiesPriorityCollisionSpline(carB, collision.SplineAID, graph) {
				return true, false
			}
			return false, true
		}
		if normalCarOccupiesPriorityCollisionSpline(carA, collision.SplineBID, graph) {
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

func normalCarOccupiesPriorityCollisionSpline(car Car, prioritySplineID int, graph *RoadGraph) bool {
	if prioritySplineID < 0 {
		return false
	}
	prioritySpline, ok := graph.splinePtrByID(prioritySplineID)
	if !ok || !prioritySpline.Priority {
		return false
	}
	return carHitboxTouchesSpline(car, prioritySpline, graph)
}

func carHitboxTouchesSpline(car Car, targetSpline *Spline, graph *RoadGraph) bool {
	currentSpline, ok := graph.splinePtrByID(car.CurrentSplineID)
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

func computeFollowingSpeedCaps(cars []Car, graph *RoadGraph) ([]float32, FollowingProfile) {
	caps := make([]float32, len(cars))
	profile := FollowingProfile{Cars: len(cars)}
	for i := range caps {
		caps[i] = math.MaxFloat32
	}

	poses := make([]carPose, len(cars))
	poseStart := time.Now()
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
	profile.PoseMS = sinceMS(poseStart)

	indexStart := time.Now()
	carsBySpline := make([][]int, len(graph.splines))
	for i := range cars {
		if splineIdx, ok := graph.indexByID[cars[i].CurrentSplineID]; ok {
			carsBySpline[splineIdx] = append(carsBySpline[splineIdx], i)
		}
	}
	profile.IndexMS = sinceMS(indexStart)

	followLookaheadSq := followLookaheadM * followLookaheadM

	scanStart := time.Now()
	var candidateRefs atomic.Int64
	parallelFor(len(cars), func(start, end int) {
		localRefs := int64(0)
		for i := start; i < end; i++ {
			car := cars[i]
			currentSplineIdx, ok := graph.indexByID[car.CurrentSplineID]
			if !ok {
				continue
			}
			hI := poses[i].heading
			pI := poses[i].pos
			desiredGap := followMinGapM + car.Speed*followTimeHeadwaySecs
			desiredGapSq := desiredGap * desiredGap
			bestDistSq := float32(math.MaxFloat32)
			bestSpeed := float32(math.MaxFloat32)

			evalSplineCars := func(splineIdx int) {
				for _, j := range carsBySpline[splineIdx] {
					localRefs++
					if j == i {
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
					dSq := diff.X*diff.X + diff.Y*diff.Y
					if dSq > followLookaheadSq {
						continue
					}
					leaderRear := cars[j].RearPosition
					if cars[j].Trailer.HasTrailer {
						leaderRear = cars[j].Trailer.RearPosition
					}
					rearDiff := vecSub(leaderRear, pI)
					gapSq := rearDiff.X*rearDiff.X + rearDiff.Y*rearDiff.Y
					if gapSq > desiredGapSq {
						continue
					}
					if dSq < bestDistSq {
						bestDistSq = dSq
						bestSpeed = cars[j].Speed
					}
				}
			}

			evalSplineCars(currentSplineIdx)

			currentSpline := graph.splines[currentSplineIdx]
			covered := currentSpline.Length - car.DistanceOnSpline
			tree, ok := graph.routeTree(car.DestinationSplineID, car.VehicleKind)
			if ok {
				curIdx := currentSplineIdx
				for steps := 0; covered < followLookaheadM && steps < len(graph.splines); steps++ {
					if curIdx < 0 || curIdx >= len(tree.nextHop) {
						break
					}
					nextIdx := tree.nextHop[curIdx]
					if nextIdx < 0 || nextIdx >= len(graph.splines) || nextIdx == curIdx {
						break
					}
					evalSplineCars(nextIdx)
					covered += graph.splines[nextIdx].Length
					curIdx = nextIdx
				}
			}
			caps[i] = bestSpeed
		}
		candidateRefs.Add(localRefs)
	})
	profile.CandidateRefs = int(candidateRefs.Load())
	profile.CandidateMS = 0
	profile.ScanMS = sinceMS(scanStart)
	return caps, profile
}

type pendingTransitionCar struct {
	originalIndex   int
	car             Car
	route           Route
	followCap       float32
	shouldHoldSpeed bool
}

type updateCarsFastStatus uint8

const (
	updateCarsFastRemove updateCarsFastStatus = iota
	updateCarsFastDwellParked
	updateCarsFastDwellBegin
	updateCarsFastAlive
	updateCarsFastTransition
)

type updateCarsFastResult struct {
	status          updateCarsFastStatus
	car             Car
	route           Route
	followCap       float32
	shouldHoldSpeed bool
	beginDwellAfter bool
}

func appendUpdatedCar(alive []Car, indexRemap []int, originalIndex int, car Car) []Car {
	indexRemap[originalIndex] = len(alive)
	return append(alive, car)
}

func applyCurrentSplineSpeedUpdate(car *Car, route Route, currentSpline *Spline, graph *RoadGraph, stoppingLightsBySpline map[int][]TrafficLight, pedestrianBlockedBySpline map[int][]float32, followCap float32, shouldHoldSpeed bool, dt float32) {
	targetSpeed := car.MaxSpeed * currentSpline.SpeedFactor
	if currentSpline.SpeedLimitKmh > 0 {
		if limitMPS := currentSpline.SpeedLimitKmh / 3.6; limitMPS < targetSpeed {
			targetSpeed = limitMPS
		}
	}
	if cs := lookupCurveSpeed(currentSpline, car.DistanceOnSpline) * car.CurveSpeedMultiplier; cs < targetSpeed {
		targetSpeed = cs
	}
	if at := computeAnticipatoryTargetSpeed(*car, currentSpline, graph); at < targetSpeed {
		targetSpeed = at
	}
	if tl := computeTrafficLightSpeedCap(*car, currentSpline, graph, stoppingLightsBySpline); tl < targetSpeed {
		targetSpeed = tl
	}
	if pc := computePedestrianCrossingSpeedCap(*car, currentSpline, graph, pedestrianBlockedBySpline); pc < targetSpeed {
		targetSpeed = pc
	}
	if bs := computeBusStopSpeedCap(*car, route, currentSpline, graph); bs < targetSpeed {
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
}

func settleCarOnCurrentSpline(car Car, currentSpline *Spline, dt float32, sampleCursor *splineSampleCursor) Car {
	splinePos, tangent, κ := sampleSplineStateAtDistance(currentSpline, car.DistanceOnSpline, sampleCursor)
	wb := car.Length * wheelbaseFrac
	targetOffset := κ * wb * wb / 6
	if car.Trailer.HasTrailer {
		trailerWb := car.Trailer.Length * wheelbaseFrac
		targetOffset = κ * (wb*wb + trailerWb*trailerWb) / 6
	}
	lerpRate := car.Speed / (2 * (wb + 0.01))
	car.LateralOffset += (targetOffset - car.LateralOffset) * clampf(lerpRate*dt, 0, 1)

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
	return car
}

func transitionCarToNextSpline(car *Car, route Route, graph *RoadGraph) bool {
	if car.CurrentSplineID == car.DestinationSplineID {
		return false
	}
	if car.LaneChanging && car.CurrentSplineID == car.LaneChangeSplineID {
		car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
		car.PrevSplineIDs[0] = car.CurrentSplineID
		car.CurrentSplineID = car.AfterSplineID
		car.DistanceOnSpline = car.AfterSplineDist + car.DistanceOnSpline
		car.LaneChanging = false
		car.LaneChangeSplineID = -1
		if car.CurrentSplineID == car.DesiredLaneSplineID {
			car.DesiredLaneSplineID = -1
			car.DesiredLaneDeadline = 0
		}
		return true
	}

	nextSplineID, ok := ChooseNextSplineOnBestPathWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
	if ok {
		if car.DesiredLaneSplineID >= 0 {
			car.DesiredLaneSplineID = -1
			car.DesiredLaneDeadline = 0
		}
	} else {
		forcedNext, desiredLane, forcedOK := FindForcedLaneChangePathWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
		if !forcedOK {
			return false
		}
		nextSplineID = forcedNext
		car.DesiredLaneSplineID = desiredLane
		if src, srcOK := graph.splinePtrByID(forcedNext); srcOK {
			car.DesiredLaneDeadline = maxf(src.Length-laneChangeForcedDistEnd, 0)
		}
	}

	car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
	car.PrevSplineIDs[0] = car.CurrentSplineID
	car.CurrentSplineID = nextSplineID
	resumeBusRouteAfterStop(route, car)

	if car.DesiredLaneSplineID < 0 && car.CurrentSplineID != car.DestinationSplineID {
		if newSpline, ok := graph.splinePtrByID(car.CurrentSplineID); ok && len(newSpline.HardCoupledIDs) > 0 {
			curTime, curOK := PathCostToDestinationWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			if curOK {
				for _, cid := range newSpline.HardCoupledIDs {
					coupledTime, coupledOK := PathCostToDestinationWithGraph(graph, cid, car.DestinationSplineID, car.VehicleKind)
					if coupledOK && curTime-coupledTime >= 20.0 {
						car.DesiredLaneSplineID = cid
						car.DesiredLaneDeadline = maxf(newSpline.Length-laneChangeForcedDistEnd, 0)
						break
					}
				}
			}
		}
	}
	return true
}

func resolveTransitionCar(pending pendingTransitionCar, graph *RoadGraph, stoppingLightsBySpline map[int][]TrafficLight, pedestrianBlockedBySpline map[int][]float32, dt float32, alive []Car, indexRemap []int) ([]Car, bool) {
	car := pending.car
	var sampleCursor splineSampleCursor
	sampleCursor.splineID = -1
	for {
		currentSpline, ok := graph.splinePtrByID(car.CurrentSplineID)
		if !ok {
			return alive, false
		}

		car.DistanceOnSpline -= currentSpline.Length
		if !transitionCarToNextSpline(&car, pending.route, graph) {
			return alive, false
		}

		currentSpline, ok = graph.splinePtrByID(car.CurrentSplineID)
		if !ok {
			return alive, false
		}
		if shouldBeginBusStopDwell(car, pending.route, currentSpline, graph) {
			beginBusStopDwell(pending.route, &car)
			return appendUpdatedCar(alive, indexRemap, pending.originalIndex, car), true
		}

		applyCurrentSplineSpeedUpdate(&car, pending.route, currentSpline, graph, stoppingLightsBySpline, pedestrianBlockedBySpline, pending.followCap, pending.shouldHoldSpeed, dt)
		if car.DistanceOnSpline <= currentSpline.Length {
			car = settleCarOnCurrentSpline(car, currentSpline, dt, &sampleCursor)
			if shouldBeginBusStopDwell(car, pending.route, currentSpline, graph) {
				beginBusStopDwell(pending.route, &car)
			}
			return appendUpdatedCar(alive, indexRemap, pending.originalIndex, car), true
		}
	}
}

func updateCars(cars []Car, routes []Route, graph *RoadGraph, brakingDecisions []bool, holdSpeedDecisions []bool, followCaps []float32, lights []TrafficLight, cycles []TrafficCycle, pedestrianBlockedBySpline map[int][]float32, dt float32) ([]Car, []int, UpdateCarsProfile) {
	profile := UpdateCarsProfile{Cars: len(cars)}
	if len(cars) == 0 {
		return cars, nil, profile
	}

	setupStart := time.Now()
	routeIndexByID := map[int]int{}
	for i, route := range routes {
		routeIndexByID[route.ID] = i
	}
	stoppingLightsBySpline := buildStoppingTrafficLightsBySpline(lights, cycles)
	profile.SetupMS = sinceMS(setupStart)

	indexRemap := make([]int, len(cars))
	for i := range indexRemap {
		indexRemap[i] = -1
	}
	alive := cars[:0]
	fastResults := make([]updateCarsFastResult, len(cars))

	fastPathStart := time.Now()
	parallelFor(len(cars), func(start, end int) {
		for i := start; i < end; i++ {
			result := &fastResults[i]
			car := cars[i]
			routeIdx, ok := routeIndexByID[car.RouteID]
			if !ok || !routes[routeIdx].Valid {
				result.status = updateCarsFastRemove
				continue
			}
			route := routes[routeIdx]
			result.route = route
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
				result.status = updateCarsFastDwellParked
				result.car = car
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

			currentSpline, ok := graph.splinePtrByID(car.CurrentSplineID)
			if !ok {
				result.status = updateCarsFastRemove
				continue
			}

			const frustrateThreshMPS = 10.0 / 3.6
			if !shouldBrake && followCap < float32(math.MaxFloat32) {
				preferredSpeed := car.MaxSpeed * currentSpline.SpeedFactor
				if currentSpline.SpeedLimitKmh > 0 {
					if limitMPS := currentSpline.SpeedLimitKmh / 3.6; limitMPS < preferredSpeed {
						preferredSpeed = limitMPS
					}
				}
				if cs := lookupCurveSpeed(currentSpline, car.DistanceOnSpline) * car.CurveSpeedMultiplier; cs < preferredSpeed {
					preferredSpeed = cs
				}
				if preferredSpeed-car.Speed >= frustrateThreshMPS {
					car.SlowedTimer += dt
				} else {
					car.SlowedTimer = 0
				}
			} else {
				car.SlowedTimer = 0
			}

			if shouldBeginBusStopDwell(car, route, currentSpline, graph) {
				result.status = updateCarsFastDwellBegin
				result.car = car
				continue
			}

			applyCurrentSplineSpeedUpdate(&car, route, currentSpline, graph, stoppingLightsBySpline, pedestrianBlockedBySpline, followCap, shouldHoldSpeed, dt)
			if car.DistanceOnSpline <= currentSpline.Length {
				var sampleCursor splineSampleCursor
				sampleCursor.splineID = -1
				car = settleCarOnCurrentSpline(car, currentSpline, dt, &sampleCursor)
				result.status = updateCarsFastAlive
				result.car = car
				result.beginDwellAfter = shouldBeginBusStopDwell(car, route, currentSpline, graph)
				continue
			}

			result.status = updateCarsFastTransition
			result.car = car
			result.followCap = followCap
			result.shouldHoldSpeed = shouldHoldSpeed
		}
	})
	pendingTransitions := make([]pendingTransitionCar, 0, len(cars)/8)
	for i, result := range fastResults {
		switch result.status {
		case updateCarsFastRemove:
			profile.RemovedCars++
		case updateCarsFastDwellParked:
			profile.DwellCars++
			alive = appendUpdatedCar(alive, indexRemap, i, result.car)
		case updateCarsFastDwellBegin:
			car := result.car
			beginBusStopDwell(result.route, &car)
			profile.DwellCars++
			alive = appendUpdatedCar(alive, indexRemap, i, car)
		case updateCarsFastAlive:
			car := result.car
			if result.beginDwellAfter {
				beginBusStopDwell(result.route, &car)
			}
			profile.FastPathCars++
			alive = appendUpdatedCar(alive, indexRemap, i, car)
		case updateCarsFastTransition:
			profile.TransitionCars++
			pendingTransitions = append(pendingTransitions, pendingTransitionCar{
				originalIndex:   i,
				car:             result.car,
				route:           result.route,
				followCap:       result.followCap,
				shouldHoldSpeed: result.shouldHoldSpeed,
			})
		}
	}
	profile.FastPathMS = sinceMS(fastPathStart)

	transitionStart := time.Now()
	for _, pending := range pendingTransitions {
		var survived bool
		alive, survived = resolveTransitionCar(pending, graph, stoppingLightsBySpline, pedestrianBlockedBySpline, dt, alive, indexRemap)
		if !survived {
			profile.RemovedCars++
		}
	}
	profile.TransitionMS = sinceMS(transitionStart)

	return alive, indexRemap, profile
}

func laneChangeFeasibleAt(src, dst Spline, distance, speed float32) bool {
	carPos, carHeading := sampleSplineAtDistance(&src, distance)
	halfDist := speed * laneChangeHalfSecs
	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(&dst, p1)
	if crossDist == 0 || dst.Length-crossDist < halfDist {
		return false
	}
	_, destHeading := sampleSplineAtDistance(&dst, crossDist+halfDist)
	return carHeading.X*destHeading.X+carHeading.Y*destHeading.Y >= laneChangeDirCos
}

func EffectiveMaxSpeedMPS(spline Spline) float32 {
	if spline.SpeedLimitKmh > 0 {
		return minf(maxCarSpeed*spline.SpeedFactor, spline.SpeedLimitKmh/3.6)
	}
	return maxCarSpeed * spline.SpeedFactor
}

func FindForcedLaneChangePathWithGraph(graph *RoadGraph, currentSplineID, destSplineID int, vehicleKind VehicleKind) (nextSplineID, desiredSplineID int, ok bool) {
	currentSpline, found := graph.splinePtrByID(currentSplineID)
	if !found {
		return 0, 0, false
	}
	for _, nextIdx := range graph.startsByNode[nodeKeyFromVec2(currentSpline.P3)] {
		nextSpline := graph.splines[nextIdx]
		if !isSplineUsableForVehicle(nextSpline, vehicleKind) {
			continue
		}
		for _, coupledID := range nextSpline.HardCoupledIDs {
			if coupledSpline, ok := graph.splinePtrByID(coupledID); ok && !isSplineUsableForVehicle(coupledSpline, vehicleKind) {
				continue
			}
			if _, pathOk := PathCostToDestinationWithGraph(graph, coupledID, destSplineID, vehicleKind); pathOk {
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
	carPos, carHeading := sampleSplineAtDistance(&srcSpline, car.DistanceOnSpline)
	halfDist := car.Speed * laneChangeHalfSecs
	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(&destSpline, p1)
	if crossDist == 0 || destSpline.Length-crossDist < halfDist {
		return 0, false
	}
	p3Dist := crossDist + halfDist
	_, destHeading := sampleSplineAtDistance(&destSpline, p3Dist)
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

// buildLaneChangeBridge synthesises a cubic Bezier that peels off the
// current spline and lands on destSplineID ~halfDist ahead of the nearest
// projection. P1 is halfDist forward along the current heading, P2 is
// halfDist back from P3 along the destination heading — this keeps the
// curve C1-continuous at both ends. The bridge is appended to lcs and
// the car is switched onto it; car.AfterSplineID/AfterSplineDist tell
// updateCars where to land when the bridge runs out. Returns lcs unchanged
// and false if no viable landing exists (not enough room, wrong heading,
// or too slow unless forced).
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
	carPos, carHeading := sampleSplineAtDistance(&srcSpline, car.DistanceOnSpline)
	effectiveSpeed := car.Speed
	if forced && effectiveSpeed < laneChangeMinSpeed {
		effectiveSpeed = laneChangeMinSpeed
	}
	halfDist := effectiveSpeed * laneChangeHalfSecs

	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(&destSpline, p1)
	if crossDist == 0 || destSpline.Length-crossDist < halfDist {
		return lcs, false
	}
	p3Dist := crossDist + halfDist
	p3, destHeading := sampleSplineAtDistance(&destSpline, p3Dist)
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

// computeLaneChanges decides, for each car not already mid-change, whether
// to start a new lane change. Three triggers, in this order:
//  1. DesiredLaneSplineID set (forced — usually because pathfinding says
//     the only route to the destination runs through a coupled sibling
//     lane). If the forced deadline has arrived, the bridge is built even
//     if it's geometrically tight.
//  2. Preference-based: every preferenceChangeCooldownS seconds, look at
//     HardCoupled+SoftCoupled siblings with a lower LanePreference and
//     consider switching if the path cost penalty is bounded.
//  3. Overtake: if the car has been stuck below its preferred speed for
//     longer than overtakeSlowThresholdS behind a slower leader, try to
//     jump to a sibling lane.
//
// Every candidate goes through laneChangeLandingDist + isLaneChangeLandingSafe
// before a bridge is synthesised.
func computeLaneChanges(cars []Car, splines []Spline, lcs []Spline, nextID *int, graph *RoadGraph, dt float32) ([]Spline, []Car) {
	splineIndexByID := graph.indexByID
	poses := make([]carPose, len(cars))
	carsBySpline := make(map[int][]int, len(cars))
	for i, car := range cars {
		if splineIdx, ok := splineIndexByID[car.CurrentSplineID]; ok {
			pos, heading := sampleSplineAtDistance(&splines[splineIdx], car.DistanceOnSpline)
			poses[i] = carPose{pos: pos, heading: heading}
		}
		carsBySpline[car.CurrentSplineID] = append(carsBySpline[car.CurrentSplineID], i)
	}

	for i := range cars {
		car := &cars[i]
		if car.ControlMode == CarControlExternal {
			continue
		}
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
			curPrefTime, curPrefTimeOk := PathCostToDestinationWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			for _, destID := range findBetterPreferenceLaneCandidates(*car, splines, splineIndexByID) {
				destTime, pathOk := PathCostToDestinationWithGraph(graph, destID, car.DestinationSplineID, car.VehicleKind)
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
			curOvertakeTime, curOvertakeTimeOk := PathCostToDestinationWithGraph(graph, car.CurrentSplineID, car.DestinationSplineID, car.VehicleKind)
			for _, destID := range findOvertakeLaneCandidates(*car, splines, splineIndexByID) {
				destTime, pathOk := PathCostToDestinationWithGraph(graph, destID, car.DestinationSplineID, car.VehicleKind)
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
	currentSpline := &splines[idx]
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
		coupled := &splines[cIdx]
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
		euclidean := sqrtf(diff.X*diff.X + diff.Y*diff.Y)
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
	currentSpline := &splines[idx]
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

// cacheSpline populates the sampled-lookup tables that the rest of the
// code assumes exist: Samples, SampleTangents, SampleCurv, CumLen are all
// length simSamples+1; Length and CurveSpeedMPS/TravelTimeS are derived
// from them. Any edit to P0..P3, SpeedLimitKmh, etc. MUST be followed by
// a RebuildSpline call or these tables will desync from the geometry.
func cacheSpline(s *Spline) {
	if len(s.SampleTangents) != simSamples+1 {
		s.SampleTangents = make([]Vec2, simSamples+1)
	}
	if len(s.SampleCurv) != simSamples+1 {
		s.SampleCurv = make([]float32, simSamples+1)
	}

	total := float32(0)
	var prev Vec2
	for i := 0; i <= simSamples; i++ {
		t := float32(i) / float32(simSamples)
		pt := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)
		s.Samples[i] = pt
		s.SampleTangents[i] = normalize(bezierDerivative(*s, t))
		if i == 0 {
			s.CumLen[i] = 0
			prev = pt
			continue
		}
		total += sqrtf(distSq(prev, pt))
		s.CumLen[i] = total
		prev = pt
	}
	for i := 0; i <= simSamples; i++ {
		s.SampleCurv[i] = sampleCurvatureAtIndex(&s.Samples, i)
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
	curvature := absf(signedCurvatureAtArcLen(s, d))
	if curvature < 1e-6 {
		return maxCarSpeed
	}
	r := 1 / curvature
	v := sqrtf(maxLateralAccelMPS2 * r)
	if v > maxCarSpeed {
		return maxCarSpeed
	}
	return v
}

func signedCurvatureAtArcLen(s *Spline, d float32) float32 {
	_, _, curvature := sampleSplineStateAtDistance(s, d, nil)
	return curvature
}

func lookupCurveSpeed(spline *Spline, dist float32) float32 {
	if len(spline.CurveSpeedMPS) == 0 {
		return maxCarSpeed
	}
	idx := int(dist / curveSpeedIntervalM)
	if idx >= len(spline.CurveSpeedMPS) {
		idx = len(spline.CurveSpeedMPS) - 1
	}
	return spline.CurveSpeedMPS[idx]
}

func buildStoppingTrafficLightsBySpline(lights []TrafficLight, cycles []TrafficCycle) map[int][]TrafficLight {
	if len(lights) == 0 || len(cycles) == 0 {
		return nil
	}

	lightsByCycle := make(map[int][]TrafficLight, len(cycles))
	for _, l := range lights {
		if l.CycleID >= 0 {
			lightsByCycle[l.CycleID] = append(lightsByCycle[l.CycleID], l)
		}
	}

	stoppingBySpline := make(map[int][]TrafficLight)
	for _, c := range cycles {
		cycleLights := lightsByCycle[c.ID]
		if len(cycleLights) == 0 || !c.Enabled || len(c.Phases) == 0 {
			continue
		}

		n := len(c.Phases)
		ei := c.PhaseIndex
		if n <= 1 || ei%2 == 0 {
			userIdx := ei / 2
			if userIdx >= n {
				userIdx = n - 1
			}
			green := make(map[int]bool, len(c.Phases[userIdx].GreenLightIDs))
			for _, lightID := range c.Phases[userIdx].GreenLightIDs {
				green[lightID] = true
			}
			for _, l := range cycleLights {
				if !green[l.ID] {
					stoppingBySpline[l.SplineID] = append(stoppingBySpline[l.SplineID], l)
				}
			}
			continue
		}

		prevIdx := (ei / 2) % n
		nextIdx := (prevIdx + 1) % n
		prevGreen := make(map[int]bool, len(c.Phases[prevIdx].GreenLightIDs))
		nextGreen := make(map[int]bool, len(c.Phases[nextIdx].GreenLightIDs))
		for _, lightID := range c.Phases[prevIdx].GreenLightIDs {
			prevGreen[lightID] = true
		}
		for _, lightID := range c.Phases[nextIdx].GreenLightIDs {
			nextGreen[lightID] = true
		}
		for _, l := range cycleLights {
			if !prevGreen[l.ID] || !nextGreen[l.ID] {
				stoppingBySpline[l.SplineID] = append(stoppingBySpline[l.SplineID], l)
			}
		}
	}
	return stoppingBySpline
}

func computeTrafficLightSpeedCap(car Car, currentSpline *Spline, graph *RoadGraph, stoppingLightsBySpline map[int][]TrafficLight) float32 {
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
		allowed := sqrtf(2 * decel * adj)
		if allowed < result {
			result = allowed
		}
	}

	for _, l := range stoppingLightsBySpline[currentSpline.ID] {
		if l.DistOnSpline <= car.DistanceOnSpline {
			continue
		}
		checkLight(l.DistOnSpline - car.DistanceOnSpline)
	}

	if remaining < lookahead {
		nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, car.DestinationSplineID, car.VehicleKind)
		if ok {
			if _, ok2 := graph.splinePtrByID(nextID); ok2 {
				for _, l := range stoppingLightsBySpline[nextID] {
					checkLight(remaining + l.DistOnSpline)
				}
			}
		}
	}

	return result
}

func computeAnticipatoryTargetSpeed(car Car, currentSpline *Spline, graph *RoadGraph) float32 {
	decel := car.Accel * 1.5
	result := float32(math.MaxFloat32)

	checkSpline := func(spline *Spline, startDist, offset float32) {
		for d := startDist + curveSpeedIntervalM; d <= spline.Length; d += curveSpeedIntervalM {
			reqSpeed := lookupCurveSpeed(spline, d) * car.CurveSpeedMultiplier
			rawDistAhead := offset + (d - startDist)
			allowedNow := sqrtf(reqSpeed*reqSpeed + 2*decel*rawDistAhead)
			if allowedNow < result {
				result = allowedNow
			}
		}
	}

	checkSpline(currentSpline, car.DistanceOnSpline, 0)

	remaining := currentSpline.Length - car.DistanceOnSpline
	if nextID, ok := ChooseNextSplineOnBestPathWithGraph(graph, currentSpline.ID, car.DestinationSplineID, car.VehicleKind); ok {
		if nextSpline, ok2 := graph.splinePtrByID(nextID); ok2 {
			checkSpline(nextSpline, 0, remaining)
		}
	}

	return result
}

func BuildVehicleCounts(cars []Car) map[int]int {
	counts := make(map[int]int, len(cars))
	for _, car := range cars {
		if car.ControlMode == CarControlExternal {
			continue
		}
		counts[car.CurrentSplineID]++
	}
	return counts
}

func isSplineUsableForVehicle(s *Spline, vehicleKind VehicleKind) bool {
	return vehicleKind == VehicleBus || !s.BusOnly
}

const routeCostInf = float32(1e30)

func buildRouteTree(graph *RoadGraph, destIdx int, vehicleKind VehicleKind) routeTreeEntry {
	costs := make([]float32, len(graph.splines))
	nextHop := make([]int, len(graph.splines))
	for i := range costs {
		costs[i] = routeCostInf
		nextHop[i] = -1
	}
	if destIdx < 0 || destIdx >= len(graph.splines) {
		return routeTreeEntry{costs: costs, nextHop: nextHop}
	}

	destSpline := graph.splines[destIdx]
	costs[destIdx] = graph.segmentCosts[destIdx]
	nextHop[destIdx] = destIdx
	if vehicleKind == VehicleCar && destSpline.BusOnly {
		return routeTreeEntry{costs: costs, nextHop: nextHop}
	}

	pq := dijkstraHeap{{idx: destIdx, dist: costs[destIdx]}}
	heap.Init(&pq)
	for pq.Len() > 0 {
		item := heap.Pop(&pq).(dijkstraItem)
		if item.dist != costs[item.idx] {
			continue
		}
		for _, prevIdx := range graph.reverseNeighbors[item.idx] {
			if vehicleKind == VehicleCar && graph.splines[prevIdx].BusOnly {
				continue
			}
			alt := item.dist + graph.segmentCosts[prevIdx]
			if alt < costs[prevIdx] {
				costs[prevIdx] = alt
				nextHop[prevIdx] = item.idx
				heap.Push(&pq, dijkstraItem{idx: prevIdx, dist: alt})
			}
		}
	}

	return routeTreeEntry{costs: costs, nextHop: nextHop}
}

// routeTree returns a cached Dijkstra tree on the reverse graph rooted at
// destinationSplineID. Every call that needs "what's the next spline from
// X toward Y for vehicle kind Z" hits this cache; on a miss it runs full
// Dijkstra. The cache lives on RoadGraph so it's discarded whenever the
// graph is rebuilt (which happens each Step — that's also why the per-frame
// rebuild reuses the cached permanent topology underneath).
func (g *RoadGraph) routeTree(destinationSplineID int, vehicleKind VehicleKind) (routeTreeEntry, bool) {
	if g == nil || len(g.splines) == 0 {
		return routeTreeEntry{}, false
	}
	key := routeTreeKey{DestinationID: destinationSplineID, VehicleKind: vehicleKind}

	g.routeCacheMu.RLock()
	entry, ok := g.routeCache[key]
	g.routeCacheMu.RUnlock()
	if ok {
		g.pathCacheHits.Add(1)
		return entry, true
	}

	destIdx, ok := g.indexByID[destinationSplineID]
	if !ok {
		g.pathCacheMisses.Add(1)
		return routeTreeEntry{}, false
	}

	g.routeCacheMu.Lock()
	if entry, ok = g.routeCache[key]; ok {
		g.pathCacheHits.Add(1)
		g.routeCacheMu.Unlock()
		return entry, true
	}
	g.pathCacheMisses.Add(1)
	entry = buildRouteTree(g, destIdx, vehicleKind)
	g.routeCache[key] = entry
	g.routeCacheMu.Unlock()
	return entry, true
}

func FindShortestPathWeightedWithGraph(graph *RoadGraph, startSplineID, endSplineID int, vehicleKind VehicleKind) ([]int, float32, bool) {
	if graph == nil || len(graph.splines) == 0 {
		return nil, 0, false
	}
	startIdx, okStart := graph.indexByID[startSplineID]
	endIdx, okEnd := graph.indexByID[endSplineID]
	if !okStart || !okEnd {
		return nil, 0, false
	}
	if vehicleKind == VehicleCar && startIdx != endIdx && graph.splines[endIdx].BusOnly {
		return nil, 0, false
	}

	tree, ok := graph.routeTree(endSplineID, vehicleKind)
	if !ok || startIdx >= len(tree.costs) || tree.costs[startIdx] >= routeCostInf/2 {
		return nil, 0, false
	}

	pathIDs := make([]int, 0, 16)
	for at, steps := startIdx, 0; ; steps++ {
		if at < 0 || at >= len(graph.splines) || steps > len(graph.splines) {
			return nil, 0, false
		}
		pathIDs = append(pathIDs, graph.splines[at].ID)
		if at == endIdx {
			return pathIDs, tree.costs[startIdx], true
		}
		nextIdx := tree.nextHop[at]
		if nextIdx < 0 || nextIdx == at {
			return nil, 0, false
		}
		at = nextIdx
	}
}

func PathCostToDestinationWithGraph(graph *RoadGraph, startSplineID, endSplineID int, vehicleKind VehicleKind) (float32, bool) {
	if graph == nil || len(graph.splines) == 0 {
		return 0, false
	}
	startIdx, okStart := graph.indexByID[startSplineID]
	endIdx, okEnd := graph.indexByID[endSplineID]
	if !okStart || !okEnd {
		return 0, false
	}
	if vehicleKind == VehicleCar && startIdx != endIdx && graph.splines[endIdx].BusOnly {
		return 0, false
	}
	tree, ok := graph.routeTree(endSplineID, vehicleKind)
	if !ok || startIdx >= len(tree.costs) || tree.costs[startIdx] >= routeCostInf/2 {
		return 0, false
	}
	return tree.costs[startIdx], true
}

func ChooseNextSplineOnBestPathWithGraph(graph *RoadGraph, currentSplineID, destinationSplineID int, vehicleKind VehicleKind) (int, bool) {
	currentSpline, ok := graph.splinePtrByID(currentSplineID)
	if !ok {
		return 0, false
	}
	candidateIndices := graph.startsByNode[nodeKeyFromVec2(currentSpline.P3)]
	if len(candidateIndices) == 0 {
		return 0, false
	}
	tree, ok := graph.routeTree(destinationSplineID, vehicleKind)
	if !ok {
		return 0, false
	}

	bestSplineID := 0
	bestCost := routeCostInf
	found := false
	for _, idx := range candidateIndices {
		candidate := graph.splines[idx]
		if !isSplineUsableForVehicle(candidate, vehicleKind) {
			continue
		}
		if idx >= len(tree.costs) || tree.costs[idx] >= routeCostInf/2 {
			continue
		}
		cost := tree.costs[idx]
		if !found || cost < bestCost {
			bestCost = cost
			bestSplineID = candidate.ID
			found = true
		}
	}
	return bestSplineID, found
}

func expandWithCoupledNeighborRefs(indices []int, splines []*Spline, indexByID map[int]int) []int {
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

func expandWithCoupledNeighbors(indices []int, splines []Spline, indexByID map[int]int) []int {
	splineRefs := make([]*Spline, len(splines))
	for i := range splines {
		splineRefs[i] = &splines[i]
	}
	return expandWithCoupledNeighborRefs(indices, splineRefs, indexByID)
}

func buildStartsByNodeRefs(splines []*Spline) map[NodeKey][]int {
	startsByNode := make(map[NodeKey][]int, len(splines))
	for i := range splines {
		key := nodeKeyFromVec2(splines[i].P0)
		startsByNode[key] = append(startsByNode[key], i)
	}
	return startsByNode
}

func buildEndsByNodeRefs(splines []*Spline) map[NodeKey][]int {
	endsByNode := make(map[NodeKey][]int, len(splines))
	for i := range splines {
		key := nodeKeyFromVec2(splines[i].P3)
		endsByNode[key] = append(endsByNode[key], i)
	}
	return endsByNode
}

func BuildStartsByNode(splines []Spline) map[NodeKey][]int {
	splineRefs := make([]*Spline, len(splines))
	for i := range splines {
		splineRefs[i] = &splines[i]
	}
	return buildStartsByNodeRefs(splineRefs)
}

func segmentTravelCost(s *Spline, vehicleCounts map[int]int) float32 {
	return s.TravelTimeS + float32(vehicleCounts[s.ID])*2.0
}

func sampleSplineAtDistance(s *Spline, distance float32) (Vec2, Vec2) {
	pos, tangent, _ := sampleSplineStateAtDistance(s, distance, nil)
	return pos, tangent
}

func sampleCurvatureAtIndex(samples *[simSamples + 1]Vec2, idx int) float32 {
	i0, i1, i2 := idx-1, idx, idx+1
	if i0 < 0 {
		i0, i1, i2 = 0, 1, 2
	}
	if i2 > simSamples {
		i0, i1, i2 = simSamples-2, simSamples-1, simSamples
	}
	A, B, C := samples[i0], samples[i1], samples[i2]
	ab := sqrtf(distSq(A, B))
	bc := sqrtf(distSq(B, C))
	ca := sqrtf(distSq(C, A))
	denom := ab * bc * ca
	if denom < 1e-6 {
		return 0
	}
	cross := (B.X-A.X)*(C.Y-A.Y) - (B.Y-A.Y)*(C.X-A.X)
	return 2 * cross / denom
}

func splineSegmentIndex(s *Spline, distance float32, cursor *splineSampleCursor) int {
	if s.Length <= 0 {
		return 0
	}
	if cursor != nil && cursor.splineID == s.ID {
		idx := cursor.idx
		if idx < 1 {
			idx = 1
		}
		if idx > simSamples {
			idx = simSamples
		}
		for idx < simSamples && distance > s.CumLen[idx] {
			idx++
		}
		for idx > 1 && distance <= s.CumLen[idx-1] {
			idx--
		}
		cursor.idx = idx
		return idx
	}
	idx := sort.Search(simSamples, func(i int) bool {
		return distance <= s.CumLen[i+1]
	}) + 1
	if idx > simSamples {
		idx = simSamples
	}
	if cursor != nil {
		cursor.splineID = s.ID
		cursor.idx = idx
	}
	return idx
}

func sampleSplineStateAtDistance(s *Spline, distance float32, cursor *splineSampleCursor) (Vec2, Vec2, float32) {
	if s.Length <= 0 {
		return s.P0, NewVec2(1, 0), 0
	}
	distance = clampf(distance, 0, s.Length)
	idx := splineSegmentIndex(s, distance, cursor)
	if idx <= 0 {
		return s.P0, NewVec2(1, 0), 0
	}

	span := s.CumLen[idx] - s.CumLen[idx-1]
	alpha := float32(0)
	if span > 0 {
		alpha = (distance - s.CumLen[idx-1]) / span
	}
	t0 := float32(idx-1) / float32(simSamples)
	t1 := float32(idx) / float32(simSamples)
	t := t0 + (t1-t0)*alpha
	pos := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)

	tangent := normalize(vecAdd(
		vecScale(s.SampleTangents[idx-1], 1-alpha),
		vecScale(s.SampleTangents[idx], alpha),
	))
	curvature := (1-alpha)*s.SampleCurv[idx-1] + alpha*s.SampleCurv[idx]
	return pos, tangent, curvature
}

func SampleSplineAtDistance(s Spline, distance float32) (Vec2, Vec2) {
	return sampleSplineAtDistance(&s, distance)
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
	for i := range splines {
		m[splines[i].ID] = i
	}
	return m
}

func buildSplinePtrIndexByID(splines []*Spline) map[int]int {
	m := make(map[int]int, len(splines))
	for i := range splines {
		m[splines[i].ID] = i
	}
	return m
}

func BuildSplineIndexByID(splines []Spline) map[int]int {
	return buildSplineIndexByID(splines)
}

func findSplinePtrByID(splines []Spline, id int) (*Spline, bool) {
	for i := range splines {
		if splines[i].ID == id {
			return &splines[i], true
		}
	}
	return nil, false
}

func findSplineByID(splines []Spline, id int) (Spline, bool) {
	spline, ok := findSplinePtrByID(splines, id)
	if !ok {
		return Spline{}, false
	}
	return *spline, true
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

func nearestSampleOnSpline(spline *Spline, pos Vec2) Vec2 {
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

func nearestSampleWithDist(spline *Spline, pos Vec2) (point Vec2, dist float32) {
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
	inv := 1 / sqrtf(lenSq)
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

func sqrtf(v float32) float32 {
	return float32(math.Sqrt(float64(v)))
}

func floorf32(v float32) int32 {
	i := int32(v)
	if v < float32(i) {
		i--
	}
	return i
}

// NodeKey quantizes a world position to 1/100 units so spline endpoints
// can be bucketed into a map. Two splines are "connected" iff their
// endpoint keys are exactly equal — there is no float tolerance. The
// editor is responsible for snapping endpoints to the same key.
type NodeKey [2]int32

func nodeKeyFromVec2(v Vec2) NodeKey {
	return NodeKey{
		int32(math.Round(float64(v.X * 100))),
		int32(math.Round(float64(v.Y * 100))),
	}
}

func randomizedSpawnDelay(spawnPerMinute float32) float32 {
	if spawnPerMinute <= 0 {
		return float32(math.MaxFloat32)
	}
	lambda := float64(spawnPerMinute) / 60.0
	u := math.Max(rand.Float64(), 1e-5)
	return float32(-math.Log(u) / lambda)
}
