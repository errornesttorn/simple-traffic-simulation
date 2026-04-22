//go:build !darwin

package main

import (
	"fmt"
	"math"
	"math/rand"
	"strconv"
	"strings"
	"time"

	"simple-traffic-simulation/internal/filepicker"
	simpkg "simple-traffic-simulation/sim"

	rl "github.com/gen2brain/raylib-go/raylib"
)

// Run with:
//   go mod init example.com/spline-editor
//   go get github.com/gen2brain/raylib-go/raylib
//   go run .
//
// Controls:
//   Mouse wheel: zoom toward the cursor
//   E: edit splines mode
//   R: route mode
//   B: bus line mode
//   G: driving mode
//   P: priority paint mode
//   D: toggle debug overlay
//   F3: toggle profiling overlay
//   Ctrl+S: save splines to a chosen JSON file
//   Ctrl+O: load splines from a chosen JSON file
//   Tab: cycle modes
//
// Edit mode:
//   Left click:  start spline / confirm P1 / set P2 / finish with P3
//   Right click: delete hovered spline, cancel current draft, or undo P2 after it is set
//
// Route mode:
//   Left click on a spline start: pick route origin
//   Left click on a spline end:   pick route destination
//   Right click / Escape: cancel selection or close the route panel
//
// Bus mode:
//   Left click on a spline start: pick line origin
//   Left click on a spline end:   pick line destination
//   In the panel: add bus stops on spline intersections
//   Right click / Escape: cancel selection or close the line panel
//
// Priority mode:
//   Left click on a hovered spline: mark it as priority
//   Right click on a hovered spline: clear priority
//
// Navigation intentionally has no pan. To move around, zoom out and then zoom back into a new point.

type EditorMode int
type EditorTool int

type Stage int

type EndKind int

type VehicleKind = simpkg.VehicleKind

const (
	ModeDraw EditorMode = iota
	ModeRules
	ModeRoute
	ModeTraffic
	ModePedestrian
	ModeDriving
)

const (
	ToolSpline EditorTool = iota
	ToolQuadratic
	ToolCut
	ToolReverse
	ToolPriority
	ToolCouple
	ToolSpeedLimit
	ToolPreference
	ToolRouteCars
	ToolRouteBuses
	ToolRouteEraser
	ToolTrafficLight
	ToolDrive
	ToolPedestrianPath
)

const (
	VehicleCar = simpkg.VehicleCar
	VehicleBus = simpkg.VehicleBus
)

const (
	StageIdle Stage = iota
	StageSetP1
	StageSetP2
	StageSetP3
)

const (
	EndNone EndKind = iota
	EndStart
	EndFinish
)

const (
	initialWidth  int32 = 1440
	initialHeight int32 = 900

	minZoom float32 = 0.05
	maxZoom float32 = 50.0

	curveSamples = 64
	hoverSamples = 48
	simSamples   = 96

	handlePixels float32 = 6.0
	linePixels   float32 = 3.0
	hoverPixels  float32 = 10.0
	snapPixels   float32 = 14.0

	spawnSliderMaxPerMinute float32 = 60.0

	// Scale: 1 world unit = metersPerUnit metres.
	// All car dimensions, speeds, and accelerations are stored in SI units (m, m/s, m/s²).
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
	// Following distance: suppress acceleration when already behind a same-direction car.
	// Desired gap = followMinGapM + speed * followTimeHeadwaySecs.
	followTimeHeadwaySecs float32 = 1.5  // seconds of headway at current speed
	followMinGapM         float32 = 2.0  // minimum gap regardless of speed (m)
	followLookaheadM      float32 = 60.0 // max distance to look for a leader (m)
	// Minimum heading alignment: dot product of unit headings must exceed this
	// (cos 40° ≈ 0.766) to be considered same-direction.
	followHeadingCos float32 = 0.766
)

const (
	// Lane changing: temporary Bézier spline bridging two coupled lanes.
	// Total manoeuvre time ≈ 2 × laneChangeHalfSecs.
	laneChangeHalfSecs float32 = 1.0 // handle arm length = speed × this (s)
	laneChangeMinSpeed float32 = 3.0 // minimum m/s to attempt a lane change
	// Minimum dot product of car heading vs destination lane tangent (cos 45° ≈ 0.71).
	// Prevents switching onto a lane going the wrong way.
	laneChangeDirCos          float32 = 0.71
	preferenceChangeCooldownS float32 = 7.5        // seconds between preference-based lane-change checks
	overtakeSlowThresholdS    float32 = 2.0        // seconds behind a leader before attempting overtake
	overtakeCooldownS         float32 = 3.5        // seconds between overtake attempts
	maxCarSpeed               float32 = 36.1       // m/s — upper bound of car MaxSpeed range (130 km/h)
	laneChangeForcedSpeedMPS  float32 = 20.0 / 3.6 // 20 km/h — speed for last-resort lane switch
	laneChangeForcedDistEnd   float32 = 15.0       // metres before spline end where last-resort fires
	curveSpeedIntervalM       float32 = 10.0       // arc-length spacing of curvature speed samples (m)
	maxLateralAccelMPS2       float32 = 5.0        // max lateral acceleration for curve speed (m/s²)
	collisionBroadPhaseSlackM float32 = 5.0

	// Pivot fractions: the spline-following point sits 20% from the front and
	// the dragged rear pivot sits 80% from the front (20% from the rear).
	// This models axle positions rather than bumpers, giving realistic turning.
	frontPivotFrac float32 = 0.20 // fraction of length from front bumper to front pivot
	rearPivotFrac  float32 = 0.80 // fraction of length from front bumper to rear pivot
	// Wheelbase = distance between the two pivots as a fraction of car length.
	wheelbaseFrac float32 = rearPivotFrac - frontPivotFrac // 0.60
)

type Vec2 = simpkg.Vec2

func NewVec2(x, y float32) Vec2 {
	return simpkg.NewVec2(x, y)
}

func toRLVec2(v Vec2) rl.Vector2 {
	return rl.Vector2{X: v.X, Y: v.Y}
}

func fromRLVec2(v rl.Vector2) Vec2 {
	return Vec2{X: v.X, Y: v.Y}
}

type Color = simpkg.Color

func NewColor(r, g, b, a uint8) Color {
	return simpkg.NewColor(r, g, b, a)
}

func toRLColor(c Color) rl.Color {
	return rl.Color(c)
}

func fromRLColor(c rl.Color) Color {
	return Color(c)
}

func drawCircleV(center Vec2, radius float32, color Color) {
	rl.DrawCircleV(toRLVec2(center), radius, color)
}

func drawRing(center Vec2, innerRadius, outerRadius, startAngle, endAngle float32, segments int32, color Color) {
	rl.DrawRing(toRLVec2(center), innerRadius, outerRadius, startAngle, endAngle, segments, color)
}

func drawLineEx(startPos, endPos Vec2, thick float32, color Color) {
	rl.DrawLineEx(toRLVec2(startPos), toRLVec2(endPos), thick, color)
}

func drawLineV(startPos, endPos Vec2, color Color) {
	rl.DrawLineV(toRLVec2(startPos), toRLVec2(endPos), color)
}

func drawCircleLinesV(center Vec2, radius float32, color Color) {
	rl.DrawCircleLinesV(toRLVec2(center), radius, color)
}

func drawRectanglePro(rec rl.Rectangle, origin Vec2, rotation float32, color Color) {
	rl.DrawRectanglePro(rec, toRLVec2(origin), rotation, color)
}

func getWorldToScreen2D(position Vec2, camera rl.Camera2D) Vec2 {
	return fromRLVec2(rl.GetWorldToScreen2D(toRLVec2(position), camera))
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

type Spline = simpkg.Spline

type asyncStepResult struct {
	World    simpkg.World
	Revision uint64
}

type playerCarState struct {
	Active          bool
	Position        Vec2
	HeadingDeg      float32
	Speed           float32
	Length          float32
	Width           float32
	Color           Color
	MaxForwardSpeed float32
	MaxReverseSpeed float32
	AccelMPS2       float32
	BrakeMPS2       float32
	CoastMPS2       float32
	BaseTurnDegPerS float32

	DestinationSplineID int
	DestinationPoint    Vec2
	HasDestination      bool
}

type editorWorldState struct {
	Splines           []Spline
	LaneChangeSplines []Spline
	Routes            []Route
	Cars              []Car
	TrafficLights     []TrafficLight
	TrafficCycles     []TrafficCycle
	PedestrianPaths   []simpkg.PedestrianPath

	NextSplineID int
	NextRouteID  int
	NextLightID  int
	NextCycleID  int

	DebugSelectedCarID   int
	DebugSelectedCarMode int
}

func newEditorWorldState(world *simpkg.World) editorWorldState {
	return editorWorldState{
		Splines:              world.Splines,
		LaneChangeSplines:    world.LaneChangeSplines,
		Routes:               world.Routes,
		Cars:                 world.Cars,
		TrafficLights:        world.TrafficLights,
		TrafficCycles:        world.TrafficCycles,
		PedestrianPaths:      world.PedestrianPaths,
		NextSplineID:         world.NextSplineID,
		NextRouteID:          world.NextRouteID,
		NextLightID:          world.NextLightID,
		NextCycleID:          world.NextCycleID,
		DebugSelectedCarID:   world.DebugSelectedCarID,
		DebugSelectedCarMode: world.DebugSelectedCarMode,
	}
}

func (s *editorWorldState) commit(world *simpkg.World) {
	world.Splines = s.Splines
	world.LaneChangeSplines = s.LaneChangeSplines
	world.Routes = s.Routes
	world.Cars = s.Cars
	world.TrafficLights = s.TrafficLights
	world.TrafficCycles = s.TrafficCycles
	world.PedestrianPaths = s.PedestrianPaths
	world.NextSplineID = s.NextSplineID
	world.NextRouteID = s.NextRouteID
	world.NextLightID = s.NextLightID
	world.NextCycleID = s.NextCycleID
	world.DebugSelectedCarID = s.DebugSelectedCarID
	world.DebugSelectedCarMode = s.DebugSelectedCarMode
}

func (s *editorWorldState) clearCarDebugSelection() {
	s.DebugSelectedCarID = -1
	s.DebugSelectedCarMode = 0
}

func (s *editorWorldState) handleTopologyChanged() {
	s.Routes = refreshRoutes(s.Routes, s.Splines)
	s.Cars = s.Cars[:0]
	s.LaneChangeSplines = s.LaneChangeSplines[:0]
	s.clearCarDebugSelection()
}

func (s *editorWorldState) handleRouteGraphChanged(clearCars bool) {
	s.Routes = refreshRoutes(s.Routes, s.Splines)
	s.LaneChangeSplines = s.LaneChangeSplines[:0]
	if clearCars {
		s.Cars = s.Cars[:0]
		s.clearCarDebugSelection()
	}
}

func (s *editorWorldState) handleCarsChanged() {
	s.clearCarDebugSelection()
}

type Draft struct {
	P0 Vec2
	P1 Vec2
	P2 Vec2
	P3 Vec2

	HasP1 bool
	HasP2 bool

	LockP1           bool
	ContinuationFrom int
	P1AxisDir        Vec2
	SnapP3           bool
	P2AxisDir        Vec2
	PerpLockActive   bool
	PerpLockOrigin   Vec2
	PerpLockAxisDir  Vec2
}

type QuadraticDraft struct {
	P0 Vec2
	M  Vec2
	P3 Vec2

	FromPrevAxis      bool
	PrevAxisDir       Vec2
	MMirroredFromPrev bool

	SnapP3          bool
	NextAxisDir     Vec2
	PerpLockActive  bool
	PerpLockOrigin  Vec2
	PerpLockAxisDir Vec2
}

// CutDraft holds state for the spline-cut mode.
// Stage StageIdle   → user is hovering, snap dot follows nearest spline point.
// Stage StageSetP1  → cut point is locked; user places the tangent handle.
type CutDraft struct {
	OriginalSplineID       int
	OriginalSplinePriority bool
	CutPoint               Vec2
	CutT                   float32
	// de Casteljau sub-spline control points
	LeftP  [4]Vec2 // left half  P0..P3
	RightP [4]Vec2 // right half P0..P3
}

type EndHit struct {
	SplineIndex int
	SplineID    int
	Kind        EndKind
	Point       Vec2
}

type DirectionWarning struct {
	Point Vec2
}

type worldRect struct {
	MinX float32
	MinY float32
	MaxX float32
	MaxY float32
}

type GeometrySnap struct {
	Active            bool
	SourceSplineID    int
	SourceSplineIndex int
	Origin            Vec2
	AxisDir           Vec2
	Point             Vec2
	HasMinT           bool
	MinT              float32
	HasMaxT           bool
	MaxT              float32
}

type BusStop = simpkg.BusStop

type Route = simpkg.Route

type RoutePanel struct {
	Open bool

	StartSplineID    int
	EndSplineID      int
	ExistingRouteID  int
	PathIDs          []int
	PathLength       float32
	SpawnPerMinute   float32
	ColorIndex       int
	VehicleKind      VehicleKind
	BusStops         []BusStop
	AddingBusStop    bool
	StopStatusNotice string

	DraggingSlider bool
}

// Trailer is an optional cargo trailer towed by a Car.
// Its front hitch is rigidly the cab's RearPosition; its own RearPosition is
// pulled toward the hitch each tick, exactly mirroring the cab's rear-pull logic.
type Trailer = simpkg.Trailer
type Car = simpkg.Car
type Pedestrian = simpkg.Pedestrian
type TrajectorySample = simpkg.TrajectorySample
type CollisionPrediction = simpkg.CollisionPrediction
type BrakingProfile = simpkg.BrakingProfile
type FollowingProfile = simpkg.FollowingProfile
type UpdateCarsProfile = simpkg.UpdateCarsProfile

type RoadGraph = simpkg.RoadGraph

type frameProfile struct {
	frameMS          float64
	inputMS          float64
	stepMS           float64
	routeVisualsMS   float64
	laneChangesMS    float64
	graphBuildMS     float64
	brakingMS        float64
	followMS         float64
	updateCarsMS     float64
	drawMS           float64
	basePathHits     int
	basePathMisses   int
	allPathHits      int
	allPathMisses    int
	brakingDetail    BrakingProfile
	followDetail     FollowingProfile
	updateCarsDetail UpdateCarsProfile
}

type profiler struct {
	current frameProfile
	smooth  frameProfile
	frames  int
}

func (p *profiler) endFrame(sample frameProfile) {
	p.current = sample
	p.frames++
	const alpha = 0.15
	if p.frames == 1 {
		p.smooth = sample
		return
	}
	p.smooth.frameMS = blendMetric(p.smooth.frameMS, sample.frameMS, alpha)
	p.smooth.inputMS = blendMetric(p.smooth.inputMS, sample.inputMS, alpha)
	p.smooth.stepMS = blendMetric(p.smooth.stepMS, sample.stepMS, alpha)
	p.smooth.routeVisualsMS = blendMetric(p.smooth.routeVisualsMS, sample.routeVisualsMS, alpha)
	p.smooth.laneChangesMS = blendMetric(p.smooth.laneChangesMS, sample.laneChangesMS, alpha)
	p.smooth.graphBuildMS = blendMetric(p.smooth.graphBuildMS, sample.graphBuildMS, alpha)
	p.smooth.brakingMS = blendMetric(p.smooth.brakingMS, sample.brakingMS, alpha)
	p.smooth.followMS = blendMetric(p.smooth.followMS, sample.followMS, alpha)
	p.smooth.updateCarsMS = blendMetric(p.smooth.updateCarsMS, sample.updateCarsMS, alpha)
	p.smooth.drawMS = blendMetric(p.smooth.drawMS, sample.drawMS, alpha)
	p.smooth.brakingDetail.MarshalMS = blendMetric(p.smooth.brakingDetail.MarshalMS, sample.brakingDetail.MarshalMS, alpha)
	p.smooth.brakingDetail.MarshalSetupMS = blendMetric(p.smooth.brakingDetail.MarshalSetupMS, sample.brakingDetail.MarshalSetupMS, alpha)
	p.smooth.brakingDetail.BasePredictMS = blendMetric(p.smooth.brakingDetail.BasePredictMS, sample.brakingDetail.BasePredictMS, alpha)
	p.smooth.brakingDetail.ConflictScanMS = blendMetric(p.smooth.brakingDetail.ConflictScanMS, sample.brakingDetail.ConflictScanMS, alpha)
	p.smooth.brakingDetail.BrakeProbeMS = blendMetric(p.smooth.brakingDetail.BrakeProbeMS, sample.brakingDetail.BrakeProbeMS, alpha)
	p.smooth.brakingDetail.HoldProbeMS = blendMetric(p.smooth.brakingDetail.HoldProbeMS, sample.brakingDetail.HoldProbeMS, alpha)
	p.smooth.brakingDetail.FinalizeMS = blendMetric(p.smooth.brakingDetail.FinalizeMS, sample.brakingDetail.FinalizeMS, alpha)
	p.smooth.brakingDetail.KernelMS = blendMetric(p.smooth.brakingDetail.KernelMS, sample.brakingDetail.KernelMS, alpha)
	p.smooth.brakingDetail.UnmarshalMS = blendMetric(p.smooth.brakingDetail.UnmarshalMS, sample.brakingDetail.UnmarshalMS, alpha)
	p.smooth.basePathHits = sample.basePathHits
	p.smooth.basePathMisses = sample.basePathMisses
	p.smooth.allPathHits = sample.allPathHits
	p.smooth.allPathMisses = sample.allPathMisses
	p.smooth.brakingDetail.Cars = sample.brakingDetail.Cars
	p.smooth.brakingDetail.BasePredictions = sample.brakingDetail.BasePredictions
	p.smooth.brakingDetail.StationaryPredictions = sample.brakingDetail.StationaryPredictions
	p.smooth.brakingDetail.EscapePredictions = sample.brakingDetail.EscapePredictions
	p.smooth.brakingDetail.FasterPredictions = sample.brakingDetail.FasterPredictions
	p.smooth.brakingDetail.TotalPredictions = sample.brakingDetail.TotalPredictions
	p.smooth.brakingDetail.TotalPredictionSamples = sample.brakingDetail.TotalPredictionSamples
	p.smooth.brakingDetail.PrimaryPairCandidates = sample.brakingDetail.PrimaryPairCandidates
	p.smooth.brakingDetail.PrimaryBroadPhasePairs = sample.brakingDetail.PrimaryBroadPhasePairs
	p.smooth.brakingDetail.PrimaryCollisionChecks = sample.brakingDetail.PrimaryCollisionChecks
	p.smooth.brakingDetail.PrimaryCollisionHits = sample.brakingDetail.PrimaryCollisionHits
	p.smooth.brakingDetail.StationaryCollisionChecks = sample.brakingDetail.StationaryCollisionChecks
	p.smooth.brakingDetail.StationaryCollisionHits = sample.brakingDetail.StationaryCollisionHits
	p.smooth.brakingDetail.EscapeCollisionChecks = sample.brakingDetail.EscapeCollisionChecks
	p.smooth.brakingDetail.EscapeCollisionHits = sample.brakingDetail.EscapeCollisionHits
	p.smooth.brakingDetail.HoldCollisionChecks = sample.brakingDetail.HoldCollisionChecks
	p.smooth.brakingDetail.HoldCollisionHits = sample.brakingDetail.HoldCollisionHits
	p.smooth.brakingDetail.InitiallyBlamedCars = sample.brakingDetail.InitiallyBlamedCars
	p.smooth.brakingDetail.BrakingCars = sample.brakingDetail.BrakingCars
	p.smooth.brakingDetail.HoldCars = sample.brakingDetail.HoldCars
	p.smooth.followDetail.Cars = sample.followDetail.Cars
	p.smooth.followDetail.PoseMS = blendMetric(p.smooth.followDetail.PoseMS, sample.followDetail.PoseMS, alpha)
	p.smooth.followDetail.IndexMS = blendMetric(p.smooth.followDetail.IndexMS, sample.followDetail.IndexMS, alpha)
	p.smooth.followDetail.CandidateMS = blendMetric(p.smooth.followDetail.CandidateMS, sample.followDetail.CandidateMS, alpha)
	p.smooth.followDetail.ScanMS = blendMetric(p.smooth.followDetail.ScanMS, sample.followDetail.ScanMS, alpha)
	p.smooth.followDetail.CandidateRefs = sample.followDetail.CandidateRefs
	p.smooth.updateCarsDetail.Cars = sample.updateCarsDetail.Cars
	p.smooth.updateCarsDetail.SetupMS = blendMetric(p.smooth.updateCarsDetail.SetupMS, sample.updateCarsDetail.SetupMS, alpha)
	p.smooth.updateCarsDetail.FastPathMS = blendMetric(p.smooth.updateCarsDetail.FastPathMS, sample.updateCarsDetail.FastPathMS, alpha)
	p.smooth.updateCarsDetail.TransitionMS = blendMetric(p.smooth.updateCarsDetail.TransitionMS, sample.updateCarsDetail.TransitionMS, alpha)
	p.smooth.updateCarsDetail.DwellCars = sample.updateCarsDetail.DwellCars
	p.smooth.updateCarsDetail.FastPathCars = sample.updateCarsDetail.FastPathCars
	p.smooth.updateCarsDetail.TransitionCars = sample.updateCarsDetail.TransitionCars
	p.smooth.updateCarsDetail.RemovedCars = sample.updateCarsDetail.RemovedCars
}

func blendMetric(prev, current, alpha float64) float64 {
	return prev*(1-alpha) + current*alpha
}

func sinceMS(start time.Time) float64 {
	return float64(time.Since(start).Microseconds()) / 1000.0
}

type DebugBlameLink = simpkg.DebugBlameLink

// ---------- traffic lights ----------

type TrafficState = simpkg.TrafficState

const (
	TrafficRed    = simpkg.TrafficRed
	TrafficYellow = simpkg.TrafficYellow
	TrafficGreen  = simpkg.TrafficGreen
)

type TrafficLight = simpkg.TrafficLight
type TrafficPhase = simpkg.TrafficPhase
type TrafficCycle = simpkg.TrafficCycle

// ---------- ui font ----------

// uiFont is the Ubuntu font used for all on-screen text.
var uiFont rl.Font

type modeToolbarItem struct {
	key      string
	label    string
	mode     EditorMode
	isDbg    bool
	isHitbox bool
	isInfo   bool
}

type toolToolbarItem struct {
	key   string
	label string
	tool  EditorTool
}

var modeToolbarItems = []modeToolbarItem{
	{"Dw", "Draw", ModeDraw, false, false, false},
	{"Ru", "Rules", ModeRules, false, false, false},
	{"Rt", "Route", ModeRoute, false, false, false},
	{"Tr", "Traffic", ModeTraffic, false, false, false},
	{"Pd", "Ped", ModePedestrian, false, false, false},
	{"G", "Drive", ModeDriving, false, false, false},
	{"I", "Info", 0, false, false, true},
	{"D", "Debug", 0, true, false, false},
	{"H", "Hitbox", 0, false, true, false},
}

var drawToolItems = []toolToolbarItem{
	{"E", "Spline", ToolSpline},
	{"Q", "Quad", ToolQuadratic},
	{"C", "Cut", ToolCut},
	{"X", "Rev", ToolReverse},
}

var rulesToolItems = []toolToolbarItem{
	{"P", "Priority", ToolPriority},
	{"L", "Couple", ToolCouple},
	{"S", "Speed", ToolSpeedLimit},
	{"V", "Prefer", ToolPreference},
}

var routeToolItems = []toolToolbarItem{
	{"R", "Cars", ToolRouteCars},
	{"B", "Buses", ToolRouteBuses},
	{"Z", "Erase", ToolRouteEraser},
}

var trafficToolItems = []toolToolbarItem{
	{"T", "Traffic", ToolTrafficLight},
}

var drivingToolItems = []toolToolbarItem{
	{"G", "Drive", ToolDrive},
}

var pedestrianToolItems = []toolToolbarItem{
	{"W", "Path", ToolPedestrianPath},
}

const (
	toolbarBtnW   = 70
	toolbarBtnH   = 75
	toolbarBtnGap = 5
	toolbarX      = 12
	toolbarY      = 12
)

func toolbarBtnRect(i int) rl.Rectangle {
	x := float32(toolbarX) + float32(i)*(toolbarBtnW+toolbarBtnGap)
	return rl.NewRectangle(x, float32(toolbarY), toolbarBtnW, toolbarBtnH)
}

func toolBtnRect(i int) rl.Rectangle {
	x := float32(toolbarX) + float32(i)*(toolbarBtnW+toolbarBtnGap)
	y := float32(toolbarY + toolbarBtnH + toolbarBtnGap)
	return rl.NewRectangle(x, y, toolbarBtnW, toolbarBtnH)
}

func toolsForMode(mode EditorMode) []toolToolbarItem {
	switch mode {
	case ModeDraw:
		return drawToolItems
	case ModeRules:
		return rulesToolItems
	case ModeRoute:
		return routeToolItems
	case ModeTraffic:
		return trafficToolItems
	case ModePedestrian:
		return pedestrianToolItems
	case ModeDriving:
		return drivingToolItems
	default:
		return nil
	}
}

func modeForTool(tool EditorTool) EditorMode {
	switch tool {
	case ToolSpline, ToolQuadratic, ToolCut, ToolReverse:
		return ModeDraw
	case ToolPriority, ToolCouple, ToolSpeedLimit, ToolPreference:
		return ModeRules
	case ToolRouteCars, ToolRouteBuses, ToolRouteEraser:
		return ModeRoute
	case ToolTrafficLight:
		return ModeTraffic
	case ToolPedestrianPath:
		return ModePedestrian
	case ToolDrive:
		return ModeDriving
	default:
		return ModeDraw
	}
}

func defaultToolForMode(mode EditorMode) EditorTool {
	switch mode {
	case ModeDraw:
		return ToolSpline
	case ModeRules:
		return ToolPriority
	case ModeRoute:
		return ToolRouteCars
	case ModeTraffic:
		return ToolTrafficLight
	case ModePedestrian:
		return ToolPedestrianPath
	case ModeDriving:
		return ToolDrive
	default:
		return ToolSpline
	}
}

func isMouseOverToolbar(mouse Vec2) bool {
	topN := len(modeToolbarItems)
	topW := float32(topN*toolbarBtnW + (topN-1)*toolbarBtnGap)
	if mouse.X >= float32(toolbarX) && mouse.X <= float32(toolbarX)+topW &&
		mouse.Y >= float32(toolbarY) && mouse.Y <= float32(toolbarY+toolbarBtnH) {
		return true
	}
	toolN := 0
	for _, mode := range []EditorMode{ModeDraw, ModeRules, ModeRoute, ModeTraffic, ModePedestrian, ModeDriving} {
		if n := len(toolsForMode(mode)); n > toolN {
			toolN = n
		}
	}
	maxToolW := float32(toolN*toolbarBtnW + (toolN-1)*toolbarBtnGap)
	return mouse.X >= float32(toolbarX) && mouse.X <= float32(toolbarX)+maxToolW &&
		mouse.Y >= float32(toolbarY+toolbarBtnH+toolbarBtnGap) && mouse.Y <= float32(toolbarY+2*toolbarBtnH+toolbarBtnGap)
}

func drawText(text string, x, y, size int32, color Color) {
	rl.DrawTextEx(uiFont, text, rl.NewVector2(float32(x), float32(y)), float32(size), 1, toRLColor(color))
}

func measureText(text string, size int32) int32 {
	return int32(rl.MeasureTextEx(uiFont, text, float32(size), 1).X)
}

func main() {
	rand.Seed(time.Now().UnixNano())

	rl.SetConfigFlags(rl.FlagWindowResizable)
	rl.InitWindow(initialWidth, initialHeight, "raylib-go traffic spline editor")
	defer rl.CloseWindow()
	rl.SetTargetFPS(144)

	uiFont = rl.LoadFontEx("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", 48, nil, 0)
	if uiFont.CharsCount == 0 {
		uiFont = rl.GetFontDefault()
	}
	rl.SetTextureFilter(uiFont.Texture, rl.FilterBilinear)
	defer rl.UnloadFont(uiFont)

	camera := rl.NewCamera2D(
		rl.NewVector2(float32(rl.GetScreenWidth())/2, float32(rl.GetScreenHeight())/2),
		rl.NewVector2(0, 0),
		0,
		1,
	)

	world := simpkg.NewWorld()
	playerCar := playerCarState{}
	driveRestoreTarget := rl.NewVector2(0, 0)

	mode := ModeDraw
	tool := ToolSpline
	stage := StageIdle
	draft := newDraft()
	quadraticDraft := newQuadraticDraft()
	cutDraft := newCutDraft()
	pedestrianDraft := pedestrianPathDraft{}
	routePanel := RoutePanel{}
	routeStartSplineID := -1
	coupleModeFirstID := -1
	debugMode := false
	hitboxDebugMode := false
	infoMode := false
	profileMode := false
	prof := profiler{}
	selectedSpeedKmh := 50
	lastPref := 0
	pendingLights := make([]TrafficLight, 0)
	editingCycleID := -1   // >= 0 when a cycle is open in the panel
	editingLights := false // true when the "Edit Lights" sub-mode is active
	editingPhaseIdx := -1  // >= 0 when a phase's light states are being toggled
	activeDurInput := -1   // >= 0 = phase index whose duration field is active
	durInputStr := ""      // current text in the active duration input
	showPhaseIdx := -1     // >= 0 = phase being previewed via Show button

	paused := false
	noticeText := ""
	noticeTimer := float32(0)
	stepResults := make(chan asyncStepResult, 1)
	stepRunning := false
	worldRevision := uint64(1)
	simAccumDT := float32(0)

	resetToolState := func() {
		stage = StageIdle
		draft = newDraft()
		quadraticDraft = newQuadraticDraft()
		cutDraft = newCutDraft()
		pedestrianDraft = pedestrianPathDraft{}
		routePanel = RoutePanel{}
		routeStartSplineID = -1
		coupleModeFirstID = -1
	}

	setTool := func(newTool EditorTool) {
		tool = newTool
		mode = modeForTool(newTool)
		resetToolState()
	}

	setMode := func(newMode EditorMode) {
		mode = newMode
		tool = defaultToolForMode(newMode)
		resetToolState()
	}

	for !rl.WindowShouldClose() {
		frameStart := time.Now()
		frameProf := frameProfile{}
		inputStart := time.Now()

		select {
		case result := <-stepResults:
			stepRunning = false
			if result.Revision == worldRevision {
				world = result.World
			}
		default:
		}

		dt := rl.GetFrameTime()
		camera.Offset = rl.NewVector2(float32(rl.GetScreenWidth())/2, float32(rl.GetScreenHeight())/2)
		editorMutatedWorld := false
		pedestrianTopologyChanged := false

		if !paused {
			simAccumDT += dt
		}

		if noticeTimer > 0 {
			noticeTimer -= dt
			if noticeTimer <= 0 {
				noticeText = ""
				noticeTimer = 0
			}
		}

		if rl.IsKeyPressed(rl.KeyTab) {
			switch mode {
			case ModeDraw:
				setMode(ModeRules)
			case ModeRules:
				setMode(ModeRoute)
			case ModeRoute:
				setMode(ModeTraffic)
			case ModeTraffic:
				setMode(ModePedestrian)
			case ModePedestrian:
				setMode(ModeDriving)
			default:
				setMode(ModeDraw)
			}
		}
		if rl.IsKeyPressed(rl.KeyE) {
			setTool(ToolSpline)
		}
		if rl.IsKeyPressed(rl.KeyQ) {
			setTool(ToolQuadratic)
		}
		if rl.IsKeyPressed(rl.KeyR) {
			setTool(ToolRouteCars)
		}
		if rl.IsKeyPressed(rl.KeyB) {
			setTool(ToolRouteBuses)
		}
		if rl.IsKeyPressed(rl.KeyZ) {
			setTool(ToolRouteEraser)
		}
		if rl.IsKeyPressed(rl.KeyP) {
			setTool(ToolPriority)
		}
		if rl.IsKeyPressed(rl.KeyL) {
			setTool(ToolCouple)
		}
		if rl.IsKeyPressed(rl.KeyC) {
			setTool(ToolCut)
		}
		if rl.IsKeyPressed(rl.KeyX) {
			setTool(ToolReverse)
		}
		if rl.IsKeyPressed(rl.KeyS) && !isCtrlDown() {
			setTool(ToolSpeedLimit)
		}
		if rl.IsKeyPressed(rl.KeyV) {
			setTool(ToolPreference)
		}
		if rl.IsKeyPressed(rl.KeyT) {
			setTool(ToolTrafficLight)
		}
		if rl.IsKeyPressed(rl.KeyG) {
			setTool(ToolDrive)
		}
		if rl.IsKeyPressed(rl.KeyW) {
			setTool(ToolPedestrianPath)
		}
		if rl.IsKeyPressed(rl.KeyD) {
			debugMode = !debugMode
		}
		if rl.IsKeyPressed(rl.KeyI) {
			infoMode = !infoMode
		}
		if rl.IsKeyPressed(rl.KeyH) {
			hitboxDebugMode = !hitboxDebugMode
		}
		if rl.IsKeyPressed(rl.KeyF3) {
			profileMode = !profileMode
		}
		if rl.IsKeyPressed(rl.KeySpace) && activeDurInput == -1 {
			paused = !paused
			editorMutatedWorld = true
		}
		if isCtrlDown() && rl.IsKeyPressed(rl.KeyS) {
			paused = true
			path, err := filepicker.PickSplineFilePath(true)
			if err != nil {
				noticeText = fmt.Sprintf("Save failed: %v", err)
				noticeTimer = 3.0
			} else if path != "" {
				if err := world.Save(path); err != nil {
					noticeText = fmt.Sprintf("Save failed: %v", err)
				} else {
					noticeText = fmt.Sprintf("Saved %d splines, %d routes, %d cars, %d lights to %s",
						len(world.Splines), len(world.Routes), len(world.Cars), len(world.TrafficLights), path)
				}
				noticeTimer = 3.0
			}
		}
		if isCtrlDown() && rl.IsKeyPressed(rl.KeyO) {
			path, err := filepicker.PickSplineFilePath(false)
			if err != nil {
				noticeText = fmt.Sprintf("Load failed: %v", err)
				noticeTimer = 3.0
			} else if path != "" {
				loadedWorld, err := simpkg.LoadWorld(path)
				if err != nil {
					noticeText = fmt.Sprintf("Load failed: %v", err)
					noticeTimer = 3.0
				} else {
					world = *loadedWorld
					paused = true
					editorMutatedWorld = true
					stage = StageIdle
					draft = newDraft()
					quadraticDraft = newQuadraticDraft()
					cutDraft = newCutDraft()
					pedestrianDraft = pedestrianPathDraft{}
					routePanel = RoutePanel{}
					routeStartSplineID = -1
					coupleModeFirstID = -1
					editingCycleID = -1
					editingLights = false
					editingPhaseIdx = -1
					showPhaseIdx = -1
					activeDurInput = -1
					durInputStr = ""
					pendingLights = pendingLights[:0]
					lastPref = maxLoadedPreference(world.Splines)
					noticeText = fmt.Sprintf("Loaded %d splines, %d routes, %d cars, %d lights from %s",
						len(world.Splines), len(world.Routes), len(world.Cars), len(world.TrafficLights), path)
					noticeTimer = 3.0
				}
			}
		}

		wheel := rl.GetMouseWheelMove()
		if wheel != 0 {
			zoomCameraToMouse(&camera, wheel)
		}

		if mode == ModeDriving && !playerCar.Active {
			driveRestoreTarget = camera.Target
			playerCar = newPlayerCarState(fromRLVec2(camera.Target))
		} else if mode != ModeDriving && playerCar.Active {
			playerCar.Active = false
			playerCar.Speed = 0
			world.ClearPlayerProxy()
			camera.Target = driveRestoreTarget
		}
		if mode == ModeDriving {
			updatePlayerCar(&playerCar, dt)
			camera.Target = toRLVec2(playerCar.Position)
			camera.Rotation = -playerCar.HeadingDeg
		} else {
			camera.Rotation = 0
		}

		mouseScreenRL := rl.GetMousePosition()
		mouseScreen := fromRLVec2(mouseScreenRL)
		mouseWorld := fromRLVec2(rl.GetScreenToWorld2D(mouseScreenRL, camera))
		mouseOnToolbar := isMouseOverToolbar(mouseScreen)

		// Toolbar click — switch mode / toggle debug
		if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && mouseOnToolbar {
			for i, item := range modeToolbarItems {
				if rl.CheckCollisionPointRec(mouseScreenRL, toolbarBtnRect(i)) {
					if item.isDbg {
						debugMode = !debugMode
					} else if item.isInfo {
						infoMode = !infoMode
					} else if item.isHitbox {
						hitboxDebugMode = !hitboxDebugMode
					} else {
						setMode(item.mode)
					}
					break
				}
			}
			for i, item := range toolsForMode(mode) {
				if rl.CheckCollisionPointRec(mouseScreenRL, toolBtnRect(i)) {
					setTool(item.tool)
					break
				}
			}
		}

		editorState := newEditorWorldState(&world)
		splines := editorState.Splines
		laneChangeSplines := editorState.LaneChangeSplines
		routes := editorState.Routes
		cars := editorState.Cars
		trafficLights := editorState.TrafficLights
		trafficCycles := editorState.TrafficCycles
		pedestrianPaths := editorState.PedestrianPaths
		nextSplineID := editorState.NextSplineID
		nextRouteID := editorState.NextRouteID
		nextLightID := editorState.NextLightID
		nextCycleID := editorState.NextCycleID

		hoverRadius := pixelsToWorld(camera.Zoom, hoverPixels)
		snapRadius := pixelsToWorld(camera.Zoom, snapPixels)
		handleRadius := pixelsToWorld(camera.Zoom, handlePixels)
		baseThickness := pixelsToWorld(camera.Zoom, linePixels)

		hoveredSpline := findHoveredSpline(splines, mouseWorld, hoverRadius)
		hoveredNode := findNearbyEndpoint(splines, mouseWorld, snapRadius)
		hoveredEnd := findNearbyEnd(splines, mouseWorld, snapRadius)
		hoveredStart := findNearbyStart(splines, mouseWorld, snapRadius)
		splineToolMouse, geometrySnap := applySplineToolSnap(mouseWorld, splines, camera.Zoom)
		if mode == ModeDriving && playerCar.Active && !mouseOnToolbar && rl.IsMouseButtonPressed(rl.MouseButtonRight) && hoveredEnd.SplineID >= 0 {
			playerCar.DestinationSplineID = hoveredEnd.SplineID
			playerCar.DestinationPoint = hoveredEnd.Point
			playerCar.HasDestination = true
		}

		frameProf.inputMS = sinceMS(inputStart)
		vehicleCounts := simpkg.BuildVehicleCounts(cars)
		baseGraph := simpkg.NewRoadGraph(splines, vehicleCounts)
		allSplines := simpkg.MergedSplines(splines, laneChangeSplines)
		debugBlameLinks := world.DebugBlameLinks
		holdBlameLinks := world.HoldBlameLinks
		debugCandidateLinks := world.DebugCandidateLinks
		frameProf.basePathHits = world.BasePathHits
		frameProf.basePathMisses = world.BasePathMisses
		frameProf.allPathHits = world.AllPathHits
		frameProf.allPathMisses = world.AllPathMisses
		frameProf.stepMS = world.StepMS
		frameProf.routeVisualsMS = world.RouteVisualsMS
		frameProf.laneChangesMS = world.LaneChangesMS
		frameProf.graphBuildMS = world.GraphBuildMS
		frameProf.brakingMS = world.BrakingMS
		frameProf.followMS = world.FollowMS
		frameProf.updateCarsMS = world.UpdateCarsMS
		frameProf.brakingDetail = world.BrakingProfile
		frameProf.followDetail = world.FollowingProfile
		frameProf.updateCarsDetail = world.UpdateCarsProfile

		if debugMode && rl.IsMouseButtonPressed(rl.MouseButtonMiddle) && !mouseOnToolbar {
			clicked := findClickedCar(cars, allSplines, simpkg.BuildSplineIndexByID(allSplines), mouseWorld)
			if clicked < 0 {
				editorState.clearCarDebugSelection()
			} else if cars[clicked].ID == editorState.DebugSelectedCarID {
				if editorState.DebugSelectedCarMode == 0 {
					editorState.DebugSelectedCarMode = 1
				} else {
					editorState.clearCarDebugSelection()
				}
			} else {
				editorState.DebugSelectedCarID = cars[clicked].ID
				editorState.DebugSelectedCarMode = 0
			}
			editorMutatedWorld = true
		}

		if routePanel.Open {
			var applied bool
			routePanel, routes, cars, applied = updateRoutePanel(routePanel, routes, cars, &nextRouteID, baseGraph, splines, mouseScreen)
			if applied {
				routeStartSplineID = -1
				editorState.handleCarsChanged()
				editorMutatedWorld = true
			}
		}

		if routePanel.Open && routePanel.VehicleKind == VehicleBus && routePanel.AddingBusStop && !mouseOnToolbar {
			panelRect := routePanelRect(routePanel)
			if !pointInRect(mouseScreen, panelRect) {
				var changed bool
				routePanel, changed = handleBusStopPlacement(routePanel, splines, baseGraph, mouseWorld, camera.Zoom)
				if changed {
					routePanel = syncRoutePanelPath(routePanel, baseGraph)
				}
			}
		}

		if !routePanel.Open && !mouseOnToolbar {
			switch tool {
			case ToolSpline:
				var topologyChanged bool
				stage, draft, splines, nextSplineID, topologyChanged = handleEditMode(stage, draft, splines, hoveredSpline, hoveredNode, splineToolMouse, geometrySnap, nextSplineID)
				if topologyChanged {
					editorState.Splines = splines
					editorState.Routes = routes
					editorState.Cars = cars
					editorState.LaneChangeSplines = laneChangeSplines
					editorState.handleTopologyChanged()
					routes = editorState.Routes
					cars = editorState.Cars
					laneChangeSplines = editorState.LaneChangeSplines
					editorMutatedWorld = true
				}
			case ToolQuadratic:
				var topologyChanged bool
				var notice string
				stage, quadraticDraft, splines, nextSplineID, topologyChanged, notice = handleQuadraticMode(stage, quadraticDraft, splines, hoveredSpline, hoveredNode, splineToolMouse, geometrySnap, nextSplineID)
				if topologyChanged {
					editorState.Splines = splines
					editorState.Routes = routes
					editorState.Cars = cars
					editorState.LaneChangeSplines = laneChangeSplines
					editorState.handleTopologyChanged()
					routes = editorState.Routes
					cars = editorState.Cars
					laneChangeSplines = editorState.LaneChangeSplines
					editorMutatedWorld = true
				}
				if notice != "" {
					noticeText = notice
					noticeTimer = 3.0
				}
			case ToolReverse:
				var topologyChanged bool
				splines, trafficLights, topologyChanged = handleReverseMode(splines, trafficLights, hoveredSpline)
				if topologyChanged {
					editorState.Splines = splines
					editorState.Routes = routes
					editorState.Cars = cars
					editorState.LaneChangeSplines = laneChangeSplines
					editorState.handleTopologyChanged()
					routes = editorState.Routes
					cars = editorState.Cars
					laneChangeSplines = editorState.LaneChangeSplines
					editorMutatedWorld = true
				}
			case ToolRouteCars:
				var notice string
				routeStartSplineID, routePanel, notice = handleRouteMode(routeStartSplineID, routePanel, routes, baseGraph, hoveredStart, hoveredEnd, VehicleCar)
				if notice != "" {
					noticeText = notice
					noticeTimer = 3.0
				}
			case ToolRouteBuses:
				var notice string
				if routeStartSplineID < 0 && rl.IsMouseButtonPressed(rl.MouseButtonRight) && hoveredSpline >= 0 {
					splines, notice = toggleBusOnlySpline(splines, hoveredSpline)
					editorState.Splines = splines
					editorState.Routes = routes
					editorState.LaneChangeSplines = laneChangeSplines
					editorState.handleRouteGraphChanged(false)
					routes = editorState.Routes
					laneChangeSplines = editorState.LaneChangeSplines
					editorMutatedWorld = true
				} else {
					routeStartSplineID, routePanel, notice = handleRouteMode(routeStartSplineID, routePanel, routes, baseGraph, hoveredStart, hoveredEnd, VehicleBus)
				}
				if notice != "" {
					noticeText = notice
					noticeTimer = 3.0
				}
			case ToolRouteEraser:
				cars = eraseCarsAtPoint(cars, allSplines, simpkg.BuildSplineIndexByID(allSplines), mouseWorld, 5.0)
				if len(cars) != len(editorState.Cars) {
					editorState.handleCarsChanged()
					editorMutatedWorld = true
				}
			case ToolPriority:
				splines = handlePriorityMode(splines, hoveredSpline)
				if hoveredSpline >= 0 && (rl.IsMouseButtonPressed(rl.MouseButtonLeft) || rl.IsMouseButtonPressed(rl.MouseButtonRight)) {
					editorMutatedWorld = true
				}
			case ToolCouple:
				var coupleNotice string
				prevFirstID := coupleModeFirstID
				coupleModeFirstID, splines, coupleNotice = handleCoupleMode(coupleModeFirstID, splines, hoveredSpline)
				if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && prevFirstID >= 0 && hoveredSpline >= 0 && splines[hoveredSpline].ID != prevFirstID {
					editorMutatedWorld = true
				}
				if coupleNotice != "" {
					noticeText = coupleNotice
					noticeTimer = 3.0
				}
			case ToolCut:
				var topologyChanged bool
				stage, cutDraft, splines, nextSplineID, topologyChanged = handleCutMode(stage, cutDraft, splines, mouseWorld, nextSplineID)
				if topologyChanged {
					editorState.Splines = splines
					editorState.Routes = routes
					editorState.Cars = cars
					editorState.LaneChangeSplines = laneChangeSplines
					editorState.handleTopologyChanged()
					routes = editorState.Routes
					cars = editorState.Cars
					laneChangeSplines = editorState.LaneChangeSplines
					editorMutatedWorld = true
				}
			case ToolSpeedLimit:
				splines = handleSpeedLimitMode(splines, hoveredSpline, selectedSpeedKmh)
				selectedSpeedKmh = updateSpeedLimitPanel(selectedSpeedKmh)
				if hoveredSpline >= 0 && (rl.IsMouseButtonPressed(rl.MouseButtonLeft) || rl.IsMouseButtonPressed(rl.MouseButtonRight)) {
					editorMutatedWorld = true
				}
			case ToolPreference:
				splines, lastPref = handlePreferenceMode(splines, hoveredSpline, lastPref)
				if hoveredSpline >= 0 && (rl.IsMouseButtonPressed(rl.MouseButtonLeft) || rl.IsMouseButtonPressed(rl.MouseButtonRight)) {
					editorMutatedWorld = true
				}
			case ToolPedestrianPath:
				var changed bool
				pedestrianDraft, pedestrianPaths, changed = handlePedestrianPathMode(pedestrianDraft, pedestrianPaths, mouseWorld, camera.Zoom)
				if changed {
					editorMutatedWorld = true
					pedestrianTopologyChanged = true
				}
			case ToolTrafficLight:
				phaseCount := 0
				if editingCycleID >= 0 {
					for _, c := range trafficCycles {
						if c.ID == editingCycleID {
							phaseCount = len(c.Phases)
							break
						}
					}
				}
				pr := trafficCyclePanelRect(editingCycleID >= 0, phaseCount, len(pendingLights))
				overPanel := rl.CheckCollisionPointRec(mouseScreenRL, pr)

				// ── keyboard ──────────────────────────────────────────────
				if rl.IsKeyPressed(rl.KeyEscape) {
					if activeDurInput != -1 {
						activeDurInput = -1
						durInputStr = ""
					} else if editingPhaseIdx >= 0 {
						editingPhaseIdx = -1
					} else if editingCycleID >= 0 {
						editingCycleID = -1
						editingLights = false
					} else {
						pendingLights = pendingLights[:0]
					}
				}

				// ── duration text input ───────────────────────────────────
				if activeDurInput != -1 {
					for {
						ch := rl.GetCharPressed()
						if ch == 0 {
							break
						}
						if (ch >= '0' && ch <= '9') || (ch == '.' && !containsRune(durInputStr, '.')) {
							if len(durInputStr) < 7 {
								durInputStr += string(ch)
							}
						}
					}
					if rl.IsKeyPressed(rl.KeyBackspace) && len(durInputStr) > 0 {
						durInputStr = durInputStr[:len(durInputStr)-1]
					}
					if rl.IsKeyPressed(rl.KeyEnter) || rl.IsKeyPressed(rl.KeyKpEnter) {
						trafficCycles = commitDurInput(trafficCycles, editingCycleID, activeDurInput, durInputStr)
						editorMutatedWorld = true
						activeDurInput = -1
						durInputStr = ""
					}
					// click outside the input field → commit
					if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
						for ci, c := range trafficCycles {
							if c.ID != editingCycleID {
								continue
							}
							var activeField rl.Rectangle
							if activeDurInput >= 0 && activeDurInput < len(c.Phases) {
								activeField = getPhaseRowBtns(pr, activeDurInput).durField
							} else if activeDurInput <= -3 {
								clrIdx := -(activeDurInput + 3)
								if clrIdx >= 0 && clrIdx < len(c.Phases) {
									activeField = getPhaseRowBtns(pr, clrIdx).clearField
								}
							}
							if !rl.CheckCollisionPointRec(mouseScreenRL, activeField) {
								trafficCycles = commitDurInput(trafficCycles, editingCycleID, activeDurInput, durInputStr)
								editorMutatedWorld = true
								activeDurInput = -1
								durInputStr = ""
							}
							_ = ci
							break
						}
					}
				}

				// ── panel button clicks ───────────────────────────────────
				// Find whether the current cycle is enabled
				cycleIsOn := false
				for _, c := range trafficCycles {
					if c.ID == editingCycleID {
						cycleIsOn = c.Enabled
						break
					}
				}
				if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && overPanel && activeDurInput < 0 {
					if editingCycleID < 0 {
						// new-cycle panel: Create button
						if len(pendingLights) > 0 {
							btn := trafficCreateBtnRect(len(pendingLights))
							if rl.CheckCollisionPointRec(mouseScreenRL, btn) {
								pendingLights, trafficLights, trafficCycles, editingCycleID = doCreateTrafficCycle(pendingLights, trafficLights, trafficCycles, &nextCycleID)
								editorMutatedWorld = true
							}
						}
					} else {
						// cycle editor panel
						onOffBtnR, editLightsBtn, closeBtnR := trafficHeaderBtnRects(pr)
						// On/Off toggle
						if rl.CheckCollisionPointRec(mouseScreenRL, onOffBtnR) {
							trafficCycles = trafficToggleCycleEnabled(trafficCycles, editingCycleID)
							editorMutatedWorld = true
							// turning on: clear all editing/preview state
							if !cycleIsOn {
								editingLights = false
								editingPhaseIdx = -1
								activeDurInput = -1
								durInputStr = ""
								showPhaseIdx = -1
							}
						}
						// Edit Lights toggle (only when cycle is off)
						if !cycleIsOn && rl.CheckCollisionPointRec(mouseScreenRL, editLightsBtn) {
							editingLights = !editingLights
							editingPhaseIdx = -1
						}
						if rl.CheckCollisionPointRec(mouseScreenRL, closeBtnR) {
							editingCycleID = -1
							editingLights = false
							editingPhaseIdx = -1
							showPhaseIdx = -1
						}
						// Add Phase button (only when cycle is off)
						if !cycleIsOn && rl.CheckCollisionPointRec(mouseScreenRL, trafficAddPhaseBtnRect(pr)) {
							trafficCycles = trafficAddPhase(trafficCycles, editingCycleID)
							editorMutatedWorld = true
						}
						// Per-phase row buttons
						for pi := 0; pi < phaseCount; pi++ {
							row := getPhaseRowBtns(pr, pi)
							// Show (cycle off) / Skip (cycle on) button: always active
							if rl.CheckCollisionPointRec(mouseScreenRL, row.showBtn) {
								if cycleIsOn {
									trafficCycles = trafficSkipToPhase(trafficCycles, editingCycleID, pi)
								} else {
									if showPhaseIdx == pi {
										showPhaseIdx = -1
									} else {
										showPhaseIdx = pi
										editingPhaseIdx = -1
									}
								}
							}
							// Editing buttons: only when cycle is off
							if !cycleIsOn {
								if rl.CheckCollisionPointRec(mouseScreenRL, row.upBtn) && pi > 0 {
									trafficCycles = trafficMovePhase(trafficCycles, editingCycleID, pi, -1)
									editorMutatedWorld = true
									if editingPhaseIdx == pi {
										editingPhaseIdx--
									}
									if showPhaseIdx == pi {
										showPhaseIdx--
									}
								}
								if rl.CheckCollisionPointRec(mouseScreenRL, row.downBtn) && pi < phaseCount-1 {
									trafficCycles = trafficMovePhase(trafficCycles, editingCycleID, pi, +1)
									editorMutatedWorld = true
									if editingPhaseIdx == pi {
										editingPhaseIdx++
									}
									if showPhaseIdx == pi {
										showPhaseIdx++
									}
								}
								if rl.CheckCollisionPointRec(mouseScreenRL, row.durField) {
									for _, c := range trafficCycles {
										if c.ID == editingCycleID && pi < len(c.Phases) {
											activeDurInput = pi
											durInputStr = fmt.Sprintf("%.1f", c.Phases[pi].DurationSecs)
											break
										}
									}
								}
								if rl.CheckCollisionPointRec(mouseScreenRL, row.clearField) {
									for _, c := range trafficCycles {
										if c.ID == editingCycleID && pi < len(c.Phases) {
											activeDurInput = -(pi + 3)
											clrDur := c.Phases[pi].ClearanceDurationSecs
											if clrDur <= 0 {
												clrDur = 3.0
											}
											durInputStr = fmt.Sprintf("%.1f", clrDur)
											break
										}
									}
								}
								if rl.CheckCollisionPointRec(mouseScreenRL, row.editBtn) {
									if editingPhaseIdx == pi {
										editingPhaseIdx = -1
									} else {
										editingPhaseIdx = pi
										editingLights = false
										showPhaseIdx = -1
									}
								}
								if rl.CheckCollisionPointRec(mouseScreenRL, row.delBtn) {
									trafficCycles = trafficDeletePhase(trafficCycles, editingCycleID, pi)
									editorMutatedWorld = true
									if editingPhaseIdx == pi {
										editingPhaseIdx = -1
									} else if editingPhaseIdx > pi {
										editingPhaseIdx--
									}
									if showPhaseIdx == pi {
										showPhaseIdx = -1
									} else if showPhaseIdx > pi {
										showPhaseIdx--
									}
								}
							}
						}
					}
				} else if !overPanel && activeDurInput < 0 {
					// ── world interactions ────────────────────────────────
					if editingPhaseIdx >= 0 {
						// toggle light state in this phase
						if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
							if hitID := trafficLightAt(trafficLights, mouseWorld, camera.Zoom); hitID >= 0 {
								trafficCycles = trafficToggleLightInPhase(trafficCycles, editingCycleID, editingPhaseIdx, hitID)
								editorMutatedWorld = true
							}
						}
					} else if editingCycleID >= 0 && editingLights {
						trafficLights, trafficCycles = handleTrafficLightEdit(splines, trafficLights, trafficCycles, editingCycleID, mouseWorld, camera.Zoom, &nextLightID)
						if rl.IsMouseButtonPressed(rl.MouseButtonLeft) || rl.IsMouseButtonPressed(rl.MouseButtonRight) {
							editorMutatedWorld = true
						}
					} else if editingCycleID < 0 {
						if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && len(pendingLights) == 0 {
							if hitID := trafficLightAt(trafficLights, mouseWorld, camera.Zoom); hitID >= 0 {
								for _, l := range trafficLights {
									if l.ID == hitID {
										editingCycleID = l.CycleID
										break
									}
								}
							} else {
								pendingLights = handleTrafficLightMode(splines, pendingLights, mouseWorld, camera.Zoom, &nextLightID)
							}
						} else {
							pendingLights = handleTrafficLightMode(splines, pendingLights, mouseWorld, camera.Zoom, &nextLightID)
						}
					}
				}
			}
		}

		editorState.Splines = splines
		editorState.LaneChangeSplines = laneChangeSplines
		editorState.Routes = routes
		editorState.Cars = cars
		editorState.TrafficLights = trafficLights
		editorState.TrafficCycles = trafficCycles
		editorState.PedestrianPaths = pedestrianPaths
		editorState.NextSplineID = nextSplineID
		editorState.NextRouteID = nextRouteID
		editorState.NextLightID = nextLightID
		editorState.NextCycleID = nextCycleID
		editorState.commit(&world)
		if pedestrianTopologyChanged {
			world.ResetPedestrianRuntime()
		}
		if mode == ModeDriving && playerCar.Active {
			world.UpdatePlayerProxy(simpkg.PlayerProxyFitInput{
				Position:            playerCar.Position,
				Heading:             playerForward(playerCar.HeadingDeg),
				Speed:               playerCar.Speed,
				Length:              playerCar.Length,
				Width:               playerCar.Width,
				CarID:               1_000_000_001,
				Color:               NewColor(230, 86, 46, 255),
				DestinationSplineID: playerCar.DestinationSplineID,
			})
		}
		cars = world.Cars
		pedestrians := world.Pedestrians

		if editorMutatedWorld {
			worldRevision++
			simAccumDT = 0
		}
		if !paused && !stepRunning && simAccumDT > 0 {
			stepWorld := world.Clone()
			stepRevision := worldRevision
			stepDT := simAccumDT
			simAccumDT = 0
			stepRunning = true
			go func(snapshot simpkg.World, revision uint64, dt float32) {
				snapshot.Step(dt)
				stepResults <- asyncStepResult{World: snapshot, Revision: revision}
			}(stepWorld, stepRevision, stepDT)
		}

		drawStart := time.Now()
		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode2D(camera)
		viewRect := cameraWorldRect(camera, pixelsToWorld(camera.Zoom, 64))
		drawGrid(camera)
		drawAxes(camera)

		drawPedestrianPaths(pedestrianPaths, viewRect)
		if tool == ToolPedestrianPath {
			drawPedestrianPathTool(pedestrianDraft, pedestrianPaths, mouseWorld, camera.Zoom)
		}
		drawPedestrians(pedestrians, pedestrianPaths, camera.Zoom, viewRect)

		splineIndexByID := simpkg.BuildSplineIndexByID(splines)
		directionWarnings := findDirectionWarnings(splines)
		for _, route := range routes {
			if !route.Valid {
				continue
			}
			drawRouteWithIndex(route, splines, splineIndexByID, pixelsToWorld(camera.Zoom, 2.0), camera.Zoom, viewRect)
		}
		if routePanel.Open && len(routePanel.PathIDs) > 0 {
			drawRouteWithIndex(Route{
				PathIDs:       routePanel.PathIDs,
				StartSplineID: routePanel.StartSplineID,
				EndSplineID:   routePanel.EndSplineID,
				Color:         simpkg.RoutePaletteColor(routePanel.ColorIndex),
				VehicleKind:   routePanel.VehicleKind,
				BusStops:      routePanel.BusStops,
				Valid:         true,
			}, splines, splineIndexByID, pixelsToWorld(camera.Zoom, 3.0), camera.Zoom, viewRect)
		}

		for i, spline := range splines {
			if !splineVisibleInWorldRect(spline, viewRect) {
				continue
			}
			color := splineDrawColor(spline)
			thickness := baseThickness
			if spline.Priority {
				thickness = maxf(thickness, pixelsToWorld(camera.Zoom, 4))
			}
			if i == hoveredSpline {
				if ((tool == ToolSpline || tool == ToolQuadratic) && stage == StageIdle) || tool == ToolPriority {
					color = NewColor(242, 153, 74, 255)
					thickness = pixelsToWorld(camera.Zoom, 5)
				}
			}

			drawSpline(spline, thickness, color)
			drawEndpoint(spline.P0, handleRadius*0.8, NewColor(60, 160, 90, 255))
			drawEndpoint(spline.P3, handleRadius, NewColor(50, 115, 225, 255))
		}
		drawDirectionWarnings(directionWarnings, camera.Zoom, viewRect)
		if tool == ToolReverse {
			drawSplineDirectionArrows(splines, camera.Zoom, viewRect)
		}
		if geometrySnap.Active && geometrySnap.SourceSplineIndex >= 0 && (tool == ToolSpline || tool == ToolQuadratic) {
			drawSpline(splines[geometrySnap.SourceSplineIndex], pixelsToWorld(camera.Zoom, 4), NewColor(74, 196, 186, 200))
		}

		if tool == ToolSpline {
			if stage == StageIdle {
				drawGeometrySnapHint(geometrySnap, splineToolMouse, camera.Zoom)
				if geometrySnap.Active {
					drawEndpoint(splineToolMouse, handleRadius*1.35, NewColor(74, 196, 186, 235))
				}
			}
			if hoveredNode.SplineIndex >= 0 && stage == StageIdle {
				drawEndpoint(hoveredNode.Point, handleRadius*1.5, NewColor(255, 196, 61, 255))
			}
			if hoveredNode.SplineIndex >= 0 && stage == StageSetP3 {
				drawEndpoint(hoveredNode.Point, handleRadius*1.5, NewColor(163, 92, 255, 255))
			}

			previewMouse := splineToolMouse
			preview, hasPreview := buildPreview(stage, draft, previewMouse, geometrySnap, hoveredNode, splines)
			if hasPreview {
				drawGeometrySnapHint(geometrySnap, previewMouse, camera.Zoom)
				drawEditSnapHint(stage, draft, hoveredNode, splines, previewMouse, camera.Zoom)
				drawDraft(stage, draft, previewMouse, camera.Zoom, geometrySnap)
				drawSpline(preview, pixelsToWorld(camera.Zoom, 4), NewColor(214, 76, 76, 255))
			}
		}
		if tool == ToolQuadratic {
			if stage == StageIdle {
				drawGeometrySnapHint(geometrySnap, splineToolMouse, camera.Zoom)
				if geometrySnap.Active {
					drawEndpoint(splineToolMouse, handleRadius*1.35, NewColor(74, 196, 186, 235))
				}
			}
			if hoveredNode.SplineIndex >= 0 && stage == StageIdle {
				drawEndpoint(hoveredNode.Point, handleRadius*1.5, NewColor(255, 196, 61, 255))
			}

			previewMouse := splineToolMouse
			preview, hasPreview := buildQuadraticPreview(stage, quadraticDraft, previewMouse, geometrySnap, hoveredNode, splines)
			if hasPreview {
				drawGeometrySnapHint(geometrySnap, previewMouse, camera.Zoom)
				drawQuadraticSnapHint(stage, quadraticDraft, hoveredNode, splines, previewMouse, camera.Zoom)
				drawQuadraticDraft(stage, quadraticDraft, previewMouse, camera.Zoom, geometrySnap)
				drawSpline(preview, pixelsToWorld(camera.Zoom, 4), NewColor(214, 76, 76, 255))
			}
		}
		if tool == ToolReverse && hoveredSpline >= 0 {
			drawSpline(splines[hoveredSpline], pixelsToWorld(camera.Zoom, 4), NewColor(255, 120, 40, 180))
		}

		if tool == ToolRouteCars {
			drawRoutePicking(routeStartSplineID, routePanel, hoveredStart, hoveredEnd, splines, splineIndexByID, baseGraph, camera.Zoom, viewRect)
		}
		if tool == ToolRouteBuses {
			drawRoutePicking(routeStartSplineID, routePanel, hoveredStart, hoveredEnd, splines, splineIndexByID, baseGraph, camera.Zoom, viewRect)
			if routePanel.Open && routePanel.AddingBusStop {
				drawBusStopPlacementPreview(routePanel, splines, baseGraph, mouseWorld, camera.Zoom)
			}
		}
		if tool == ToolRouteEraser {
			r := float32(5.0)
			drawCircleV(mouseWorld, r, NewColor(220, 70, 70, 30))
			drawRing(mouseWorld, r*0.94, r, 0, 360, 28, NewColor(220, 70, 70, 220))
		}

		if tool == ToolCouple {
			drawCoupleMode(splines, coupleModeFirstID, hoveredSpline, camera.Zoom)
		}
		if tool == ToolSpeedLimit {
			drawSpeedLimitWorld(splines, hoveredSpline, camera.Zoom)
		}
		if tool == ToolCut {
			drawCutMode(stage, cutDraft, splines, mouseWorld, camera.Zoom)
		}
		if mode == ModeDriving && playerCar.HasDestination {
			drawDrivingDestination(playerCar, camera.Zoom, viewRect)
		}
		if mode == ModeDriving && world.HasPlayerProxy {
			drawPlayerProxyAttachment(world.PlayerProxyCar, splines, splineIndexByID, camera.Zoom, viewRect)
		}
		allSplineIndexByID := simpkg.BuildSplineIndexByID(allSplines)
		drawCars(cars, allSplines, allSplineIndexByID, camera.Zoom, debugMode, viewRect)
		if hitboxDebugMode {
			drawCarHitboxes(cars, allSplines, allSplineIndexByID, viewRect)
		}
		if debugMode {
			drawDebugBlameLinks(debugBlameLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, NewColor(220, 50, 50, 220), viewRect)
			drawDebugBlameLinks(holdBlameLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, NewColor(255, 165, 0, 220), viewRect)
			if world.DebugSelectedCarMode == 0 {
				drawDebugBlameLinks(debugCandidateLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, NewColor(50, 220, 50, 200), viewRect)
			} else {
				drawDebugBlameLinks(debugCandidateLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, NewColor(180, 50, 220, 200), viewRect)
			}
			if sel := simpkg.FindCarIndexByID(cars, world.DebugSelectedCarID); sel >= 0 && sel < len(cars) {
				if _, center, _, ok := carBodyPose(cars[sel], allSplines, allSplineIndexByID); ok {
					r := pixelsToWorld(camera.Zoom, 8)
					ringColor := NewColor(50, 220, 50, 255)
					if world.DebugSelectedCarMode == 1 {
						ringColor = NewColor(180, 50, 220, 255)
					}
					drawRing(center, r*0.6, r, 0, 360, 24, ringColor)
				}
			}
			drawLaneChangeSplines(laneChangeSplines, camera.Zoom)
		}
		drawTrafficLights(trafficLights, pendingLights, trafficCycles, editingCycleID, editingPhaseIdx, showPhaseIdx, camera.Zoom, viewRect)
		if tool == ToolTrafficLight && (editingCycleID < 0 || editingLights) && editingPhaseIdx < 0 {
			if _, _, snap, found := findNearestSplinePoint(splines, mouseWorld); found {
				r := pixelsToWorld(camera.Zoom, 8)
				drawCircleV(snap, r, NewColor(255, 200, 0, 180))
				drawRing(snap, r*0.6, r, 0, 360, 16, NewColor(220, 160, 0, 220))
			}
		}
		rl.EndMode2D()
		if mode == ModeDriving && playerCar.Active {
			drawPlayerCarScreen(playerCar, camera.Zoom)
		}
		drawDirectionWarningLabels(directionWarnings, camera, viewRect)

		drawScaleBar(camera.Zoom)
		if tool == ToolSpline {
			previewMouse := splineToolMouse
			if preview, hasPreview := buildPreview(stage, draft, previewMouse, geometrySnap, hoveredNode, splines); hasPreview {
				drawDraftInfo(stage, draft, previewMouse, geometrySnap, preview, camera)
			}
		}
		if tool == ToolQuadratic {
			previewMouse := splineToolMouse
			if preview, hasPreview := buildQuadraticPreview(stage, quadraticDraft, previewMouse, geometrySnap, hoveredNode, splines); hasPreview {
				drawQuadraticDraftInfo(stage, quadraticDraft, previewMouse, geometrySnap, preview, camera)
			}
		}
		if tool == ToolSpeedLimit {
			drawSpeedLimitLabels(splines, camera, viewRect)
			drawCarSpeedLabels(cars, allSplines, allSplineIndexByID, camera, viewRect)
		}
		if tool == ToolPreference {
			drawPreferenceLabels(splines, camera, viewRect)
		}
		drawHud(mode, tool, stage, draft, quadraticDraft, hoveredSpline, routeStartSplineID, coupleModeFirstID, debugMode, hitboxDebugMode, infoMode, paused, camera.Zoom, len(splines), len(routes), len(cars))
		if profileMode {
			drawProfilerOverlay(prof)
		}
		if tool == ToolTrafficLight {
			drawTrafficCyclePanel(pendingLights, trafficLights, trafficCycles, editingCycleID, editingLights, editingPhaseIdx, activeDurInput, durInputStr, showPhaseIdx)
		}
		if tool == ToolSpeedLimit {
			drawSpeedLimitPanel(selectedSpeedKmh)
		}
		if routePanel.Open {
			drawRoutePanel(routePanel, routes)
		}
		if paused {
			drawNotice("⏸  Paused  (Space to resume)")
		} else if noticeText != "" {
			drawNotice(noticeText)
		}
		rl.DrawFPS(int32(rl.GetScreenWidth()-90), 10)
		frameProf.drawMS = sinceMS(drawStart)
		frameProf.frameMS = sinceMS(frameStart)
		prof.endFrame(frameProf)
		rl.EndDrawing()
	}
}

func snapToGrid(v Vec2, gridSize float32) Vec2 {
	return Vec2{
		X: float32(math.Round(float64(v.X)/float64(gridSize))) * gridSize,
		Y: float32(math.Round(float64(v.Y)/float64(gridSize))) * gridSize,
	}
}

func shiftDown() bool {
	return rl.IsKeyDown(rl.KeyLeftShift) || rl.IsKeyDown(rl.KeyRightShift)
}

func ctrlDown() bool {
	return rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl)
}

func leftAltDown() bool {
	return rl.IsKeyDown(rl.KeyLeftAlt)
}

func axisSnapStep() float32 {
	return 4.0
}

func projectPointOntoAxis(point, origin, axisDir Vec2) Vec2 {
	if vectorLengthSq(axisDir) <= 1e-9 {
		return origin
	}
	dir := normalize(axisDir)
	dist := dot(vecSub(point, origin), dir)
	return vecAdd(origin, vecScale(dir, dist))
}

func projectPointOntoAxisSnapped(point, origin, axisDir Vec2, step float32) Vec2 {
	if vectorLengthSq(axisDir) <= 1e-9 {
		return origin
	}
	dir := normalize(axisDir)
	dist := dot(vecSub(point, origin), dir)
	if step > 0 {
		dist = float32(math.Round(float64(dist)/float64(step))) * step
	}
	return vecAdd(origin, vecScale(dir, dist))
}

func lineIntersection(originA, dirA, originB, dirB Vec2) (Vec2, bool) {
	denom := cross2D(dirA, dirB)
	if absf(denom) <= 1e-6 {
		return Vec2{}, false
	}
	delta := vecSub(originB, originA)
	t := cross2D(delta, dirB) / denom
	return vecAdd(originA, vecScale(dirA, t)), true
}

type geometryAxisCandidate struct {
	Origin  Vec2
	AxisDir Vec2
	HasMinT bool
	MinT    float32
	HasMaxT bool
	MaxT    float32
}

func perpendicular(v Vec2) Vec2 {
	return NewVec2(-v.Y, v.X)
}

func appendGeometryLineAxis(out []geometryAxisCandidate, origin, axisDir Vec2) []geometryAxisCandidate {
	if vectorLengthSq(axisDir) <= 1e-9 {
		return out
	}
	return append(out, geometryAxisCandidate{Origin: origin, AxisDir: axisDir})
}

func appendGeometryRayAxis(out []geometryAxisCandidate, origin, axisDir Vec2) []geometryAxisCandidate {
	if vectorLengthSq(axisDir) <= 1e-9 {
		return out
	}
	return append(out, geometryAxisCandidate{
		Origin:  origin,
		AxisDir: axisDir,
		HasMinT: true,
		MinT:    0,
	})
}

func quadraticControlPointForSpline(spline Spline) (Vec2, bool) {
	fromStart := vecAdd(spline.P0, vecScale(vecSub(spline.P1, spline.P0), 1.5))
	fromEnd := vecAdd(spline.P3, vecScale(vecSub(spline.P2, spline.P3), 1.5))
	tol := maxf(0.05, spline.Length*0.01)
	if distSq(fromStart, fromEnd) > tol*tol {
		return Vec2{}, false
	}
	return vecScale(vecAdd(fromStart, fromEnd), 0.5), true
}

func projectPointOntoCandidate(point Vec2, axis geometryAxisCandidate) Vec2 {
	if vectorLengthSq(axis.AxisDir) <= 1e-9 {
		return axis.Origin
	}
	dir := normalize(axis.AxisDir)
	t := dot(vecSub(point, axis.Origin), dir)
	if ctrlDown() && shiftDown() {
		t = float32(math.Round(float64(t)/float64(axisSnapStep()))) * axisSnapStep()
	}
	if axis.HasMinT {
		t = maxf(t, axis.MinT)
	}
	if axis.HasMaxT {
		t = minf(t, axis.MaxT)
	}
	return vecAdd(axis.Origin, vecScale(dir, t))
}

func collectSplineGeometryAxes(spline Spline) []geometryAxisCandidate {
	var axes []geometryAxisCandidate
	startDir := vecSub(spline.P1, spline.P0)
	endDir := vecSub(spline.P3, spline.P2)
	axes = appendGeometryRayAxis(axes, spline.P0, vecScale(startDir, -1))
	axes = appendGeometryLineAxis(axes, spline.P0, perpendicular(startDir))
	axes = appendGeometryRayAxis(axes, spline.P3, endDir)
	axes = appendGeometryLineAxis(axes, spline.P3, perpendicular(endDir))
	if m, ok := quadraticControlPointForSpline(spline); ok {
		spanDir := vecSub(spline.P3, spline.P0)
		axes = appendGeometryLineAxis(axes, m, perpendicular(spanDir))
	}
	return axes
}

func findGeometrySnap(splines []Spline, point Vec2, radius float32) GeometrySnap {
	best := GeometrySnap{}
	bestDistSq := radius * radius
	for i, spline := range splines {
		for _, axis := range collectSplineGeometryAxes(spline) {
			projected := projectPointOntoCandidate(point, axis)
			if d := distSq(point, projected); d <= bestDistSq {
				bestDistSq = d
				best = GeometrySnap{
					Active:            true,
					SourceSplineID:    spline.ID,
					SourceSplineIndex: i,
					Origin:            axis.Origin,
					AxisDir:           axis.AxisDir,
					Point:             projected,
					HasMinT:           axis.HasMinT,
					MinT:              axis.MinT,
					HasMaxT:           axis.HasMaxT,
					MaxT:              axis.MaxT,
				}
			}
		}
	}
	return best
}

func applySplineToolSnap(mouse Vec2, splines []Spline, zoom float32) (Vec2, GeometrySnap) {
	if ctrlDown() && !shiftDown() {
		mouse = snapToGrid(mouse, 4.0)
	}
	if !shiftDown() {
		return mouse, GeometrySnap{}
	}
	snap := findGeometrySnap(splines, mouse, pixelsToWorld(zoom, snapPixels))
	if !snap.Active {
		return mouse, GeometrySnap{}
	}
	return snap.Point, snap
}

func axisConstrainedPoint(mouse, axisOrigin, axisDir Vec2, geometrySnap GeometrySnap) Vec2 {
	if geometrySnap.Active {
		if point, ok := lineIntersection(axisOrigin, axisDir, geometrySnap.Origin, geometrySnap.AxisDir); ok {
			return point
		}
	}
	if ctrlDown() && shiftDown() {
		return projectPointOntoAxisSnapped(mouse, axisOrigin, axisDir, axisSnapStep())
	}
	return projectPointOntoAxis(mouse, axisOrigin, axisDir)
}

func startPerpendicularLock(selectedPoint Vec2, geometrySnap GeometrySnap) (bool, Vec2, Vec2) {
	if !leftAltDown() || !geometrySnap.Active {
		return false, Vec2{}, Vec2{}
	}
	return true, selectedPoint, perpendicular(geometrySnap.AxisDir)
}

func pendingPerpendicularPoint(mouse Vec2, active bool, origin, axisDir Vec2, geometrySnap GeometrySnap) Vec2 {
	if !active {
		return mouse
	}
	return axisConstrainedPoint(mouse, origin, axisDir, geometrySnap)
}

func mirroredP1FromPrevSpline(prev Spline) Vec2 {
	return vecAdd(prev.P3, vecSub(prev.P3, prev.P2))
}

func mirroredP2FromNextSpline(next Spline) Vec2 {
	return vecSub(vecScale(next.P0, 2), next.P1)
}

func endpointAnchor(spline Spline, kind EndKind) Vec2 {
	if kind == EndStart {
		return spline.P0
	}
	return spline.P3
}

func endpointAxisDir(spline Spline, kind EndKind) Vec2 {
	if kind == EndStart {
		return vecSub(spline.P0, spline.P1)
	}
	return vecSub(spline.P3, spline.P2)
}

func mirroredHandleAtEndpoint(spline Spline, kind EndKind) Vec2 {
	anchor := endpointAnchor(spline, kind)
	if kind == EndStart {
		return vecSub(vecScale(anchor, 2), spline.P1)
	}
	return vecSub(vecScale(anchor, 2), spline.P2)
}

func draftP1Preview(draft Draft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.LockP1 {
		return axisConstrainedPoint(mouse, draft.P0, draft.P1AxisDir, geometrySnap)
	}
	if draft.PerpLockActive {
		return pendingPerpendicularPoint(mouse, true, draft.PerpLockOrigin, draft.PerpLockAxisDir, geometrySnap)
	}
	return mouse
}

func draftP2FreePreview(draft Draft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.PerpLockActive {
		return pendingPerpendicularPoint(mouse, true, draft.PerpLockOrigin, draft.PerpLockAxisDir, geometrySnap)
	}
	return mouse
}

func draftP2Preview(draft Draft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.SnapP3 {
		return axisConstrainedPoint(mouse, draft.P3, draft.P2AxisDir, geometrySnap)
	}
	return mouse
}

func draftP3Preview(draft Draft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.PerpLockActive {
		return pendingPerpendicularPoint(mouse, true, draft.PerpLockOrigin, draft.PerpLockAxisDir, geometrySnap)
	}
	return mouse
}

func newQuadraticDraft() QuadraticDraft {
	return QuadraticDraft{}
}

func newQuadraticSpline(id int, p0, m, p3 Vec2) Spline {
	p1 := vecAdd(p0, vecScale(vecSub(m, p0), 2.0/3.0))
	p2 := vecAdd(p3, vecScale(vecSub(m, p3), 2.0/3.0))
	return simpkg.NewSpline(id, p0, p1, p2, p3)
}

func quadraticMirroredMFromPrevSpline(prev Spline) Vec2 {
	return vecAdd(prev.P3, vecSub(prev.P3, prev.P2))
}

func quadraticMirroredMFromNextSpline(next Spline) Vec2 {
	return vecSub(vecScale(next.P0, 2), next.P1)
}

func quadraticMOnPrevAxis(draft QuadraticDraft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.FromPrevAxis {
		return axisConstrainedPoint(mouse, draft.P0, draft.PrevAxisDir, geometrySnap)
	}
	if draft.PerpLockActive {
		return pendingPerpendicularPoint(mouse, true, draft.PerpLockOrigin, draft.PerpLockAxisDir, geometrySnap)
	}
	return mouse
}

func quadraticMOnNextAxis(draft QuadraticDraft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.SnapP3 {
		return axisConstrainedPoint(mouse, draft.P3, draft.NextAxisDir, geometrySnap)
	}
	return draft.M
}

func quadraticP3Preview(draft QuadraticDraft, mouse Vec2, geometrySnap GeometrySnap) Vec2 {
	if draft.PerpLockActive {
		return pendingPerpendicularPoint(mouse, true, draft.PerpLockOrigin, draft.PerpLockAxisDir, geometrySnap)
	}
	return mouse
}

func handleQuadraticMode(stage Stage, draft QuadraticDraft, splines []Spline, hoveredSpline int, hoveredNode EndHit, mouseWorld Vec2, geometrySnap GeometrySnap, nextSplineID int) (Stage, QuadraticDraft, []Spline, int, bool, string) {
	topologyChanged := false
	notice := ""

	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		switch stage {
		case StageIdle:
			if hoveredSpline >= 0 {
				deletedID := splines[hoveredSpline].ID
				splines = append(splines[:hoveredSpline], splines[hoveredSpline+1:]...)
				splines = removeSplineFromCouplings(splines, deletedID)
				topologyChanged = true
			}
		case StageSetP1:
			stage = StageIdle
			draft = newQuadraticDraft()
		case StageSetP2:
			stage = StageSetP1
			draft.M = Vec2{}
			draft.P3 = Vec2{}
			draft.SnapP3 = false
			draft.NextAxisDir = Vec2{}
			draft.MMirroredFromPrev = false
			draft.PerpLockActive = false
			draft.PerpLockOrigin = Vec2{}
			draft.PerpLockAxisDir = Vec2{}
		}
	}

	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		switch stage {
		case StageIdle:
			draft = newQuadraticDraft()
			if hoveredNode.SplineIndex >= 0 {
				prev := splines[hoveredNode.SplineIndex]
				draft.P0 = endpointAnchor(prev, hoveredNode.Kind)
				draft.FromPrevAxis = true
				draft.PrevAxisDir = endpointAxisDir(prev, hoveredNode.Kind)
			} else {
				draft.P0 = mouseWorld
				draft.PerpLockActive, draft.PerpLockOrigin, draft.PerpLockAxisDir = startPerpendicularLock(draft.P0, geometrySnap)
			}
			stage = StageSetP1

		case StageSetP1:
			if hoveredNode.SplineIndex >= 0 {
				next := splines[hoveredNode.SplineIndex]
				nextAnchor := endpointAnchor(next, hoveredNode.Kind)
				nextAxis := endpointAxisDir(next, hoveredNode.Kind)
				if draft.FromPrevAxis {
					m, ok := lineIntersection(draft.P0, draft.PrevAxisDir, nextAnchor, nextAxis)
					if !ok {
						notice = "A quadratic spline can't be drawn this way."
						return stage, draft, splines, nextSplineID, topologyChanged, notice
					}
					spline := newQuadraticSpline(nextSplineID, draft.P0, m, nextAnchor)
					nextSplineID++
					splines = append(splines, spline)
					stage = StageIdle
					draft = newQuadraticDraft()
					topologyChanged = true
					break
				}
				draft.P3 = nextAnchor
				draft.SnapP3 = true
				draft.NextAxisDir = nextAxis
				draft.PerpLockActive = false
				draft.PerpLockOrigin = Vec2{}
				draft.PerpLockAxisDir = Vec2{}
				stage = StageSetP2
				break
			}

			draft.M = quadraticMOnPrevAxis(draft, mouseWorld, geometrySnap)
			draft.MMirroredFromPrev = false
			draft.PerpLockActive, draft.PerpLockOrigin, draft.PerpLockAxisDir = startPerpendicularLock(draft.M, geometrySnap)
			stage = StageSetP2

		case StageSetP2:
			if draft.SnapP3 {
				draft.M = quadraticMOnNextAxis(draft, mouseWorld, geometrySnap)
				spline := newQuadraticSpline(nextSplineID, draft.P0, draft.M, draft.P3)
				nextSplineID++
				splines = append(splines, spline)
				stage = StageIdle
				draft = newQuadraticDraft()
				topologyChanged = true
				break
			}

			p3 := quadraticP3Preview(draft, mouseWorld, geometrySnap)
			m := draft.M
			if hoveredNode.SplineIndex >= 0 {
				next := splines[hoveredNode.SplineIndex]
				nextAnchor := endpointAnchor(next, hoveredNode.Kind)
				nextAxis := endpointAxisDir(next, hoveredNode.Kind)
				p3 = nextAnchor
				if draft.MMirroredFromPrev {
					notice = "A quadratic spline can't be drawn this way."
					return stage, draft, splines, nextSplineID, topologyChanged, notice
				}
				if draft.FromPrevAxis {
					var ok bool
					m, ok = lineIntersection(draft.P0, draft.PrevAxisDir, nextAnchor, nextAxis)
					if !ok {
						notice = "A quadratic spline can't be drawn this way."
						return stage, draft, splines, nextSplineID, topologyChanged, notice
					}
				} else {
					m = mirroredHandleAtEndpoint(next, hoveredNode.Kind)
				}
			}
			spline := newQuadraticSpline(nextSplineID, draft.P0, m, p3)
			nextSplineID++
			splines = append(splines, spline)
			stage = StageIdle
			draft = newQuadraticDraft()
			topologyChanged = true
		}
	}

	return stage, draft, splines, nextSplineID, topologyChanged, notice
}

func handleEditMode(stage Stage, draft Draft, splines []Spline, hoveredSpline int, hoveredNode EndHit, mouseWorld Vec2, geometrySnap GeometrySnap, nextSplineID int) (Stage, Draft, []Spline, int, bool) {
	topologyChanged := false

	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		switch stage {
		case StageIdle:
			if hoveredSpline >= 0 {
				deletedID := splines[hoveredSpline].ID
				splines = append(splines[:hoveredSpline], splines[hoveredSpline+1:]...)
				splines = removeSplineFromCouplings(splines, deletedID)
				topologyChanged = true
			}
		case StageSetP1, StageSetP2:
			stage = StageIdle
			draft = newDraft()
		case StageSetP3:
			draft.HasP2 = false
			draft.SnapP3 = false
			draft.P2AxisDir = Vec2{}
			draft.P3 = Vec2{}
			draft.PerpLockActive = false
			draft.PerpLockOrigin = Vec2{}
			draft.PerpLockAxisDir = Vec2{}
			stage = StageSetP2
		}
	}

	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		switch stage {
		case StageIdle:
			draft = newDraft()
			if hoveredNode.SplineIndex >= 0 {
				prev := splines[hoveredNode.SplineIndex]
				draft.P0 = endpointAnchor(prev, hoveredNode.Kind)
				draft.ContinuationFrom = prev.ID
				draft.LockP1 = true
				draft.P1AxisDir = endpointAxisDir(prev, hoveredNode.Kind)
			} else {
				draft.P0 = mouseWorld
				draft.PerpLockActive, draft.PerpLockOrigin, draft.PerpLockAxisDir = startPerpendicularLock(draft.P0, geometrySnap)
			}
			stage = StageSetP1

		case StageSetP1:
			if !draft.LockP1 && hoveredNode.SplineIndex >= 0 {
				prev := splines[hoveredNode.SplineIndex]
				draft.P0 = endpointAnchor(prev, hoveredNode.Kind)
				draft.P1 = mirroredHandleAtEndpoint(prev, hoveredNode.Kind)
				draft.HasP1 = true
				draft.LockP1 = false
				draft.P1AxisDir = Vec2{}
				draft.PerpLockActive = false
				draft.PerpLockOrigin = Vec2{}
				draft.PerpLockAxisDir = Vec2{}
			} else {
				draft.P1 = draftP1Preview(draft, mouseWorld, geometrySnap)
				draft.HasP1 = true
				draft.PerpLockActive, draft.PerpLockOrigin, draft.PerpLockAxisDir = startPerpendicularLock(draft.P1, geometrySnap)
			}
			stage = StageSetP2

		case StageSetP2:
			if hoveredNode.SplineIndex >= 0 {
				next := splines[hoveredNode.SplineIndex]
				draft.SnapP3 = true
				draft.P3 = endpointAnchor(next, hoveredNode.Kind)
				draft.P2AxisDir = endpointAxisDir(next, hoveredNode.Kind)
				draft.HasP2 = false
				draft.PerpLockActive = false
				draft.PerpLockOrigin = Vec2{}
				draft.PerpLockAxisDir = Vec2{}
			} else {
				draft.P2 = draftP2FreePreview(draft, mouseWorld, geometrySnap)
				draft.HasP2 = true
				draft.SnapP3 = false
				draft.P2AxisDir = Vec2{}
				draft.P3 = Vec2{}
				draft.PerpLockActive, draft.PerpLockOrigin, draft.PerpLockAxisDir = startPerpendicularLock(draft.P2, geometrySnap)
			}
			stage = StageSetP3

		case StageSetP3:
			p2 := draft.P2
			p3 := draftP3Preview(draft, mouseWorld, geometrySnap)
			if draft.SnapP3 {
				p3 = draft.P3
				p2 = draftP2Preview(draft, mouseWorld, geometrySnap)
			} else if hoveredNode.SplineIndex >= 0 {
				next := splines[hoveredNode.SplineIndex]
				p3 = endpointAnchor(next, hoveredNode.Kind)
				p2 = mirroredHandleAtEndpoint(next, hoveredNode.Kind)
			}
			spline := simpkg.NewSpline(nextSplineID, draft.P0, draft.P1, p2, p3)
			nextSplineID++
			splines = append(splines, spline)
			stage = StageIdle
			draft = newDraft()
			topologyChanged = true
		}
	}

	return stage, draft, splines, nextSplineID, topologyChanged
}

func handleRouteMode(routeStartSplineID int, routePanel RoutePanel, routes []Route, graph *RoadGraph, hoveredStart EndHit, hoveredEnd EndHit, vehicleKind VehicleKind) (int, RoutePanel, string) {
	if rl.IsKeyPressed(rl.KeyEscape) || rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		return -1, RoutePanel{}, ""
	}

	if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		return routeStartSplineID, routePanel, ""
	}

	if routeStartSplineID < 0 {
		if hoveredStart.SplineID >= 0 {
			if vehicleKind == VehicleCar {
				if spline, ok := graph.SplineByID(hoveredStart.SplineID); ok && spline.BusOnly {
					return routeStartSplineID, routePanel, "Bus-only splines cannot be used for car routes."
				}
			}
			return hoveredStart.SplineID, routePanel, ""
		}
		return routeStartSplineID, routePanel, ""
	}

	if hoveredEnd.SplineID < 0 {
		return routeStartSplineID, routePanel, ""
	}
	if vehicleKind == VehicleCar {
		if spline, ok := graph.SplineByID(hoveredEnd.SplineID); ok && spline.BusOnly {
			return routeStartSplineID, routePanel, "Bus-only splines cannot be used for car routes."
		}
	}

	panel := RoutePanel{
		Open:            true,
		StartSplineID:   routeStartSplineID,
		EndSplineID:     hoveredEnd.SplineID,
		VehicleKind:     vehicleKind,
		SpawnPerMinute:  defaultSpawnPerMinute(vehicleKind),
		ColorIndex:      simpkg.PickNextColorIndex(routes),
		ExistingRouteID: findRouteID(routes, routeStartSplineID, hoveredEnd.SplineID, vehicleKind),
	}
	if panel.ExistingRouteID >= 0 {
		if idx := findRouteIndexByID(routes, panel.ExistingRouteID); idx >= 0 {
			panel.SpawnPerMinute = routes[idx].SpawnPerMinute
			panel.ColorIndex = routes[idx].ColorIndex
			panel.BusStops = append([]BusStop(nil), routes[idx].BusStops...)
		}
	}
	panel = syncRoutePanelPath(panel, graph)
	if !panel.ValidPath() {
		return -1, RoutePanel{}, "No valid path between the selected start and destination."
	}

	return -1, panel, ""
}

func (p RoutePanel) ValidPath() bool {
	return len(p.PathIDs) > 0
}

func defaultSpawnPerMinute(vehicleKind VehicleKind) float32 {
	if vehicleKind == VehicleBus {
		return 4.0
	}
	return 15.0
}

func syncRoutePanelPath(panel RoutePanel, graph *RoadGraph) RoutePanel {
	preview := Route{
		StartSplineID: panel.StartSplineID,
		EndSplineID:   panel.EndSplineID,
		VehicleKind:   panel.VehicleKind,
		BusStops:      append([]BusStop(nil), panel.BusStops...),
	}
	pathIDs, pathLength, notice, _ := simpkg.ComputeRoutePathWithGraph(preview, graph)
	panel.PathIDs = pathIDs
	panel.PathLength = pathLength
	if notice != "" {
		panel.StopStatusNotice = notice
	} else if strings.HasPrefix(panel.StopStatusNotice, "Stop ") || strings.HasPrefix(panel.StopStatusNotice, "No valid path") {
		panel.StopStatusNotice = ""
	}
	return panel
}

func findBusStopCandidate(panel RoutePanel, splines []Spline, graph *RoadGraph, mouseWorld Vec2, zoom float32) (BusStop, bool) {
	if panel.VehicleKind != VehicleBus || graph == nil {
		return BusStop{}, false
	}
	threshold := pixelsToWorld(zoom, 16)
	thresholdSq := threshold * threshold
	startsByNode := buildStartsByNode(splines)
	endsByNode := buildEndsByNode(splines)

	fromSplineID := panel.StartSplineID
	if n := len(panel.BusStops); n > 0 {
		fromSplineID = panel.BusStops[n-1].SplineID
	}

	bestDist := thresholdSq
	var bestNodeKey simpkg.NodeKey
	bestPoint := Vec2{}
	found := false
	for nodeKey, starts := range startsByNode {
		if len(starts) == 0 || len(endsByNode[nodeKey]) == 0 {
			continue
		}
		point := splines[starts[0]].P0
		d := distSq(mouseWorld, point)
		if d <= bestDist {
			bestDist = d
			bestNodeKey = nodeKey
			bestPoint = point
			found = true
		}
	}
	if !found {
		return BusStop{}, false
	}

	stop, ok := simpkg.PickBestBusStopSpline(graph, bestNodeKey, fromSplineID, panel.EndSplineID)
	if !ok {
		return BusStop{}, false
	}
	stop.WorldPos = bestPoint
	return stop, true
}

func handleBusStopPlacement(panel RoutePanel, splines []Spline, graph *RoadGraph, mouseWorld Vec2, zoom float32) (RoutePanel, bool) {
	if !panel.AddingBusStop {
		return panel, false
	}
	if rl.IsKeyPressed(rl.KeyEscape) {
		panel.AddingBusStop = false
		panel.StopStatusNotice = ""
		return panel, false
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		panel.AddingBusStop = false
		panel.StopStatusNotice = ""
		return panel, false
	}
	if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		return panel, false
	}

	stop, ok := findBusStopCandidate(panel, splines, graph, mouseWorld, zoom)
	if !ok {
		panel.StopStatusNotice = "Pick an intersection where buses can arrive and depart."
		return panel, false
	}
	for _, existing := range panel.BusStops {
		if existing.SplineID == stop.SplineID {
			panel.StopStatusNotice = "That stop is already on the line."
			return panel, false
		}
	}
	panel.BusStops = append(panel.BusStops, stop)
	panel.AddingBusStop = false
	panel.StopStatusNotice = ""
	return panel, true
}

func buildEndsByNode(splines []Spline) map[simpkg.NodeKey][]int {
	endsByNode := make(map[simpkg.NodeKey][]int, len(splines))
	for i, spline := range splines {
		key := nodeKeyFromVec2(spline.P3)
		endsByNode[key] = append(endsByNode[key], i)
	}
	return endsByNode
}

func buildStartsByNode(splines []Spline) map[simpkg.NodeKey][]int {
	startsByNode := make(map[simpkg.NodeKey][]int, len(splines))
	for i, spline := range splines {
		key := nodeKeyFromVec2(spline.P0)
		startsByNode[key] = append(startsByNode[key], i)
	}
	return startsByNode
}

func findDirectionWarnings(splines []Spline) []DirectionWarning {
	type nodeCounts struct {
		point  Vec2
		starts int
		ends   int
	}
	nodes := map[simpkg.NodeKey]*nodeCounts{}
	for _, spline := range splines {
		startKey := nodeKeyFromVec2(spline.P0)
		if nodes[startKey] == nil {
			nodes[startKey] = &nodeCounts{point: spline.P0}
		}
		nodes[startKey].starts++

		endKey := nodeKeyFromVec2(spline.P3)
		if nodes[endKey] == nil {
			nodes[endKey] = &nodeCounts{point: spline.P3}
		}
		nodes[endKey].ends++
	}

	warnings := make([]DirectionWarning, 0)
	for _, node := range nodes {
		if node.starts >= 2 && node.ends == 0 {
			warnings = append(warnings, DirectionWarning{Point: node.point})
		} else if node.ends >= 2 && node.starts == 0 {
			warnings = append(warnings, DirectionWarning{Point: node.point})
		}
	}
	return warnings
}

func drawBusStopPlacementPreview(panel RoutePanel, splines []Spline, graph *RoadGraph, mouseWorld Vec2, zoom float32) {
	stop, ok := findBusStopCandidate(panel, splines, graph, mouseWorld, zoom)
	if !ok {
		return
	}
	r := pixelsToWorld(zoom, 9)
	drawCircleV(stop.WorldPos, r, NewColor(255, 210, 70, 220))
	drawRing(stop.WorldPos, r*0.65, r*1.25, 0, 360, 24, NewColor(210, 150, 0, 220))
}

func toggleBusOnlySpline(splines []Spline, hoveredSpline int) ([]Spline, string) {
	if hoveredSpline < 0 {
		return splines, ""
	}
	splines[hoveredSpline].BusOnly = !splines[hoveredSpline].BusOnly
	if splines[hoveredSpline].BusOnly {
		return splines, fmt.Sprintf("Spline #%d marked bus-only", splines[hoveredSpline].ID)
	}
	return splines, fmt.Sprintf("Spline #%d open to all vehicles", splines[hoveredSpline].ID)
}

func handlePriorityMode(splines []Spline, hoveredSpline int) []Spline {
	if hoveredSpline < 0 {
		return splines
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		splines[hoveredSpline].Priority = true
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		splines[hoveredSpline].Priority = false
	}
	return splines
}

func handlePreferenceMode(splines []Spline, hoveredSpline, lastPref int) ([]Spline, int) {
	if hoveredSpline < 0 {
		return splines, lastPref
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		splines[hoveredSpline].LanePreference = 1
		lastPref = 1
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		if splines[hoveredSpline].LanePreference > 0 {
			splines[hoveredSpline].LanePreference = 0
		} else {
			lastPref++
			splines[hoveredSpline].LanePreference = lastPref
		}
	}
	return splines, lastPref
}

// ---------- traffic light mode handlers ----------

func handleTrafficLightMode(splines []Spline, pending []TrafficLight, mouseWorld Vec2, zoom float32, nextLightID *int) []TrafficLight {
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		si, t, point, found := findNearestSplinePoint(splines, mouseWorld)
		if found {
			idx := int(t * float32(simSamples))
			if idx > simSamples {
				idx = simSamples
			}
			dist := splines[si].CumLen[idx]
			pending = append(pending, TrafficLight{
				ID:           *nextLightID,
				SplineID:     splines[si].ID,
				DistOnSpline: dist,
				WorldPos:     point,
				CycleID:      -1,
			})
			*nextLightID++
		}
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		threshold := 15 / zoom
		bestDSq := threshold * threshold
		bestIdx := -1
		for i, l := range pending {
			d := distSq(l.WorldPos, mouseWorld)
			if d < bestDSq {
				bestDSq = d
				bestIdx = i
			}
		}
		if bestIdx >= 0 {
			pending = append(pending[:bestIdx], pending[bestIdx+1:]...)
		}
	}
	return pending
}

func doCreateTrafficCycle(pending []TrafficLight, lights []TrafficLight, cycles []TrafficCycle, nextCycleID *int) ([]TrafficLight, []TrafficLight, []TrafficCycle, int) {
	cycleID := *nextCycleID
	*nextCycleID++
	ids := make([]int, len(pending))
	for i, l := range pending {
		l.CycleID = cycleID
		ids[i] = l.ID
		lights = append(lights, l)
	}
	cycles = append(cycles, TrafficCycle{
		ID:         cycleID,
		LightIDs:   ids,
		Phases:     nil,
		Timer:      0,
		PhaseIndex: 0,
		Enabled:    false,
	})
	return pending[:0], lights, cycles, cycleID
}

// effectivePhaseDur returns the duration of effective phase index ei in a cycle.
// When n > 1, even ei = user phase, odd ei = clearance phase (phase[ei/2].ClearanceDurationSecs).
// When n == 1, there is only one effective phase (the single user phase).
func trafficToggleCycleEnabled(cycles []TrafficCycle, cycleID int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		cycles[i].Enabled = !cycles[i].Enabled
		if cycles[i].Enabled {
			cycles[i].Timer = 0
			cycles[i].PhaseIndex = 0
		}
		break
	}
	return cycles
}

// trafficSkipToPhase jumps the cycle immediately to user phase userIdx (effective index 2*userIdx).
func trafficSkipToPhase(cycles []TrafficCycle, cycleID, userIdx int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		n := len(cycles[i].Phases)
		if userIdx < 0 || userIdx >= n {
			break
		}
		cycles[i].PhaseIndex = 2 * userIdx
		cycles[i].Timer = 0
		break
	}
	return cycles
}

// ---------- traffic light panel ----------

const trafficPanelW = 300

// trafficCyclePanelRect computes the panel bounding rect.
// editing=true means the cycle editor (with phase list); false = new-cycle creator.
func trafficCyclePanelRect(editing bool, phaseCount, pendingCount int) rl.Rectangle {
	x := float32(rl.GetScreenWidth()) - trafficPanelW - 12
	y := float32(toolbarY+toolbarBtnH) + 30
	var h float32
	if !editing {
		rows := pendingCount
		if rows == 0 {
			rows = 1
		}
		h = 44 + float32(rows)*22 + 48
	} else {
		rows := phaseCount
		if rows == 0 {
			rows = 1
		}
		h = 44 + 36 + float32(rows)*52 + 26
	}
	return rl.NewRectangle(x, y, trafficPanelW, h)
}

// trafficCreateBtnRect — "Create Traffic Cycle" button in new-cycle mode.
func trafficCreateBtnRect(pendingCount int) rl.Rectangle {
	pr := trafficCyclePanelRect(false, 0, pendingCount)
	return rl.NewRectangle(pr.X+12, pr.Y+pr.Height-42, pr.Width-24, 32)
}

// trafficHeaderBtnRects returns [On/Off], [Edit Lights] and [Close] in the cycle-editor header.
func trafficHeaderBtnRects(pr rl.Rectangle) (onOffBtn, editLightsBtn, closeBtn rl.Rectangle) {
	closeBtn = rl.NewRectangle(pr.X+pr.Width-12-48, pr.Y+8, 48, 26)
	editLightsBtn = rl.NewRectangle(pr.X+pr.Width-12-48-6-72, pr.Y+8, 72, 26)
	onOffBtn = rl.NewRectangle(pr.X+pr.Width-12-48-6-72-6-46, pr.Y+8, 46, 26)
	return
}

// trafficAddPhaseBtnRect — "+ Add Phase" button.
func trafficAddPhaseBtnRect(pr rl.Rectangle) rl.Rectangle {
	return rl.NewRectangle(pr.X+12, pr.Y+42, pr.Width-24, 28)
}

// phaseRowBtns holds all button/field rects for one phase row.
// Each row has two sub-rows: (0) label + clearance field + duration field, (1) action buttons.
type phaseRowBtns struct {
	clearField               rl.Rectangle
	durField                 rl.Rectangle
	upBtn, downBtn           rl.Rectangle
	showBtn, editBtn, delBtn rl.Rectangle
}

// getPhaseRowBtns returns the rects for phase row idx inside panel pr.
// Each row is 52px tall: label+fields on top 24px, buttons on bottom 22px.
func getPhaseRowBtns(pr rl.Rectangle, idx int) phaseRowBtns {
	rowY := pr.Y + 78 + float32(idx)*52
	x := pr.X + 12
	return phaseRowBtns{
		// sub-row 1: clearance field then duration field, both right-aligned
		clearField: rl.NewRectangle(pr.X+pr.Width-127, rowY+2, 45, 20),
		durField:   rl.NewRectangle(pr.X+pr.Width-66, rowY+2, 48, 20),
		// sub-row 2: buttons left-to-right
		upBtn:   rl.NewRectangle(x, rowY+26, 32, 20),
		downBtn: rl.NewRectangle(x+36, rowY+26, 38, 20),
		showBtn: rl.NewRectangle(x+84, rowY+26, 36, 20),
		editBtn: rl.NewRectangle(x+124, rowY+26, 42, 20),
		delBtn:  rl.NewRectangle(x+170, rowY+26, 44, 20),
	}
}

func drawSmallBtn(r rl.Rectangle, label string, bg, fg Color) {
	rl.DrawRectangleRec(r, bg)
	rl.DrawRectangleLinesEx(r, 1, NewColor(0, 0, 0, 40))
	lw := measureText(label, 12)
	drawText(label, int32(r.X)+int32(r.Width)/2-lw/2, int32(r.Y)+int32(r.Height)/2-7, 12, fg)
}

func drawTrafficCyclePanel(pending []TrafficLight, lights []TrafficLight, cycles []TrafficCycle,
	editingCycleID int, editingLights bool, editingPhaseIdx, activeDurInput int, durInputStr string, showPhaseIdx int) {
	bg := NewColor(248, 248, 250, 245)
	outline := NewColor(210, 210, 215, 255)
	dark := NewColor(28, 28, 33, 255)
	muted := NewColor(100, 100, 110, 255)
	sep := NewColor(220, 220, 224, 255)
	disabledFg := NewColor(170, 170, 178, 255)
	disabledBg := NewColor(220, 220, 224, 255)

	if editingCycleID < 0 {
		// ── New-cycle creator ────────────────────────────────────────────
		pr := trafficCyclePanelRect(false, 0, len(pending))
		rl.DrawRectangleRec(pr, bg)
		rl.DrawRectangleLinesEx(pr, 1, outline)
		drawText("New Traffic Cycle", int32(pr.X)+12, int32(pr.Y)+12, 15, dark)
		drawLineEx(NewVec2(pr.X+1, pr.Y+36), NewVec2(pr.X+pr.Width-1, pr.Y+36), 1, sep)

		y := int32(pr.Y) + 44
		if len(pending) == 0 {
			drawText("No lights selected yet", int32(pr.X)+12, y, 13, muted)
		} else {
			for _, l := range pending {
				drawText(fmt.Sprintf("• Spline #%d  at %.1f m", l.SplineID, l.DistOnSpline), int32(pr.X)+12, y, 13, dark)
				y += 22
			}
		}
		btn := trafficCreateBtnRect(len(pending))
		btnBg := NewColor(47, 120, 60, 255)
		if len(pending) == 0 {
			btnBg = NewColor(160, 160, 165, 255)
		}
		rl.DrawRectangleRec(btn, btnBg)
		rl.DrawRectangleLinesEx(btn, 1, NewColor(0, 0, 0, 40))
		lbl := "Create Traffic Cycle"
		lw := measureText(lbl, 13)
		drawText(lbl, int32(btn.X)+int32(btn.Width)/2-lw/2, int32(btn.Y)+9, 13, rl.White)
		if len(cycles) > 0 {
			drawText(fmt.Sprintf("%d cycle(s) active", len(cycles)), int32(pr.X)+12, int32(pr.Y+pr.Height)+8, 12, muted)
		}
		return
	}

	// ── Cycle editor ─────────────────────────────────────────────────────
	var cycle TrafficCycle
	var found bool
	for _, c := range cycles {
		if c.ID == editingCycleID {
			cycle = c
			found = true
			break
		}
	}
	if !found {
		return
	}

	cycleOn := cycle.Enabled

	pr := trafficCyclePanelRect(true, len(cycle.Phases), 0)
	rl.DrawRectangleRec(pr, bg)
	rl.DrawRectangleLinesEx(pr, 1, outline)

	// Header row
	drawText(fmt.Sprintf("Cycle #%d", editingCycleID), int32(pr.X)+12, int32(pr.Y)+12, 15, dark)
	onOffBtn, elBtn, clBtn := trafficHeaderBtnRects(pr)

	// On/Off toggle — label shows current state
	onOffBg := NewColor(190, 50, 50, 255)
	onOffLbl := "Off"
	if cycleOn {
		onOffBg = NewColor(47, 140, 60, 255)
		onOffLbl = "On"
	}
	drawSmallBtn(onOffBtn, onOffLbl, onOffBg, rl.White)

	// Edit Lights button (disabled when cycle is on)
	elBg := NewColor(47, 96, 198, 255)
	elFg := Color(rl.White)
	if cycleOn {
		elBg = disabledBg
		elFg = disabledFg
	} else if editingLights {
		elBg = NewColor(28, 62, 155, 255)
	}
	elLbl := "Edit Lights"
	if editingLights && !cycleOn {
		elLbl = "Done"
	}
	drawSmallBtn(elBtn, elLbl, elBg, elFg)
	drawSmallBtn(clBtn, "Close", NewColor(200, 60, 60, 255), rl.White)

	drawLineEx(NewVec2(pr.X+1, pr.Y+38), NewVec2(pr.X+pr.Width-1, pr.Y+38), 1, sep)

	// "+ Add Phase" button (disabled when cycle is on)
	addBtn := trafficAddPhaseBtnRect(pr)
	if cycleOn {
		drawSmallBtn(addBtn, "+ Add Phase", disabledBg, disabledFg)
	} else {
		drawSmallBtn(addBtn, "+ Add Phase", NewColor(70, 140, 80, 255), rl.White)
	}

	drawLineEx(NewVec2(pr.X+1, pr.Y+74), NewVec2(pr.X+pr.Width-1, pr.Y+74), 1, sep)

	// Phase rows
	if len(cycle.Phases) == 0 {
		drawText("No phases yet", int32(pr.X)+12, int32(pr.Y)+80, 13, muted)
	} else {
		for pi, phase := range cycle.Phases {
			row := getPhaseRowBtns(pr, pi)
			rowY := int32(pr.Y) + 78 + int32(pi)*52

			// Determine whether this row is the current or transitioning-out phase.
			// cycle.PhaseIndex is an effective index in the 2*N sequence:
			//   even = user phase (ei/2), odd = transition (prev=ei/2, next=(ei/2+1)%n)
			ei := cycle.PhaseIndex
			inTransition := cycleOn && len(cycle.Phases) > 1 && ei%2 == 1
			currentUserPhase := ei / 2 // integer division works for both even and odd
			isCurrentPhase := cycleOn && pi == currentUserPhase && !inTransition
			isLeavingPhase := cycleOn && inTransition && pi == currentUserPhase
			isEnteringPhase := cycleOn && inTransition && pi == (currentUserPhase+1)%len(cycle.Phases)

			if isCurrentPhase {
				rl.DrawRectangle(int32(pr.X)+2, rowY, int32(pr.Width)-4, 50,
					NewColor(255, 240, 180, 160))
			} else if isLeavingPhase {
				rl.DrawRectangle(int32(pr.X)+2, rowY, int32(pr.Width)-4, 50,
					NewColor(255, 210, 100, 100))
			} else if isEnteringPhase {
				rl.DrawRectangle(int32(pr.X)+2, rowY, int32(pr.Width)-4, 50,
					NewColor(200, 255, 200, 100))
			}

			// Phase label (sub-row 1)
			labelColor := dark
			phaseLabel := fmt.Sprintf("Phase %d", pi+1)
			if isCurrentPhase {
				labelColor = NewColor(140, 90, 0, 255)
				phaseLabel = fmt.Sprintf("Phase %d  ▶", pi+1)
			} else if isLeavingPhase {
				labelColor = NewColor(160, 100, 0, 255)
				phaseLabel = fmt.Sprintf("Phase %d  →", pi+1)
			} else if isEnteringPhase {
				labelColor = NewColor(30, 120, 30, 255)
				phaseLabel = fmt.Sprintf("Phase %d  ←", pi+1)
			}
			drawText(phaseLabel, int32(pr.X)+12, rowY+3, 12, labelColor)

			yellowColor := NewColor(180, 120, 0, 255)

			// Clearance field (sub-row 1, to the left of the duration field)
			clrActiveInput := -(pi + 3)
			clrBg := NewColor(255, 252, 240, 255) // subtle amber tint
			if cycleOn {
				clrBg = NewColor(235, 235, 238, 255)
			} else if activeDurInput == clrActiveInput {
				clrBg = NewColor(255, 248, 220, 255)
			}
			drawText("→", int32(row.clearField.X)-16, rowY+3, 12, yellowColor)
			rl.DrawRectangleRec(row.clearField, clrBg)
			rl.DrawRectangleLinesEx(row.clearField, 1, NewColor(200, 170, 100, 255))
			clrDur := phase.ClearanceDurationSecs
			if clrDur <= 0 {
				clrDur = 3.0
			}
			clrStr := fmt.Sprintf("%.1f", clrDur)
			if !cycleOn && activeDurInput == clrActiveInput {
				clrStr = durInputStr
			}
			clrColor := yellowColor
			if cycleOn {
				clrColor = disabledFg
			}
			drawText(clrStr, int32(row.clearField.X)+4, int32(row.clearField.Y)+3, 12, clrColor)
			if !cycleOn && activeDurInput == clrActiveInput {
				cw := measureText(clrStr, 12)
				rl.DrawRectangle(int32(row.clearField.X)+4+cw+1, int32(row.clearField.Y)+3, 1, 14, dark)
			}
			drawText("s", int32(row.clearField.X)+int32(row.clearField.Width)+3, int32(row.clearField.Y)+3, 12, muted)

			// Duration input field (sub-row 1, right side) — disabled when cycle on
			fieldBg := NewColor(255, 255, 255, 255)
			if cycleOn {
				fieldBg = NewColor(235, 235, 238, 255)
			} else if activeDurInput == pi {
				fieldBg = NewColor(240, 248, 255, 255)
			}
			rl.DrawRectangleRec(row.durField, fieldBg)
			rl.DrawRectangleLinesEx(row.durField, 1, NewColor(180, 180, 190, 255))
			durStr := fmt.Sprintf("%.1f", phase.DurationSecs)
			if !cycleOn && activeDurInput == pi {
				durStr = durInputStr
			}
			durColor := dark
			if cycleOn {
				durColor = disabledFg
			}
			drawText(durStr, int32(row.durField.X)+4, int32(row.durField.Y)+3, 12, durColor)
			if !cycleOn && activeDurInput == pi {
				cw := measureText(durStr, 12)
				rl.DrawRectangle(int32(row.durField.X)+4+cw+1, int32(row.durField.Y)+3, 1, 14, dark)
			}
			drawText("s", int32(row.durField.X)+int32(row.durField.Width)+3, int32(row.durField.Y)+3, 12, muted)

			// Sub-row 2: action buttons
			if cycleOn {
				// Disabled Up/Down/Edit/Delete
				drawSmallBtn(row.upBtn, "Up", disabledBg, disabledFg)
				drawSmallBtn(row.downBtn, "Down", disabledBg, disabledFg)
				drawSmallBtn(row.editBtn, "Edit", disabledBg, disabledFg)
				drawSmallBtn(row.delBtn, "Delete", disabledBg, disabledFg)
			} else {
				upBg := NewColor(200, 200, 205, 255)
				downBg := NewColor(200, 200, 205, 255)
				if pi == 0 {
					upBg = disabledBg
				}
				if pi == len(cycle.Phases)-1 {
					downBg = disabledBg
				}
				drawSmallBtn(row.upBtn, "Up", upBg, dark)
				drawSmallBtn(row.downBtn, "Down", downBg, dark)

				editActive := editingPhaseIdx == pi
				editBg := NewColor(47, 96, 198, 255)
				editLbl := "Edit"
				if editActive {
					editBg = NewColor(28, 62, 155, 255)
					editLbl = "Done"
				}
				drawSmallBtn(row.editBtn, editLbl, editBg, rl.White)
				drawSmallBtn(row.delBtn, "Delete", NewColor(200, 60, 60, 255), rl.White)
			}

			// Show (cycle off) / Skip (cycle on) button
			if cycleOn {
				drawSmallBtn(row.showBtn, "Skip", NewColor(60, 120, 190, 255), rl.White)
			} else {
				showActive := showPhaseIdx == pi
				showBg := NewColor(130, 80, 180, 255)
				if showActive {
					showBg = NewColor(90, 40, 140, 255)
				}
				drawSmallBtn(row.showBtn, "Show", showBg, rl.White)
			}
		}
	}

}

// ---------- traffic light world drawing ----------

func containsRune(s string, r rune) bool {
	for _, c := range s {
		if c == r {
			return true
		}
	}
	return false
}

func commitDurInput(cycles []TrafficCycle, cycleID, phaseIdx int, str string) []TrafficCycle {
	val, err := strconv.ParseFloat(str, 32)
	if err != nil || val <= 0 {
		return cycles
	}
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		if phaseIdx >= 0 && phaseIdx < len(cycles[i].Phases) {
			cycles[i].Phases[phaseIdx].DurationSecs = float32(val)
		} else if phaseIdx <= -3 {
			clrIdx := -(phaseIdx + 3)
			if clrIdx >= 0 && clrIdx < len(cycles[i].Phases) {
				cycles[i].Phases[clrIdx].ClearanceDurationSecs = float32(val)
			}
		}
		break
	}
	return cycles
}

func trafficAddPhase(cycles []TrafficCycle, cycleID int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		cycles[i].Phases = append(cycles[i].Phases, TrafficPhase{DurationSecs: 5, ClearanceDurationSecs: 3.0, GreenLightIDs: nil})
		break
	}
	return cycles
}

func trafficMovePhase(cycles []TrafficCycle, cycleID, idx, dir int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		phases := cycles[i].Phases
		target := idx + dir
		if target < 0 || target >= len(phases) {
			break
		}
		phases[idx], phases[target] = phases[target], phases[idx]
		break
	}
	return cycles
}

func trafficDeletePhase(cycles []TrafficCycle, cycleID, idx int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		phases := cycles[i].Phases
		if idx < 0 || idx >= len(phases) {
			break
		}
		cycles[i].Phases = append(phases[:idx], phases[idx+1:]...)
		// Reset to start; editing only happens when cycle is off (PhaseIndex=0).
		cycles[i].PhaseIndex = 0
		cycles[i].Timer = 0
		break
	}
	return cycles
}

func trafficToggleLightInPhase(cycles []TrafficCycle, cycleID, phaseIdx, lightID int) []TrafficCycle {
	for i := range cycles {
		if cycles[i].ID != cycleID {
			continue
		}
		if phaseIdx < 0 || phaseIdx >= len(cycles[i].Phases) {
			break
		}
		ids := cycles[i].Phases[phaseIdx].GreenLightIDs
		for j, id := range ids {
			if id == lightID {
				cycles[i].Phases[phaseIdx].GreenLightIDs = append(ids[:j], ids[j+1:]...)
				return cycles
			}
		}
		cycles[i].Phases[phaseIdx].GreenLightIDs = append(ids, lightID)
		break
	}
	return cycles
}

func drawTrafficLights(lights []TrafficLight, pending []TrafficLight, cycles []TrafficCycle, editingCycleID int, editingPhaseIdx int, showPhaseIdx int, zoom float32, viewRect worldRect) {
	r := pixelsToWorld(zoom, 8)
	all := append(lights, pending...)
	for _, l := range all {
		if !circleVisibleInWorldRect(l.WorldPos, r*1.5, viewRect) {
			continue
		}
		var fill Color
		if l.CycleID < 0 {
			fill = NewColor(255, 165, 0, 220) // amber = pending
		} else {
			// Determine which phase index to display, if any override is active
			displayPhase := -1
			if editingCycleID >= 0 && l.CycleID == editingCycleID {
				if showPhaseIdx >= 0 {
					displayPhase = showPhaseIdx
				} else if editingPhaseIdx >= 0 {
					displayPhase = editingPhaseIdx
				}
			}
			if displayPhase >= 0 {
				isGreen := false
				for _, c := range cycles {
					if c.ID != l.CycleID {
						continue
					}
					if displayPhase < len(c.Phases) {
						for _, id := range c.Phases[displayPhase].GreenLightIDs {
							if id == l.ID {
								isGreen = true
								break
							}
						}
					}
					break
				}
				if isGreen {
					fill = NewColor(40, 180, 60, 240)
				} else {
					fill = NewColor(210, 35, 35, 240)
				}
			} else {
				switch simpkg.TrafficLightState(l.ID, l.CycleID, cycles) {
				case TrafficRed:
					fill = NewColor(210, 35, 35, 240)
				case TrafficYellow:
					fill = NewColor(230, 175, 0, 240)
				case TrafficGreen:
					fill = NewColor(40, 180, 60, 240)
				}
			}
		}
		drawCircleV(l.WorldPos, r, NewColor(20, 20, 20, 220))
		drawCircleV(l.WorldPos, r*0.7, fill)
		// White ring around lights belonging to the currently-open cycle
		if editingCycleID >= 0 && l.CycleID == editingCycleID {
			drawRing(l.WorldPos, r*1.1, r*1.45, 0, 360, 24, NewColor(255, 255, 255, 200))
		}
	}
}

// trafficLightAt returns the ID of the nearest placed light within click range,
// or -1 if none.
func trafficLightAt(lights []TrafficLight, pos Vec2, zoom float32) int {
	thresh := 15 / zoom
	threshSq := thresh * thresh
	for _, l := range lights {
		if distSq(l.WorldPos, pos) < threshSq {
			return l.ID
		}
	}
	return -1
}

// trafficCycleLightCount returns how many placed lights belong to cycleID.
func trafficCycleLightCount(cycleID int, lights []TrafficLight) int {
	n := 0
	for _, l := range lights {
		if l.CycleID == cycleID {
			n++
		}
	}
	return n
}

// handleTrafficLightEdit adds or removes placed lights from an existing cycle.
func handleTrafficLightEdit(splines []Spline, lights []TrafficLight, cycles []TrafficCycle, cycleID int, mouseWorld Vec2, zoom float32, nextLightID *int) ([]TrafficLight, []TrafficCycle) {
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		si, t, point, found := findNearestSplinePoint(splines, mouseWorld)
		if found {
			idx := int(t * float32(simSamples))
			if idx > simSamples {
				idx = simSamples
			}
			dist := splines[si].CumLen[idx]
			newLight := TrafficLight{
				ID:           *nextLightID,
				SplineID:     splines[si].ID,
				DistOnSpline: dist,
				WorldPos:     point,
				CycleID:      cycleID,
			}
			*nextLightID++
			lights = append(lights, newLight)
			for ci := range cycles {
				if cycles[ci].ID == cycleID {
					cycles[ci].LightIDs = append(cycles[ci].LightIDs, newLight.ID)
					break
				}
			}
		}
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		thresh := 15 / zoom
		threshSq := thresh * thresh
		bestDSq := threshSq
		bestIdx := -1
		removedID := -1
		for i, l := range lights {
			if l.CycleID != cycleID {
				continue
			}
			if d := distSq(l.WorldPos, mouseWorld); d < bestDSq {
				bestDSq = d
				bestIdx = i
				removedID = l.ID
			}
		}
		if bestIdx >= 0 {
			lights = append(lights[:bestIdx], lights[bestIdx+1:]...)
			for ci := range cycles {
				if cycles[ci].ID == cycleID {
					ids := cycles[ci].LightIDs
					for i, id := range ids {
						if id == removedID {
							cycles[ci].LightIDs = append(ids[:i], ids[i+1:]...)
							break
						}
					}
					break
				}
			}
		}
	}
	return lights, cycles
}

func handleSpeedLimitMode(splines []Spline, hoveredSpline, selectedSpeedKmh int) []Spline {
	if hoveredSpline < 0 {
		return splines
	}
	mousePos := rl.GetMousePosition()
	if isMouseOverSpeedLimitPanel(fromRLVec2(mousePos)) {
		return splines
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		splines[hoveredSpline].SpeedLimitKmh = float32(selectedSpeedKmh)
		simpkg.RebuildSpline(&splines[hoveredSpline])
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		splines[hoveredSpline].SpeedLimitKmh = 0
		simpkg.RebuildSpline(&splines[hoveredSpline])
	}
	return splines
}

// updateSpeedLimitPanel handles clicks on the speed picker panel and returns
// the newly selected speed in km/h.
func updateSpeedLimitPanel(selectedSpeedKmh int) int {
	if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		return selectedSpeedKmh
	}
	mouse := rl.GetMousePosition()
	px, _ := speedLimitPanelOrigin()
	for i, kmh := range speedLimitSteps() {
		col := i / 7
		row := i % 7
		bx := px + float32(col)*84
		by := float32(180 + row*32)
		if mouse.X >= bx && mouse.X <= bx+76 && mouse.Y >= by && mouse.Y <= by+24 {
			return kmh
		}
	}
	return selectedSpeedKmh
}

func isMouseOverSpeedLimitPanel(mouse Vec2) bool {
	px, _ := speedLimitPanelOrigin()
	panelW := float32(172)
	panelH := float32(7*32 + 32)
	return mouse.X >= px-8 && mouse.X <= px-8+panelW && mouse.Y >= float32(172) && mouse.Y <= float32(172)+panelH
}

func speedLimitPanelOrigin() (float32, float32) {
	return float32(rl.GetScreenWidth()) - 188, 180
}

func speedLimitSteps() []int {
	steps := make([]int, 0, 14)
	for kmh := 10; kmh <= 140; kmh += 10 {
		steps = append(steps, kmh)
	}
	return steps
}

// drawSpeedLimitWorld highlights the hovered spline in speed-limit mode.
func drawSpeedLimitWorld(splines []Spline, hoveredSpline int, zoom float32) {
	if hoveredSpline < 0 {
		return
	}
	drawSpline(splines[hoveredSpline], pixelsToWorld(zoom, 4), NewColor(255, 200, 50, 180))
}

// drawSpeedLimitLabels draws speed limit badges on any spline that has a limit,
// in screen space, so they stay readable at any zoom level.
func drawSpeedLimitLabels(splines []Spline, camera rl.Camera2D, viewRect worldRect) {
	for _, spline := range splines {
		if spline.SpeedLimitKmh <= 0 {
			continue
		}
		mid := spline.Samples[simSamples/2]
		if !worldRectContainsPoint(viewRect, mid) {
			continue
		}
		screen := getWorldToScreen2D(mid, camera)
		label := fmt.Sprintf("%d", int(spline.SpeedLimitKmh))
		fontSize := int32(14)
		textW := measureText(label, fontSize)
		r := int32(14)
		cx, cy := int32(screen.X), int32(screen.Y)
		rl.DrawCircle(cx, cy, float32(r), rl.White)
		rl.DrawCircleLines(cx, cy, float32(r), NewColor(200, 30, 30, 255))
		drawText(label, cx-textW/2, cy-fontSize/2, fontSize, NewColor(200, 30, 30, 255))
	}
}

// drawPreferenceLabels draws preference badges on splines that have a preference assigned,
// in screen space, so they stay readable at any zoom level.
func drawPreferenceLabels(splines []Spline, camera rl.Camera2D, viewRect worldRect) {
	color := NewColor(30, 150, 60, 255)
	for _, spline := range splines {
		if spline.LanePreference <= 0 {
			continue
		}
		mid := spline.Samples[simSamples/2]
		if !worldRectContainsPoint(viewRect, mid) {
			continue
		}
		screen := getWorldToScreen2D(mid, camera)
		label := fmt.Sprintf("%d", spline.LanePreference)
		fontSize := int32(14)
		textW := measureText(label, fontSize)
		r := int32(14)
		cx, cy := int32(screen.X), int32(screen.Y)
		rl.DrawCircle(cx, cy, float32(r), rl.White)
		rl.DrawCircleLines(cx, cy, float32(r), color)
		drawText(label, cx-textW/2, cy-fontSize/2, fontSize, color)
	}
}

// maxLoadedPreference returns the highest LanePreference value across all splines,
// used to resume the preference counter after loading a file.
func maxLoadedPreference(splines []Spline) int {
	max := 0
	for _, s := range splines {
		if s.LanePreference > max {
			max = s.LanePreference
		}
	}
	return max
}

// drawSpeedLimitPanel draws the speed picker UI in screen space.
func drawSpeedLimitPanel(selectedSpeedKmh int) {
	bg := NewColor(248, 248, 250, 245)
	outline := NewColor(210, 210, 215, 255)
	text := NewColor(30, 30, 35, 255)
	selBg := NewColor(200, 30, 30, 220)
	selText := rl.White

	px, _ := speedLimitPanelOrigin()
	panelX := int32(px - 8)
	panelW := int32(172)
	panelH := int32(7*32 + 32)
	rl.DrawRectangle(panelX, 172, panelW, panelH, bg)
	rl.DrawRectangleLines(panelX, 172, panelW, panelH, outline)
	drawText("Speed limit", panelX+8, 178, 16, text)

	for i, kmh := range speedLimitSteps() {
		col := i / 7
		row := i % 7
		bx := int32(px) + int32(col)*84
		by := int32(180 + row*32)
		label := fmt.Sprintf("%d km/h", kmh)
		if kmh == selectedSpeedKmh {
			rl.DrawRectangle(bx, by, 76, 24, selBg)
			rl.DrawRectangleLines(bx, by, 76, 24, NewColor(150, 20, 20, 255))
			drawText(label, bx+6, by+5, 14, selText)
		} else {
			rl.DrawRectangle(bx, by, 76, 24, NewColor(235, 236, 240, 255))
			rl.DrawRectangleLines(bx, by, 76, 24, outline)
			drawText(label, bx+6, by+5, 14, text)
		}
	}
}

func handleCoupleMode(firstID int, splines []Spline, hoveredSpline int) (int, []Spline, string) {
	if rl.IsKeyPressed(rl.KeyEscape) {
		return -1, splines, ""
	}

	leftClicked := rl.IsMouseButtonPressed(rl.MouseButtonLeft) && hoveredSpline >= 0
	rightClicked := rl.IsMouseButtonPressed(rl.MouseButtonRight) && hoveredSpline >= 0

	if !leftClicked && !rightClicked {
		return firstID, splines, ""
	}

	clickedID := splines[hoveredSpline].ID

	if firstID < 0 {
		if leftClicked {
			return clickedID, splines, ""
		}
		return firstID, splines, ""
	}
	if clickedID == firstID {
		return -1, splines, ""
	}

	if leftClicked {
		idxA := findSplineIndexByID(splines, firstID)
		idxB := findSplineIndexByID(splines, clickedID)
		if idxA >= 0 && idxB >= 0 {
			splineA := splines[idxA]
			splineB := splines[idxB]
			speedA := simpkg.EffectiveMaxSpeedMPS(splineA)
			speedB := simpkg.EffectiveMaxSpeedMPS(splineB)
			if !laneChangeFeasible(splineA, splineB, speedA) || !laneChangeFeasible(splineB, splineA, speedB) {
				return -1, splines, "Hard couple rejected: lane change not feasible at max speed from the start of both splines"
			}
		}
		splines = toggleHardCoupling(splines, firstID, clickedID)
	} else {
		splines = toggleSoftCoupling(splines, firstID, clickedID)
	}
	return -1, splines, ""
}

func laneChangeFeasibleAt(src, dst Spline, distance, speed float32) bool {
	carPos, carHeading := simpkg.SampleSplineAtDistance(src, distance)
	halfDist := speed * laneChangeHalfSecs
	p1 := vecAdd(carPos, vecScale(carHeading, halfDist))
	_, crossDist := nearestSampleWithDist(dst, p1)
	if crossDist == 0 || dst.Length-crossDist < halfDist {
		return false
	}
	_, destHeading := simpkg.SampleSplineAtDistance(dst, crossDist+halfDist)
	return carHeading.X*destHeading.X+carHeading.Y*destHeading.Y >= laneChangeDirCos
}

func laneChangeFeasible(src, dst Spline, speed float32) bool {
	return laneChangeFeasibleAt(src, dst, 0, speed)
}

func toggleHardCoupling(splines []Spline, idA, idB int) []Spline {
	return toggleCouplingList(splines, idA, idB, true)
}

func toggleSoftCoupling(splines []Spline, idA, idB int) []Spline {
	return toggleCouplingList(splines, idA, idB, false)
}

// toggleCouplingList adds or removes a bidirectional hard or soft coupling.
// Toggling one type also removes the other, so a pair can only have one coupling type at a time.
func toggleCouplingList(splines []Spline, idA, idB int, hard bool) []Spline {
	idxA := findSplineIndexByID(splines, idA)
	idxB := findSplineIndexByID(splines, idB)
	if idxA < 0 || idxB < 0 {
		return splines
	}

	var primaryA, otherA, primaryB, otherB *[]int
	if hard {
		primaryA, otherA = &splines[idxA].HardCoupledIDs, &splines[idxA].SoftCoupledIDs
		primaryB, otherB = &splines[idxB].HardCoupledIDs, &splines[idxB].SoftCoupledIDs
	} else {
		primaryA, otherA = &splines[idxA].SoftCoupledIDs, &splines[idxA].HardCoupledIDs
		primaryB, otherB = &splines[idxB].SoftCoupledIDs, &splines[idxB].HardCoupledIDs
	}

	alreadyCoupled := false
	for _, id := range *primaryA {
		if id == idB {
			alreadyCoupled = true
			break
		}
	}

	if alreadyCoupled {
		*primaryA = removeInt(*primaryA, idB)
		*primaryB = removeInt(*primaryB, idA)
	} else {
		// Remove from the opposite list first (can't be both hard and soft).
		*otherA = removeInt(*otherA, idB)
		*otherB = removeInt(*otherB, idA)
		*primaryA = append(*primaryA, idB)
		*primaryB = append(*primaryB, idA)
	}
	return splines
}

// removeSplineFromCouplings removes all references to deletedID from every spline's coupling lists.
func removeSplineFromCouplings(splines []Spline, deletedID int) []Spline {
	for i := range splines {
		splines[i].HardCoupledIDs = removeInt(splines[i].HardCoupledIDs, deletedID)
		splines[i].SoftCoupledIDs = removeInt(splines[i].SoftCoupledIDs, deletedID)
	}
	return splines
}

func reverseSpline(spline Spline) Spline {
	reversed := simpkg.NewSpline(spline.ID, spline.P3, spline.P2, spline.P1, spline.P0)
	reversed.Priority = spline.Priority
	reversed.BusOnly = spline.BusOnly
	reversed.HardCoupledIDs = append([]int(nil), spline.HardCoupledIDs...)
	reversed.SoftCoupledIDs = append([]int(nil), spline.SoftCoupledIDs...)
	reversed.SpeedLimitKmh = spline.SpeedLimitKmh
	reversed.LanePreference = spline.LanePreference
	return reversed
}

func handleReverseMode(splines []Spline, lights []TrafficLight, hoveredSpline int) ([]Spline, []TrafficLight, bool) {
	if hoveredSpline < 0 || !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		return splines, lights, false
	}
	oldSpline := splines[hoveredSpline]
	reversed := reverseSpline(oldSpline)
	splines[hoveredSpline] = reversed
	for i := range lights {
		if lights[i].SplineID == reversed.ID {
			lights[i].DistOnSpline = maxf(reversed.Length-lights[i].DistOnSpline, 0)
		}
	}
	return splines, lights, true
}

func findSplineIndexByID(splines []Spline, id int) int {
	for i, s := range splines {
		if s.ID == id {
			return i
		}
	}
	return -1
}

func removeInt(slice []int, val int) []int {
	out := slice[:0]
	for _, v := range slice {
		if v != val {
			out = append(out, v)
		}
	}
	return out
}

// nearestSampleOnSpline returns the position of the precomputed sample on spline
// that is closest (Euclidean) to pos.
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

// nearestSampleWithDist returns both the world position and the arc-length
// distance along the spline for the sample closest to pos.
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

// drawLaneChangeSplines renders active lane-change bridge splines in a
// distinctive magenta colour, visible only when debug mode is on.
func drawLaneChangeSplines(lcs []Spline, zoom float32) {
	if len(lcs) == 0 {
		return
	}
	color := NewColor(230, 60, 230, 220)
	dimColor := NewColor(230, 60, 230, 100)
	thickness := pixelsToWorld(zoom, 2.5)
	handleR := pixelsToWorld(zoom, 4)
	armThick := pixelsToWorld(zoom, 1)

	for _, s := range lcs {
		drawSpline(s, thickness, color)
		// Draw the Bézier handles so it's clear where the curve is steered.
		drawLineEx(s.P0, s.P1, armThick, dimColor)
		drawLineEx(s.P3, s.P2, armThick, dimColor)
		drawCircleV(s.P1, handleR, color)
		drawCircleV(s.P2, handleR, color)
	}
}

// drawCoupleMode draws coupling relationship lines between coupled splines
// and highlights the currently selected first spline.
func drawCoupleMode(splines []Spline, firstSelectedID int, hoveredSpline int, zoom float32) {
	hardColor := NewColor(80, 180, 255, 180)  // blue — hard coupling
	softColor := NewColor(180, 120, 255, 180) // purple — soft coupling
	selectedColor := NewColor(255, 200, 50, 255)
	hoveredColor := NewColor(255, 140, 30, 200)
	thickness := pixelsToWorld(zoom, 2)
	r := pixelsToWorld(zoom, 5)

	drawLinks := func(ids []int, spline Spline, color Color) {
		midA := spline.Samples[simSamples/2]
		for _, coupledID := range ids {
			if coupledID <= spline.ID {
				continue // draw each pair once
			}
			idx := findSplineIndexByID(splines, coupledID)
			if idx < 0 {
				continue
			}
			midB := splines[idx].Samples[simSamples/2]
			drawLineEx(midA, midB, thickness, color)
			drawCircleV(midA, r, color)
			drawCircleV(midB, r, color)
		}
	}

	// Draw all coupling links (draw once per unique pair by only drawing when idA < idB).
	for _, spline := range splines {
		drawLinks(spline.HardCoupledIDs, spline, hardColor)
		drawLinks(spline.SoftCoupledIDs, spline, softColor)
	}

	// Highlight the hovered spline.
	if hoveredSpline >= 0 {
		mid := splines[hoveredSpline].Samples[simSamples/2]
		drawCircleV(mid, pixelsToWorld(zoom, 8), hoveredColor)
	}

	// Highlight the first selected spline.
	if firstSelectedID >= 0 {
		idx := findSplineIndexByID(splines, firstSelectedID)
		if idx >= 0 {
			mid := splines[idx].Samples[simSamples/2]
			drawCircleV(mid, pixelsToWorld(zoom, 10), selectedColor)
		}
	}
}

func newCutDraft() CutDraft {
	return CutDraft{OriginalSplineID: -1}
}

// splitBezierAt splits a cubic Bézier at parameter t using de Casteljau's algorithm.
// Returns control points for the left (t=0..t) and right (t..1) sub-curves.
func splitBezierAt(p0, p1, p2, p3 Vec2, t float32) ([4]Vec2, [4]Vec2) {
	lerp2 := func(a, b Vec2, t float32) Vec2 {
		return Vec2{X: a.X + (b.X-a.X)*t, Y: a.Y + (b.Y-a.Y)*t}
	}
	a1 := lerp2(p0, p1, t)
	a2 := lerp2(p1, p2, t)
	a3 := lerp2(p2, p3, t)
	b1 := lerp2(a1, a2, t)
	b2 := lerp2(a2, a3, t)
	cut := lerp2(b1, b2, t)
	left := [4]Vec2{p0, a1, b1, cut}
	right := [4]Vec2{cut, b2, a3, p3}
	return left, right
}

// findNearestSplinePoint searches all precomputed samples across all splines and
// returns the one closest to pos. Returns splineIndex, sample t (0..1), world position.
func findNearestSplinePoint(splines []Spline, pos Vec2) (splineIndex int, t float32, point Vec2, found bool) {
	bestDSq := float32(math.MaxFloat32)
	for si, spline := range splines {
		for i := 0; i <= simSamples; i++ {
			d := distSq(spline.Samples[i], pos)
			if d < bestDSq {
				bestDSq = d
				splineIndex = si
				t = float32(i) / float32(simSamples)
				point = spline.Samples[i]
				found = true
			}
		}
	}
	return
}

func handleCutMode(stage Stage, cd CutDraft, splines []Spline, mouseWorld Vec2, nextSplineID int) (Stage, CutDraft, []Spline, int, bool) {
	if rl.IsKeyPressed(rl.KeyEscape) || rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		return StageIdle, newCutDraft(), splines, nextSplineID, false
	}

	switch stage {
	case StageIdle:
		if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
			break
		}
		si, t, point, found := findNearestSplinePoint(splines, mouseWorld)
		if !found {
			break
		}
		spline := splines[si]
		left, right := splitBezierAt(spline.P0, spline.P1, spline.P2, spline.P3, t)
		return StageSetP1, CutDraft{
			OriginalSplineID:       spline.ID,
			OriginalSplinePriority: spline.Priority,
			CutPoint:               point,
			CutT:                   t,
			LeftP:                  left,
			RightP:                 right,
		}, splines, nextSplineID, false

	case StageSetP1:
		if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
			break
		}
		H := mouseWorld
		mirror := Vec2{X: 2*cd.CutPoint.X - H.X, Y: 2*cd.CutPoint.Y - H.Y}

		// Build the two final splines. Left keeps its P0/P1, gets new P2=mirror.
		// Right gets new P1=H, keeps its P2/P3.
		newLeft := simpkg.NewSpline(nextSplineID, cd.LeftP[0], cd.LeftP[1], mirror, cd.LeftP[3])
		newLeft.Priority = cd.OriginalSplinePriority
		nextSplineID++
		newRight := simpkg.NewSpline(nextSplineID, cd.RightP[0], H, cd.RightP[2], cd.RightP[3])
		newRight.Priority = cd.OriginalSplinePriority
		nextSplineID++

		// Remove original spline (find by ID in case slice shifted).
		idx := findSplineIndexByID(splines, cd.OriginalSplineID)
		if idx >= 0 {
			splines = removeSplineFromCouplings(splines, cd.OriginalSplineID)
			splines = append(splines[:idx], splines[idx+1:]...)
		}
		splines = append(splines, newLeft, newRight)

		return StageIdle, newCutDraft(), splines, nextSplineID, true
	}

	return stage, cd, splines, nextSplineID, false
}

func drawCutMode(stage Stage, cd CutDraft, splines []Spline, mouseWorld Vec2, zoom float32) {
	snapColor := NewColor(255, 180, 0, 255)
	handleColor := NewColor(255, 220, 60, 220)
	leftColor := NewColor(80, 210, 120, 255)
	rightColor := NewColor(80, 140, 230, 255)
	thickness := pixelsToWorld(zoom, 3)
	handleR := pixelsToWorld(zoom, 5)
	snapR := pixelsToWorld(zoom, 8)

	switch stage {
	case StageIdle:
		// Snap dot follows the nearest point on any spline.
		_, _, point, found := findNearestSplinePoint(splines, mouseWorld)
		if found {
			drawCircleV(point, snapR, snapColor)
			// Small crosshair lines.
			arm := pixelsToWorld(zoom, 10)
			drawLineEx(Vec2{X: point.X - arm, Y: point.Y}, Vec2{X: point.X + arm, Y: point.Y}, pixelsToWorld(zoom, 1.5), snapColor)
			drawLineEx(Vec2{X: point.X, Y: point.Y - arm}, Vec2{X: point.X, Y: point.Y + arm}, pixelsToWorld(zoom, 1.5), snapColor)
		}

	case StageSetP1:
		H := mouseWorld
		mirror := Vec2{X: 2*cd.CutPoint.X - H.X, Y: 2*cd.CutPoint.Y - H.Y}

		// Preview left spline (green): original P0/P1, new P2=mirror, cut point.
		previewLeft := simpkg.NewSpline(0, cd.LeftP[0], cd.LeftP[1], mirror, cd.LeftP[3])
		// Preview right spline (blue): cut point, H, original P2, original P3.
		previewRight := simpkg.NewSpline(0, cd.RightP[0], H, cd.RightP[2], cd.RightP[3])

		drawSpline(previewLeft, thickness, leftColor)
		drawSpline(previewRight, thickness, rightColor)

		// Handle lines from cut point to both control handles.
		drawLineEx(cd.CutPoint, H, pixelsToWorld(zoom, 1.5), handleColor)
		drawLineEx(cd.CutPoint, mirror, pixelsToWorld(zoom, 1.5), handleColor)
		drawCircleV(H, handleR, rightColor)
		drawCircleV(mirror, handleR, leftColor)

		// Cut point marker.
		drawCircleV(cd.CutPoint, snapR, snapColor)
	}
}

func routePanelRect(panel RoutePanel) rl.Rectangle {
	height := float32(234)
	if panel.VehicleKind == VehicleBus {
		rows := len(panel.BusStops)
		if rows < 1 {
			rows = 1
		}
		height = 326 + float32(rows)*28
	}
	return rl.NewRectangle(float32(rl.GetScreenWidth())-360, 18, 340, height)
}

func routePanelSliderRect(panel RoutePanel) rl.Rectangle {
	pr := routePanelRect(panel)
	return rl.NewRectangle(pr.X+18, pr.Y+82, pr.Width-36, 22)
}

func routePanelBusAddRect(panel RoutePanel) rl.Rectangle {
	pr := routePanelRect(panel)
	return rl.NewRectangle(pr.X+18, pr.Y+180, pr.Width-36, 28)
}

func routePanelBusStopRowRect(panel RoutePanel, idx int) rl.Rectangle {
	pr := routePanelRect(panel)
	return rl.NewRectangle(pr.X+18, pr.Y+216+float32(idx)*28, pr.Width-36, 24)
}

func routePanelApplyRect(panel RoutePanel) rl.Rectangle {
	pr := routePanelRect(panel)
	return rl.NewRectangle(pr.X+18, pr.Y+pr.Height-54, 120, 32)
}

func routePanelCancelRect(panel RoutePanel) rl.Rectangle {
	pr := routePanelRect(panel)
	return rl.NewRectangle(pr.X+202, pr.Y+pr.Height-54, 120, 32)
}

func updateRoutePanel(panel RoutePanel, routes []Route, cars []Car, nextRouteID *int, graph *RoadGraph, splines []Spline, mouse Vec2) (RoutePanel, []Route, []Car, bool) {
	panelRect := routePanelRect(panel)
	sliderRect := routePanelSliderRect(panel)
	applyRect := routePanelApplyRect(panel)
	cancelRect := routePanelCancelRect(panel)
	applied := false

	if panel.VehicleKind == VehicleBus && panel.AddingBusStop && (rl.IsKeyPressed(rl.KeyEscape) || rl.IsMouseButtonPressed(rl.MouseButtonRight)) {
		panel.AddingBusStop = false
		panel.StopStatusNotice = ""
		return panel, routes, cars, false
	}
	if rl.IsKeyPressed(rl.KeyEscape) || rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		return RoutePanel{}, routes, cars, false
	}
	if rl.IsKeyPressed(rl.KeyEnter) {
		panel, routes, cars = applyRoutePanel(panel, routes, cars, nextRouteID)
		return panel, routes, cars, true
	}

	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		if pointInRect(mouse, sliderRect) {
			panel.DraggingSlider = true
			panel.SpawnPerMinute = sliderValueFromMouse(mouse.X, sliderRect, 0, spawnSliderMaxPerMinute, routeSpawnSliderStep(panel.VehicleKind))
		}
		// Colour swatch clicks
		swatchSize := float32(20)
		swatchStride := swatchSize + 4
		for i := range routePalette {
			col := i % 8
			row := i / 8
			sx := panelRect.X + 18 + float32(col)*swatchStride
			sy := panelRect.Y + 128 + float32(row)*24
			swatchRect := rl.NewRectangle(sx, sy, swatchSize, swatchSize)
			if pointInRect(mouse, swatchRect) {
				panel.ColorIndex = i
			}
		}
		if panel.VehicleKind == VehicleBus {
			addRect := routePanelBusAddRect(panel)
			if pointInRect(mouse, addRect) {
				panel.AddingBusStop = !panel.AddingBusStop
				panel.StopStatusNotice = ""
			}
			for i := range panel.BusStops {
				rowRect := routePanelBusStopRowRect(panel, i)
				delRect := rl.NewRectangle(rowRect.X+rowRect.Width-58, rowRect.Y+2, 54, rowRect.Height-4)
				if pointInRect(mouse, delRect) {
					panel.BusStops = append(panel.BusStops[:i], panel.BusStops[i+1:]...)
					panel.AddingBusStop = false
					panel = syncRoutePanelPath(panel, graph)
					break
				}
			}
		}
		if pointInRect(mouse, applyRect) {
			panel, routes, cars = applyRoutePanel(panel, routes, cars, nextRouteID)
			applied = true
		}
		if pointInRect(mouse, cancelRect) {
			panel = RoutePanel{}
		}
	}
	if panel.DraggingSlider {
		if rl.IsMouseButtonDown(rl.MouseButtonLeft) {
			panel.SpawnPerMinute = sliderValueFromMouse(mouse.X, sliderRect, 0, spawnSliderMaxPerMinute, routeSpawnSliderStep(panel.VehicleKind))
		} else {
			panel.DraggingSlider = false
		}
	}
	panel = syncRoutePanelPath(panel, graph)
	_ = splines

	return panel, routes, cars, applied
}

func applyRoutePanel(panel RoutePanel, routes []Route, cars []Car, nextRouteID *int) (RoutePanel, []Route, []Car) {
	if panel.SpawnPerMinute > 0.01 && !panel.ValidPath() {
		return panel, routes, cars
	}
	spawn := panel.SpawnPerMinute
	if spawn <= 0.01 {
		if panel.ExistingRouteID >= 0 {
			routes = removeRouteByID(routes, panel.ExistingRouteID)
			cars = removeCarsForRoute(cars, panel.ExistingRouteID)
		}
		return RoutePanel{}, routes, cars
	}

	if panel.ExistingRouteID >= 0 {
		if idx := findRouteIndexByID(routes, panel.ExistingRouteID); idx >= 0 {
			routes[idx].SpawnPerMinute = spawn
			routes[idx].PathIDs = append([]int(nil), panel.PathIDs...)
			routes[idx].PathLength = panel.PathLength
			routes[idx].Valid = len(panel.PathIDs) > 0
			routes[idx].ColorIndex = panel.ColorIndex
			routes[idx].Color = simpkg.RoutePaletteColor(panel.ColorIndex)
			routes[idx].VehicleKind = panel.VehicleKind
			routes[idx].BusStops = append([]BusStop(nil), panel.BusStops...)
			if routes[idx].NextSpawnIn <= 0 {
				routes[idx].NextSpawnIn = randomizedSpawnDelay(spawn)
			}
			if panel.VehicleKind == VehicleBus {
				cars = removeCarsForRoute(cars, panel.ExistingRouteID)
			}
		}
		return RoutePanel{}, routes, cars
	}

	routeID := *nextRouteID
	*nextRouteID = *nextRouteID + 1
	route := Route{
		ID:             routeID,
		StartSplineID:  panel.StartSplineID,
		EndSplineID:    panel.EndSplineID,
		PathIDs:        append([]int(nil), panel.PathIDs...),
		PathLength:     panel.PathLength,
		SpawnPerMinute: spawn,
		NextSpawnIn:    randomizedSpawnDelay(spawn),
		Valid:          len(panel.PathIDs) > 0,
		ColorIndex:     panel.ColorIndex,
		Color:          simpkg.RoutePaletteColor(panel.ColorIndex),
		VehicleKind:    panel.VehicleKind,
		BusStops:       append([]BusStop(nil), panel.BusStops...),
	}
	routes = append(routes, route)
	return RoutePanel{}, routes, cars
}

func refreshRoutes(routes []Route, splines []Spline) []Route {
	return updateRouteVisuals(routes, splines, map[int]int{})
}

func updateRouteVisuals(routes []Route, splines []Spline, vehicleCounts map[int]int) []Route {
	graph := simpkg.NewRoadGraph(splines, vehicleCounts)
	return simpkg.UpdateRouteVisualsWithGraph(routes, graph)
}

func carBodyPose(car Car, splines []Spline, splineIndexByID map[int]int) (frontPos, center, heading Vec2, ok bool) {
	splineIdx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return Vec2{}, Vec2{}, Vec2{}, false
	}
	splinePos, splineTangent := simpkg.SampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
	rightNormal := Vec2{X: splineTangent.Y, Y: -splineTangent.X}
	frontPos = vecAdd(splinePos, vecScale(rightNormal, car.LateralOffset))
	heading = normalize(vecSub(frontPos, car.RearPosition))
	if vectorLengthSq(vecSub(frontPos, car.RearPosition)) <= 1e-9 {
		heading = splineTangent
	}
	center = vecScale(vecAdd(frontPos, car.RearPosition), 0.5)
	return frontPos, center, heading, true
}

func eraseCarsAtPoint(cars []Car, splines []Spline, splineIndexByID map[int]int, center Vec2, radius float32) []Car {
	if !rl.IsMouseButtonDown(rl.MouseButtonLeft) {
		return cars
	}
	out := cars[:0]
	radiusSq := radius * radius
	for _, car := range cars {
		_, bodyCenter, _, ok := carBodyPose(car, splines, splineIndexByID)
		if !ok {
			out = append(out, car)
			continue
		}
		erase := distSq(bodyCenter, center) <= radiusSq
		if !erase && car.Trailer.HasTrailer {
			trailerCenter := vecScale(vecAdd(car.RearPosition, car.Trailer.RearPosition), 0.5)
			erase = distSq(trailerCenter, center) <= radiusSq
		}
		if !erase {
			out = append(out, car)
		}
	}
	return out
}

func findClickedCar(cars []Car, splines []Spline, splineIndexByID map[int]int, point Vec2) int {
	bestIdx := -1
	bestDistSq := float32(math.MaxFloat32)
	for i, car := range cars {
		_, center, _, ok := carBodyPose(car, splines, splineIndexByID)
		if !ok {
			continue
		}
		halfDiag := (car.Length + car.Width) / 2
		dSq := distSq(center, point)
		if dSq <= halfDiag*halfDiag && dSq < bestDistSq {
			bestDistSq = dSq
			bestIdx = i
		}
	}
	return bestIdx
}

func drawCars(cars []Car, splines []Spline, splineIndexByID map[int]int, zoom float32, debugMode bool, viewRect worldRect) {
	if len(cars) == 0 {
		return
	}
	for _, car := range cars {
		_, center, bodyHeading, ok := carBodyPose(car, splines, splineIndexByID)
		if !ok {
			continue
		}
		bodyVisible := circleVisibleInWorldRect(center, (car.Length+car.Width)/2, viewRect)
		trailerVisible := false
		var trailerCenter Vec2
		var trailerHeading Vec2
		if car.Trailer.HasTrailer {
			hitchPos := car.RearPosition
			trailerHeading = normalize(vecSub(hitchPos, car.Trailer.RearPosition))
			if vectorLengthSq(vecSub(hitchPos, car.Trailer.RearPosition)) <= 1e-9 {
				trailerHeading = bodyHeading
			}
			trailerCenter = vecScale(vecAdd(hitchPos, car.Trailer.RearPosition), 0.5)
			trailerVisible = circleVisibleInWorldRect(trailerCenter, (car.Trailer.Length+car.Trailer.Width)/2, viewRect)
		}
		if !bodyVisible && !trailerVisible {
			continue
		}
		angle := float32(math.Atan2(float64(bodyHeading.Y), float64(bodyHeading.X)) * 180 / math.Pi)
		rect := rl.NewRectangle(center.X, center.Y, car.Length, car.Width)
		origin := NewVec2(car.Length/2, car.Width/2)
		// Draw trailer first so the cab renders on top at the hitch point.
		if car.Trailer.HasTrailer && trailerVisible {
			trailerAngle := float32(math.Atan2(float64(trailerHeading.Y), float64(trailerHeading.X)) * 180 / math.Pi)
			trailerRect := rl.NewRectangle(trailerCenter.X, trailerCenter.Y, car.Trailer.Length, car.Trailer.Width)
			trailerOrigin := NewVec2(car.Trailer.Length/2, car.Trailer.Width/2)
			drawRectanglePro(trailerRect, trailerOrigin, trailerAngle, car.Trailer.Color)
		}
		if bodyVisible {
			drawRectanglePro(rect, origin, angle, car.Color)
		}
		if car.VehicleKind == VehicleBus {
			windowColor := NewColor(235, 243, 250, 220)
			windowRect := rl.NewRectangle(center.X, center.Y, car.Length*0.72, car.Width*0.42)
			windowOrigin := NewVec2(windowRect.Width/2, windowRect.Height/2)
			if bodyVisible {
				drawRectanglePro(windowRect, windowOrigin, angle, windowColor)
			}
			frontRect := rl.NewRectangle(center.X+bodyHeading.X*car.Length*0.28, center.Y+bodyHeading.Y*car.Length*0.28, car.Length*0.12, car.Width*0.78)
			frontOrigin := NewVec2(frontRect.Width/2, frontRect.Height/2)
			if bodyVisible {
				drawRectanglePro(frontRect, frontOrigin, angle, NewColor(30, 30, 35, 90))
			}
		}
		if bodyVisible && car.Braking {
			drawCircleV(center, maxf(car.Width*0.22, pixelsToWorld(zoom, 2)), NewColor(220, 50, 50, 255))
		} else if bodyVisible && debugMode && car.SoftSlowing {
			drawCircleV(center, maxf(car.Width*0.22, pixelsToWorld(zoom, 2)), NewColor(60, 120, 220, 255))
		}
		if bodyVisible && car.VehicleKind == VehicleBus && car.BusStopTimer > 0 {
			drawCircleV(center, maxf(car.Width*0.24, pixelsToWorld(zoom, 2.5)), NewColor(255, 200, 40, 255))
		}
	}
}

func newPlayerCarState(spawnPos Vec2) playerCarState {
	return playerCarState{
		Active:              true,
		Position:            spawnPos,
		HeadingDeg:          0,
		Speed:               0,
		Length:              4.6,
		Width:               1.9,
		Color:               NewColor(36, 142, 212, 255),
		MaxForwardSpeed:     16.0,
		MaxReverseSpeed:     4.0,
		AccelMPS2:           4.25,
		BrakeMPS2:           6.0,
		CoastMPS2:           1.5,
		BaseTurnDegPerS:     165.0,
		DestinationSplineID: -1,
	}
}

func playerForward(headingDeg float32) Vec2 {
	rad := float64(headingDeg) * math.Pi / 180
	return Vec2{
		X: float32(math.Sin(rad)),
		Y: -float32(math.Cos(rad)),
	}
}

func updatePlayerCar(player *playerCarState, dt float32) {
	if player == nil || !player.Active || dt <= 0 {
		return
	}

	if rl.IsKeyDown(rl.KeyUp) {
		player.Speed += player.AccelMPS2 * dt
	} else if rl.IsKeyDown(rl.KeyDown) {
		player.Speed -= player.BrakeMPS2 * dt
	} else if player.Speed > 0 {
		player.Speed = maxf(0, player.Speed-player.CoastMPS2*dt)
	} else if player.Speed < 0 {
		player.Speed = minf(0, player.Speed+player.CoastMPS2*dt)
	}

	player.Speed = clampf(player.Speed, -player.MaxReverseSpeed, player.MaxForwardSpeed)

	turnInput := float32(0)
	if rl.IsKeyDown(rl.KeyLeft) {
		turnInput -= 1
	}
	if rl.IsKeyDown(rl.KeyRight) {
		turnInput += 1
	}
	if turnInput != 0 {
		turnScale := 1.0 / (1.0 + absf(player.Speed)*0.12)
		turnScale = maxf(turnScale, 0.2)
		if player.Speed < 0 {
			turnInput = -turnInput
		}
		player.HeadingDeg += turnInput * player.BaseTurnDegPerS * turnScale * dt
	}

	player.Position = vecAdd(player.Position, vecScale(playerForward(player.HeadingDeg), player.Speed*dt))
}

func drawPlayerCarScreen(player playerCarState, zoom float32) {
	if !player.Active {
		return
	}

	center := NewVec2(float32(rl.GetScreenWidth())/2, float32(rl.GetScreenHeight())/2)
	lengthPx := maxf(player.Length*zoom, 34)
	widthPx := maxf(player.Width*zoom, 18)

	bodyRect := rl.NewRectangle(center.X, center.Y, lengthPx, widthPx)
	bodyOrigin := NewVec2(lengthPx/2, widthPx/2)
	drawRectanglePro(bodyRect, bodyOrigin, -90, player.Color)

	windowRect := rl.NewRectangle(center.X, center.Y-widthPx*0.02, lengthPx*0.54, widthPx*0.56)
	windowOrigin := NewVec2(windowRect.Width/2, windowRect.Height/2)
	drawRectanglePro(windowRect, windowOrigin, -90, NewColor(222, 238, 246, 230))

	grilleRect := rl.NewRectangle(center.X, center.Y-lengthPx*0.28, lengthPx*0.12, widthPx*0.74)
	grilleOrigin := NewVec2(grilleRect.Width/2, grilleRect.Height/2)
	drawRectanglePro(grilleRect, grilleOrigin, -90, NewColor(26, 34, 38, 110))

	lightOffsetX := widthPx * 0.26
	lightOffsetY := lengthPx * 0.43
	lightR := maxf(widthPx*0.08, 2.5)
	drawCircleV(NewVec2(center.X-lightOffsetX, center.Y-lightOffsetY), lightR, NewColor(255, 245, 190, 255))
	drawCircleV(NewVec2(center.X+lightOffsetX, center.Y-lightOffsetY), lightR, NewColor(255, 245, 190, 255))
}

func drawPlayerProxyAttachment(car Car, splines []Spline, splineIndexByID map[int]int, zoom float32, viewRect worldRect) {
	splineIdx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return
	}
	spline := splines[splineIdx]
	if !splineVisibleInWorldRect(spline, viewRect) {
		return
	}

	highlightColor := NewColor(255, 122, 68, 220)
	drawSpline(spline, pixelsToWorld(zoom, 6), highlightColor)

	anchorPos, _ := simpkg.SampleSplineAtDistance(spline, car.DistanceOnSpline)
	if !worldRectContainsPoint(viewRect, anchorPos) {
		return
	}
	rOuter := pixelsToWorld(zoom, 9)
	rInner := pixelsToWorld(zoom, 4)
	drawCircleV(anchorPos, rInner, NewColor(255, 244, 232, 255))
	drawRing(anchorPos, rInner*1.15, rOuter, 0, 360, 24, highlightColor)
}

func drawDrivingDestination(player playerCarState, zoom float32, viewRect worldRect) {
	if !player.HasDestination {
		return
	}
	if !worldRectContainsPoint(viewRect, player.DestinationPoint) {
		return
	}
	color := NewColor(60, 190, 90, 240)
	rOuter := pixelsToWorld(zoom, 11)
	rInner := pixelsToWorld(zoom, 4.5)
	drawCircleV(player.DestinationPoint, rInner, NewColor(236, 255, 240, 255))
	drawRing(player.DestinationPoint, rInner*1.2, rOuter, 0, 360, 24, color)
}

// drawCarHitboxes renders the multi-circle collision hitbox of every car.
func drawCarHitboxes(cars []Car, splines []Spline, splineIndexByID map[int]int, viewRect worldRect) {
	fill := NewColor(255, 80, 255, 50)
	outline := NewColor(255, 80, 255, 200)
	pivotColor := NewColor(255, 220, 0, 220)
	for _, car := range cars {
		splineIdx, ok := splineIndexByID[car.CurrentSplineID]
		if !ok {
			continue
		}
		splinePos2, splineTangent := simpkg.SampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
		rightNormal2 := Vec2{X: splineTangent.Y, Y: -splineTangent.X}
		frontPos := vecAdd(splinePos2, vecScale(rightNormal2, car.LateralOffset))
		bodyHeading := normalize(vecSub(frontPos, car.RearPosition))
		if vectorLengthSq(vecSub(frontPos, car.RearPosition)) <= 1e-9 {
			bodyHeading = splineTangent
		}
		center := vecScale(vecAdd(frontPos, car.RearPosition), 0.5)
		r := collisionRadius(car)
		bodyVisible := circleVisibleInWorldRect(center, (car.Length+car.Width)/2+r, viewRect)
		trailerVisible := false
		var trailerCenter Vec2
		var trailerHeading Vec2
		for _, offset := range collisionCircleOffsets(car) {
			if bodyVisible {
				circlePos := vecAdd(center, vecScale(bodyHeading, offset))
				drawCircleV(circlePos, r, fill)
				drawCircleLinesV(circlePos, r, outline)
			}
		}
		// Mark front and rear pivots.
		if bodyVisible {
			drawCircleV(frontPos, r*0.25, pivotColor)
			drawCircleV(car.RearPosition, r*0.25, pivotColor)
		}

		if car.Trailer.HasTrailer {
			hitchPos := car.RearPosition
			trailerHeading = normalize(vecSub(hitchPos, car.Trailer.RearPosition))
			if vectorLengthSq(vecSub(hitchPos, car.Trailer.RearPosition)) <= 1e-9 {
				trailerHeading = bodyHeading
			}
			trailerCenter = vecScale(vecAdd(hitchPos, car.Trailer.RearPosition), 0.5)
			rT := hitboxRadius(car.Trailer.Width)
			trailerVisible = circleVisibleInWorldRect(trailerCenter, (car.Trailer.Length+car.Trailer.Width)/2+rT, viewRect)
			for _, offset := range hitboxCircleOffsets(car.Trailer.Length, car.Trailer.Width) {
				if trailerVisible {
					circlePos := vecAdd(trailerCenter, vecScale(trailerHeading, offset))
					drawCircleV(circlePos, rT, fill)
					drawCircleLinesV(circlePos, rT, outline)
				}
			}
			// Trailer rear pivot (hitch is already marked as cab rear pivot above).
			if trailerVisible {
				drawCircleV(car.Trailer.RearPosition, rT*0.25, pivotColor)
			}
		}
	}
}

// drawCarSpeedLabels draws each car's current speed in km/h in screen space,
// centred just above the car's position.
func drawCarSpeedLabels(cars []Car, splines []Spline, splineIndexByID map[int]int, camera rl.Camera2D, viewRect worldRect) {
	fontSize := int32(12)
	for _, car := range cars {
		splineIdx, ok := splineIndexByID[car.CurrentSplineID]
		if !ok {
			continue
		}
		frontPos, _ := simpkg.SampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
		center := vecScale(vecAdd(frontPos, car.RearPosition), 0.5)
		if !worldRectContainsPoint(viewRect, center) {
			continue
		}
		screen := getWorldToScreen2D(center, camera)
		label := fmt.Sprintf("%d", int(car.Speed*3.6))
		textW := measureText(label, fontSize)
		drawText(label, int32(screen.X)-textW/2, int32(screen.Y)-18, fontSize, rl.Black)
	}
}

func drawDebugBlameLinks(links []DebugBlameLink, cars []Car, splines []Spline, splineIndexByID map[int]int, zoom float32, lineColor Color, viewRect worldRect) {
	if len(links) == 0 || len(cars) == 0 {
		return
	}

	lineThickness := pixelsToWorld(zoom, 2.0)

	for _, link := range links {
		if link.FromCarIndex < 0 || link.FromCarIndex >= len(cars) || link.ToCarIndex < 0 || link.ToCarIndex >= len(cars) {
			continue
		}

		fromSplineIdx, okFrom := splineIndexByID[cars[link.FromCarIndex].CurrentSplineID]
		toSplineIdx, okTo := splineIndexByID[cars[link.ToCarIndex].CurrentSplineID]
		if !okFrom || !okTo {
			continue
		}

		fromFront, _ := simpkg.SampleSplineAtDistance(splines[fromSplineIdx], cars[link.FromCarIndex].DistanceOnSpline)
		toFront, _ := simpkg.SampleSplineAtDistance(splines[toSplineIdx], cars[link.ToCarIndex].DistanceOnSpline)
		fromPos := vecScale(vecAdd(fromFront, cars[link.FromCarIndex].RearPosition), 0.5)
		toPos := vecScale(vecAdd(toFront, cars[link.ToCarIndex].RearPosition), 0.5)
		if !worldRectIntersectsAABB(viewRect, minf(fromPos.X, toPos.X)-lineThickness, minf(fromPos.Y, toPos.Y)-lineThickness, maxf(fromPos.X, toPos.X)+lineThickness, maxf(fromPos.Y, toPos.Y)+lineThickness) {
			continue
		}
		drawLineEx(fromPos, toPos, lineThickness, lineColor)
		drawCircleV(fromPos, pixelsToWorld(zoom, 3.5), lineColor)
	}
}

func drawRouteWithIndex(route Route, splines []Spline, indexByID map[int]int, thickness float32, zoom float32, viewRect worldRect) {
	color := NewColor(route.Color.R, route.Color.G, route.Color.B, 90)
	for _, pathID := range route.PathIDs {
		idx, ok := indexByID[pathID]
		if !ok {
			continue
		}
		if !splineVisibleInWorldRect(splines[idx], viewRect) {
			continue
		}
		drawSpline(splines[idx], thickness, color)
	}
	spawnR := pixelsToWorld(zoom, 10)
	destR := pixelsToWorld(zoom, 10)
	if idx, ok := indexByID[route.StartSplineID]; ok && circleVisibleInWorldRect(splines[idx].P0, spawnR, viewRect) {
		drawEndpoint(splines[idx].P0, spawnR, route.Color)
	}
	if idx, ok := indexByID[route.EndSplineID]; ok && circleVisibleInWorldRect(splines[idx].P3, destR, viewRect) {
		drawEndpoint(splines[idx].P3, destR, color)
	}
	if route.VehicleKind == VehicleBus {
		stopFill := NewColor(route.Color.R, route.Color.G, route.Color.B, 220)
		stopOuter := NewColor(20, 20, 25, 220)
		stopR := pixelsToWorld(zoom, 7)
		for _, stop := range route.BusStops {
			if !circleVisibleInWorldRect(stop.WorldPos, stopR, viewRect) {
				continue
			}
			drawCircleV(stop.WorldPos, stopR, stopOuter)
			drawCircleV(stop.WorldPos, stopR*0.72, stopFill)
		}
	}
}

func drawRoute(route Route, splines []Spline, thickness float32, zoom float32) {
	drawRouteWithIndex(route, splines, simpkg.BuildSplineIndexByID(splines), thickness, zoom, worldRect{
		MinX: -math.MaxFloat32,
		MinY: -math.MaxFloat32,
		MaxX: math.MaxFloat32,
		MaxY: math.MaxFloat32,
	})
}

func drawRoutePicking(routeStartSplineID int, routePanel RoutePanel, hoveredStart EndHit, hoveredEnd EndHit, splines []Spline, indexByID map[int]int, graph *RoadGraph, zoom float32, viewRect worldRect) {
	handleRadius := pixelsToWorld(zoom, handlePixels)
	if hoveredStart.SplineIndex >= 0 && routeStartSplineID < 0 {
		drawEndpoint(hoveredStart.Point, handleRadius*1.6, NewColor(255, 196, 61, 255))
	}
	if routeStartSplineID >= 0 {
		if startSpline, ok := simpkg.FindSplineByID(splines, routeStartSplineID); ok {
			drawEndpoint(startSpline.P0, handleRadius*1.8, NewColor(214, 76, 76, 255))
		}
		if hoveredEnd.SplineIndex >= 0 {
			drawEndpoint(hoveredEnd.Point, handleRadius*1.5, NewColor(35, 85, 175, 255))
			preview := Route{
				StartSplineID: routeStartSplineID,
				EndSplineID:   hoveredEnd.SplineID,
				VehicleKind:   routePanel.VehicleKind,
			}
			if pathIDs, _, _, ok := simpkg.ComputeRoutePathWithGraph(preview, graph); ok {
				previewRoute := Route{PathIDs: pathIDs, Color: simpkg.RoutePaletteColor(routePanel.ColorIndex), Valid: true}
				drawRouteWithIndex(previewRoute, splines, indexByID, pixelsToWorld(zoom, 4), zoom, viewRect)
			}
		}
	}
}

func drawRoutePanel(panel RoutePanel, routes []Route) {
	panelRect := routePanelRect(panel)
	sliderRect := routePanelSliderRect(panel)
	applyRect := routePanelApplyRect(panel)
	cancelRect := routePanelCancelRect(panel)

	bg := NewColor(248, 248, 250, 245)
	outline := NewColor(210, 210, 215, 255)
	textCol := NewColor(30, 30, 35, 255)
	muted := NewColor(90, 90, 100, 255)
	accent := NewColor(70, 110, 220, 255)
	button := NewColor(235, 236, 240, 255)
	danger := NewColor(200, 60, 60, 255)

	rl.DrawRectangle(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), bg)
	rl.DrawRectangleLines(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), outline)
	title := "Create route"
	if panel.ExistingRouteID >= 0 {
		title = "Edit existing route"
	}
	unitLabel := "cars"
	if panel.VehicleKind == VehicleBus {
		if panel.ExistingRouteID >= 0 {
			title = "Edit bus line"
		} else {
			title = "Create bus line"
		}
		unitLabel = "buses"
	}
	drawText(title, int32(panelRect.X+18), int32(panelRect.Y+16), 22, textCol)
	drawText(fmt.Sprintf("Start spline #%d → end spline #%d", panel.StartSplineID, panel.EndSplineID), int32(panelRect.X+18), int32(panelRect.Y+44), 18, muted)
	meanSeconds := float32(0)
	if panel.SpawnPerMinute > 0 {
		meanSeconds = 60 / panel.SpawnPerMinute
	}
	drawText(fmt.Sprintf("Average spawn frequency: %.1f %s/min  (mean %.1fs)", panel.SpawnPerMinute, unitLabel, meanSeconds), int32(panelRect.X+18), int32(panelRect.Y+64), 18, textCol)

	rl.DrawRectangle(int32(sliderRect.X), int32(sliderRect.Y), int32(sliderRect.Width), int32(sliderRect.Height), NewColor(229, 229, 234, 255))
	fillWidth := sliderRect.Width * clampf(panel.SpawnPerMinute/spawnSliderMaxPerMinute, 0, 1)
	if fillWidth > 0 {
		fillRect := rl.NewRectangle(sliderRect.X, sliderRect.Y, fillWidth, sliderRect.Height)
		rl.DrawRectangle(int32(fillRect.X), int32(fillRect.Y), int32(fillRect.Width), int32(fillRect.Height), accent)
	}
	knobX := sliderRect.X + fillWidth
	knobRect := rl.NewRectangle(knobX-7, sliderRect.Y-4, 14, sliderRect.Height+8)
	rl.DrawRectangle(int32(knobRect.X), int32(knobRect.Y), int32(knobRect.Width), int32(knobRect.Height), rl.White)
	rl.DrawRectangleLines(int32(knobRect.X), int32(knobRect.Y), int32(knobRect.Width), int32(knobRect.Height), outline)

	// ── Colour picker ──────────────────────────────────────────────────────
	drawText("Route colour:", int32(panelRect.X+18), int32(panelRect.Y+112), 14, muted)
	swatchSize := float32(20)
	swatchStride := swatchSize + 4
	for i, col := range routePalette {
		swatchCol := i % 8
		swatchRow := i / 8
		sx := panelRect.X + 18 + float32(swatchCol)*swatchStride
		sy := panelRect.Y + 128 + float32(swatchRow)*24
		sr := rl.NewRectangle(sx, sy, swatchSize, swatchSize)
		rl.DrawRectangleRec(sr, col)
		if i == panel.ColorIndex {
			rl.DrawRectangleLinesEx(sr, 2, rl.White)
			rl.DrawRectangleLinesEx(rl.NewRectangle(sx-1, sy-1, swatchSize+2, swatchSize+2), 1, NewColor(40, 40, 40, 200))
		} else {
			rl.DrawRectangleLinesEx(sr, 1, NewColor(0, 0, 0, 40))
		}
	}

	if panel.VehicleKind == VehicleBus {
		addRect := routePanelBusAddRect(panel)
		addLabel := "Add bus stop"
		addColor := NewColor(47, 120, 60, 255)
		if panel.AddingBusStop {
			addLabel = "Pick in map..."
			addColor = NewColor(210, 150, 20, 255)
		}
		drawText("Bus stops:", int32(panelRect.X+18), int32(panelRect.Y+166), 14, muted)
		rl.DrawRectangleRec(addRect, addColor)
		rl.DrawRectangleLinesEx(addRect, 1, NewColor(0, 0, 0, 40))
		addW := measureText(addLabel, 14)
		drawText(addLabel, int32(addRect.X)+int32(addRect.Width)/2-addW/2, int32(addRect.Y)+7, 14, rl.White)

		if len(panel.BusStops) == 0 {
			drawText("No stops yet. Add intersections in travel order.", int32(panelRect.X+18), int32(panelRect.Y+220), 13, muted)
		} else {
			for i, stop := range panel.BusStops {
				rowRect := routePanelBusStopRowRect(panel, i)
				rl.DrawRectangleRec(rowRect, NewColor(242, 243, 246, 255))
				rl.DrawRectangleLinesEx(rowRect, 1, outline)
				drawText(fmt.Sprintf("%d. stop on spline #%d", i+1, stop.SplineID), int32(rowRect.X)+8, int32(rowRect.Y)+5, 13, textCol)
				delRect := rl.NewRectangle(rowRect.X+rowRect.Width-58, rowRect.Y+2, 54, rowRect.Height-4)
				rl.DrawRectangleRec(delRect, danger)
				rl.DrawRectangleLinesEx(delRect, 1, NewColor(0, 0, 0, 40))
				drawText("Remove", int32(delRect.X)+7, int32(delRect.Y)+4, 12, rl.White)
			}
		}

		if panel.StopStatusNotice != "" {
			drawText(panel.StopStatusNotice, int32(panelRect.X+18), int32(applyRect.Y)-28, 12, muted)
		}
	}

	applyLabel := "Apply"
	if panel.SpawnPerMinute <= 0.01 {
		if panel.ExistingRouteID >= 0 {
			applyLabel = "Delete"
		} else {
			applyLabel = "Cancel"
		}
	}
	rl.DrawRectangle(int32(applyRect.X), int32(applyRect.Y), int32(applyRect.Width), int32(applyRect.Height), button)
	rl.DrawRectangle(int32(cancelRect.X), int32(cancelRect.Y), int32(cancelRect.Width), int32(cancelRect.Height), button)
	rl.DrawRectangleLines(int32(applyRect.X), int32(applyRect.Y), int32(applyRect.Width), int32(applyRect.Height), outline)
	rl.DrawRectangleLines(int32(cancelRect.X), int32(cancelRect.Y), int32(cancelRect.Width), int32(cancelRect.Height), outline)
	drawText(applyLabel, int32(applyRect.X+34), int32(applyRect.Y+7), 20, textCol)
	drawText("Close", int32(cancelRect.X+34), int32(cancelRect.Y+7), 20, textCol)
	if panel.VehicleKind == VehicleBus {
		drawText(fmt.Sprintf("Current weighted cost: %.0f", panel.PathLength), int32(panelRect.X+18), int32(panelRect.Y+panelRect.Height)-26, 16, muted)
	} else {
		drawText(fmt.Sprintf("Current weighted cost: %.0f", panel.PathLength), int32(panelRect.X+18), int32(panelRect.Y+220), 16, muted)
	}
	_ = routes
}

func drawNotice(text string) {
	width := measureText(text, 18) + 28
	x := int32((int32(rl.GetScreenWidth()) - width) / 2)
	y := int32(rl.GetScreenHeight() - 54)
	bg := NewColor(40, 44, 52, 230)
	rl.DrawRectangle(x, y, width, 34, bg)
	drawText(text, x+14, y+8, 18, rl.White)
}

func modeStatusText(mode EditorMode, tool EditorTool, stage Stage, draft Draft, quadraticDraft QuadraticDraft, routeStartSplineID, coupleModeFirstID int) string {
	switch tool {
	case ToolSpline:
		return stageLabel(stage, draft)
	case ToolQuadratic:
		switch stage {
		case StageIdle:
			return "Idle"
		case StageSetP1:
			if quadraticDraft.FromPrevAxis {
				return "Choose M on the previous spline axis"
			}
			return "Choose M or click a spline end/start"
		case StageSetP2:
			if quadraticDraft.SnapP3 {
				return "Choose M on the next spline axis"
			}
			return "Choose P3"
		default:
			return "Quadratic tool"
		}
	case ToolReverse:
		return "Left click a spline to reverse its direction"
	case ToolRouteCars:
		if routeStartSplineID >= 0 {
			return fmt.Sprintf("Pick destination for start spline #%d", routeStartSplineID)
		}
		return "Click a spline start to begin a route"
	case ToolRouteBuses:
		if routeStartSplineID >= 0 {
			return fmt.Sprintf("Pick destination for bus line start spline #%d", routeStartSplineID)
		}
		return "Left click: begin a bus line   Right click on a spline: toggle bus-only"
	case ToolRouteEraser:
		return "Hold left mouse to erase vehicles in a 5 m brush"
	case ToolPriority:
		return "Left click: set priority   Right click: clear"
	case ToolCouple:
		if coupleModeFirstID >= 0 {
			return fmt.Sprintf("Click second spline to couple/decouple with #%d  (right-click cancels)", coupleModeFirstID)
		}
		return "Click a spline to select it"
	case ToolCut:
		if stage == StageSetP1 {
			return "Place tangent handle at cut point  (right-click cancels)"
		}
		return "Click a spline to cut it"
	case ToolSpeedLimit:
		return "Left click: apply speed limit   Right click: remove"
	case ToolPreference:
		return "Left click: assign 1 (reset counter)   Right click on empty: next number   Right click on assigned: remove"
	case ToolTrafficLight:
		return "Left click on spline: add light to cycle   Right click on light: remove from cycle   Then press Create"
	case ToolDrive:
		return "Use arrow keys to drive. Right click a spline end to set the destination."
	case ToolPedestrianPath:
		return "Left click: first edge, second edge places a 4 m path   Right click: cancel draft or delete hovered path"
	}
	switch mode {
	case ModeDraw:
		return "Draw tools"
	case ModeRules:
		return "Road rule tools"
	case ModeRoute:
		return "Route tools"
	case ModeTraffic:
		return "Traffic tools"
	case ModePedestrian:
		return "Pedestrian tools"
	case ModeDriving:
		return "Driving"
	default:
		return ""
	}
}

func modeName(mode EditorMode) string {
	switch mode {
	case ModeDraw:
		return "Draw"
	case ModeRules:
		return "Rules"
	case ModeRoute:
		return "Route"
	case ModeTraffic:
		return "Traffic"
	case ModePedestrian:
		return "Pedestrian"
	case ModeDriving:
		return "Drive"
	default:
		return "Unknown"
	}
}

func toolName(tool EditorTool) string {
	switch tool {
	case ToolSpline:
		return "Spline"
	case ToolQuadratic:
		return "Quadratic"
	case ToolCut:
		return "Cut"
	case ToolReverse:
		return "Reverse"
	case ToolPriority:
		return "Priority"
	case ToolCouple:
		return "Couple"
	case ToolSpeedLimit:
		return "Speed"
	case ToolPreference:
		return "Prefer"
	case ToolRouteCars:
		return "Cars"
	case ToolRouteBuses:
		return "Buses"
	case ToolRouteEraser:
		return "Erase"
	case ToolTrafficLight:
		return "Traffic"
	case ToolDrive:
		return "Drive"
	case ToolPedestrianPath:
		return "Ped Path"
	default:
		return "Unknown"
	}
}

func hudInfoLines(mode EditorMode, tool EditorTool, stage Stage, draft Draft, quadraticDraft QuadraticDraft, hoveredSpline int, routeStartSplineID, coupleModeFirstID int, debugMode bool, hitboxDebugMode bool, paused bool) []string {
	controls := []string{
		"Global: mouse wheel zooms to cursor, Tab cycles modes, Space pauses, Ctrl+S saves, Ctrl+O loads, F3 toggles profiler.",
	}
	switch tool {
	case ToolSpline:
		controls = append(controls,
			"Spline tool: left click places P0, then P1, then P2, then P3. Right click deletes the hovered spline when idle, cancels the current draft, or steps one stage back.",
			"Continuation: clicking an existing spline endpoint can start from it, mirror a handle, or finish by snapping into another spline endpoint.",
			"Geometry snap: hold Shift to snap the current point to nearby spline-derived axes. Hold Ctrl+Shift for the same axis snap with 4 m steps along the axis.",
			"Perpendicular follow-up: if a point was snapped with Shift, hold Left Alt while confirming it to force the next free point onto a perpendicular axis through that selected point.",
		)
	case ToolQuadratic:
		controls = append(controls,
			"Quadratic tool: left click places P0, then the single middle control point M, then P3. The tool stores a normal cubic internally, so it behaves like the rest of the road network.",
			"Continuation: clicking an existing endpoint can start from it, mirror M from a previous spline, or finish into another spline when the required axes are compatible.",
			"Geometry snap: hold Shift to snap P0, M, or P3 to nearby geometry axes. Hold Ctrl+Shift for axis snapping with 4 m steps. Left Alt on a snapped point constrains the next free point perpendicularly.",
			"If the current combination of previous and next spline axes is impossible for a quadratic spline, the editor will warn instead of creating invalid geometry.",
		)
	case ToolCut:
		controls = append(controls,
			"Cut tool: left click a spline to pick the cut point, then place the tangent handle. Right click cancels.",
			"The spline is split into two new splines that preserve the original curve shape around the cut.",
		)
	case ToolReverse:
		controls = append(controls,
			"Reverse tool: left click a spline to flip its direction. The orange arrows show current direction before you click.",
			"Exclamation marks highlight suspicious nodes where multiple splines start together or end together without an opposite-direction connection.",
		)
	case ToolPriority:
		controls = append(controls,
			"Priority tool: left click marks the hovered spline as priority, right click clears priority.",
			"Priority splines are drawn differently and are favored during conflict handling.",
		)
	case ToolCouple:
		controls = append(controls,
			"Couple tool: left click a spline to select it, then left click another spline to couple or decouple the pair. Right click or Escape cancels the pending selection.",
			"Coupled splines are used by lane changes and lane preference logic.",
		)
	case ToolSpeedLimit:
		controls = append(controls,
			"Speed tool: choose a speed in the panel, then left click splines to apply it. Right click a spline to clear its custom speed limit.",
			"These limits feed directly into vehicle behavior and route timing.",
		)
	case ToolPreference:
		controls = append(controls,
			"Prefer tool: left click assigns the current preference number to the hovered spline.",
			"Right click on empty space advances the current number. Right click a numbered spline removes its assignment.",
			"Preference numbers help vehicles choose between coupled lanes.",
		)
	case ToolRouteCars:
		controls = append(controls,
			"Car routes: left click a spline start endpoint to pick the origin, then left click a spline end endpoint to pick the destination.",
			"The route panel sets spawn rate and color. Existing start/end pairs reopen for editing instead of creating duplicates.",
			"Right click or Escape cancels route picking. Bus-only splines cannot be used in car routes.",
		)
	case ToolRouteBuses:
		controls = append(controls,
			"Bus routes: left click a spline start endpoint, then a spline end endpoint, to open the line panel.",
			"In the panel, add bus stops in travel order by choosing map intersections. Right click or Escape cancels bus-stop placement or route picking.",
			"While idle in this tool, right click a hovered spline to toggle whether it is bus-only.",
		)
	case ToolRouteEraser:
		controls = append(controls,
			"Route eraser: hold left mouse button to delete vehicles inside a 5 m brush.",
			"The brush removes cars and buses directly from the live simulation without changing routes or road geometry.",
		)
	case ToolTrafficLight:
		controls = append(controls,
			"Traffic tool: left click a spline to add a light candidate to the current cycle, or open the cycle panel workflow and then create phases from the selected lights.",
			"Right click a placed light removes it. The cycle panel is used to create, edit, preview, and assign per-phase light states and durations.",
			"Escape backs out of the current traffic editing sub-step before it closes the overall cycle UI.",
		)
	case ToolDrive:
		controls = append(controls,
			"Driving mode: use the arrow keys to accelerate, brake, and steer the player car.",
			"The camera follows the player, keeps the car centered, and rotates the world so the car always points toward the top of the screen.",
			"Right click a spline end to set the current destination node for later route/path logic.",
			"Leaving driving mode removes the player car and restores the normal editor camera orientation.",
		)
	case ToolPedestrianPath:
		controls = append(controls,
			"Pedestrian path tool: left click places the first edge of a 4 m-wide path, second left click places the opposite edge and commits it.",
			"Endpoints snap to existing pedestrian path endpoints within a small radius, so multiple paths can share a junction.",
			"Right click with a draft in progress cancels it; right click on a hovered path removes that path.",
		)
	default:
		controls = append(controls, fmt.Sprintf("%s mode: use the toolbar or shortcuts to switch to a tool for editing.", modeName(mode)))
	}

	_ = hoveredSpline
	_ = routeStartSplineID
	_ = coupleModeFirstID
	_ = debugMode
	_ = hitboxDebugMode
	_ = paused
	return controls
}

func drawInfoBox(title string, x, y, width int32, lines []string) {
	if len(lines) == 0 {
		return
	}
	lineHeight := int32(16)
	height := int32(52) + int32(len(lines))*lineHeight
	rect := rl.NewRectangle(float32(x), float32(y), float32(width), float32(height))
	bg := NewColor(247, 247, 250, 240)
	border := NewColor(202, 206, 216, 255)
	titleCol := NewColor(34, 38, 48, 255)
	textCol := NewColor(66, 72, 86, 255)

	rl.DrawRectangleRec(rect, bg)
	rl.DrawRectangleLinesEx(rect, 1, border)
	drawText(title, x+12, y+10, 18, titleCol)
	for i, line := range lines {
		drawText(line, x+12, y+36+int32(i)*lineHeight, 13, textCol)
	}
}

func drawHud(mode EditorMode, tool EditorTool, stage Stage, draft Draft, quadraticDraft QuadraticDraft, hoveredSpline int, routeStartSplineID int, coupleModeFirstID int, debugMode bool, hitboxDebugMode bool, infoMode bool, paused bool, zoom float32, splineCount, routeCount, carCount int) {
	mouse := rl.GetMousePosition()

	bgNormal := NewColor(245, 245, 248, 245)
	bgActive := NewColor(47, 96, 198, 255)
	bgHover := NewColor(218, 224, 238, 245)
	outNormal := NewColor(200, 200, 206, 255)
	outActive := NewColor(28, 62, 155, 255)
	txtDark := NewColor(28, 28, 33, 255)
	txtMuted := NewColor(100, 100, 110, 255)

	for i, item := range modeToolbarItems {
		r := toolbarBtnRect(i)
		isActive := (!item.isDbg && !item.isHitbox && !item.isInfo && item.mode == mode) ||
			(item.isDbg && debugMode) ||
			(item.isHitbox && hitboxDebugMode) ||
			(item.isInfo && infoMode)
		isHovered := rl.CheckCollisionPointRec(mouse, r)

		bg, out, fg := bgNormal, outNormal, txtDark
		if isActive {
			bg, out, fg = bgActive, outActive, rl.White
		} else if isHovered {
			bg, out, fg = bgHover, outNormal, txtDark
		}

		rl.DrawRectangleRec(r, bg)
		rl.DrawRectangleLinesEx(r, 1, out)

		// Key letter — big, centred
		keyW := measureText(item.key, 36)
		drawText(item.key, int32(r.X)+int32(r.Width)/2-keyW/2, int32(r.Y)+10, 36, fg)

		// Mode label — small, centred near bottom
		lblW := measureText(item.label, 13)
		drawText(item.label, int32(r.X)+int32(r.Width)/2-lblW/2, int32(r.Y)+int32(r.Height)-18, 13, fg)
	}

	for i, item := range toolsForMode(mode) {
		r := toolBtnRect(i)
		isActive := item.tool == tool
		isHovered := rl.CheckCollisionPointRec(mouse, r)

		bg, out, fg := bgNormal, outNormal, txtDark
		if isActive {
			bg, out, fg = bgActive, outActive, rl.White
		} else if isHovered {
			bg, out, fg = bgHover, outNormal, txtDark
		}

		rl.DrawRectangleRec(r, bg)
		rl.DrawRectangleLinesEx(r, 1, out)

		keyW := measureText(item.key, 36)
		drawText(item.key, int32(r.X)+int32(r.Width)/2-keyW/2, int32(r.Y)+10, 36, fg)

		lblW := measureText(item.label, 13)
		drawText(item.label, int32(r.X)+int32(r.Width)/2-lblW/2, int32(r.Y)+int32(r.Height)-18, 13, fg)
	}

	// Stats — top right
	stats := fmt.Sprintf("Splines: %d   Routes: %d   Cars: %d   Zoom: %.1f×", splineCount, routeCount, carCount, zoom)
	statsW := measureText(stats, 13)
	drawText(stats, int32(rl.GetScreenWidth())-statsW-12, 14, 13, txtMuted)

	if !infoMode {
		return
	}

	controlLines := hudInfoLines(mode, tool, stage, draft, quadraticDraft, hoveredSpline, routeStartSplineID, coupleModeFirstID, debugMode, hitboxDebugMode, paused)
	drawInfoBox("Controls", toolbarX, toolbarY+2*toolbarBtnH+toolbarBtnGap+8, 760, controlLines)
}

func drawProfilerOverlay(prof profiler) {
	bg := NewColor(18, 22, 31, 225)
	border := NewColor(60, 76, 112, 255)
	title := NewColor(236, 239, 244, 255)
	text := NewColor(206, 214, 228, 255)
	muted := NewColor(150, 162, 184, 255)

	cur := prof.current
	avg := prof.smooth
	lines := []string{
		fmt.Sprintf("Frame      %6.2f ms   avg %6.2f", cur.frameMS, avg.frameMS),
		fmt.Sprintf("Input      %6.2f ms   avg %6.2f", cur.inputMS, avg.inputMS),
		fmt.Sprintf("Step       %6.2f ms   avg %6.2f", cur.stepMS, avg.stepMS),
		fmt.Sprintf("RouteVis   %6.2f ms   avg %6.2f", cur.routeVisualsMS, avg.routeVisualsMS),
		fmt.Sprintf("LaneChange %6.2f ms   avg %6.2f", cur.laneChangesMS, avg.laneChangesMS),
		fmt.Sprintf("GraphBuild %6.2f ms   avg %6.2f", cur.graphBuildMS, avg.graphBuildMS),
		fmt.Sprintf("Braking    %6.2f ms   avg %6.2f", cur.brakingMS, avg.brakingMS),
		fmt.Sprintf("Following  %6.2f ms   avg %6.2f", cur.followMS, avg.followMS),
		fmt.Sprintf("FolPose    %6.2f ms   avg %6.2f", cur.followDetail.PoseMS, avg.followDetail.PoseMS),
		fmt.Sprintf("FolIndex   %6.2f ms   avg %6.2f", cur.followDetail.IndexMS, avg.followDetail.IndexMS),
		fmt.Sprintf("FolCand    %6.2f ms   avg %6.2f", cur.followDetail.CandidateMS, avg.followDetail.CandidateMS),
		fmt.Sprintf("FolScan    %6.2f ms   avg %6.2f", cur.followDetail.ScanMS, avg.followDetail.ScanMS),
		fmt.Sprintf("FolRefs    %d", cur.followDetail.CandidateRefs),
		fmt.Sprintf("UpdateCars %6.2f ms   avg %6.2f", cur.updateCarsMS, avg.updateCarsMS),
		fmt.Sprintf("UpdSetup   %6.2f ms   avg %6.2f", cur.updateCarsDetail.SetupMS, avg.updateCarsDetail.SetupMS),
		fmt.Sprintf("UpdFast    %6.2f ms   avg %6.2f", cur.updateCarsDetail.FastPathMS, avg.updateCarsDetail.FastPathMS),
		fmt.Sprintf("UpdTrans   %6.2f ms   avg %6.2f", cur.updateCarsDetail.TransitionMS, avg.updateCarsDetail.TransitionMS),
		fmt.Sprintf("UpdMix     dwell %d  fast %d  trans %d  rm %d", cur.updateCarsDetail.DwellCars, cur.updateCarsDetail.FastPathCars, cur.updateCarsDetail.TransitionCars, cur.updateCarsDetail.RemovedCars),
		fmt.Sprintf("Draw       %6.2f ms   avg %6.2f", cur.drawMS, avg.drawMS),
		fmt.Sprintf("Cars       %4d", cur.brakingDetail.Cars),
		fmt.Sprintf("BrkMarshal %6.2f ms   avg %6.2f", cur.brakingDetail.MarshalMS, avg.brakingDetail.MarshalMS),
		fmt.Sprintf("BrkRoute   %6.2f ms   avg %6.2f", cur.brakingDetail.RouteTreeMS, avg.brakingDetail.RouteTreeMS),
		fmt.Sprintf("BrkSetup   %6.2f ms   avg %6.2f", cur.brakingDetail.MarshalSetupMS, avg.brakingDetail.MarshalSetupMS),
		fmt.Sprintf("BrkBase    %6.2f ms   avg %6.2f", cur.brakingDetail.BasePredictMS, avg.brakingDetail.BasePredictMS),
		fmt.Sprintf("BrkScan    %6.2f ms   avg %6.2f", cur.brakingDetail.ConflictScanMS, avg.brakingDetail.ConflictScanMS),
		fmt.Sprintf("BrkEscape  %6.2f ms   avg %6.2f", cur.brakingDetail.BrakeProbeMS, avg.brakingDetail.BrakeProbeMS),
		fmt.Sprintf("BrkHold    %6.2f ms   avg %6.2f", cur.brakingDetail.HoldProbeMS, avg.brakingDetail.HoldProbeMS),
		fmt.Sprintf("BrkFinal   %6.2f ms   avg %6.2f", cur.brakingDetail.FinalizeMS, avg.brakingDetail.FinalizeMS),
		fmt.Sprintf("BrkKernel  %6.2f ms   avg %6.2f", cur.brakingDetail.KernelMS, avg.brakingDetail.KernelMS),
		fmt.Sprintf("BrkUnpack  %6.2f ms   avg %6.2f", cur.brakingDetail.UnmarshalMS, avg.brakingDetail.UnmarshalMS),
		fmt.Sprintf("Preds      base %d  stat %d  esc %d  fast %d", cur.brakingDetail.BasePredictions, cur.brakingDetail.StationaryPredictions, cur.brakingDetail.EscapePredictions, cur.brakingDetail.FasterPredictions),
		fmt.Sprintf("PredSamp   total %d  calls %d", cur.brakingDetail.TotalPredictionSamples, cur.brakingDetail.TotalPredictions),
		fmt.Sprintf("PrimPairs  cand %d  broad %d", cur.brakingDetail.PrimaryPairCandidates, cur.brakingDetail.PrimaryBroadPhasePairs),
		fmt.Sprintf("PrimColl   chk %d  hit %d", cur.brakingDetail.PrimaryCollisionChecks, cur.brakingDetail.PrimaryCollisionHits),
		fmt.Sprintf("Rechecks   stat %d/%d  esc %d/%d  hold %d/%d", cur.brakingDetail.StationaryCollisionChecks, cur.brakingDetail.StationaryCollisionHits, cur.brakingDetail.EscapeCollisionChecks, cur.brakingDetail.EscapeCollisionHits, cur.brakingDetail.HoldCollisionChecks, cur.brakingDetail.HoldCollisionHits),
		fmt.Sprintf("BrakeFlags blamed %d  brake %d  hold %d", cur.brakingDetail.InitiallyBlamedCars, cur.brakingDetail.BrakingCars, cur.brakingDetail.HoldCars),
		fmt.Sprintf("Base path cache  hits %d  miss %d", cur.basePathHits, cur.basePathMisses),
		fmt.Sprintf("All  path cache  hits %d  miss %d", cur.allPathHits, cur.allPathMisses),
	}

	rectHeight := float32(54 + len(lines)*15)
	rect := rl.NewRectangle(float32(rl.GetScreenWidth())-420, 42, 400, rectHeight)
	rl.DrawRectangleRec(rect, bg)
	rl.DrawRectangleLinesEx(rect, 1, border)

	drawText("Profiler", int32(rect.X)+12, int32(rect.Y)+10, 18, title)
	drawText("F3 to toggle", int32(rect.X)+190, int32(rect.Y)+13, 12, muted)

	for i, line := range lines {
		drawText(line, int32(rect.X)+12, int32(rect.Y)+38+int32(i*15), 12, text)
	}
}

func stageLabel(stage Stage, draft Draft) string {
	switch stage {
	case StageIdle:
		return "Idle"
	case StageSetP1:
		if draft.LockP1 {
			return "Choose P1 on the previous spline axis"
		}
		return "Choose P1"
	case StageSetP2:
		return "Choose P2 or click a next spline start"
	case StageSetP3:
		if draft.SnapP3 {
			return "Choose P2 on the next spline axis"
		}
		return "Choose P3"
	default:
		return "Unknown"
	}
}

func newDraft() Draft {
	return Draft{ContinuationFrom: -1}
}

func splineDrawColor(s Spline) Color {
	if s.Priority && s.BusOnly {
		return NewColor(95, 125, 195, 255)
	}
	if s.Priority {
		return NewColor(130, 75, 215, 255)
	}
	if s.BusOnly {
		return NewColor(28, 145, 125, 255)
	}
	return NewColor(35, 85, 175, 255)
}

func buildQuadraticPreview(stage Stage, draft QuadraticDraft, mouse Vec2, geometrySnap GeometrySnap, hoveredNode EndHit, splines []Spline) (Spline, bool) {
	switch stage {
	case StageSetP1:
		return newQuadraticSpline(-1, draft.P0, quadraticMOnPrevAxis(draft, mouse, geometrySnap), mouse), true
	case StageSetP2:
		if draft.SnapP3 {
			return newQuadraticSpline(-1, draft.P0, quadraticMOnNextAxis(draft, mouse, geometrySnap), draft.P3), true
		}
		if hoveredNode.SplineIndex >= 0 {
			next := splines[hoveredNode.SplineIndex]
			p3 := endpointAnchor(next, hoveredNode.Kind)
			if draft.MMirroredFromPrev {
				return newQuadraticSpline(-1, draft.P0, draft.M, mouse), true
			}
			if draft.FromPrevAxis {
				if m, ok := lineIntersection(draft.P0, draft.PrevAxisDir, p3, endpointAxisDir(next, hoveredNode.Kind)); ok {
					return newQuadraticSpline(-1, draft.P0, m, p3), true
				}
				return newQuadraticSpline(-1, draft.P0, draft.M, mouse), true
			}
			return newQuadraticSpline(-1, draft.P0, mirroredHandleAtEndpoint(next, hoveredNode.Kind), p3), true
		}
		return newQuadraticSpline(-1, draft.P0, draft.M, quadraticP3Preview(draft, mouse, geometrySnap)), true
	default:
		return Spline{}, false
	}
}

func drawQuadraticDraft(stage Stage, draft QuadraticDraft, mouse Vec2, zoom float32, geometrySnap GeometrySnap) {
	lineThickness := pixelsToWorld(zoom, 1.5)
	handleRadius := pixelsToWorld(zoom, handlePixels)
	guide := NewColor(140, 140, 140, 255)
	locked := NewColor(130, 75, 215, 255)

	drawEndpoint(draft.P0, handleRadius, NewColor(215, 67, 67, 255))

	switch stage {
	case StageSetP1:
		m := quadraticMOnPrevAxis(draft, mouse, geometrySnap)
		color := guide
		if draft.FromPrevAxis {
			color = locked
		}
		drawLineEx(draft.P0, m, lineThickness, color)
		drawLineEx(m, mouse, lineThickness, guide)
		drawEndpoint(m, handleRadius, color)
		drawEndpoint(mouse, handleRadius, NewColor(35, 85, 175, 255))
	case StageSetP2:
		m := draft.M
		p3 := quadraticP3Preview(draft, mouse, geometrySnap)
		if draft.SnapP3 {
			m = quadraticMOnNextAxis(draft, mouse, geometrySnap)
			p3 = draft.P3
		}
		drawLineEx(draft.P0, m, lineThickness, guide)
		drawLineEx(m, p3, lineThickness, guide)
		drawEndpoint(m, handleRadius, mapBoolColor(draft.FromPrevAxis || draft.SnapP3, locked, guide))
		drawEndpoint(p3, handleRadius, NewColor(35, 85, 175, 255))
	}
}

func drawQuadraticDraftInfo(stage Stage, draft QuadraticDraft, mouseWorld Vec2, geometrySnap GeometrySnap, preview Spline, camera rl.Camera2D) {
	switch stage {
	case StageSetP1:
		drawSegmentLabel(draft.P0, quadraticMOnPrevAxis(draft, mouseWorld, geometrySnap), camera)
		drawArcLabel(preview, camera)
	case StageSetP2:
		if draft.SnapP3 {
			drawSegmentLabel(quadraticMOnNextAxis(draft, mouseWorld, geometrySnap), draft.P3, camera)
		} else {
			drawSegmentLabel(draft.M, quadraticP3Preview(draft, mouseWorld, geometrySnap), camera)
		}
		drawArcLabel(preview, camera)
	}
}

func drawQuadraticSnapHint(stage Stage, draft QuadraticDraft, hoveredNode EndHit, splines []Spline, mouse Vec2, zoom float32) {
	anchorColor := NewColor(255, 196, 61, 220)
	ghostColor := NewColor(163, 92, 255, 220)

	switch stage {
	case StageIdle:
		if hoveredNode.SplineIndex < 0 {
			return
		}
		prev := splines[hoveredNode.SplineIndex]
		drawAxisHint(endpointAnchor(prev, hoveredNode.Kind), endpointAxisDir(prev, hoveredNode.Kind), mouse, zoom, anchorColor)
	case StageSetP1:
		if hoveredNode.SplineIndex >= 0 {
			next := splines[hoveredNode.SplineIndex]
			nextAnchor := endpointAnchor(next, hoveredNode.Kind)
			nextAxis := endpointAxisDir(next, hoveredNode.Kind)
			if draft.FromPrevAxis {
				if m, ok := lineIntersection(draft.P0, draft.PrevAxisDir, nextAnchor, nextAxis); ok {
					drawRing(nextAnchor, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, anchorColor)
					drawEndpoint(m, pixelsToWorld(zoom, 4.5), ghostColor)
				}
				return
			}
			drawAxisHint(nextAnchor, nextAxis, mouse, zoom, ghostColor)
			return
		}
		if draft.FromPrevAxis {
			drawAxisHint(draft.P0, draft.PrevAxisDir, mouse, zoom, ghostColor)
			return
		}
		if draft.PerpLockActive {
			drawAxisHint(draft.PerpLockOrigin, draft.PerpLockAxisDir, mouse, zoom, ghostColor)
			return
		}
		if hoveredNode.SplineIndex < 0 {
			return
		}
		prev := splines[hoveredNode.SplineIndex]
		anchor := endpointAnchor(prev, hoveredNode.Kind)
		m := mirroredHandleAtEndpoint(prev, hoveredNode.Kind)
		drawLineEx(anchor, m, pixelsToWorld(zoom, 1.5), anchorColor)
		drawRing(anchor, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, anchorColor)
		drawEndpoint(m, pixelsToWorld(zoom, 4.5), ghostColor)
	case StageSetP2:
		if draft.SnapP3 {
			drawAxisHint(draft.P3, draft.NextAxisDir, mouse, zoom, ghostColor)
			return
		}
		if draft.PerpLockActive {
			drawAxisHint(draft.PerpLockOrigin, draft.PerpLockAxisDir, mouse, zoom, ghostColor)
		}
		if hoveredNode.SplineIndex < 0 {
			return
		}
		next := splines[hoveredNode.SplineIndex]
		anchor := endpointAnchor(next, hoveredNode.Kind)
		drawRing(anchor, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, anchorColor)
		if draft.MMirroredFromPrev {
			return
		}
		if draft.FromPrevAxis {
			if m, ok := lineIntersection(draft.P0, draft.PrevAxisDir, anchor, endpointAxisDir(next, hoveredNode.Kind)); ok {
				drawEndpoint(m, pixelsToWorld(zoom, 4.5), ghostColor)
			}
			return
		}
		m := mirroredHandleAtEndpoint(next, hoveredNode.Kind)
		drawLineEx(m, anchor, pixelsToWorld(zoom, 1.5), ghostColor)
		drawEndpoint(m, pixelsToWorld(zoom, 4.5), ghostColor)
	}
}

func buildPreview(stage Stage, draft Draft, mouse Vec2, geometrySnap GeometrySnap, hoveredNode EndHit, splines []Spline) (Spline, bool) {
	switch stage {
	case StageSetP1:
		return simpkg.NewSpline(-1, draft.P0, draftP1Preview(draft, mouse, geometrySnap), mouse, mouse), true
	case StageSetP2:
		p2 := draftP2FreePreview(draft, mouse, geometrySnap)
		return simpkg.NewSpline(-1, draft.P0, draft.P1, p2, p2), true
	case StageSetP3:
		if draft.SnapP3 {
			return simpkg.NewSpline(-1, draft.P0, draft.P1, draftP2Preview(draft, mouse, geometrySnap), draft.P3), true
		}
		if hoveredNode.SplineIndex >= 0 {
			next := splines[hoveredNode.SplineIndex]
			p3 := endpointAnchor(next, hoveredNode.Kind)
			p2 := mirroredHandleAtEndpoint(next, hoveredNode.Kind)
			return simpkg.NewSpline(-1, draft.P0, draft.P1, p2, p3), true
		}
		return simpkg.NewSpline(-1, draft.P0, draft.P1, draft.P2, draftP3Preview(draft, mouse, geometrySnap)), true
	default:
		return Spline{}, false
	}
}

func drawDraft(stage Stage, draft Draft, mouse Vec2, zoom float32, geometrySnap GeometrySnap) {
	lineThickness := pixelsToWorld(zoom, 1.5)
	handleRadius := pixelsToWorld(zoom, handlePixels)
	guide := NewColor(140, 140, 140, 255)
	locked := NewColor(130, 75, 215, 255)

	drawEndpoint(draft.P0, handleRadius, NewColor(215, 67, 67, 255))

	switch stage {
	case StageSetP1:
		if draft.LockP1 {
			p1 := draftP1Preview(draft, mouse, geometrySnap)
			drawLineEx(draft.P0, p1, lineThickness, locked)
			drawEndpoint(p1, handleRadius, locked)
		} else {
			drawLineEx(draft.P0, mouse, lineThickness, guide)
			drawEndpoint(mouse, handleRadius, guide)
		}
	case StageSetP2:
		p2 := draftP2FreePreview(draft, mouse, geometrySnap)
		drawLineEx(draft.P0, draft.P1, lineThickness, guide)
		drawLineEx(draft.P1, p2, lineThickness, guide)
		drawEndpoint(draft.P1, handleRadius, mapBoolColor(draft.LockP1, locked, guide))
		drawEndpoint(p2, handleRadius, guide)
	case StageSetP3:
		p2 := draft.P2
		p3 := draftP3Preview(draft, mouse, geometrySnap)
		if draft.SnapP3 {
			p2 = draftP2Preview(draft, mouse, geometrySnap)
			p3 = draft.P3
		}
		drawLineEx(draft.P0, draft.P1, lineThickness, guide)
		drawLineEx(draft.P1, p2, lineThickness, guide)
		drawLineEx(p2, p3, lineThickness, guide)
		drawEndpoint(draft.P1, handleRadius, mapBoolColor(draft.LockP1, locked, guide))
		drawEndpoint(p2, handleRadius, guide)
		drawEndpoint(p3, handleRadius, NewColor(35, 85, 175, 255))
	}
}

func drawAxisHint(origin, axisDir, reference Vec2, zoom float32, color Color) Vec2 {
	if vectorLengthSq(axisDir) <= 1e-9 {
		drawEndpoint(origin, pixelsToWorld(zoom, 5), color)
		return origin
	}
	projected := projectPointOntoAxis(reference, origin, axisDir)
	dir := normalize(axisDir)
	dist := float32(math.Sqrt(float64(distSq(origin, projected))))
	arm := maxf(pixelsToWorld(zoom, 34), dist)
	start := vecSub(origin, vecScale(dir, arm))
	end := vecAdd(origin, vecScale(dir, arm))
	drawLineEx(start, end, pixelsToWorld(zoom, 1.5), color)
	drawRing(origin, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, color)
	drawEndpoint(projected, pixelsToWorld(zoom, 4.5), color)
	return projected
}

func drawGeometrySnapHint(geometrySnap GeometrySnap, reference Vec2, zoom float32) {
	if !geometrySnap.Active {
		return
	}
	color := NewColor(74, 196, 186, 220)
	if vectorLengthSq(geometrySnap.AxisDir) <= 1e-9 {
		drawEndpoint(geometrySnap.Origin, pixelsToWorld(zoom, 5), color)
		return
	}
	dir := normalize(geometrySnap.AxisDir)
	projected := geometrySnap.Point
	arm := maxf(pixelsToWorld(zoom, 34), float32(math.Sqrt(float64(distSq(geometrySnap.Origin, projected)))))
	start := vecSub(geometrySnap.Origin, vecScale(dir, arm))
	end := vecAdd(geometrySnap.Origin, vecScale(dir, arm))
	if geometrySnap.HasMinT && !geometrySnap.HasMaxT && absf(geometrySnap.MinT) <= 1e-6 {
		start = geometrySnap.Origin
		end = vecAdd(geometrySnap.Origin, vecScale(dir, arm))
	}
	if geometrySnap.HasMaxT && !geometrySnap.HasMinT && absf(geometrySnap.MaxT) <= 1e-6 {
		start = vecSub(geometrySnap.Origin, vecScale(dir, arm))
		end = geometrySnap.Origin
	}
	drawLineEx(start, end, pixelsToWorld(zoom, 1.5), color)
	drawRing(geometrySnap.Origin, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, color)
	drawEndpoint(projected, pixelsToWorld(zoom, 4.5), color)
}

func drawEditSnapHint(stage Stage, draft Draft, hoveredNode EndHit, splines []Spline, mouse Vec2, zoom float32) {
	anchorColor := NewColor(255, 196, 61, 220)
	ghostColor := NewColor(163, 92, 255, 220)

	switch stage {
	case StageIdle:
		if hoveredNode.SplineIndex < 0 {
			return
		}
		prev := splines[hoveredNode.SplineIndex]
		drawAxisHint(endpointAnchor(prev, hoveredNode.Kind), endpointAxisDir(prev, hoveredNode.Kind), mouse, zoom, anchorColor)
	case StageSetP1:
		if draft.LockP1 {
			drawAxisHint(draft.P0, draft.P1AxisDir, mouse, zoom, ghostColor)
			return
		}
		if draft.PerpLockActive {
			drawAxisHint(draft.PerpLockOrigin, draft.PerpLockAxisDir, mouse, zoom, ghostColor)
			return
		}
		if hoveredNode.SplineIndex < 0 {
			return
		}
		prev := splines[hoveredNode.SplineIndex]
		anchor := endpointAnchor(prev, hoveredNode.Kind)
		p1 := mirroredHandleAtEndpoint(prev, hoveredNode.Kind)
		drawLineEx(anchor, p1, pixelsToWorld(zoom, 1.5), anchorColor)
		drawRing(anchor, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, anchorColor)
		drawEndpoint(p1, pixelsToWorld(zoom, 4.5), ghostColor)
	case StageSetP2:
		if draft.PerpLockActive {
			drawAxisHint(draft.PerpLockOrigin, draft.PerpLockAxisDir, mouse, zoom, ghostColor)
		}
		if hoveredNode.SplineIndex < 0 {
			return
		}
		next := splines[hoveredNode.SplineIndex]
		drawAxisHint(endpointAnchor(next, hoveredNode.Kind), endpointAxisDir(next, hoveredNode.Kind), mouse, zoom, ghostColor)
	case StageSetP3:
		if draft.SnapP3 {
			drawAxisHint(draft.P3, draft.P2AxisDir, mouse, zoom, ghostColor)
			return
		}
		if draft.PerpLockActive {
			drawAxisHint(draft.PerpLockOrigin, draft.PerpLockAxisDir, mouse, zoom, ghostColor)
		}
		if hoveredNode.SplineIndex < 0 {
			return
		}
		next := splines[hoveredNode.SplineIndex]
		anchor := endpointAnchor(next, hoveredNode.Kind)
		p2 := mirroredHandleAtEndpoint(next, hoveredNode.Kind)
		drawLineEx(p2, anchor, pixelsToWorld(zoom, 1.5), ghostColor)
		drawEndpoint(p2, pixelsToWorld(zoom, 4.5), ghostColor)
		drawRing(anchor, pixelsToWorld(zoom, 5.5), pixelsToWorld(zoom, 7.5), 0, 360, 20, anchorColor)
	}
}

// segmentAngleDeg returns the clockwise angle from East for the vector from→to,
// normalised to [0, 360). Screen Y is down, so this matches screen intuition.
func segmentAngleDeg(from, to Vec2) float32 {
	dx := to.X - from.X
	dy := to.Y - from.Y
	a := float32(math.Atan2(float64(dy), float64(dx)) * 180 / math.Pi)
	if a < 0 {
		a += 360
	}
	return a
}

// drawSegmentLabel draws a length + angle label at the midpoint of a segment,
// offset perpendicular to the segment so it doesn't overlap the line.
func drawSegmentLabel(from, to Vec2, camera rl.Camera2D) {
	mx := (from.X + to.X) / 2
	my := (from.Y + to.Y) / 2
	mid := getWorldToScreen2D(NewVec2(mx, my), camera)

	dx := to.X - from.X
	dy := to.Y - from.Y
	length := float32(math.Sqrt(float64(dx*dx+dy*dy))) * metersPerUnit
	angle := segmentAngleDeg(from, to)

	// Perpendicular offset in screen space (rotate segment dir 90° left).
	segScreenFrom := getWorldToScreen2D(from, camera)
	segScreenTo := getWorldToScreen2D(to, camera)
	sdx := segScreenTo.X - segScreenFrom.X
	sdy := segScreenTo.Y - segScreenFrom.Y
	slen := float32(math.Sqrt(float64(sdx*sdx + sdy*sdy)))
	ox, oy := float32(0), float32(-14) // default: shift up
	if slen > 0.01 {
		// Unit perpendicular (rotate 90° CCW in screen space)
		px := -sdy / slen
		py := sdx / slen
		ox = px * 14
		oy = py * 14
	}

	text := fmt.Sprintf("%.1f m  %.0f°", length, angle)
	tw := measureText(text, 14) + 6
	tx := int32(mid.X+ox) - tw/2
	ty := int32(mid.Y+oy) - 9

	rl.DrawRectangle(tx-2, ty-2, tw+4, 20, NewColor(30, 30, 35, 180))
	drawText(text, tx+3, ty+1, 14, rl.White)
}

// drawArcLabel draws the arc length at the midpoint of the preview spline.
func drawArcLabel(preview Spline, camera rl.Camera2D) {
	if preview.Length <= 0 {
		return
	}
	midPos, tangent := simpkg.SampleSplineAtDistance(preview, preview.Length/2)
	screen := getWorldToScreen2D(midPos, camera)

	// Offset perpendicular to tangent so the label doesn't sit on the curve.
	px := -tangent.Y
	py := tangent.X
	ox := px * 16
	oy := py * 16

	text := fmt.Sprintf("arc %.1f m", preview.Length*metersPerUnit)
	tw := measureText(text, 14) + 6
	tx := int32(screen.X+ox) - tw/2
	ty := int32(screen.Y+oy) - 9

	rl.DrawRectangle(tx-2, ty-2, tw+4, 20, NewColor(20, 80, 160, 200))
	drawText(text, tx+3, ty+1, 14, NewColor(180, 220, 255, 255))
}

// drawDraftInfo draws measurement labels directly on the draft segments and arc.
func drawDraftInfo(stage Stage, draft Draft, mouseWorld Vec2, geometrySnap GeometrySnap, preview Spline, camera rl.Camera2D) {
	switch stage {
	case StageSetP1:
		if draft.LockP1 {
			drawSegmentLabel(draft.P0, draftP1Preview(draft, mouseWorld, geometrySnap), camera)
			drawArcLabel(preview, camera)
		} else {
			drawSegmentLabel(draft.P0, mouseWorld, camera)
		}
	case StageSetP2:
		drawSegmentLabel(draft.P1, draftP2FreePreview(draft, mouseWorld, geometrySnap), camera)
		drawArcLabel(preview, camera)
	case StageSetP3:
		if draft.SnapP3 {
			drawSegmentLabel(draftP2Preview(draft, mouseWorld, geometrySnap), draft.P3, camera)
		} else {
			drawSegmentLabel(draft.P2, draftP3Preview(draft, mouseWorld, geometrySnap), camera)
		}
		drawArcLabel(preview, camera)
	}
}

func mapBoolColor(condition bool, whenTrue, whenFalse Color) Color {
	if condition {
		return whenTrue
	}
	return whenFalse
}

func cameraScreenWorldBounds(camera rl.Camera2D) (minX, minY, maxX, maxY float32) {
	screenW := float32(rl.GetScreenWidth())
	screenH := float32(rl.GetScreenHeight())
	corners := [4]rl.Vector2{
		rl.NewVector2(0, 0),
		rl.NewVector2(screenW, 0),
		rl.NewVector2(0, screenH),
		rl.NewVector2(screenW, screenH),
	}
	first := rl.GetScreenToWorld2D(corners[0], camera)
	minX, maxX = first.X, first.X
	minY, maxY = first.Y, first.Y
	for i := 1; i < len(corners); i++ {
		p := rl.GetScreenToWorld2D(corners[i], camera)
		minX = minf(minX, p.X)
		maxX = maxf(maxX, p.X)
		minY = minf(minY, p.Y)
		maxY = maxf(maxY, p.Y)
	}
	return minX, minY, maxX, maxY
}

func drawGrid(camera rl.Camera2D) {
	minX, minY, maxX, maxY := cameraScreenWorldBounds(camera)
	zoom := camera.Zoom

	type level struct {
		spacing float32
		color   Color
		pxThick float32 // constant screen-pixel thickness
		showMin float32 // only show when zoom >= showMin (0 = no minimum)
		showMax float32 // only show when zoom <= showMax (0 = no maximum)
	}

	// Draw thinnest first so thicker lines paint on top.
	levels := []level{
		{4, NewColor(220, 220, 226, 255), 1.0, 6.0, 0},
		{20, NewColor(185, 185, 196, 255), 1.5, 0.8, 0},
		{100, NewColor(148, 148, 162, 255), 2.0, 0, 0},
	}

	for _, lvl := range levels {
		if lvl.showMin > 0 && zoom < lvl.showMin {
			continue
		}
		if lvl.showMax > 0 && zoom > lvl.showMax {
			continue
		}
		thick := pixelsToWorld(zoom, lvl.pxThick)
		sp := lvl.spacing
		for x := float32(math.Floor(float64(minX/sp))) * sp; x <= maxX; x += sp {
			drawLineEx(NewVec2(x, minY), NewVec2(x, maxY), thick, lvl.color)
		}
		for y := float32(math.Floor(float64(minY/sp))) * sp; y <= maxY; y += sp {
			drawLineEx(NewVec2(minX, y), NewVec2(maxX, y), thick, lvl.color)
		}
	}
}

func drawAxes(camera rl.Camera2D) {
	minX, minY, maxX, maxY := cameraScreenWorldBounds(camera)

	axis := NewColor(180, 180, 185, 255)
	drawLineV(NewVec2(0, minY), NewVec2(0, maxY), axis)
	drawLineV(NewVec2(minX, 0), NewVec2(maxX, 0), axis)
}

// formatDistance formats a distance in metres to a human-readable string.
func formatDistance(metres float32) string {
	if metres >= 1000 {
		km := metres / 1000
		if km == float32(int(km)) {
			return fmt.Sprintf("%d km", int(km))
		}
		return fmt.Sprintf("%.1f km", km)
	}
	return fmt.Sprintf("%d m", int(metres))
}

// drawScaleBar draws a fixed-size scale indicator in the bottom-left corner of the screen.
// It shows what the current major grid spacing represents in real-world metres.
func drawScaleBar(zoom float32) {
	major := chooseGridSpacing(zoom) * metersPerUnit // real-world metres per major cell
	barPx := major * zoom / metersPerUnit            // screen pixels for one major cell

	margin := float32(20)
	screenH := float32(rl.GetScreenHeight())
	barY := screenH - margin - 8
	barX := margin

	label := formatDistance(major)
	barColor := NewColor(50, 50, 55, 220)

	// Horizontal bar with end ticks
	drawLineEx(NewVec2(barX, barY), NewVec2(barX+barPx, barY), 2, barColor)
	drawLineEx(NewVec2(barX, barY-5), NewVec2(barX, barY+5), 2, barColor)
	drawLineEx(NewVec2(barX+barPx, barY-5), NewVec2(barX+barPx, barY+5), 2, barColor)

	labelW := measureText(label, 16)
	drawText(label, int32(barX+barPx/2)-labelW/2, int32(barY-20), 16, barColor)
}

func chooseGridSpacing(zoom float32) float32 {
	desired := 120.0 / float64(zoom)
	base := math.Pow(10, math.Floor(math.Log10(desired)))
	for _, mult := range []float64{1, 2, 5, 10} {
		spacing := mult * base
		if spacing >= desired {
			return float32(spacing)
		}
	}
	return float32(10 * base)
}

func zoomCameraToMouse(camera *rl.Camera2D, wheel float32) {
	mouse := rl.GetMousePosition()
	before := rl.GetScreenToWorld2D(mouse, *camera)

	factor := float32(math.Pow(1.18, float64(wheel)))
	camera.Zoom = clampf(camera.Zoom*factor, minZoom, maxZoom)

	after := rl.GetScreenToWorld2D(mouse, *camera)
	camera.Target = toRLVec2(vecAdd(fromRLVec2(camera.Target), vecSub(fromRLVec2(before), fromRLVec2(after))))
}

func drawSpline(s Spline, thickness float32, color Color) {
	prev := s.P0
	for i := 1; i <= curveSamples; i++ {
		t := float32(i) / float32(curveSamples)
		curr := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)
		drawLineEx(prev, curr, thickness, color)
		prev = curr
	}
}

func drawSplineDirectionArrow(s Spline, zoom float32, color Color) {
	if s.Length <= 0 {
		return
	}
	center, tangent := simpkg.SampleSplineAtDistance(s, s.Length*0.5)
	arrowLen := pixelsToWorld(zoom, 16)
	arrowHalfWidth := pixelsToWorld(zoom, 6)
	tip := vecAdd(center, vecScale(tangent, arrowLen*0.5))
	base := vecSub(center, vecScale(tangent, arrowLen*0.5))
	normal := Vec2{X: tangent.Y, Y: -tangent.X}
	left := vecAdd(base, vecScale(normal, arrowHalfWidth))
	right := vecSub(base, vecScale(normal, arrowHalfWidth))
	drawLineEx(left, tip, pixelsToWorld(zoom, 2.2), color)
	drawLineEx(right, tip, pixelsToWorld(zoom, 2.2), color)
	drawLineEx(left, right, pixelsToWorld(zoom, 1.8), color)
}

func drawSplineDirectionArrows(splines []Spline, zoom float32, viewRect worldRect) {
	color := NewColor(255, 120, 40, 230)
	for _, spline := range splines {
		if !splineVisibleInWorldRect(spline, viewRect) {
			continue
		}
		drawSplineDirectionArrow(spline, zoom, color)
	}
}

func drawEndpoint(p Vec2, radius float32, color Color) {
	drawCircleV(p, radius, color)
}

func drawDirectionWarnings(warnings []DirectionWarning, zoom float32, viewRect worldRect) {
	for _, warning := range warnings {
		r := pixelsToWorld(zoom, 10)
		if !circleVisibleInWorldRect(warning.Point, r*1.15, viewRect) {
			continue
		}
		drawCircleV(warning.Point, r, NewColor(255, 236, 150, 235))
		drawRing(warning.Point, r*0.75, r*1.15, 0, 360, 20, NewColor(230, 120, 20, 230))
	}
}

func drawDirectionWarningLabels(warnings []DirectionWarning, camera rl.Camera2D, viewRect worldRect) {
	for _, warning := range warnings {
		if !worldRectContainsPoint(viewRect, warning.Point) {
			continue
		}
		screen := getWorldToScreen2D(warning.Point, camera)
		drawText("!", int32(screen.X)-4, int32(screen.Y)-11, 18, NewColor(170, 60, 10, 255))
	}
}

func endpointHoverScore(mouse, anchor, axisDir Vec2) float32 {
	offset := vecSub(mouse, anchor)
	if vectorLengthSq(offset) <= 1e-9 || vectorLengthSq(axisDir) <= 1e-9 {
		return 0
	}
	return dot(normalize(offset), normalize(axisDir))
}

func findNearbyEnd(splines []Spline, point Vec2, radius float32) EndHit {
	best := EndHit{SplineIndex: -1, SplineID: -1, Kind: EndNone}
	bestDistSq := radius * radius
	bestScore := float32(-2)
	const distTieEps float32 = 1e-4
	for i, spline := range splines {
		anchor := spline.P3
		d := distSq(point, anchor)
		score := endpointHoverScore(point, anchor, endpointAxisDir(spline, EndFinish))
		if d < bestDistSq-distTieEps || (absf(d-bestDistSq) <= distTieEps && score > bestScore) {
			bestDistSq = d
			bestScore = score
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndFinish, Point: anchor}
		}
	}
	return best
}

func findNearbyStart(splines []Spline, point Vec2, radius float32) EndHit {
	best := EndHit{SplineIndex: -1, SplineID: -1, Kind: EndNone}
	bestDistSq := radius * radius
	bestScore := float32(-2)
	const distTieEps float32 = 1e-4
	for i, spline := range splines {
		anchor := spline.P0
		d := distSq(point, anchor)
		score := endpointHoverScore(point, anchor, endpointAxisDir(spline, EndStart))
		if d < bestDistSq-distTieEps || (absf(d-bestDistSq) <= distTieEps && score > bestScore) {
			bestDistSq = d
			bestScore = score
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndStart, Point: anchor}
		}
	}
	return best
}

func findNearbyEndpoint(splines []Spline, point Vec2, radius float32) EndHit {
	best := EndHit{SplineIndex: -1, SplineID: -1, Kind: EndNone}
	bestDistSq := radius * radius
	bestScore := float32(-2)
	const distTieEps float32 = 1e-4
	for i, spline := range splines {
		startAnchor := spline.P0
		if d := distSq(point, startAnchor); d < bestDistSq-distTieEps || (absf(d-bestDistSq) <= distTieEps && endpointHoverScore(point, startAnchor, endpointAxisDir(spline, EndStart)) > bestScore) {
			bestDistSq = d
			bestScore = endpointHoverScore(point, startAnchor, endpointAxisDir(spline, EndStart))
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndStart, Point: startAnchor}
		}
		finishAnchor := spline.P3
		if d := distSq(point, finishAnchor); d < bestDistSq-distTieEps || (absf(d-bestDistSq) <= distTieEps && endpointHoverScore(point, finishAnchor, endpointAxisDir(spline, EndFinish)) > bestScore) {
			bestDistSq = d
			bestScore = endpointHoverScore(point, finishAnchor, endpointAxisDir(spline, EndFinish))
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndFinish, Point: finishAnchor}
		}
	}
	return best
}

func findHoveredSpline(splines []Spline, point Vec2, radius float32) int {
	bestIndex := -1
	bestDistSq := radius * radius
	for i, spline := range splines {
		d := splineDistanceSq(spline, point)
		if d <= bestDistSq {
			bestDistSq = d
			bestIndex = i
		}
	}
	return bestIndex
}

func splineDistanceSq(s Spline, point Vec2) float32 {
	best := float32(math.MaxFloat32)
	prev := s.P0
	for i := 1; i <= hoverSamples; i++ {
		t := float32(i) / float32(hoverSamples)
		curr := bezierPoint(s.P0, s.P1, s.P2, s.P3, t)
		d := pointSegmentDistanceSq(point, prev, curr)
		if d < best {
			best = d
		}
		prev = curr
	}
	return best
}

func pointSegmentDistanceSq(p, a, b Vec2) float32 {
	ab := vecSub(b, a)
	ap := vecSub(p, a)
	denom := dot(ab, ab)
	if denom == 0 {
		return distSq(p, a)
	}
	t := clampf(dot(ap, ab)/denom, 0, 1)
	closest := vecAdd(a, vecScale(ab, t))
	return distSq(p, closest)
}

func findRouteID(routes []Route, startSplineID, endSplineID int, vehicleKind VehicleKind) int {
	for _, route := range routes {
		if route.StartSplineID == startSplineID && route.EndSplineID == endSplineID && route.VehicleKind == vehicleKind {
			return route.ID
		}
	}
	return -1
}

func findRouteIndexByID(routes []Route, id int) int {
	for i, route := range routes {
		if route.ID == id {
			return i
		}
	}
	return -1
}

func removeRouteByID(routes []Route, id int) []Route {
	for i, route := range routes {
		if route.ID == id {
			return append(routes[:i], routes[i+1:]...)
		}
	}
	return routes
}

func removeCarsForRoute(cars []Car, routeID int) []Car {
	kept := cars[:0]
	for _, car := range cars {
		if car.RouteID != routeID {
			kept = append(kept, car)
		}
	}
	return kept
}

func isCtrlDown() bool {
	return rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl)
}

func nodeKeyFromVec2(v Vec2) simpkg.NodeKey {
	return simpkg.NodeKey{
		int32(math.Round(float64(v.X * 100))),
		int32(math.Round(float64(v.Y * 100))),
	}
}

func pointInRect(p Vec2, r rl.Rectangle) bool {
	return p.X >= r.X && p.X <= r.X+r.Width && p.Y >= r.Y && p.Y <= r.Y+r.Height
}

func cameraWorldRect(camera rl.Camera2D, pad float32) worldRect {
	minX, minY, maxX, maxY := cameraScreenWorldBounds(camera)
	return worldRect{
		MinX: minX - pad,
		MinY: minY - pad,
		MaxX: maxX + pad,
		MaxY: maxY + pad,
	}
}

func worldRectContainsPoint(r worldRect, p Vec2) bool {
	return p.X >= r.MinX && p.X <= r.MaxX && p.Y >= r.MinY && p.Y <= r.MaxY
}

func worldRectIntersectsAABB(r worldRect, minX, minY, maxX, maxY float32) bool {
	return maxX >= r.MinX && minX <= r.MaxX && maxY >= r.MinY && minY <= r.MaxY
}

func circleVisibleInWorldRect(center Vec2, radius float32, r worldRect) bool {
	return worldRectIntersectsAABB(r, center.X-radius, center.Y-radius, center.X+radius, center.Y+radius)
}

func splineVisibleInWorldRect(s Spline, r worldRect) bool {
	minX := minf(minf(s.P0.X, s.P1.X), minf(s.P2.X, s.P3.X))
	minY := minf(minf(s.P0.Y, s.P1.Y), minf(s.P2.Y, s.P3.Y))
	maxX := maxf(maxf(s.P0.X, s.P1.X), maxf(s.P2.X, s.P3.X))
	maxY := maxf(maxf(s.P0.Y, s.P1.Y), maxf(s.P2.Y, s.P3.Y))
	return worldRectIntersectsAABB(r, minX, minY, maxX, maxY)
}

func routeSpawnSliderStep(vehicleKind VehicleKind) float32 {
	if vehicleKind == VehicleBus {
		return 0.5
	}
	return 3.0
}

func sliderValueFromMouse(mouseX float32, rect rl.Rectangle, minValue, maxValue, step float32) float32 {
	t := clampf((mouseX-rect.X)/rect.Width, 0, 1)
	value := minValue + t*(maxValue-minValue)
	if step <= 0 {
		return value
	}
	return float32(math.Round(float64(value/step)) * float64(step))
}

func randomizedSpawnDelay(spawnPerMinute float32) float32 {
	if spawnPerMinute <= 0 {
		return float32(math.MaxFloat32)
	}
	lambda := float64(spawnPerMinute) / 60.0
	u := math.Max(rand.Float64(), 1e-5)
	return float32(-math.Log(u) / lambda)
}

var routePalette = []Color{
	NewColor(224, 94, 94, 255),   // Red
	NewColor(76, 150, 230, 255),  // Blue
	NewColor(99, 190, 123, 255),  // Green
	NewColor(225, 169, 76, 255),  // Orange
	NewColor(154, 108, 224, 255), // Purple
	NewColor(76, 191, 188, 255),  // Cyan
	NewColor(213, 104, 171, 255), // Pink
	NewColor(218, 205, 60, 255),  // Yellow
	NewColor(140, 205, 70, 255),  // Lime
	NewColor(98, 118, 228, 255),  // Indigo
	NewColor(230, 112, 82, 255),  // Coral
	NewColor(62, 178, 152, 255),  // Teal
	NewColor(178, 132, 228, 255), // Lavender
	NewColor(225, 82, 128, 255),  // Rose
	NewColor(228, 158, 48, 255),  // Amber
}

// hitboxRadius returns the circle radius for a body of the given width.
// Circles protrude exactly 0.5 m beyond the body's sides.
func hitboxRadius(width float32) float32 {
	return width/2 + 0.5
}

// hitboxCircleOffsets returns circle-centre offsets along the body heading for
// an object of the given length and width. Circles are placed so adjacent
// edges have a gap of at most 1 m and never overlap. Minimum two circles.
func hitboxCircleOffsets(length, width float32) []float32 {
	r := hitboxRadius(width)
	span := 2 * (length/2 + 1.0 - r)
	if span < 0 {
		span = 0
	}
	// Maximum centre-to-centre distance that keeps the edge gap ≤ 1 m.
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

func pixelsToWorld(zoom, pixels float32) float32 {
	return pixels / zoom
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

// ---------- pedestrian paths ----------

// pedestrianPathDraft tracks the two-click placement state: HasP0=false means
// the next left click places the first endpoint; HasP0=true means the next
// left click commits a path from P0 to the current cursor position.
type pedestrianPathDraft struct {
	P0    Vec2
	HasP0 bool
}

// pedestrianEndpointSnapPixels is the screen-space snap radius for locking a
// new endpoint onto an existing pedestrian path endpoint.
const pedestrianEndpointSnapPixels float32 = 14.0

// pedestrianMinSegmentLengthM rejects zero- or near-zero-length paths from
// accidental double clicks. 0.5 m keeps the rule intuitive for 4 m-wide paths.
const pedestrianMinSegmentLengthM float32 = 0.5

func snapPedestrianEndpoint(point Vec2, paths []simpkg.PedestrianPath, zoom float32) Vec2 {
	radius := pixelsToWorld(zoom, pedestrianEndpointSnapPixels)
	radiusSq := radius * radius
	best := point
	bestSq := radiusSq
	for _, p := range paths {
		if d := distSq(point, p.P0); d < bestSq {
			bestSq = d
			best = p.P0
		}
		if d := distSq(point, p.P1); d < bestSq {
			bestSq = d
			best = p.P1
		}
	}
	return best
}

func findNearestPedestrianEndpoint(point Vec2, paths []simpkg.PedestrianPath, radius float32) (Vec2, bool) {
	radiusSq := radius * radius
	bestSq := radiusSq
	best := Vec2{}
	found := false
	for _, p := range paths {
		if d := distSq(point, p.P0); d < bestSq {
			bestSq = d
			best = p.P0
			found = true
		}
		if d := distSq(point, p.P1); d < bestSq {
			bestSq = d
			best = p.P1
			found = true
		}
	}
	return best, found
}

func pointSegmentDistSq(p, a, b Vec2) float32 {
	ab := vecSub(b, a)
	lenSq := dot(ab, ab)
	if lenSq <= 1e-9 {
		return distSq(p, a)
	}
	t := dot(vecSub(p, a), ab) / lenSq
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return distSq(p, vecAdd(a, vecScale(ab, t)))
}

func findPedestrianPathAt(paths []simpkg.PedestrianPath, point Vec2, radius float32) int {
	halfWidth := simpkg.PedestrianPathWidthM * 0.5
	tol := halfWidth + radius
	bestSq := tol * tol
	idx := -1
	for i, p := range paths {
		if d := pointSegmentDistSq(point, p.P0, p.P1); d < bestSq {
			bestSq = d
			idx = i
		}
	}
	return idx
}

func handlePedestrianPathMode(draft pedestrianPathDraft, paths []simpkg.PedestrianPath, mouseWorld Vec2, zoom float32) (pedestrianPathDraft, []simpkg.PedestrianPath, bool) {
	changed := false
	if rl.IsKeyPressed(rl.KeyEscape) && draft.HasP0 {
		draft = pedestrianPathDraft{}
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		if draft.HasP0 {
			draft = pedestrianPathDraft{}
		} else {
			radius := pixelsToWorld(zoom, hoverPixels)
			if idx := findPedestrianPathAt(paths, mouseWorld, radius); idx >= 0 {
				paths = append(paths[:idx], paths[idx+1:]...)
				changed = true
			}
		}
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		snapped := snapPedestrianEndpoint(mouseWorld, paths, zoom)
		if !draft.HasP0 {
			draft.P0 = snapped
			draft.HasP0 = true
		} else {
			if distSq(draft.P0, snapped) >= pedestrianMinSegmentLengthM*pedestrianMinSegmentLengthM {
				paths = append(paths, simpkg.PedestrianPath{P0: draft.P0, P1: snapped})
				changed = true
			}
			draft = pedestrianPathDraft{}
		}
	}
	return draft, paths, changed
}

// ---------- pedestrian path rendering ----------

var pedestrianPathColor = NewColor(196, 178, 140, 220) // tan

func drawPedestrianPaths(paths []simpkg.PedestrianPath, viewRect worldRect) {
	if len(paths) == 0 {
		return
	}
	for _, p := range paths {
		if !segmentVisibleInWorldRect(p.P0, p.P1, simpkg.PedestrianPathWidthM, viewRect) {
			continue
		}
		drawLineEx(p.P0, p.P1, simpkg.PedestrianPathWidthM, pedestrianPathColor)
	}
}

func drawPedestrians(pedestrians []Pedestrian, paths []simpkg.PedestrianPath, zoom float32, viewRect worldRect) {
	if len(pedestrians) == 0 || len(paths) == 0 {
		return
	}
	bodyColor := NewColor(46, 74, 93, 230)
	headColor := NewColor(236, 205, 177, 245)
	outlineColor := NewColor(245, 245, 245, 150)
	for _, ped := range pedestrians {
		pos, heading, ok := simpkg.PedestrianPose(paths, ped)
		if !ok {
			continue
		}
		bodyRadius := maxf(ped.Radius, pixelsToWorld(zoom, 3.5))
		if !circleVisibleInWorldRect(pos, bodyRadius*2.0, viewRect) {
			continue
		}
		headPos := vecAdd(pos, vecScale(heading, bodyRadius*0.85))
		drawCircleV(pos, bodyRadius, bodyColor)
		drawCircleV(headPos, bodyRadius*0.58, headColor)
		drawCircleLinesV(pos, bodyRadius, outlineColor)
	}
}

func drawPedestrianPathTool(draft pedestrianPathDraft, paths []simpkg.PedestrianPath, mouseWorld Vec2, zoom float32) {
	endpointRadius := pixelsToWorld(zoom, handlePixels)
	endpointColor := NewColor(120, 90, 40, 255)
	for _, p := range paths {
		drawCircleV(p.P0, endpointRadius*0.9, endpointColor)
		drawCircleV(p.P1, endpointRadius*0.9, endpointColor)
	}
	snapped := snapPedestrianEndpoint(mouseWorld, paths, zoom)
	snappedToExisting := distSq(snapped, mouseWorld) > 1e-6
	if snappedToExisting {
		drawCircleLinesV(snapped, endpointRadius*1.5, NewColor(255, 196, 61, 255))
	}
	if draft.HasP0 {
		previewColor := NewColor(214, 76, 76, 180)
		drawLineEx(draft.P0, snapped, simpkg.PedestrianPathWidthM, previewColor)
		drawCircleV(draft.P0, endpointRadius, NewColor(214, 76, 76, 255))
	}
}

// segmentVisibleInWorldRect tests whether a thick line segment's AABB overlaps
// the view rectangle. Cheap reject for offscreen paths.
func segmentVisibleInWorldRect(a, b Vec2, thickness float32, viewRect worldRect) bool {
	pad := thickness * 0.5
	minX := minf(a.X, b.X) - pad
	maxX := maxf(a.X, b.X) + pad
	minY := minf(a.Y, b.Y) - pad
	maxY := maxf(a.Y, b.Y) + pad
	if maxX < viewRect.MinX || minX > viewRect.MaxX {
		return false
	}
	if maxY < viewRect.MinY || minY > viewRect.MaxY {
		return false
	}
	return true
}
