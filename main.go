package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"

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
//   P: priority paint mode
//   D: toggle debug overlay
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
// Priority mode:
//   Left click on a hovered spline: mark it as priority
//   Right click on a hovered spline: clear priority
//
// Navigation intentionally has no pan. To move around, zoom out and then zoom back into a new point.

type EditorMode int

type Stage int

type EndKind int

const (
	ModeEdit EditorMode = iota
	ModeRoute
	ModePriority
	ModeCouple
	ModeCut
	ModeSpeedLimit
	ModePreference
	ModeTrafficLight
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

	predictionHorizonSeconds float32 = 2.0
	predictionStepSeconds    float32 = 0.15
	blameAngleThresholdDeg   float32 = 45.0
	brakeDecelMultiplier     float32 = 2.5
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
)

type Spline struct {
	ID       int
	Priority bool

	P0 rl.Vector2
	P1 rl.Vector2
	P2 rl.Vector2
	P3 rl.Vector2

	Length         float32
	SpeedFactor    float32
	Samples        [simSamples + 1]rl.Vector2
	CumLen         [simSamples + 1]float32
	HardCoupledIDs []int     // parallel lanes that also act as pathfinding neighbours
	SoftCoupledIDs []int     // parallel lanes used for lane-changes only, not routing
	SpeedLimitKmh  float32   // 0 = no limit
	LanePreference int       // 0 = none; lower = higher preference
	CurveSpeedMPS  []float32 // max speed at each curveSpeedIntervalM mark along the spline
}

type Draft struct {
	P0 rl.Vector2
	P1 rl.Vector2
	P2 rl.Vector2

	HasP1 bool
	HasP2 bool

	LockP1           bool
	ContinuationFrom int
}

// CutDraft holds state for the spline-cut mode.
// Stage StageIdle   → user is hovering, snap dot follows nearest spline point.
// Stage StageSetP1  → cut point is locked; user places the tangent handle.
type CutDraft struct {
	OriginalSplineID       int
	OriginalSplinePriority bool
	CutPoint               rl.Vector2
	CutT                   float32
	// de Casteljau sub-spline control points
	LeftP  [4]rl.Vector2 // left half  P0..P3
	RightP [4]rl.Vector2 // right half P0..P3
}

type EndHit struct {
	SplineIndex int
	SplineID    int
	Kind        EndKind
	Point       rl.Vector2
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
	Color          rl.Color
}

type RoutePanel struct {
	Open bool

	StartSplineID   int
	EndSplineID     int
	ExistingRouteID int
	PathIDs         []int
	PathLength      float32
	SpawnPerMinute  float32
	ColorIndex      int

	DraggingSlider bool
}

type Car struct {
	RouteID int

	CurrentSplineID      int
	DestinationSplineID  int
	PrevSplineIDs        [2]int // last two splines before current; -1 = none
	DistanceOnSpline     float32
	Speed                float32
	MaxSpeed             float32
	Accel                float32
	Length               float32
	Width                float32
	CurveSpeedMultiplier float32 // per-car multiplier on curvature speed limits (0.8–1.2)
	Color                rl.Color
	Braking              bool
	SoftSlowing          bool // capped by a leading car's speed (follow-cap mechanism)

	// Lane-change state.
	LaneChanging       bool
	LaneChangeSplineID int     // ID of the temporary bridging spline; -1 = none
	AfterSplineID      int     // destination spline to continue on after the bridge
	AfterSplineDist    float32 // arc-length on AfterSplineID where the bridge lands
	// Forced lane-switch state (set when no direct path exists but a hard-coupled
	// neighbour of the next physical spline does have a path).
	DesiredLaneSplineID int     // spline to merge into; -1 = none
	DesiredLaneDeadline float32 // arc-length on current spline beyond which the switch must start

	PreferenceCooldown float32 // seconds until the next preference-based lane-change check
	SlowedTimer        float32 // seconds the car has been held back by a leader
	OvertakeCooldown   float32 // seconds until the next overtake attempt is allowed
}

type TrajectorySample struct {
	Time     float32
	Position rl.Vector2
	Heading  rl.Vector2
	Priority bool
	SplineID int
}

type CollisionPrediction struct {
	Time      float32
	PosA      rl.Vector2
	PosB      rl.Vector2
	PrevPosA  rl.Vector2
	PrevPosB  rl.Vector2
	HeadingA  rl.Vector2
	HeadingB  rl.Vector2
	PriorityA bool
	PriorityB bool
	SplineAID int
	SplineBID int
}

type DebugBlameLink struct {
	FromCarIndex int
	ToCarIndex   int
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
	ID             int        `json:"id"`
	Priority       bool       `json:"priority"`
	P0             rl.Vector2 `json:"p0"`
	P1             rl.Vector2 `json:"p1"`
	P2             rl.Vector2 `json:"p2"`
	P3             rl.Vector2 `json:"p3"`
	HardCoupledIDs []int      `json:"hard_coupled_ids,omitempty"`
	SoftCoupledIDs []int      `json:"soft_coupled_ids,omitempty"`
	SpeedLimitKmh  float32    `json:"speed_limit_kmh,omitempty"`
	LanePreference int        `json:"lane_preference,omitempty"`
}

type SavedRoute struct {
	ID             int     `json:"id"`
	StartSplineID  int     `json:"start_spline_id"`
	EndSplineID    int     `json:"end_spline_id"`
	PathIDs        []int   `json:"path_ids"`
	SpawnPerMinute float32 `json:"spawn_per_minute"`
	ColorIndex     int     `json:"color_index,omitempty"`
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
}

// ---------- traffic lights ----------

type TrafficState int

const (
	TrafficRed    TrafficState = iota
	TrafficYellow TrafficState = iota
	TrafficGreen  TrafficState = iota
)

// TrafficLight is a stop/go point placed on a spline.
type TrafficLight struct {
	ID           int
	SplineID     int
	DistOnSpline float32
	WorldPos     rl.Vector2
	CycleID      int // -1 = pending (not yet in any cycle)
}

// TrafficPhase is one step of a cycle. Each light not listed in GreenLightIDs is red.
type TrafficPhase struct {
	DurationSecs          float32
	ClearanceDurationSecs float32 // duration of the yellow clearance phase automatically inserted after this phase
	GreenLightIDs         []int   // IDs of lights that are green; all others are red
}

// TrafficCycle groups lights that share a timed phase sequence.
type TrafficCycle struct {
	ID         int
	LightIDs   []int
	Phases     []TrafficPhase
	Timer      float32
	PhaseIndex int  // which phase is currently active
	Enabled    bool // when false the cycle is paused and all lights show red
}

// ---------- ui font ----------

// uiFont is the Ubuntu font used for all on-screen text.
var uiFont rl.Font

// toolbarItem describes one button in the mode toolbar.
type toolbarItem struct {
	key   string
	label string
	mode  EditorMode
	isDbg bool // true for the debug toggle button
}

// toolbarItems is the ordered list of toolbar buttons.
var toolbarItems = []toolbarItem{
	{"E", "Edit", ModeEdit, false},
	{"R", "Route", ModeRoute, false},
	{"P", "Priority", ModePriority, false},
	{"L", "Couple", ModeCouple, false},
	{"C", "Cut", ModeCut, false},
	{"S", "Speed", ModeSpeedLimit, false},
	{"V", "Prefer", ModePreference, false},
	{"T", "Traffic", ModeTrafficLight, false},
	{"D", "Debug", 0, true},
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

func isMouseOverToolbar(mouse rl.Vector2) bool {
	n := len(toolbarItems)
	totalW := float32(n*toolbarBtnW + (n-1)*toolbarBtnGap)
	return mouse.X >= float32(toolbarX) && mouse.X <= float32(toolbarX)+totalW &&
		mouse.Y >= float32(toolbarY) && mouse.Y <= float32(toolbarY+toolbarBtnH)
}

func drawText(text string, x, y, size int32, color rl.Color) {
	rl.DrawTextEx(uiFont, text, rl.NewVector2(float32(x), float32(y)), float32(size), 1, color)
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

	splines := make([]Spline, 0, 128)
	laneChangeSplines := make([]Spline, 0, 32)
	routes := make([]Route, 0, 32)
	cars := make([]Car, 0, 256)

	mode := ModeEdit
	stage := StageIdle
	draft := newDraft()
	cutDraft := newCutDraft()
	routePanel := RoutePanel{}
	routeStartSplineID := -1
	coupleModeFirstID := -1
	debugMode := false
	selectedSpeedKmh := 50
	lastPref := 0
	nextSplineID := 1
	nextRouteID := 1
	nextLightID := 1
	nextCycleID := 1
	trafficLights := make([]TrafficLight, 0)
	trafficCycles := make([]TrafficCycle, 0)
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
	routeVisualsTimer := float32(0)

	for !rl.WindowShouldClose() {
		dt := rl.GetFrameTime()
		camera.Offset = rl.NewVector2(float32(rl.GetScreenWidth())/2, float32(rl.GetScreenHeight())/2)

		if noticeTimer > 0 {
			noticeTimer -= dt
			if noticeTimer <= 0 {
				noticeText = ""
				noticeTimer = 0
			}
		}

		if rl.IsKeyPressed(rl.KeyTab) {
			switch mode {
			case ModeEdit:
				mode = ModeRoute
			case ModeRoute:
				mode = ModePriority
			case ModePriority:
				mode = ModeCouple
			case ModeCouple:
				mode = ModeCut
			case ModeCut:
				mode = ModeSpeedLimit
			case ModeSpeedLimit:
				mode = ModePreference
			case ModePreference:
				mode = ModeTrafficLight
			default:
				mode = ModeEdit
			}
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyE) {
			mode = ModeEdit
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyR) {
			mode = ModeRoute
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyP) {
			mode = ModePriority
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyL) {
			mode = ModeCouple
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyC) {
			mode = ModeCut
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyS) && !isCtrlDown() {
			mode = ModeSpeedLimit
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyV) {
			mode = ModePreference
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyT) {
			mode = ModeTrafficLight
			stage = StageIdle
			draft = newDraft()
			cutDraft = newCutDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
			coupleModeFirstID = -1
		}
		if rl.IsKeyPressed(rl.KeyD) {
			debugMode = !debugMode
		}
		if rl.IsKeyPressed(rl.KeySpace) && activeDurInput == -1 {
			paused = !paused
		}
		if isCtrlDown() && rl.IsKeyPressed(rl.KeyS) {
			path, err := pickSplineFilePath(true)
			if err != nil {
				noticeText = fmt.Sprintf("Save failed: %v", err)
				noticeTimer = 3.0
			} else if path != "" {
				if err := saveSplineFile(splines, routes, cars, trafficLights, trafficCycles, path); err != nil {
					noticeText = fmt.Sprintf("Save failed: %v", err)
				} else {
					noticeText = fmt.Sprintf("Saved %d splines, %d routes, %d cars, %d lights to %s",
						len(splines), len(routes), len(cars), len(trafficLights), path)
				}
				noticeTimer = 3.0
			}
		}
		if isCtrlDown() && rl.IsKeyPressed(rl.KeyO) {
			path, err := pickSplineFilePath(false)
			if err != nil {
				noticeText = fmt.Sprintf("Load failed: %v", err)
				noticeTimer = 3.0
			} else if path != "" {
				loadedSplines, loadedRoutes, loadedCars, loadedLights, loadedCycles,
					loadedNextSplineID, loadedNextRouteID, loadedNextLightID, loadedNextCycleID, err := loadSplineFile(path)
				if err != nil {
					noticeText = fmt.Sprintf("Load failed: %v", err)
					noticeTimer = 3.0
				} else {
					splines = loadedSplines
					routes = loadedRoutes
					cars = loadedCars
					trafficLights = loadedLights
					trafficCycles = loadedCycles
					stage = StageIdle
					draft = newDraft()
					cutDraft = newCutDraft()
					laneChangeSplines = laneChangeSplines[:0]
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
					nextSplineID = loadedNextSplineID
					nextRouteID = loadedNextRouteID
					nextLightID = loadedNextLightID
					nextCycleID = loadedNextCycleID
					lastPref = maxLoadedPreference(loadedSplines)
					noticeText = fmt.Sprintf("Loaded %d splines, %d routes, %d cars, %d lights from %s",
						len(splines), len(routes), len(cars), len(trafficLights), path)
					noticeTimer = 3.0
				}
			}
		}

		wheel := rl.GetMouseWheelMove()
		if wheel != 0 {
			zoomCameraToMouse(&camera, wheel)
		}

		mouseScreen := rl.GetMousePosition()
		mouseWorld := rl.GetScreenToWorld2D(mouseScreen, camera)
		mouseOnToolbar := isMouseOverToolbar(mouseScreen)

		// Toolbar click — switch mode / toggle debug
		if rl.IsMouseButtonPressed(rl.MouseButtonLeft) && mouseOnToolbar {
			for i, item := range toolbarItems {
				if rl.CheckCollisionPointRec(mouseScreen, toolbarBtnRect(i)) {
					if item.isDbg {
						debugMode = !debugMode
					} else {
						mode = item.mode
						stage = StageIdle
						draft = newDraft()
						cutDraft = newCutDraft()
						routePanel = RoutePanel{}
						routeStartSplineID = -1
						coupleModeFirstID = -1
					}
					break
				}
			}
		}

		hoverRadius := pixelsToWorld(camera.Zoom, hoverPixels)
		snapRadius := pixelsToWorld(camera.Zoom, snapPixels)
		handleRadius := pixelsToWorld(camera.Zoom, handlePixels)
		baseThickness := pixelsToWorld(camera.Zoom, linePixels)

		hoveredSpline := findHoveredSpline(splines, mouseWorld, hoverRadius)
		hoveredEnd := findNearbyEnd(splines, mouseWorld, snapRadius)
		hoveredStart := findNearbyStart(splines, mouseWorld, snapRadius)

		vehicleCounts := buildVehicleCounts(cars)
		routeVisualsTimer -= dt
		if routeVisualsTimer <= 0 {
			routes = updateRouteVisuals(routes, splines, vehicleCounts)
			routeVisualsTimer = 0.5
		}
		if !paused {
			laneChangeSplines, cars = computeLaneChanges(cars, splines, laneChangeSplines, &nextSplineID, vehicleCounts, dt)
		}
		allSplines := mergedSplines(splines, laneChangeSplines)
		var brakingDecisions, holdSpeedDecisions []bool
		var debugBlameLinks, holdBlameLinks []DebugBlameLink
		if !paused {
			brakingDecisions, holdSpeedDecisions, debugBlameLinks, holdBlameLinks = computeBrakingDecisions(cars, allSplines, vehicleCounts)
			followCaps := computeFollowingSpeedCaps(cars, allSplines, vehicleCounts)
			cars = updateCars(cars, routes, allSplines, vehicleCounts, brakingDecisions, holdSpeedDecisions, followCaps, trafficLights, trafficCycles, dt)
			laneChangeSplines = gcLaneChangeSplines(laneChangeSplines, cars)
			routes, cars = updateRouteSpawning(routes, cars, splines, dt)
			trafficCycles = updateTrafficCycles(trafficCycles, dt)
		}

		if routePanel.Open {
			var applied bool
			routePanel, routes, cars, applied = updateRoutePanel(routePanel, routes, cars, &nextRouteID)
			if applied {
				routeStartSplineID = -1
			}
		}

		if !routePanel.Open && !mouseOnToolbar {
			switch mode {
			case ModeEdit:
				var topologyChanged bool
				stage, draft, splines, nextSplineID, topologyChanged = handleEditMode(stage, draft, splines, hoveredSpline, hoveredEnd, hoveredStart, mouseWorld, nextSplineID)
				if topologyChanged {
					routes = refreshRoutes(routes, splines)
					cars = cars[:0]
					laneChangeSplines = laneChangeSplines[:0]
				}
			case ModeRoute:
				var notice string
				routeStartSplineID, routePanel, notice = handleRouteMode(routeStartSplineID, routePanel, routes, splines, vehicleCounts, hoveredStart, hoveredEnd)
				if notice != "" {
					noticeText = notice
					noticeTimer = 3.0
				}
			case ModePriority:
				splines = handlePriorityMode(splines, hoveredSpline)
			case ModeCouple:
				var coupleNotice string
				coupleModeFirstID, splines, coupleNotice = handleCoupleMode(coupleModeFirstID, splines, hoveredSpline)
				if coupleNotice != "" {
					noticeText = coupleNotice
					noticeTimer = 3.0
				}
			case ModeCut:
				var topologyChanged bool
				stage, cutDraft, splines, nextSplineID, topologyChanged = handleCutMode(stage, cutDraft, splines, mouseWorld, nextSplineID)
				if topologyChanged {
					routes = refreshRoutes(routes, splines)
					cars = cars[:0]
					laneChangeSplines = laneChangeSplines[:0]
				}
			case ModeSpeedLimit:
				splines = handleSpeedLimitMode(splines, hoveredSpline, selectedSpeedKmh)
				selectedSpeedKmh = updateSpeedLimitPanel(selectedSpeedKmh)
			case ModePreference:
				splines, lastPref = handlePreferenceMode(splines, hoveredSpline, lastPref)
			case ModeTrafficLight:
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
				overPanel := rl.CheckCollisionPointRec(mouseScreen, pr)

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
							if !rl.CheckCollisionPointRec(mouseScreen, activeField) {
								trafficCycles = commitDurInput(trafficCycles, editingCycleID, activeDurInput, durInputStr)
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
							if rl.CheckCollisionPointRec(mouseScreen, btn) {
								pendingLights, trafficLights, trafficCycles, editingCycleID = doCreateTrafficCycle(pendingLights, trafficLights, trafficCycles, &nextCycleID)
							}
						}
					} else {
						// cycle editor panel
						onOffBtnR, editLightsBtn, closeBtnR := trafficHeaderBtnRects(pr)
						// On/Off toggle
						if rl.CheckCollisionPointRec(mouseScreen, onOffBtnR) {
							trafficCycles = trafficToggleCycleEnabled(trafficCycles, editingCycleID)
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
						if !cycleIsOn && rl.CheckCollisionPointRec(mouseScreen, editLightsBtn) {
							editingLights = !editingLights
							editingPhaseIdx = -1
						}
						if rl.CheckCollisionPointRec(mouseScreen, closeBtnR) {
							editingCycleID = -1
							editingLights = false
							editingPhaseIdx = -1
							showPhaseIdx = -1
						}
						// Add Phase button (only when cycle is off)
						if !cycleIsOn && rl.CheckCollisionPointRec(mouseScreen, trafficAddPhaseBtnRect(pr)) {
							trafficCycles = trafficAddPhase(trafficCycles, editingCycleID)
						}
						// Per-phase row buttons
						for pi := 0; pi < phaseCount; pi++ {
							row := getPhaseRowBtns(pr, pi)
							// Show (cycle off) / Skip (cycle on) button: always active
							if rl.CheckCollisionPointRec(mouseScreen, row.showBtn) {
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
								if rl.CheckCollisionPointRec(mouseScreen, row.upBtn) && pi > 0 {
									trafficCycles = trafficMovePhase(trafficCycles, editingCycleID, pi, -1)
									if editingPhaseIdx == pi {
										editingPhaseIdx--
									}
									if showPhaseIdx == pi {
										showPhaseIdx--
									}
								}
								if rl.CheckCollisionPointRec(mouseScreen, row.downBtn) && pi < phaseCount-1 {
									trafficCycles = trafficMovePhase(trafficCycles, editingCycleID, pi, +1)
									if editingPhaseIdx == pi {
										editingPhaseIdx++
									}
									if showPhaseIdx == pi {
										showPhaseIdx++
									}
								}
								if rl.CheckCollisionPointRec(mouseScreen, row.durField) {
									for _, c := range trafficCycles {
										if c.ID == editingCycleID && pi < len(c.Phases) {
											activeDurInput = pi
											durInputStr = fmt.Sprintf("%.1f", c.Phases[pi].DurationSecs)
											break
										}
									}
								}
								if rl.CheckCollisionPointRec(mouseScreen, row.clearField) {
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
								if rl.CheckCollisionPointRec(mouseScreen, row.editBtn) {
									if editingPhaseIdx == pi {
										editingPhaseIdx = -1
									} else {
										editingPhaseIdx = pi
										editingLights = false
										showPhaseIdx = -1
									}
								}
								if rl.CheckCollisionPointRec(mouseScreen, row.delBtn) {
									trafficCycles = trafficDeletePhase(trafficCycles, editingCycleID, pi)
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
							}
						}
					} else if editingCycleID >= 0 && editingLights {
						trafficLights, trafficCycles = handleTrafficLightEdit(splines, trafficLights, trafficCycles, editingCycleID, mouseWorld, camera.Zoom, &nextLightID)
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

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode2D(camera)
		drawGrid(camera)
		drawAxes(camera)

		splineIndexByID := buildSplineIndexByID(splines)
		for _, route := range routes {
			if !route.Valid {
				continue
			}
			drawRouteWithIndex(route, splines, splineIndexByID, pixelsToWorld(camera.Zoom, 2.0), camera.Zoom)
		}

		for i, spline := range splines {
			color := splineDrawColor(spline)
			thickness := baseThickness
			if spline.Priority {
				thickness = maxf(thickness, pixelsToWorld(camera.Zoom, 4))
			}
			if i == hoveredSpline {
				if (mode == ModeEdit && stage == StageIdle) || mode == ModePriority {
					color = rl.NewColor(242, 153, 74, 255)
					thickness = pixelsToWorld(camera.Zoom, 5)
				}
			}

			drawSpline(spline, thickness, color)
			drawEndpoint(spline.P0, handleRadius*0.8, rl.NewColor(60, 160, 90, 255))
			drawEndpoint(spline.P3, handleRadius, rl.NewColor(50, 115, 225, 255))
		}

		if mode == ModeEdit {
			if hoveredEnd.SplineIndex >= 0 && stage == StageIdle {
				drawEndpoint(hoveredEnd.Point, handleRadius*1.5, rl.NewColor(255, 196, 61, 255))
			}
			if hoveredStart.SplineIndex >= 0 && stage == StageSetP3 {
				drawEndpoint(hoveredStart.Point, handleRadius*1.5, rl.NewColor(163, 92, 255, 255))
			}

			previewMouse := mouseWorld
			if rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl) {
				previewMouse = snapToGrid(mouseWorld, 4.0)
			}
			preview, hasPreview := buildPreview(stage, draft, previewMouse, hoveredStart, splines)
			if hasPreview {
				drawDraft(stage, draft, previewMouse, camera.Zoom)
				drawSpline(preview, pixelsToWorld(camera.Zoom, 4), rl.NewColor(214, 76, 76, 255))
			}
		}

		if mode == ModeRoute {
			drawRoutePicking(routeStartSplineID, routePanel, hoveredStart, hoveredEnd, splines, vehicleCounts, camera.Zoom)
		}

		if mode == ModeCouple {
			drawCoupleMode(splines, coupleModeFirstID, hoveredSpline, camera.Zoom)
		}
		if mode == ModeSpeedLimit {
			drawSpeedLimitWorld(splines, hoveredSpline, camera.Zoom)
		}
		if mode == ModeCut {
			drawCutMode(stage, cutDraft, splines, mouseWorld, camera.Zoom)
		}
		allSplineIndexByID := buildSplineIndexByID(allSplines)
		drawCars(cars, allSplines, allSplineIndexByID, camera.Zoom, debugMode)
		if debugMode {
			drawDebugBlameLinks(debugBlameLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, rl.NewColor(220, 50, 50, 220))
			drawDebugBlameLinks(holdBlameLinks, cars, allSplines, allSplineIndexByID, camera.Zoom, rl.NewColor(255, 165, 0, 220))
			drawLaneChangeSplines(laneChangeSplines, camera.Zoom)
		}
		drawTrafficLights(trafficLights, pendingLights, trafficCycles, editingCycleID, editingPhaseIdx, showPhaseIdx, camera.Zoom)
		if mode == ModeTrafficLight && (editingCycleID < 0 || editingLights) && editingPhaseIdx < 0 {
			if _, _, snap, found := findNearestSplinePoint(splines, mouseWorld); found {
				r := pixelsToWorld(camera.Zoom, 8)
				rl.DrawCircleV(snap, r, rl.NewColor(255, 200, 0, 180))
				rl.DrawRing(snap, r*0.6, r, 0, 360, 16, rl.NewColor(220, 160, 0, 220))
			}
		}
		rl.EndMode2D()

		drawScaleBar(camera.Zoom)
		if mode == ModeEdit {
			previewMouse := mouseWorld
			if rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl) {
				previewMouse = snapToGrid(mouseWorld, 4.0)
			}
			if preview, hasPreview := buildPreview(stage, draft, previewMouse, hoveredStart, splines); hasPreview {
				drawDraftInfo(stage, draft, previewMouse, preview, camera)
			}
		}
		if mode == ModeSpeedLimit {
			drawSpeedLimitLabels(splines, camera)
			drawCarSpeedLabels(cars, allSplines, allSplineIndexByID, camera)
		}
		if mode == ModePreference {
			drawPreferenceLabels(splines, camera)
		}
		drawHud(mode, stage, draft, hoveredSpline, routeStartSplineID, coupleModeFirstID, debugMode, camera.Zoom, len(splines), len(routes), len(cars))
		if mode == ModeTrafficLight {
			drawTrafficCyclePanel(pendingLights, trafficLights, trafficCycles, editingCycleID, editingLights, editingPhaseIdx, activeDurInput, durInputStr, showPhaseIdx)
		}
		if mode == ModeSpeedLimit {
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
		rl.EndDrawing()
	}
}

func snapToGrid(v rl.Vector2, gridSize float32) rl.Vector2 {
	return rl.Vector2{
		X: float32(math.Round(float64(v.X)/float64(gridSize))) * gridSize,
		Y: float32(math.Round(float64(v.Y)/float64(gridSize))) * gridSize,
	}
}

func handleEditMode(stage Stage, draft Draft, splines []Spline, hoveredSpline int, hoveredEnd EndHit, hoveredStart EndHit, mouseWorld rl.Vector2, nextSplineID int) (Stage, Draft, []Spline, int, bool) {
	topologyChanged := false

	if rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl) {
		mouseWorld = snapToGrid(mouseWorld, 4.0)
	}

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
			stage = StageSetP2
		}
	}

	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		switch stage {
		case StageIdle:
			draft = newDraft()
			if hoveredEnd.SplineIndex >= 0 {
				prev := splines[hoveredEnd.SplineIndex]
				draft.P0 = prev.P3
				draft.ContinuationFrom = prev.ID
				draft.LockP1 = true
				draft.P1 = vecAdd(draft.P0, vecSub(prev.P3, prev.P2))
				draft.HasP1 = true
			} else {
				draft.P0 = mouseWorld
			}
			stage = StageSetP1

		case StageSetP1:
			if !draft.LockP1 {
				draft.P1 = mouseWorld
				draft.HasP1 = true
			}
			stage = StageSetP2

		case StageSetP2:
			draft.P2 = mouseWorld
			draft.HasP2 = true
			stage = StageSetP3

		case StageSetP3:
			p2 := draft.P2
			p3 := mouseWorld
			if hoveredStart.SplineIndex >= 0 {
				next := splines[hoveredStart.SplineIndex]
				p3 = next.P0
				p2 = vecSub(vecScale(p3, 2), next.P1)
			}
			spline := newSpline(nextSplineID, draft.P0, draft.P1, p2, p3)
			nextSplineID++
			splines = append(splines, spline)
			stage = StageIdle
			draft = newDraft()
			topologyChanged = true
		}
	}

	return stage, draft, splines, nextSplineID, topologyChanged
}

func handleRouteMode(routeStartSplineID int, routePanel RoutePanel, routes []Route, splines []Spline, vehicleCounts map[int]int, hoveredStart EndHit, hoveredEnd EndHit) (int, RoutePanel, string) {
	if rl.IsKeyPressed(rl.KeyEscape) || rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		return -1, RoutePanel{}, ""
	}

	if !rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		return routeStartSplineID, routePanel, ""
	}

	if routeStartSplineID < 0 {
		if hoveredStart.SplineID >= 0 {
			return hoveredStart.SplineID, routePanel, ""
		}
		return routeStartSplineID, routePanel, ""
	}

	if hoveredEnd.SplineID < 0 {
		return routeStartSplineID, routePanel, ""
	}

	pathIDs, pathLength, ok := findShortestPathWeighted(splines, routeStartSplineID, hoveredEnd.SplineID, vehicleCounts)
	if !ok {
		return -1, RoutePanel{}, "No valid path between the selected start and destination."
	}

	existingRouteID := findRouteID(routes, routeStartSplineID, hoveredEnd.SplineID)
	spawnPerMinute := float32(12.0)
	colorIndex := pickNextColorIndex(routes)
	if existingRouteID >= 0 {
		if idx := findRouteIndexByID(routes, existingRouteID); idx >= 0 {
			spawnPerMinute = routes[idx].SpawnPerMinute
			colorIndex = routes[idx].ColorIndex
		}
	}

	return -1, RoutePanel{
		Open:            true,
		StartSplineID:   routeStartSplineID,
		EndSplineID:     hoveredEnd.SplineID,
		ExistingRouteID: existingRouteID,
		PathIDs:         pathIDs,
		PathLength:      pathLength,
		SpawnPerMinute:  spawnPerMinute,
		ColorIndex:      colorIndex,
	}, ""
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

func handleTrafficLightMode(splines []Spline, pending []TrafficLight, mouseWorld rl.Vector2, zoom float32, nextLightID *int) []TrafficLight {
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
func effectivePhaseDur(c *TrafficCycle, ei int) float32 {
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

func updateTrafficCycles(cycles []TrafficCycle, dt float32) []TrafficCycle {
	for i := range cycles {
		if !cycles[i].Enabled || len(cycles[i].Phases) == 0 {
			continue
		}
		total := effectivePhaseCount(&cycles[i])
		cycles[i].Timer += dt
		dur := effectivePhaseDur(&cycles[i], cycles[i].PhaseIndex)
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

func trafficLightState(lightID, cycleID int, cycles []TrafficCycle) TrafficState {
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
			// User phase
			userIdx := ei / 2
			if userIdx >= n {
				userIdx = n - 1
			}
			if phaseHasGreen(c.Phases[userIdx], lightID) {
				return TrafficGreen
			}
			return TrafficRed
		}
		// Transition phase between prevIdx and nextIdx
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

func drawSmallBtn(r rl.Rectangle, label string, bg, fg rl.Color) {
	rl.DrawRectangleRec(r, bg)
	rl.DrawRectangleLinesEx(r, 1, rl.NewColor(0, 0, 0, 40))
	lw := measureText(label, 12)
	drawText(label, int32(r.X)+int32(r.Width)/2-lw/2, int32(r.Y)+int32(r.Height)/2-7, 12, fg)
}

func drawTrafficCyclePanel(pending []TrafficLight, lights []TrafficLight, cycles []TrafficCycle,
	editingCycleID int, editingLights bool, editingPhaseIdx, activeDurInput int, durInputStr string, showPhaseIdx int) {
	bg := rl.NewColor(248, 248, 250, 245)
	outline := rl.NewColor(210, 210, 215, 255)
	dark := rl.NewColor(28, 28, 33, 255)
	muted := rl.NewColor(100, 100, 110, 255)
	sep := rl.NewColor(220, 220, 224, 255)
	disabledFg := rl.NewColor(170, 170, 178, 255)
	disabledBg := rl.NewColor(220, 220, 224, 255)

	if editingCycleID < 0 {
		// ── New-cycle creator ────────────────────────────────────────────
		pr := trafficCyclePanelRect(false, 0, len(pending))
		rl.DrawRectangleRec(pr, bg)
		rl.DrawRectangleLinesEx(pr, 1, outline)
		drawText("New Traffic Cycle", int32(pr.X)+12, int32(pr.Y)+12, 15, dark)
		rl.DrawLineEx(rl.NewVector2(pr.X+1, pr.Y+36), rl.NewVector2(pr.X+pr.Width-1, pr.Y+36), 1, sep)

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
		btnBg := rl.NewColor(47, 120, 60, 255)
		if len(pending) == 0 {
			btnBg = rl.NewColor(160, 160, 165, 255)
		}
		rl.DrawRectangleRec(btn, btnBg)
		rl.DrawRectangleLinesEx(btn, 1, rl.NewColor(0, 0, 0, 40))
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
	onOffBg := rl.NewColor(190, 50, 50, 255)
	onOffLbl := "Off"
	if cycleOn {
		onOffBg = rl.NewColor(47, 140, 60, 255)
		onOffLbl = "On"
	}
	drawSmallBtn(onOffBtn, onOffLbl, onOffBg, rl.White)

	// Edit Lights button (disabled when cycle is on)
	elBg := rl.NewColor(47, 96, 198, 255)
	elFg := rl.Color(rl.White)
	if cycleOn {
		elBg = disabledBg
		elFg = disabledFg
	} else if editingLights {
		elBg = rl.NewColor(28, 62, 155, 255)
	}
	elLbl := "Edit Lights"
	if editingLights && !cycleOn {
		elLbl = "Done"
	}
	drawSmallBtn(elBtn, elLbl, elBg, elFg)
	drawSmallBtn(clBtn, "Close", rl.NewColor(200, 60, 60, 255), rl.White)

	rl.DrawLineEx(rl.NewVector2(pr.X+1, pr.Y+38), rl.NewVector2(pr.X+pr.Width-1, pr.Y+38), 1, sep)

	// "+ Add Phase" button (disabled when cycle is on)
	addBtn := trafficAddPhaseBtnRect(pr)
	if cycleOn {
		drawSmallBtn(addBtn, "+ Add Phase", disabledBg, disabledFg)
	} else {
		drawSmallBtn(addBtn, "+ Add Phase", rl.NewColor(70, 140, 80, 255), rl.White)
	}

	rl.DrawLineEx(rl.NewVector2(pr.X+1, pr.Y+74), rl.NewVector2(pr.X+pr.Width-1, pr.Y+74), 1, sep)

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
					rl.NewColor(255, 240, 180, 160))
			} else if isLeavingPhase {
				rl.DrawRectangle(int32(pr.X)+2, rowY, int32(pr.Width)-4, 50,
					rl.NewColor(255, 210, 100, 100))
			} else if isEnteringPhase {
				rl.DrawRectangle(int32(pr.X)+2, rowY, int32(pr.Width)-4, 50,
					rl.NewColor(200, 255, 200, 100))
			}

			// Phase label (sub-row 1)
			labelColor := dark
			phaseLabel := fmt.Sprintf("Phase %d", pi+1)
			if isCurrentPhase {
				labelColor = rl.NewColor(140, 90, 0, 255)
				phaseLabel = fmt.Sprintf("Phase %d  ▶", pi+1)
			} else if isLeavingPhase {
				labelColor = rl.NewColor(160, 100, 0, 255)
				phaseLabel = fmt.Sprintf("Phase %d  →", pi+1)
			} else if isEnteringPhase {
				labelColor = rl.NewColor(30, 120, 30, 255)
				phaseLabel = fmt.Sprintf("Phase %d  ←", pi+1)
			}
			drawText(phaseLabel, int32(pr.X)+12, rowY+3, 12, labelColor)

			yellowColor := rl.NewColor(180, 120, 0, 255)

			// Clearance field (sub-row 1, to the left of the duration field)
			clrActiveInput := -(pi + 3)
			clrBg := rl.NewColor(255, 252, 240, 255) // subtle amber tint
			if cycleOn {
				clrBg = rl.NewColor(235, 235, 238, 255)
			} else if activeDurInput == clrActiveInput {
				clrBg = rl.NewColor(255, 248, 220, 255)
			}
			drawText("→", int32(row.clearField.X)-16, rowY+3, 12, yellowColor)
			rl.DrawRectangleRec(row.clearField, clrBg)
			rl.DrawRectangleLinesEx(row.clearField, 1, rl.NewColor(200, 170, 100, 255))
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
			fieldBg := rl.NewColor(255, 255, 255, 255)
			if cycleOn {
				fieldBg = rl.NewColor(235, 235, 238, 255)
			} else if activeDurInput == pi {
				fieldBg = rl.NewColor(240, 248, 255, 255)
			}
			rl.DrawRectangleRec(row.durField, fieldBg)
			rl.DrawRectangleLinesEx(row.durField, 1, rl.NewColor(180, 180, 190, 255))
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
				upBg := rl.NewColor(200, 200, 205, 255)
				downBg := rl.NewColor(200, 200, 205, 255)
				if pi == 0 {
					upBg = disabledBg
				}
				if pi == len(cycle.Phases)-1 {
					downBg = disabledBg
				}
				drawSmallBtn(row.upBtn, "Up", upBg, dark)
				drawSmallBtn(row.downBtn, "Down", downBg, dark)

				editActive := editingPhaseIdx == pi
				editBg := rl.NewColor(47, 96, 198, 255)
				editLbl := "Edit"
				if editActive {
					editBg = rl.NewColor(28, 62, 155, 255)
					editLbl = "Done"
				}
				drawSmallBtn(row.editBtn, editLbl, editBg, rl.White)
				drawSmallBtn(row.delBtn, "Delete", rl.NewColor(200, 60, 60, 255), rl.White)
			}

			// Show (cycle off) / Skip (cycle on) button
			if cycleOn {
				drawSmallBtn(row.showBtn, "Skip", rl.NewColor(60, 120, 190, 255), rl.White)
			} else {
				showActive := showPhaseIdx == pi
				showBg := rl.NewColor(130, 80, 180, 255)
				if showActive {
					showBg = rl.NewColor(90, 40, 140, 255)
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

func drawTrafficLights(lights []TrafficLight, pending []TrafficLight, cycles []TrafficCycle, editingCycleID int, editingPhaseIdx int, showPhaseIdx int, zoom float32) {
	r := pixelsToWorld(zoom, 8)
	all := append(lights, pending...)
	for _, l := range all {
		var fill rl.Color
		if l.CycleID < 0 {
			fill = rl.NewColor(255, 165, 0, 220) // amber = pending
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
					fill = rl.NewColor(40, 180, 60, 240)
				} else {
					fill = rl.NewColor(210, 35, 35, 240)
				}
			} else {
				switch trafficLightState(l.ID, l.CycleID, cycles) {
				case TrafficRed:
					fill = rl.NewColor(210, 35, 35, 240)
				case TrafficYellow:
					fill = rl.NewColor(230, 175, 0, 240)
				case TrafficGreen:
					fill = rl.NewColor(40, 180, 60, 240)
				}
			}
		}
		rl.DrawCircleV(l.WorldPos, r, rl.NewColor(20, 20, 20, 220))
		rl.DrawCircleV(l.WorldPos, r*0.7, fill)
		// White ring around lights belonging to the currently-open cycle
		if editingCycleID >= 0 && l.CycleID == editingCycleID {
			rl.DrawRing(l.WorldPos, r*1.1, r*1.45, 0, 360, 24, rl.NewColor(255, 255, 255, 200))
		}
	}
}

// trafficLightAt returns the ID of the nearest placed light within click range,
// or -1 if none.
func trafficLightAt(lights []TrafficLight, pos rl.Vector2, zoom float32) int {
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
func handleTrafficLightEdit(splines []Spline, lights []TrafficLight, cycles []TrafficCycle, cycleID int, mouseWorld rl.Vector2, zoom float32, nextLightID *int) ([]TrafficLight, []TrafficCycle) {
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
	if isMouseOverSpeedLimitPanel(mousePos) {
		return splines
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonLeft) {
		splines[hoveredSpline].SpeedLimitKmh = float32(selectedSpeedKmh)
		splines[hoveredSpline].CurveSpeedMPS = buildCurveSpeedProfile(&splines[hoveredSpline])
	}
	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		splines[hoveredSpline].SpeedLimitKmh = 0
		splines[hoveredSpline].CurveSpeedMPS = buildCurveSpeedProfile(&splines[hoveredSpline])
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

func isMouseOverSpeedLimitPanel(mouse rl.Vector2) bool {
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
	drawSpline(splines[hoveredSpline], pixelsToWorld(zoom, 4), rl.NewColor(255, 200, 50, 180))
}

// drawSpeedLimitLabels draws speed limit badges on any spline that has a limit,
// in screen space, so they stay readable at any zoom level.
func drawSpeedLimitLabels(splines []Spline, camera rl.Camera2D) {
	for _, spline := range splines {
		if spline.SpeedLimitKmh <= 0 {
			continue
		}
		mid := spline.Samples[simSamples/2]
		screen := rl.GetWorldToScreen2D(mid, camera)
		label := fmt.Sprintf("%d", int(spline.SpeedLimitKmh))
		fontSize := int32(14)
		textW := measureText(label, fontSize)
		r := int32(14)
		cx, cy := int32(screen.X), int32(screen.Y)
		rl.DrawCircle(cx, cy, float32(r), rl.White)
		rl.DrawCircleLines(cx, cy, float32(r), rl.NewColor(200, 30, 30, 255))
		drawText(label, cx-textW/2, cy-fontSize/2, fontSize, rl.NewColor(200, 30, 30, 255))
	}
}

// drawPreferenceLabels draws preference badges on splines that have a preference assigned,
// in screen space, so they stay readable at any zoom level.
func drawPreferenceLabels(splines []Spline, camera rl.Camera2D) {
	color := rl.NewColor(30, 150, 60, 255)
	for _, spline := range splines {
		if spline.LanePreference <= 0 {
			continue
		}
		mid := spline.Samples[simSamples/2]
		screen := rl.GetWorldToScreen2D(mid, camera)
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
	bg := rl.NewColor(248, 248, 250, 245)
	outline := rl.NewColor(210, 210, 215, 255)
	text := rl.NewColor(30, 30, 35, 255)
	selBg := rl.NewColor(200, 30, 30, 220)
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
			rl.DrawRectangleLines(bx, by, 76, 24, rl.NewColor(150, 20, 20, 255))
			drawText(label, bx+6, by+5, 14, selText)
		} else {
			rl.DrawRectangle(bx, by, 76, 24, rl.NewColor(235, 236, 240, 255))
			rl.DrawRectangleLines(bx, by, 76, 24, outline)
			drawText(label, bx+6, by+5, 14, text)
		}
	}
}

// effectiveMaxSpeedMPS returns the fastest a car can ever travel on a spline,
// taking the global max car speed, the spline's speed factor, and any speed
// limit into account.
func effectiveMaxSpeedMPS(spline Spline) float32 {
	cap := maxCarSpeed
	if spline.SpeedLimitKmh > 0 {
		if limitMPS := spline.SpeedLimitKmh / 3.6; limitMPS < cap {
			cap = limitMPS
		}
	}
	return cap * spline.SpeedFactor
}

// laneChangeFeasibleAt checks whether a lane change from a specific arc-length
// distance on src to dst is geometrically possible at the given speed.
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

// laneChangeFeasible checks whether a lane change from the very start of src to
// dst is geometrically possible at the given speed, using the same rules as the
// runtime lane-change system.
func laneChangeFeasible(src, dst Spline, speed float32) bool {
	return laneChangeFeasibleAt(src, dst, 0, speed)
}

// findForcedLaneChangePath looks for a physically reachable next spline whose
// hard-coupled partners include one that has a valid path to the destination.
// Returns the ID of the next spline to enter and the ID of the desired target
// spline to switch to once on that next spline.
func findForcedLaneChangePath(splines []Spline, currentSplineID, destSplineID int, vehicleCounts map[int]int) (nextSplineID, desiredSplineID int, ok bool) {
	currentSpline, found := findSplineByID(splines, currentSplineID)
	if !found {
		return 0, 0, false
	}
	startsByNode := buildStartsByNode(splines)
	for _, nextIdx := range startsByNode[pointKey(currentSpline.P3)] {
		nextSpline := splines[nextIdx]
		for _, coupledID := range nextSpline.HardCoupledIDs {
			if _, _, pathOk := findShortestPathWeighted(splines, coupledID, destSplineID, vehicleCounts); pathOk {
				return nextSpline.ID, coupledID, true
			}
		}
	}
	return 0, 0, false
}

// laneChangeLandingDist computes the arc-length position on destSpline where a
// lane change from car's current position would land, applying the same geometry
// rules as buildLaneChangeBridge. Returns (p3Dist, true) when feasible.
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

// isLaneChangeLandingSafe returns true if landing at p3Dist on destSplineID
// would not immediately collide with cars already there.
// Rear cars are given a speed-adjusted gap (2 s of closure distance).
func isLaneChangeLandingSafe(p3Dist float32, destSplineID int, switchingCar Car, cars []Car) bool {
	T := 2 * laneChangeHalfSecs // lane change duration in seconds
	const safetyMargin = 2.0
	for _, other := range cars {
		if other.CurrentSplineID != destSplineID {
			continue
		}
		halfLengths := (switchingCar.Length + other.Length) / 2
		// Predict where each car will be when the merge completes.
		// Switching car lands at p3Dist. Other car travels other.Speed*T further.
		otherPosAtLanding := other.DistanceOnSpline + other.Speed*T
		gapAtLanding := (otherPosAtLanding - p3Dist) // positive = other is ahead, negative = behind
		if gapAtLanding >= 0 {
			// Other car will be ahead at landing — bumper gap must be positive.
			if gapAtLanding < halfLengths+safetyMargin {
				return false
			}
		} else {
			// Other car will be behind at landing — switching car rear must clear its front.
			if -gapAtLanding < halfLengths+safetyMargin {
				return false
			}
		}
	}
	return true
}

// buildLaneChangeBridge attempts to build a Bézier bridge from the car's
// current position to destSplineID. Modifies *car and appends to lcs on
// success. Returns the updated lcs slice and whether it succeeded.
// When forced is true the minimum speed check is skipped and a floor of
// laneChangeMinSpeed is used for halfDist, so slow cars can still switch.
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
	lcs = append(lcs, newSpline(id, carPos, p1, p2, p3))

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
			speedA := effectiveMaxSpeedMPS(splineA)
			speedB := effectiveMaxSpeedMPS(splineB)
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
func nearestSampleOnSpline(spline Spline, pos rl.Vector2) rl.Vector2 {
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
func nearestSampleWithDist(spline Spline, pos rl.Vector2) (point rl.Vector2, dist float32) {
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

// mergedSplines returns a slice containing all permanent splines followed by
// all lane-change splines. Returns permanent directly if there are no temporaries.
func mergedSplines(permanent, temporary []Spline) []Spline {
	if len(temporary) == 0 {
		return permanent
	}
	all := make([]Spline, 0, len(permanent)+len(temporary))
	all = append(all, permanent...)
	all = append(all, temporary...)
	return all
}

// gcLaneChangeSplines removes any lane-change splines that no car is currently
// travelling on, freeing memory and keeping the slice compact.
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

// computeLaneChanges handles desired-lane switches for each car.
// Cars with a DesiredLaneSplineID attempt to merge as soon as a safe gap
// exists; at the deadline the switch is forced regardless of gap.
// Preference-based switches are attempted at most every preferenceChangeCooldownS seconds.
func computeLaneChanges(cars []Car, splines []Spline, lcs []Spline, nextID *int, vehicleCounts map[int]int, dt float32) ([]Spline, []Car) {
	splineIndexByID := buildSplineIndexByID(splines)

	for i := range cars {
		car := &cars[i]

		if car.LaneChanging {
			continue
		}

		// Desired lane switch: attempt as soon as a safe gap exists on the
		// target spline. At the deadline (10 m before end) force it regardless.
		if car.DesiredLaneSplineID >= 0 {
			if car.DistanceOnSpline >= car.DesiredLaneDeadline {
				// Last resort — ignore gap, just switch.
				if newLcs, ok := buildLaneChangeBridge(car, car.DesiredLaneSplineID, splines, splineIndexByID, lcs, nextID, true); ok {
					lcs = newLcs
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
			} else {
				// Desired switch: only commit if the landing zone is clear.
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

		// Preference-based lane change: opportunistically move to a coupled lane
		// with a higher preference (lower number). Gated by a cooldown so the
		// behaviour is soft — cars drift toward preferred lanes, not snap to them.
		car.PreferenceCooldown -= dt
		if car.PreferenceCooldown <= 0 {
			car.PreferenceCooldown = preferenceChangeCooldownS
			for _, destID := range findBetterPreferenceLaneCandidates(*car, splines, splineIndexByID) {
				if _, _, pathOk := findShortestPathWeighted(splines, destID, car.DestinationSplineID, vehicleCounts); !pathOk {
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

		// Overtake: if the car has been held back by a leader long enough, try
		// moving to an adjacent lane that is exactly one preference step less
		// preferred (number one higher). Never forced; requires a safe gap and a
		// valid path to the destination. Resets PreferenceCooldown on success so
		// the car does not immediately try to return to the preferred lane.
		car.OvertakeCooldown -= dt
		leaderSpeed, leaderFound := nearestLeaderSpeed(i, cars, splines, splineIndexByID)
		if car.SlowedTimer > overtakeSlowThresholdS && car.OvertakeCooldown <= 0 &&
			(!leaderFound || leaderSpeed <= car.Speed) {
			car.OvertakeCooldown = overtakeCooldownS
			for _, destID := range findOvertakeLaneCandidates(*car, splines, splineIndexByID) {
				if _, _, pathOk := findShortestPathWeighted(splines, destID, car.DestinationSplineID, vehicleCounts); !pathOk {
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

// findBetterPreferenceLaneCandidates returns coupled lanes ordered from most to
// least preferred among lanes that are strictly better than the current one.
// Lanes with no preference (0) are never considered as targets.
func findBetterPreferenceLaneCandidates(car Car, splines []Spline, splineIndexByID map[int]int) []int {
	idx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return nil
	}
	currentSpline := splines[idx]
	currentPref := currentSpline.LanePreference // 0 = no preference assigned

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
			continue // unpreferred lanes are never a target
		}
		// Must be strictly better (lower number) than the current lane.
		// A car on an unpreferred lane (0) considers any preferred lane better.
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

// nearestLeaderSpeed returns the speed of the closest car ahead on the same
// spline as cars[carIdx], and true. Returns 0, false if no such car is found
// within followLookaheadM.
func nearestLeaderSpeed(carIdx int, cars []Car, splines []Spline, splineIndexByID map[int]int) (float32, bool) {
	car := cars[carIdx]
	splineIdx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return 0, false
	}
	spline := splines[splineIdx]
	carPos, carHeading := sampleSplineAtDistance(spline, car.DistanceOnSpline)

	bestDist := float32(math.MaxFloat32)
	bestSpeed := float32(0)
	found := false
	for j, other := range cars {
		if j == carIdx {
			continue
		}
		if other.CurrentSplineID != car.CurrentSplineID {
			continue
		}
		otherIdx, ok2 := splineIndexByID[other.CurrentSplineID]
		if !ok2 {
			continue
		}
		otherPos, _ := sampleSplineAtDistance(splines[otherIdx], other.DistanceOnSpline)
		diff := rl.Vector2{X: otherPos.X - carPos.X, Y: otherPos.Y - carPos.Y}
		proj := diff.X*carHeading.X + diff.Y*carHeading.Y
		if proj <= 0 {
			continue // behind us
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

// findOvertakeLaneCandidates returns coupled lanes whose LanePreference is
// exactly currentPref+1 (one step less preferred — the overtaking lane).
// Only works when the car is on a numbered lane; lanes with no preference are
// never considered as targets.
func findOvertakeLaneCandidates(car Car, splines []Spline, splineIndexByID map[int]int) []int {
	idx, ok := splineIndexByID[car.CurrentSplineID]
	if !ok {
		return nil
	}
	currentSpline := splines[idx]
	currentPref := currentSpline.LanePreference
	if currentPref <= 0 {
		return nil // can only overtake from a lane that has a preference number
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

// drawLaneChangeSplines renders active lane-change bridge splines in a
// distinctive magenta colour, visible only when debug mode is on.
func drawLaneChangeSplines(lcs []Spline, zoom float32) {
	if len(lcs) == 0 {
		return
	}
	color := rl.NewColor(230, 60, 230, 220)
	dimColor := rl.NewColor(230, 60, 230, 100)
	thickness := pixelsToWorld(zoom, 2.5)
	handleR := pixelsToWorld(zoom, 4)
	armThick := pixelsToWorld(zoom, 1)

	for _, s := range lcs {
		drawSpline(s, thickness, color)
		// Draw the Bézier handles so it's clear where the curve is steered.
		rl.DrawLineEx(s.P0, s.P1, armThick, dimColor)
		rl.DrawLineEx(s.P3, s.P2, armThick, dimColor)
		rl.DrawCircleV(s.P1, handleR, color)
		rl.DrawCircleV(s.P2, handleR, color)
	}
}

// drawCoupleMode draws coupling relationship lines between coupled splines
// and highlights the currently selected first spline.
func drawCoupleMode(splines []Spline, firstSelectedID int, hoveredSpline int, zoom float32) {
	hardColor := rl.NewColor(80, 180, 255, 180)  // blue — hard coupling
	softColor := rl.NewColor(180, 120, 255, 180) // purple — soft coupling
	selectedColor := rl.NewColor(255, 200, 50, 255)
	hoveredColor := rl.NewColor(255, 140, 30, 200)
	thickness := pixelsToWorld(zoom, 2)
	r := pixelsToWorld(zoom, 5)

	drawLinks := func(ids []int, spline Spline, color rl.Color) {
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
			rl.DrawLineEx(midA, midB, thickness, color)
			rl.DrawCircleV(midA, r, color)
			rl.DrawCircleV(midB, r, color)
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
		rl.DrawCircleV(mid, pixelsToWorld(zoom, 8), hoveredColor)
	}

	// Highlight the first selected spline.
	if firstSelectedID >= 0 {
		idx := findSplineIndexByID(splines, firstSelectedID)
		if idx >= 0 {
			mid := splines[idx].Samples[simSamples/2]
			rl.DrawCircleV(mid, pixelsToWorld(zoom, 10), selectedColor)
		}
	}
}

func newCutDraft() CutDraft {
	return CutDraft{OriginalSplineID: -1}
}

// splitBezierAt splits a cubic Bézier at parameter t using de Casteljau's algorithm.
// Returns control points for the left (t=0..t) and right (t..1) sub-curves.
func splitBezierAt(p0, p1, p2, p3 rl.Vector2, t float32) ([4]rl.Vector2, [4]rl.Vector2) {
	lerp2 := func(a, b rl.Vector2, t float32) rl.Vector2 {
		return rl.Vector2{X: a.X + (b.X-a.X)*t, Y: a.Y + (b.Y-a.Y)*t}
	}
	a1 := lerp2(p0, p1, t)
	a2 := lerp2(p1, p2, t)
	a3 := lerp2(p2, p3, t)
	b1 := lerp2(a1, a2, t)
	b2 := lerp2(a2, a3, t)
	cut := lerp2(b1, b2, t)
	left := [4]rl.Vector2{p0, a1, b1, cut}
	right := [4]rl.Vector2{cut, b2, a3, p3}
	return left, right
}

// findNearestSplinePoint searches all precomputed samples across all splines and
// returns the one closest to pos. Returns splineIndex, sample t (0..1), world position.
func findNearestSplinePoint(splines []Spline, pos rl.Vector2) (splineIndex int, t float32, point rl.Vector2, found bool) {
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

func handleCutMode(stage Stage, cd CutDraft, splines []Spline, mouseWorld rl.Vector2, nextSplineID int) (Stage, CutDraft, []Spline, int, bool) {
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
		mirror := rl.Vector2{X: 2*cd.CutPoint.X - H.X, Y: 2*cd.CutPoint.Y - H.Y}

		// Build the two final splines. Left keeps its P0/P1, gets new P2=mirror.
		// Right gets new P1=H, keeps its P2/P3.
		newLeft := newSpline(nextSplineID, cd.LeftP[0], cd.LeftP[1], mirror, cd.LeftP[3])
		newLeft.Priority = cd.OriginalSplinePriority
		nextSplineID++
		newRight := newSpline(nextSplineID, cd.RightP[0], H, cd.RightP[2], cd.RightP[3])
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

func drawCutMode(stage Stage, cd CutDraft, splines []Spline, mouseWorld rl.Vector2, zoom float32) {
	snapColor := rl.NewColor(255, 180, 0, 255)
	handleColor := rl.NewColor(255, 220, 60, 220)
	leftColor := rl.NewColor(80, 210, 120, 255)
	rightColor := rl.NewColor(80, 140, 230, 255)
	thickness := pixelsToWorld(zoom, 3)
	handleR := pixelsToWorld(zoom, 5)
	snapR := pixelsToWorld(zoom, 8)

	switch stage {
	case StageIdle:
		// Snap dot follows the nearest point on any spline.
		_, _, point, found := findNearestSplinePoint(splines, mouseWorld)
		if found {
			rl.DrawCircleV(point, snapR, snapColor)
			// Small crosshair lines.
			arm := pixelsToWorld(zoom, 10)
			rl.DrawLineEx(rl.Vector2{X: point.X - arm, Y: point.Y}, rl.Vector2{X: point.X + arm, Y: point.Y}, pixelsToWorld(zoom, 1.5), snapColor)
			rl.DrawLineEx(rl.Vector2{X: point.X, Y: point.Y - arm}, rl.Vector2{X: point.X, Y: point.Y + arm}, pixelsToWorld(zoom, 1.5), snapColor)
		}

	case StageSetP1:
		H := mouseWorld
		mirror := rl.Vector2{X: 2*cd.CutPoint.X - H.X, Y: 2*cd.CutPoint.Y - H.Y}

		// Preview left spline (green): original P0/P1, new P2=mirror, cut point.
		previewLeft := newSpline(0, cd.LeftP[0], cd.LeftP[1], mirror, cd.LeftP[3])
		// Preview right spline (blue): cut point, H, original P2, original P3.
		previewRight := newSpline(0, cd.RightP[0], H, cd.RightP[2], cd.RightP[3])

		drawSpline(previewLeft, thickness, leftColor)
		drawSpline(previewRight, thickness, rightColor)

		// Handle lines from cut point to both control handles.
		rl.DrawLineEx(cd.CutPoint, H, pixelsToWorld(zoom, 1.5), handleColor)
		rl.DrawLineEx(cd.CutPoint, mirror, pixelsToWorld(zoom, 1.5), handleColor)
		rl.DrawCircleV(H, handleR, rightColor)
		rl.DrawCircleV(mirror, handleR, leftColor)

		// Cut point marker.
		rl.DrawCircleV(cd.CutPoint, snapR, snapColor)
	}
}

func updateRoutePanel(panel RoutePanel, routes []Route, cars []Car, nextRouteID *int) (RoutePanel, []Route, []Car, bool) {
	panelRect := rl.NewRectangle(float32(rl.GetScreenWidth())-360, 18, 340, 234)
	sliderRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+82, panelRect.Width-36, 22)
	applyRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+180, 120, 32)
	cancelRect := rl.NewRectangle(panelRect.X+202, panelRect.Y+180, 120, 32)
	mouse := rl.GetMousePosition()
	applied := false

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
			panel.SpawnPerMinute = sliderValueFromMouse(mouse.X, sliderRect, 0, spawnSliderMaxPerMinute)
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
			panel.SpawnPerMinute = sliderValueFromMouse(mouse.X, sliderRect, 0, spawnSliderMaxPerMinute)
		} else {
			panel.DraggingSlider = false
		}
	}

	return panel, routes, cars, applied
}

func applyRoutePanel(panel RoutePanel, routes []Route, cars []Car, nextRouteID *int) (RoutePanel, []Route, []Car) {
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
			routes[idx].Color = routePaletteColor(panel.ColorIndex)
			if routes[idx].NextSpawnIn <= 0 {
				routes[idx].NextSpawnIn = randomizedSpawnDelay(spawn)
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
		Color:          routePaletteColor(panel.ColorIndex),
	}
	routes = append(routes, route)
	return RoutePanel{}, routes, cars
}

func refreshRoutes(routes []Route, splines []Spline) []Route {
	return updateRouteVisuals(routes, splines, map[int]int{})
}

func updateRouteVisuals(routes []Route, splines []Spline, vehicleCounts map[int]int) []Route {
	for i := range routes {
		pathIDs, pathLength, ok := findShortestPathWeighted(splines, routes[i].StartSplineID, routes[i].EndSplineID, vehicleCounts)
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
		candidate := spawnCar(routes[i])
		if !spawnBlocked(candidate, cars, splines) {
			cars = append(cars, candidate)
			routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
		}
		// If blocked, NextSpawnIn stays ≤ 0 so we retry next frame.
	}
	return routes, cars
}

// spawnBlocked returns true if placing candidate at its spawn point would
// immediately overlap with any existing car using the two-circle hitbox.
func spawnBlocked(candidate Car, cars []Car, splines []Spline) bool {
	spline, ok := findSplineByID(splines, candidate.CurrentSplineID)
	if !ok {
		return false
	}
	pos, heading := sampleSplineAtDistance(spline, 0)
	rC := collisionRadius(candidate)
	offC := circleOffset(candidate)
	cFront := vecAdd(pos, vecScale(heading, offC))
	cBack := vecSub(pos, vecScale(heading, offC))

	for _, other := range cars {
		if other.CurrentSplineID != candidate.CurrentSplineID {
			continue // only cars on the same spline can overlap with a fresh spawn
		}
		otherSpline, ok := findSplineByID(splines, other.CurrentSplineID)
		if !ok {
			continue
		}
		oPos, oHeading := sampleSplineAtDistance(otherSpline, other.DistanceOnSpline)
		rO := collisionRadius(other)
		thresh := rC + rO
		threshSq := thresh * thresh
		offO := circleOffset(other)
		oFront := vecAdd(oPos, vecScale(oHeading, offO))
		oBack := vecSub(oPos, vecScale(oHeading, offO))

		if distSq(cFront, oFront) <= threshSq ||
			distSq(cFront, oBack) <= threshSq ||
			distSq(cBack, oFront) <= threshSq ||
			distSq(cBack, oBack) <= threshSq {
			return true
		}
	}
	return false
}

func spawnCar(route Route) Car {
	// All values in SI units: metres, m/s, m/s².
	// Typical passenger car: 4.0-4.8 m long, 1.8-2.0 m wide.
	// Speed range: ~50-130 km/h = 13.9-36.1 m/s.
	// Acceleration: 2.5-4.5 m/s² (comfortable urban driving).
	return Car{
		RouteID:              route.ID,
		CurrentSplineID:      route.StartSplineID,
		DestinationSplineID:  route.EndSplineID,
		PrevSplineIDs:        [2]int{-1, -1},
		DistanceOnSpline:     0,
		Speed:                randRange(0, 2),                     // m/s — starts nearly stationary
		MaxSpeed:             randRange(22.2, 36.1),               // m/s — 80–130 km/h
		Accel:                randRange(2.5, 4.5),                 // m/s²
		Length:               randRange(4.0, 4.8) / metersPerUnit, // world units
		Width:                randRange(1.8, 2.0) / metersPerUnit, // world units
		CurveSpeedMultiplier: randRange(0.8, 1.2),
		Color:                route.Color,
		Braking:              false,
		LaneChangeSplineID:   -1,
		AfterSplineID:        -1,

		DesiredLaneSplineID: -1,
		PreferenceCooldown:  rand.Float32() * preferenceChangeCooldownS,
		OvertakeCooldown:    rand.Float32() * overtakeCooldownS,
	}
}

// recentlyLeft returns true if the car was on splineID within its last two transitions.
// Used to suppress false blame when a car exits a priority lane onto a normal lane.
func recentlyLeft(car Car, splineID int) bool {
	return splineID >= 0 && (car.PrevSplineIDs[0] == splineID || car.PrevSplineIDs[1] == splineID)
}

func computeBrakingDecisions(cars []Car, splines []Spline, vehicleCounts map[int]int) ([]bool, []bool, []DebugBlameLink, []DebugBlameLink) {
	flags := make([]bool, len(cars))
	holdSpeed := make([]bool, len(cars))
	initialBlame := make([]bool, len(cars))
	tentativeLinks := make([]DebugBlameLink, 0)
	holdLinks := make([]DebugBlameLink, 0)
	if len(cars) < 2 {
		return flags, holdSpeed, tentativeLinks, holdLinks
	}

	predictions := make([][]TrajectorySample, len(cars))
	stationaryPredictions := make([][]TrajectorySample, len(cars)) // lazily populated on first blame check
	for i, car := range cars {
		predictions[i] = predictCarTrajectory(car, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
	}

	for i := 0; i < len(cars); i++ {
		if len(predictions[i]) == 0 {
			continue
		}
		for j := i + 1; j < len(cars); j++ {
			if len(predictions[j]) == 0 {
				continue
			}
			collision, ok := predictCollision(predictions[i], predictions[j], cars[i], cars[j])
			if !ok {
				continue
			}
			blameI, blameJ := determineBlame(collision, cars[i], cars[j], splines)
			// Suppress blame if the blamed car recently left the other car's current spline —
			// it has already cleared that segment and the other car is following behind.
			if blameI && recentlyLeft(cars[i], cars[j].CurrentSplineID) {
				blameI = false
			}
			if blameJ && recentlyLeft(cars[j], cars[i].CurrentSplineID) {
				blameJ = false
			}
			// Suppress blame if the conflict would happen even when the blamed car is
			// standing still — braking can never resolve such a conflict.
			if blameI {
				if stationaryPredictions[i] == nil {
					sc := cars[i]
					sc.Speed = 0
					stationaryPredictions[i] = predictCarTrajectory(sc, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
				}
				if _, still := predictCollision(stationaryPredictions[i], predictions[j], cars[i], cars[j]); still {
					blameI = false
				}
			}
			if blameJ {
				if stationaryPredictions[j] == nil {
					sc := cars[j]
					sc.Speed = 0
					stationaryPredictions[j] = predictCarTrajectory(sc, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
				}
				if _, still := predictCollision(stationaryPredictions[j], predictions[i], cars[j], cars[i]); still {
					blameJ = false
				}
			}
			if blameI {
				initialBlame[i] = true
				tentativeLinks = append(tentativeLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
			}
			if blameJ {
				initialBlame[j] = true
				tentativeLinks = append(tentativeLinks, DebugBlameLink{FromCarIndex: j, ToCarIndex: i})
			}
		}
	}

	for i := range cars {
		if !initialBlame[i] {
			continue
		}
		if shouldBrakeForBlamedConflicts(i, cars, splines, vehicleCounts, predictions) {
			flags[i] = true
		}
	}

	// For cars that are not braking, check whether they would be blamed for a
	// collision if travelling slightly faster (current speed + 0.25s of acceleration).
	// If so, hold the current speed rather than accelerating into the conflict.
	for i, car := range cars {
		if flags[i] || len(predictions[i]) == 0 {
			continue
		}
		fasterCar := car
		fasterCar.Speed += 0.25 * fasterCar.Accel
		if fasterCar.Speed <= car.Speed+1e-4 {
			continue // already at or near max — nothing to hold back
		}
		fasterPred := predictCarTrajectory(fasterCar, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
		if len(fasterPred) == 0 {
			continue
		}
		for j := range cars {
			if i == j || len(predictions[j]) == 0 {
				continue
			}
			collision, ok := predictCollision(fasterPred, predictions[j], fasterCar, cars[j])
			if !ok {
				continue
			}
			blameI, _ := determineBlame(collision, fasterCar, cars[j], splines)
			if blameI && !recentlyLeft(fasterCar, cars[j].CurrentSplineID) {
				// Suppress if the collision is unavoidable even when standing still —
				// consistent with how braking blame is suppressed above.
				if stationaryPredictions[i] == nil {
					sc := cars[i]
					sc.Speed = 0
					stationaryPredictions[i] = predictCarTrajectory(sc, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
				}
				if _, still := predictCollision(stationaryPredictions[i], predictions[j], cars[i], cars[j]); !still {
					holdSpeed[i] = true
					holdLinks = append(holdLinks, DebugBlameLink{FromCarIndex: i, ToCarIndex: j})
					break
				}
			}
		}
	}

	debugLinks := make([]DebugBlameLink, 0, len(tentativeLinks))
	for _, link := range tentativeLinks {
		if link.FromCarIndex >= 0 && link.FromCarIndex < len(flags) && flags[link.FromCarIndex] {
			debugLinks = append(debugLinks, link)
		}
	}

	return flags, holdSpeed, debugLinks, holdLinks
}

func shouldBrakeForBlamedConflicts(carIndex int, cars []Car, splines []Spline, vehicleCounts map[int]int, predictions [][]TrajectorySample) bool {
	if carIndex < 0 || carIndex >= len(cars) {
		return false
	}

	car := cars[carIndex]
	currentSpline, ok := findSplineByID(splines, car.CurrentSplineID)
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

		testPrediction := predictCarTrajectory(testCar, splines, vehicleCounts, predictionHorizonSeconds, predictionStepSeconds)
		if len(testPrediction) == 0 {
			return true
		}
		if hasBlamedConflictWithPrediction(carIndex, testCar, testPrediction, cars, splines, predictions) {
			return true
		}
	}

	return false
}

func hasBlamedConflictWithPrediction(carIndex int, testCar Car, testPrediction []TrajectorySample, cars []Car, splines []Spline, predictions [][]TrajectorySample) bool {
	for otherIndex, otherCar := range cars {
		if otherIndex == carIndex || otherIndex >= len(predictions) || len(predictions[otherIndex]) == 0 {
			continue
		}
		if recentlyLeft(testCar, otherCar.CurrentSplineID) {
			continue
		}

		collision, ok := predictCollision(testPrediction, predictions[otherIndex], testCar, otherCar)
		if !ok {
			continue
		}

		blameTestCar, _ := determineBlame(collision, testCar, otherCar, splines)
		if blameTestCar {
			return true
		}
	}
	return false
}

func predictCarTrajectory(car Car, splines []Spline, vehicleCounts map[int]int, horizon, step float32) []TrajectorySample {
	if len(splines) == 0 || car.CurrentSplineID < 0 {
		return nil
	}

	steps := int(math.Ceil(float64(horizon / step)))
	samples := make([]TrajectorySample, 0, steps+1)
	simCar := car
	speed := maxf(car.Speed, 0)
	active := true

	for stepIndex := 0; stepIndex <= steps; stepIndex++ {
		spline, ok := findSplineByID(splines, simCar.CurrentSplineID)
		if !ok {
			break
		}
		pos, heading := sampleSplineAtDistance(spline, simCar.DistanceOnSpline)
		samples = append(samples, TrajectorySample{
			Time:     float32(stepIndex) * step,
			Position: pos,
			Heading:  heading,
			Priority: spline.Priority,
			SplineID: spline.ID,
		})

		if !active {
			break
		}
		if stepIndex == steps || speed <= 0.01 {
			continue
		}

		move := speed * step
		for move > 0 && active {
			currentSpline, ok := findSplineByID(splines, simCar.CurrentSplineID)
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
			simCar.DistanceOnSpline = 0

			if simCar.CurrentSplineID == simCar.DestinationSplineID {
				active = false
				break
			}

			// Lane-change bridge completed in trajectory sim.
			if simCar.LaneChanging && simCar.CurrentSplineID == simCar.LaneChangeSplineID {
				simCar.PrevSplineIDs[1] = simCar.PrevSplineIDs[0]
				simCar.PrevSplineIDs[0] = simCar.CurrentSplineID
				simCar.CurrentSplineID = simCar.AfterSplineID
				simCar.DistanceOnSpline = simCar.AfterSplineDist
				simCar.LaneChanging = false
				simCar.LaneChangeSplineID = -1
				continue
			}

			nextSplineID, ok := chooseNextSplineOnBestPath(splines, simCar.CurrentSplineID, simCar.DestinationSplineID, vehicleCounts)
			if !ok {
				forcedNext, _, forcedOk := findForcedLaneChangePath(splines, simCar.CurrentSplineID, simCar.DestinationSplineID, vehicleCounts)
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

func predictCollision(aSamples, bSamples []TrajectorySample, carA, carB Car) (CollisionPrediction, bool) {
	count := len(aSamples)
	if len(bSamples) < count {
		count = len(bSamples)
	}
	if count == 0 {
		return CollisionPrediction{}, false
	}

	// Two-circle approximation: one circle at L/4 ahead of centre, one at L/4 behind.
	// Radius for each circle: sqrt((L/4)² + (W/2)²) — covers the half-rectangle corners.
	rA := collisionRadius(carA)
	rB := collisionRadius(carB)
	circleDist := rA + rB
	circleDistSq := circleDist * circleDist
	offA := circleOffset(carA)
	offB := circleOffset(carB)

	for i := 0; i < count; i++ {
		pA, hA := aSamples[i].Position, aSamples[i].Heading
		pB, hB := bSamples[i].Position, bSamples[i].Heading

		aFront := vecAdd(pA, vecScale(hA, offA))
		aBack := vecSub(pA, vecScale(hA, offA))
		bFront := vecAdd(pB, vecScale(hB, offB))
		bBack := vecSub(pB, vecScale(hB, offB))

		if distSq(aFront, bFront) > circleDistSq &&
			distSq(aFront, bBack) > circleDistSq &&
			distSq(aBack, bFront) > circleDistSq &&
			distSq(aBack, bBack) > circleDistSq {
			continue
		}
		prevIndex := i - 1
		if prevIndex < 0 {
			prevIndex = 0
		}
		return CollisionPrediction{
			Time:      aSamples[i].Time,
			PosA:      pA,
			PosB:      pB,
			PrevPosA:  aSamples[prevIndex].Position,
			PrevPosB:  bSamples[prevIndex].Position,
			HeadingA:  hA,
			HeadingB:  hB,
			PriorityA: aSamples[i].Priority,
			PriorityB: bSamples[i].Priority,
			SplineAID: aSamples[i].SplineID,
			SplineBID: bSamples[i].SplineID,
		}, true
	}

	return CollisionPrediction{}, false
}

func determineBlame(collision CollisionPrediction, carA, carB Car, splines []Spline) (bool, bool) {
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
	pos, heading := sampleSplineAtDistance(currentSpline, car.DistanceOnSpline)
	offset := circleOffset(car)
	radius := collisionRadius(car)
	radiusSq := radius * radius

	front := vecAdd(pos, vecScale(heading, offset))
	back := vecSub(pos, vecScale(heading, offset))

	nearestFront := nearestSampleOnSpline(targetSpline, front)
	if distSq(front, nearestFront) <= radiusSq {
		return true
	}
	nearestBack := nearestSampleOnSpline(targetSpline, back)
	return distSq(back, nearestBack) <= radiusSq
}

// computeFollowingSpeedCaps returns, for each car, the maximum speed it should
// target due to a leading car ahead on the same path. The cap equals the
// leader's current speed. Returns math.MaxFloat32 (no cap) when no leader is
// found within followLookaheadM or the car is already braking.
func computeFollowingSpeedCaps(cars []Car, splines []Spline, vehicleCounts map[int]int) []float32 {
	caps := make([]float32, len(cars))
	for i := range caps {
		caps[i] = math.MaxFloat32
	}

	splineIndexByID := buildSplineIndexByID(splines)

	// Pre-compute world positions and headings for all cars.
	type carPose struct {
		pos     rl.Vector2
		heading rl.Vector2
	}
	poses := make([]carPose, len(cars))
	for i, car := range cars {
		splineIdx, ok := splineIndexByID[car.CurrentSplineID]
		if !ok {
			continue
		}
		p, h := sampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
		poses[i] = carPose{p, h}
	}

	// Pre-compute the set of spline IDs each car will traverse within followLookaheadM.
	pathSets := make([]map[int]bool, len(cars))
	for i, car := range cars {
		set := map[int]bool{car.CurrentSplineID: true}
		covered := float32(0)
		curIdx, ok := splineIndexByID[car.CurrentSplineID]
		if ok {
			// Distance remaining on the current spline.
			covered -= (splines[curIdx].Length - car.DistanceOnSpline)
			curID := car.CurrentSplineID
			for covered < followLookaheadM {
				nextID, ok2 := chooseNextSplineOnBestPath(splines, curID, car.DestinationSplineID, vehicleCounts)
				if !ok2 {
					break
				}
				set[nextID] = true
				nextIdx, ok3 := splineIndexByID[nextID]
				if !ok3 {
					break
				}
				covered += splines[nextIdx].Length
				curID = nextID
			}
		}
		pathSets[i] = set
	}

	for i, car := range cars {
		hI := poses[i].heading
		pI := poses[i].pos
		desiredGap := followMinGapM + car.Speed*followTimeHeadwaySecs

		bestDist := float32(math.MaxFloat32)
		bestSpeed := float32(math.MaxFloat32)

		for j, other := range cars {
			if i == j {
				continue
			}
			// Other car must be on this car's predicted path.
			if !pathSets[i][other.CurrentSplineID] {
				continue
			}
			// Must be going in roughly the same direction.
			hJ := poses[j].heading
			dot := hI.X*hJ.X + hI.Y*hJ.Y
			if dot < followHeadingCos {
				continue
			}
			// Other car must be ahead (positive projection along our heading).
			diff := rl.Vector2{X: poses[j].pos.X - pI.X, Y: poses[j].pos.Y - pI.Y}
			proj := diff.X*hI.X + diff.Y*hI.Y
			if proj <= 0 {
				continue
			}
			// Within lookahead range.
			euclidean := float32(math.Sqrt(float64(diff.X*diff.X + diff.Y*diff.Y)))
			if euclidean > followLookaheadM {
				continue
			}
			// Gap = euclidean distance minus half-lengths of both cars.
			gap := euclidean - car.Length/2 - other.Length/2
			if gap > desiredGap {
				continue
			}
			// Pick the closest leader.
			if euclidean < bestDist {
				bestDist = euclidean
				bestSpeed = other.Speed
			}
		}
		caps[i] = bestSpeed
		_ = bestDist
	}
	return caps
}

func updateCars(cars []Car, routes []Route, splines []Spline, vehicleCounts map[int]int, brakingDecisions []bool, holdSpeedDecisions []bool, followCaps []float32, lights []TrafficLight, cycles []TrafficCycle, dt float32) []Car {
	if len(cars) == 0 {
		return cars
	}

	routeIndexByID := map[int]int{}
	for i, route := range routes {
		routeIndexByID[route.ID] = i
	}

	alive := cars[:0]
	for i, car := range cars {
		routeIdx, ok := routeIndexByID[car.RouteID]
		if !ok || !routes[routeIdx].Valid {
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

		// Accumulate frustration time: only counts when a leader is present AND
		// the car is at least 10 km/h below its preferred speed on this spline.
		const frustrateThreshMPS = 10.0 / 3.6
		if !shouldBrake && followCap < float32(math.MaxFloat32) {
			frustrated := false
			if spline, ok := findSplineByID(splines, car.CurrentSplineID); ok {
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
			currentSpline, ok := findSplineByID(splines, car.CurrentSplineID)
			if !ok {
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
			if at := computeAnticipatoryTargetSpeed(car, currentSpline, splines, vehicleCounts); at < targetSpeed {
				targetSpeed = at
			}
			if tl := computeTrafficLightSpeedCap(car, currentSpline, splines, lights, cycles, vehicleCounts); tl < targetSpeed {
				targetSpeed = tl
			}
			// If a forced lane switch is pending, begin decelerating to
			// laneChangeForcedSpeedMPS in time to reach the deadline position.
			if car.DesiredLaneSplineID >= 0 {
				remaining := car.DesiredLaneDeadline - car.DistanceOnSpline
				if remaining >= 0 && car.Speed > laneChangeForcedSpeedMPS {
					// Use the normal coast-down deceleration (accel * 1.5), not the
					// emergency-brake multiplier, since that's what actually applies
					// here. A small safety factor (0.9) ensures we start a little early.
					normalDecel := car.Accel * 1.5 * 0.9
					brakingDist := (car.Speed*car.Speed - laneChangeForcedSpeedMPS*laneChangeForcedSpeedMPS) /
						(2 * normalDecel)
					if remaining <= brakingDist && targetSpeed > laneChangeForcedSpeedMPS {
						targetSpeed = laneChangeForcedSpeedMPS
					}
				}
			}
			if car.Braking {
				targetSpeed = 0
			} else if followCap < targetSpeed {
				// Following distance: don't accelerate beyond leader's speed.
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
				alive = append(alive, car)
				break
			}

			overshoot := car.DistanceOnSpline - currentSpline.Length
			car.DistanceOnSpline = overshoot
			if car.CurrentSplineID == car.DestinationSplineID {
				break
			}

			// Lane-change bridge completed: resume on the destination lane.
			if car.LaneChanging && car.CurrentSplineID == car.LaneChangeSplineID {
				car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
				car.PrevSplineIDs[0] = car.CurrentSplineID
				car.CurrentSplineID = car.AfterSplineID
				car.DistanceOnSpline = car.AfterSplineDist + overshoot
				car.LaneChanging = false
				car.LaneChangeSplineID = -1
				// If the car arrived on its desired lane, clear the forced-switch state.
				if car.CurrentSplineID == car.DesiredLaneSplineID {
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
				continue
			}

			nextSplineID, ok := chooseNextSplineOnBestPath(splines, car.CurrentSplineID, car.DestinationSplineID, vehicleCounts)
			if ok {
				// Normal path found — any pending forced-switch is no longer needed.
				if car.DesiredLaneSplineID >= 0 {
					car.DesiredLaneSplineID = -1
					car.DesiredLaneDeadline = 0
				}
			} else {
				// No direct path: check whether entering a physical next spline whose
				// hard-coupled partner has a path can rescue the car.
				forcedNext, desiredLane, forcedOk := findForcedLaneChangePath(splines, car.CurrentSplineID, car.DestinationSplineID, vehicleCounts)
				if !forcedOk {
					break
				}
				nextSplineID = forcedNext
				car.DesiredLaneSplineID = desiredLane
				// Deadline: 10 m before the end of the forced spline.
				if src, srcOk := findSplineByID(splines, forcedNext); srcOk {
					car.DesiredLaneDeadline = maxf(src.Length-laneChangeForcedDistEnd, 0)
				}
			}
			car.PrevSplineIDs[1] = car.PrevSplineIDs[0]
			car.PrevSplineIDs[0] = car.CurrentSplineID
			car.CurrentSplineID = nextSplineID
		}
	}
	return alive
}

func drawCars(cars []Car, splines []Spline, splineIndexByID map[int]int, zoom float32, debugMode bool) {
	if len(cars) == 0 {
		return
	}
	for _, car := range cars {
		splineIdx, ok := splineIndexByID[car.CurrentSplineID]
		if !ok {
			continue
		}
		pos, tangent := sampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
		angle := float32(math.Atan2(float64(tangent.Y), float64(tangent.X)) * 180 / math.Pi)
		rect := rl.NewRectangle(pos.X, pos.Y, car.Length, car.Width)
		origin := rl.NewVector2(car.Length/2, car.Width/2)
		rl.DrawRectanglePro(rect, origin, angle, car.Color)
		if car.Braking {
			rl.DrawCircleV(pos, maxf(car.Width*0.22, pixelsToWorld(zoom, 2)), rl.NewColor(220, 50, 50, 255))
		} else if debugMode && car.SoftSlowing {
			rl.DrawCircleV(pos, maxf(car.Width*0.22, pixelsToWorld(zoom, 2)), rl.NewColor(60, 120, 220, 255))
		}
	}
}

// drawCarSpeedLabels draws each car's current speed in km/h in screen space,
// centred just above the car's position.
func drawCarSpeedLabels(cars []Car, splines []Spline, splineIndexByID map[int]int, camera rl.Camera2D) {
	fontSize := int32(12)
	for _, car := range cars {
		splineIdx, ok := splineIndexByID[car.CurrentSplineID]
		if !ok {
			continue
		}
		pos, _ := sampleSplineAtDistance(splines[splineIdx], car.DistanceOnSpline)
		screen := rl.GetWorldToScreen2D(pos, camera)
		label := fmt.Sprintf("%d", int(car.Speed*3.6))
		textW := measureText(label, fontSize)
		drawText(label, int32(screen.X)-textW/2, int32(screen.Y)-18, fontSize, rl.Black)
	}
}

func drawDebugBlameLinks(links []DebugBlameLink, cars []Car, splines []Spline, splineIndexByID map[int]int, zoom float32, lineColor rl.Color) {
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

		fromPos, _ := sampleSplineAtDistance(splines[fromSplineIdx], cars[link.FromCarIndex].DistanceOnSpline)
		toPos, _ := sampleSplineAtDistance(splines[toSplineIdx], cars[link.ToCarIndex].DistanceOnSpline)
		rl.DrawLineEx(fromPos, toPos, lineThickness, lineColor)
		rl.DrawCircleV(fromPos, pixelsToWorld(zoom, 3.5), lineColor)
	}
}

func drawRouteWithIndex(route Route, splines []Spline, indexByID map[int]int, thickness float32, zoom float32) {
	color := rl.NewColor(route.Color.R, route.Color.G, route.Color.B, 90)
	for _, pathID := range route.PathIDs {
		idx, ok := indexByID[pathID]
		if !ok {
			continue
		}
		drawSpline(splines[idx], thickness, color)
	}
	spawnR := pixelsToWorld(zoom, 10)
	destR := pixelsToWorld(zoom, 10)
	if start, ok := findSplineByID(splines, route.StartSplineID); ok {
		drawEndpoint(start.P0, spawnR, route.Color)
	}
	if end, ok := findSplineByID(splines, route.EndSplineID); ok {
		drawEndpoint(end.P3, destR, color)
	}
}

func drawRoute(route Route, splines []Spline, thickness float32, zoom float32) {
	drawRouteWithIndex(route, splines, buildSplineIndexByID(splines), thickness, zoom)
}

func drawRoutePicking(routeStartSplineID int, routePanel RoutePanel, hoveredStart EndHit, hoveredEnd EndHit, splines []Spline, vehicleCounts map[int]int, zoom float32) {
	handleRadius := pixelsToWorld(zoom, handlePixels)
	if hoveredStart.SplineIndex >= 0 && routeStartSplineID < 0 {
		drawEndpoint(hoveredStart.Point, handleRadius*1.6, rl.NewColor(255, 196, 61, 255))
	}
	if routeStartSplineID >= 0 {
		if startSpline, ok := findSplineByID(splines, routeStartSplineID); ok {
			drawEndpoint(startSpline.P0, handleRadius*1.8, rl.NewColor(214, 76, 76, 255))
		}
		if hoveredEnd.SplineIndex >= 0 {
			drawEndpoint(hoveredEnd.Point, handleRadius*1.5, rl.NewColor(35, 85, 175, 255))
			if pathIDs, _, ok := findShortestPathWeighted(splines, routeStartSplineID, hoveredEnd.SplineID, vehicleCounts); ok {
				previewRoute := Route{PathIDs: pathIDs, Color: routePaletteColor(routePanel.ColorIndex), Valid: true}
				drawRoute(previewRoute, splines, pixelsToWorld(zoom, 4), zoom)
			}
		}
	}
}

func drawRoutePanel(panel RoutePanel, routes []Route) {
	panelRect := rl.NewRectangle(float32(rl.GetScreenWidth())-360, 18, 340, 234)
	sliderRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+82, panelRect.Width-36, 22)
	applyRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+180, 120, 32)
	cancelRect := rl.NewRectangle(panelRect.X+202, panelRect.Y+180, 120, 32)

	bg := rl.NewColor(248, 248, 250, 245)
	outline := rl.NewColor(210, 210, 215, 255)
	textCol := rl.NewColor(30, 30, 35, 255)
	muted := rl.NewColor(90, 90, 100, 255)
	accent := rl.NewColor(70, 110, 220, 255)
	button := rl.NewColor(235, 236, 240, 255)

	rl.DrawRectangle(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), bg)
	rl.DrawRectangleLines(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), outline)
	title := "Create route"
	if panel.ExistingRouteID >= 0 {
		title = "Edit existing route"
	}
	drawText(title, int32(panelRect.X+18), int32(panelRect.Y+16), 22, textCol)
	drawText(fmt.Sprintf("Start spline #%d → end spline #%d", panel.StartSplineID, panel.EndSplineID), int32(panelRect.X+18), int32(panelRect.Y+44), 18, muted)
	meanSeconds := float32(0)
	if panel.SpawnPerMinute > 0 {
		meanSeconds = 60 / panel.SpawnPerMinute
	}
	drawText(fmt.Sprintf("Average spawn frequency: %.1f cars/min  (mean %.1fs)", panel.SpawnPerMinute, meanSeconds), int32(panelRect.X+18), int32(panelRect.Y+64), 18, textCol)

	rl.DrawRectangle(int32(sliderRect.X), int32(sliderRect.Y), int32(sliderRect.Width), int32(sliderRect.Height), rl.NewColor(229, 229, 234, 255))
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
			rl.DrawRectangleLinesEx(rl.NewRectangle(sx-1, sy-1, swatchSize+2, swatchSize+2), 1, rl.NewColor(40, 40, 40, 200))
		} else {
			rl.DrawRectangleLinesEx(sr, 1, rl.NewColor(0, 0, 0, 40))
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
	drawText(fmt.Sprintf("Current weighted cost: %.0f", panel.PathLength), int32(panelRect.X+18), int32(panelRect.Y+220), 16, muted)
	_ = routes
}

func drawNotice(text string) {
	width := measureText(text, 18) + 28
	x := int32((int32(rl.GetScreenWidth()) - width) / 2)
	y := int32(rl.GetScreenHeight() - 54)
	bg := rl.NewColor(40, 44, 52, 230)
	rl.DrawRectangle(x, y, width, 34, bg)
	drawText(text, x+14, y+8, 18, rl.White)
}

func modeStatusText(mode EditorMode, stage Stage, draft Draft, routeStartSplineID, coupleModeFirstID int) string {
	switch mode {
	case ModeEdit:
		return stageLabel(stage, draft)
	case ModeRoute:
		if routeStartSplineID >= 0 {
			return fmt.Sprintf("Pick destination for start spline #%d", routeStartSplineID)
		}
		return "Click a spline start to begin a route"
	case ModePriority:
		return "Left click: set priority   Right click: clear"
	case ModeCouple:
		if coupleModeFirstID >= 0 {
			return fmt.Sprintf("Click second spline to couple/decouple with #%d  (right-click cancels)", coupleModeFirstID)
		}
		return "Click a spline to select it"
	case ModeCut:
		if stage == StageSetP1 {
			return "Place tangent handle at cut point  (right-click cancels)"
		}
		return "Click a spline to cut it"
	case ModeSpeedLimit:
		return "Left click: apply speed limit   Right click: remove"
	case ModePreference:
		return "Left click: assign 1 (reset counter)   Right click on empty: next number   Right click on assigned: remove"
	case ModeTrafficLight:
		return "Left click on spline: add light to cycle   Right click on light: remove from cycle   Then press Create"
	}
	return ""
}

func drawHud(mode EditorMode, stage Stage, draft Draft, hoveredSpline int, routeStartSplineID int, coupleModeFirstID int, debugMode bool, zoom float32, splineCount, routeCount, carCount int) {
	mouse := rl.GetMousePosition()

	bgNormal := rl.NewColor(245, 245, 248, 245)
	bgActive := rl.NewColor(47, 96, 198, 255)
	bgHover := rl.NewColor(218, 224, 238, 245)
	outNormal := rl.NewColor(200, 200, 206, 255)
	outActive := rl.NewColor(28, 62, 155, 255)
	txtDark := rl.NewColor(28, 28, 33, 255)
	txtMuted := rl.NewColor(100, 100, 110, 255)

	for i, item := range toolbarItems {
		r := toolbarBtnRect(i)
		isActive := (!item.isDbg && item.mode == mode) || (item.isDbg && debugMode)
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

	// Status text below toolbar
	status := modeStatusText(mode, stage, draft, routeStartSplineID, coupleModeFirstID)
	drawText(status, int32(toolbarX), int32(toolbarY+toolbarBtnH+7), 13, txtMuted)

	// Stats — top right
	stats := fmt.Sprintf("Splines: %d   Routes: %d   Cars: %d   Zoom: %.1f×", splineCount, routeCount, carCount, zoom)
	statsW := measureText(stats, 13)
	drawText(stats, int32(rl.GetScreenWidth())-statsW-12, 14, 13, txtMuted)
}

func stageLabel(stage Stage, draft Draft) string {
	switch stage {
	case StageIdle:
		return "Idle"
	case StageSetP1:
		if draft.LockP1 {
			return "P1 locked for continuity (click to confirm)"
		}
		return "Choose P1"
	case StageSetP2:
		return "Choose P2"
	case StageSetP3:
		return "Choose P3"
	default:
		return "Unknown"
	}
}

func newDraft() Draft {
	return Draft{ContinuationFrom: -1}
}

func newSpline(id int, p0, p1, p2, p3 rl.Vector2) Spline {
	s := Spline{ID: id, P0: p0, P1: p1, P2: p2, P3: p3}
	cacheSpline(&s)
	return s
}

func splineDrawColor(s Spline) rl.Color {
	if s.Priority {
		return rl.NewColor(130, 75, 215, 255)
	}
	return rl.NewColor(35, 85, 175, 255)
}

func cacheSpline(s *Spline) {
	s.Samples[0] = s.P0
	s.CumLen[0] = 0
	total := float32(0)
	prev := s.P0
	for i := 1; i <= simSamples; i++ {
		t := float32(i) / float32(simSamples)
		pt := rl.GetSplinePointBezierCubic(s.P0, s.P1, s.P2, s.P3, t)
		total += float32(math.Sqrt(float64(distSq(prev, pt))))
		s.Samples[i] = pt
		s.CumLen[i] = total
		prev = pt
	}
	s.Length = total
	s.SpeedFactor = 1.0
	s.CurveSpeedMPS = buildCurveSpeedProfile(s)
}

// buildCurveSpeedProfile computes the maximum speed at every curveSpeedIntervalM
// metres along the spline. Speed is derived from local curvature and capped by
// the spline's SpeedLimitKmh (if set), so anticipatory braking applies to speed
// limits the same way it applies to curves.
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

// curveSpeedAtArcLen returns the maximum speed at arc-length d based on local
// curvature. Uses the circumradius of three neighbouring samples to estimate
// the radius of curvature, then applies v = sqrt(maxLateralAccelMPS2 * r).
func curveSpeedAtArcLen(s *Spline, d float32) float32 {
	// Find sample index nearest to arc-length d.
	idx := 0
	for idx < simSamples && s.CumLen[idx+1] < d {
		idx++
	}
	// Choose three consecutive samples, clamped to valid range.
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

	// |cross| = 2 * triangle area; circumradius r = (ab*bc*ca) / (2*|cross|)
	cross := absf((B.X-A.X)*(C.Y-A.Y) - (B.Y-A.Y)*(C.X-A.X))
	if cross < 1e-6 {
		return maxCarSpeed // essentially straight
	}
	r := ab * bc * ca / (2 * cross)
	v := float32(math.Sqrt(float64(maxLateralAccelMPS2 * r)))
	if v > maxCarSpeed {
		return maxCarSpeed
	}
	return v
}

// lookupCurveSpeed returns the precomputed curvature speed limit at the given
// arc-length on the spline. Returns maxCarSpeed if no profile is available.
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

// trafficLightShouldStop returns true when the light is part of an enabled cycle
// that is currently showing red or yellow — i.e. the car should try to stop.
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
		state := trafficLightState(l.ID, l.CycleID, cycles)
		return state == TrafficRed || state == TrafficYellow
	}
	return false
}

// computeTrafficLightSpeedCap looks ahead on the car's path and returns the
// maximum speed the car should be doing now so it can decelerate to a stop at
// each upcoming red/yellow light.  Uses the same kinematic formula as
// computeAnticipatoryTargetSpeed (reqSpeed=0 case): allowed = sqrt(2·decel·d).
// If physics genuinely can't stop the car in time (light just turned red very
// close ahead) the car will still decelerate as hard as it can and coast
// through — that is the natural "run the light" edge case.
func computeTrafficLightSpeedCap(car Car, currentSpline Spline, splines []Spline,
	lights []TrafficLight, cycles []TrafficCycle, vehicleCounts map[int]int) float32 {

	decel := car.Accel * 1.5
	result := float32(math.MaxFloat32)
	remaining := currentSpline.Length - car.DistanceOnSpline

	// Lookahead: stopping distance from current speed + small buffer, capped at 200 m.
	lookahead := car.Speed*car.Speed/(2*decel) + 20
	if lookahead > 200 {
		lookahead = 200
	}

	checkLight := func(rawDistAhead float32) {
		// DistanceOnSpline is measured from the car's centre, so subtract a
		// full car length to make the front stop at least car.Length before
		// the light position.
		adj := rawDistAhead - car.Length
		if adj <= 0 {
			// Car centre is already within car.Length of the light — must stop
			// immediately and stay stopped while the light is red.
			result = 0
			return
		}
		allowed := float32(math.Sqrt(float64(2 * decel * adj)))
		if allowed < result {
			result = allowed
		}
	}

	for _, l := range lights {
		if l.SplineID != currentSpline.ID {
			continue
		}
		if l.DistOnSpline <= car.DistanceOnSpline {
			continue // already passed
		}
		if !trafficLightShouldStop(l, cycles) {
			continue
		}
		checkLight(l.DistOnSpline - car.DistanceOnSpline)
	}

	// Also check lights on the next spline if the lookahead extends past the end.
	if remaining < lookahead {
		nextID, ok := chooseNextSplineOnBestPath(splines, currentSpline.ID, car.DestinationSplineID, vehicleCounts)
		if ok {
			if _, ok2 := findSplineByID(splines, nextID); ok2 {
				for _, l := range lights {
					if l.SplineID != nextID {
						continue
					}
					if !trafficLightShouldStop(l, cycles) {
						continue
					}
					checkLight(remaining + l.DistOnSpline)
				}
			}
		}
	}

	return result
}

// computeAnticipatoryTargetSpeed looks ahead along the car's path and returns
// the maximum speed the car should target NOW so that it can decelerate in time
// for every upcoming speed constraint within lookahead range.
//
// For each constraint at distance d ahead with required speed v_req:
//
//	allowed_now = sqrt(v_req² + 2·decel·d)
//
// The minimum across all checked points is returned.
// Lookahead extends from the car's position to at most 150 m (or the braking
// distance from current speed, whichever is smaller). If the lookahead reaches
// the end of the current spline, the next most-likely spline is scanned too.
func computeAnticipatoryTargetSpeed(car Car, currentSpline Spline, splines []Spline, vehicleCounts map[int]int) float32 {
	decel := car.Accel * 1.5
	// Look far enough to brake from the current speed, plus a couple of extra
	// fragments as buffer. Cap at 150 m.
	lookahead := car.Speed*car.Speed/(2*decel) + 2*curveSpeedIntervalM
	if lookahead > 150 {
		lookahead = 150
	}

	result := float32(math.MaxFloat32)
	remaining := currentSpline.Length - car.DistanceOnSpline

	checkConstraint := func(reqSpeed, distAhead float32) {
		allowed := float32(math.Sqrt(float64(reqSpeed*reqSpeed + 2*decel*distAhead)))
		if allowed < result {
			result = allowed
		}
	}

	splineReqSpeed := func(s Spline, pos float32) float32 {
		spd := lookupCurveSpeed(s, pos) * car.CurveSpeedMultiplier
		if s.SpeedLimitKmh > 0 {
			if lim := s.SpeedLimitKmh / 3.6; lim < spd {
				spd = lim
			}
		}
		return spd
	}

	// Scan ahead on the current spline in curveSpeedIntervalM steps.
	for d := curveSpeedIntervalM; d <= lookahead; d += curveSpeedIntervalM {
		if d > remaining {
			break
		}
		checkConstraint(splineReqSpeed(currentSpline, car.DistanceOnSpline+d), d)
	}

	// If the lookahead extends past the end of the current spline, scan the
	// next most-likely spline from its beginning.
	if remaining < lookahead {
		nextID, ok := chooseNextSplineOnBestPath(splines, currentSpline.ID, car.DestinationSplineID, vehicleCounts)
		if ok {
			if nextSpline, ok2 := findSplineByID(splines, nextID); ok2 {
				budget := lookahead - remaining
				for pos := float32(0); pos <= minf(nextSpline.Length, budget); pos += curveSpeedIntervalM {
					checkConstraint(splineReqSpeed(nextSpline, pos), remaining+pos)
				}
			}
		}
	}

	if result == float32(math.MaxFloat32) {
		return maxCarSpeed
	}
	return result
}

func buildPreview(stage Stage, draft Draft, mouse rl.Vector2, snapStart EndHit, splines []Spline) (Spline, bool) {
	switch stage {
	case StageSetP1:
		if draft.LockP1 {
			return newSpline(-1, draft.P0, draft.P1, mouse, mouse), true
		}
		return newSpline(-1, draft.P0, mouse, mouse, mouse), true
	case StageSetP2:
		return newSpline(-1, draft.P0, draft.P1, mouse, mouse), true
	case StageSetP3:
		if snapStart.SplineIndex >= 0 {
			next := splines[snapStart.SplineIndex]
			p3 := next.P0
			p2 := vecSub(vecScale(p3, 2), next.P1)
			return newSpline(-1, draft.P0, draft.P1, p2, p3), true
		}
		return newSpline(-1, draft.P0, draft.P1, draft.P2, mouse), true
	default:
		return Spline{}, false
	}
}

func drawDraft(stage Stage, draft Draft, mouse rl.Vector2, zoom float32) {
	lineThickness := pixelsToWorld(zoom, 1.5)
	handleRadius := pixelsToWorld(zoom, handlePixels)
	guide := rl.NewColor(140, 140, 140, 255)
	locked := rl.NewColor(130, 75, 215, 255)

	drawEndpoint(draft.P0, handleRadius, rl.NewColor(215, 67, 67, 255))

	switch stage {
	case StageSetP1:
		if draft.LockP1 {
			rl.DrawLineEx(draft.P0, draft.P1, lineThickness, locked)
			drawEndpoint(draft.P1, handleRadius, locked)
		} else {
			rl.DrawLineEx(draft.P0, mouse, lineThickness, guide)
			drawEndpoint(mouse, handleRadius, guide)
		}
	case StageSetP2:
		rl.DrawLineEx(draft.P0, draft.P1, lineThickness, guide)
		rl.DrawLineEx(draft.P1, mouse, lineThickness, guide)
		drawEndpoint(draft.P1, handleRadius, mapBoolColor(draft.LockP1, locked, guide))
		drawEndpoint(mouse, handleRadius, guide)
	case StageSetP3:
		rl.DrawLineEx(draft.P0, draft.P1, lineThickness, guide)
		rl.DrawLineEx(draft.P1, draft.P2, lineThickness, guide)
		rl.DrawLineEx(draft.P2, mouse, lineThickness, guide)
		drawEndpoint(draft.P1, handleRadius, mapBoolColor(draft.LockP1, locked, guide))
		drawEndpoint(draft.P2, handleRadius, guide)
		drawEndpoint(mouse, handleRadius, rl.NewColor(35, 85, 175, 255))
	}
}

// segmentAngleDeg returns the clockwise angle from East for the vector from→to,
// normalised to [0, 360). Screen Y is down, so this matches screen intuition.
func segmentAngleDeg(from, to rl.Vector2) float32 {
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
func drawSegmentLabel(from, to rl.Vector2, camera rl.Camera2D) {
	mx := (from.X + to.X) / 2
	my := (from.Y + to.Y) / 2
	mid := rl.GetWorldToScreen2D(rl.NewVector2(mx, my), camera)

	dx := to.X - from.X
	dy := to.Y - from.Y
	length := float32(math.Sqrt(float64(dx*dx+dy*dy))) * metersPerUnit
	angle := segmentAngleDeg(from, to)

	// Perpendicular offset in screen space (rotate segment dir 90° left).
	segScreenFrom := rl.GetWorldToScreen2D(from, camera)
	segScreenTo := rl.GetWorldToScreen2D(to, camera)
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

	rl.DrawRectangle(tx-2, ty-2, tw+4, 20, rl.NewColor(30, 30, 35, 180))
	drawText(text, tx+3, ty+1, 14, rl.White)
}

// drawArcLabel draws the arc length at the midpoint of the preview spline.
func drawArcLabel(preview Spline, camera rl.Camera2D) {
	if preview.Length <= 0 {
		return
	}
	midPos, tangent := sampleSplineAtDistance(preview, preview.Length/2)
	screen := rl.GetWorldToScreen2D(midPos, camera)

	// Offset perpendicular to tangent so the label doesn't sit on the curve.
	px := -tangent.Y
	py := tangent.X
	ox := px * 16
	oy := py * 16

	text := fmt.Sprintf("arc %.1f m", preview.Length*metersPerUnit)
	tw := measureText(text, 14) + 6
	tx := int32(screen.X+ox) - tw/2
	ty := int32(screen.Y+oy) - 9

	rl.DrawRectangle(tx-2, ty-2, tw+4, 20, rl.NewColor(20, 80, 160, 200))
	drawText(text, tx+3, ty+1, 14, rl.NewColor(180, 220, 255, 255))
}

// drawDraftInfo draws measurement labels directly on the draft segments and arc.
func drawDraftInfo(stage Stage, draft Draft, mouseWorld rl.Vector2, preview Spline, camera rl.Camera2D) {
	switch stage {
	case StageSetP1:
		if draft.LockP1 {
			drawSegmentLabel(draft.P1, mouseWorld, camera)
			drawArcLabel(preview, camera)
		} else {
			drawSegmentLabel(draft.P0, mouseWorld, camera)
		}
	case StageSetP2:
		drawSegmentLabel(draft.P1, mouseWorld, camera)
		drawArcLabel(preview, camera)
	case StageSetP3:
		drawSegmentLabel(draft.P2, mouseWorld, camera)
		drawArcLabel(preview, camera)
	}
}

func mapBoolColor(condition bool, whenTrue, whenFalse rl.Color) rl.Color {
	if condition {
		return whenTrue
	}
	return whenFalse
}

func drawGrid(camera rl.Camera2D) {
	topLeft := rl.GetScreenToWorld2D(rl.NewVector2(0, 0), camera)
	bottomRight := rl.GetScreenToWorld2D(rl.NewVector2(float32(rl.GetScreenWidth()), float32(rl.GetScreenHeight())), camera)
	minX := minf(topLeft.X, bottomRight.X)
	maxX := maxf(topLeft.X, bottomRight.X)
	minY := minf(topLeft.Y, bottomRight.Y)
	maxY := maxf(topLeft.Y, bottomRight.Y)
	zoom := camera.Zoom

	type level struct {
		spacing float32
		color   rl.Color
		pxThick float32 // constant screen-pixel thickness
		showMin float32 // only show when zoom >= showMin (0 = no minimum)
		showMax float32 // only show when zoom <= showMax (0 = no maximum)
	}

	// Draw thinnest first so thicker lines paint on top.
	levels := []level{
		{4, rl.NewColor(220, 220, 226, 255), 1.0, 6.0, 0},
		{20, rl.NewColor(185, 185, 196, 255), 1.5, 0.8, 0},
		{100, rl.NewColor(148, 148, 162, 255), 2.0, 0, 0},
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
			rl.DrawLineEx(rl.NewVector2(x, minY), rl.NewVector2(x, maxY), thick, lvl.color)
		}
		for y := float32(math.Floor(float64(minY/sp))) * sp; y <= maxY; y += sp {
			rl.DrawLineEx(rl.NewVector2(minX, y), rl.NewVector2(maxX, y), thick, lvl.color)
		}
	}
}

func drawAxes(camera rl.Camera2D) {
	topLeft := rl.GetScreenToWorld2D(rl.NewVector2(0, 0), camera)
	bottomRight := rl.GetScreenToWorld2D(rl.NewVector2(float32(rl.GetScreenWidth()), float32(rl.GetScreenHeight())), camera)
	minX := minf(topLeft.X, bottomRight.X)
	maxX := maxf(topLeft.X, bottomRight.X)
	minY := minf(topLeft.Y, bottomRight.Y)
	maxY := maxf(topLeft.Y, bottomRight.Y)

	axis := rl.NewColor(180, 180, 185, 255)
	rl.DrawLineV(rl.NewVector2(0, minY), rl.NewVector2(0, maxY), axis)
	rl.DrawLineV(rl.NewVector2(minX, 0), rl.NewVector2(maxX, 0), axis)
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
	barColor := rl.NewColor(50, 50, 55, 220)

	// Horizontal bar with end ticks
	rl.DrawLineEx(rl.NewVector2(barX, barY), rl.NewVector2(barX+barPx, barY), 2, barColor)
	rl.DrawLineEx(rl.NewVector2(barX, barY-5), rl.NewVector2(barX, barY+5), 2, barColor)
	rl.DrawLineEx(rl.NewVector2(barX+barPx, barY-5), rl.NewVector2(barX+barPx, barY+5), 2, barColor)

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
	camera.Target = vecAdd(camera.Target, vecSub(before, after))
}

func drawSpline(s Spline, thickness float32, color rl.Color) {
	prev := s.P0
	for i := 1; i <= curveSamples; i++ {
		t := float32(i) / float32(curveSamples)
		curr := rl.GetSplinePointBezierCubic(s.P0, s.P1, s.P2, s.P3, t)
		rl.DrawLineEx(prev, curr, thickness, color)
		prev = curr
	}
}

func drawEndpoint(p rl.Vector2, radius float32, color rl.Color) {
	rl.DrawCircleV(p, radius, color)
}

func findNearbyEnd(splines []Spline, point rl.Vector2, radius float32) EndHit {
	best := EndHit{SplineIndex: -1, SplineID: -1, Kind: EndNone}
	bestDistSq := radius * radius
	for i, spline := range splines {
		d := distSq(point, spline.P3)
		if d <= bestDistSq {
			bestDistSq = d
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndFinish, Point: spline.P3}
		}
	}
	return best
}

func findNearbyStart(splines []Spline, point rl.Vector2, radius float32) EndHit {
	best := EndHit{SplineIndex: -1, SplineID: -1, Kind: EndNone}
	bestDistSq := radius * radius
	for i, spline := range splines {
		d := distSq(point, spline.P0)
		if d <= bestDistSq {
			bestDistSq = d
			best = EndHit{SplineIndex: i, SplineID: spline.ID, Kind: EndStart, Point: spline.P0}
		}
	}
	return best
}

func findHoveredSpline(splines []Spline, point rl.Vector2, radius float32) int {
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

func splineDistanceSq(s Spline, point rl.Vector2) float32 {
	best := float32(math.MaxFloat32)
	prev := s.P0
	for i := 1; i <= hoverSamples; i++ {
		t := float32(i) / float32(hoverSamples)
		curr := rl.GetSplinePointBezierCubic(s.P0, s.P1, s.P2, s.P3, t)
		d := pointSegmentDistanceSq(point, prev, curr)
		if d < best {
			best = d
		}
		prev = curr
	}
	return best
}

func pointSegmentDistanceSq(p, a, b rl.Vector2) float32 {
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

func buildVehicleCounts(cars []Car) map[int]int {
	counts := make(map[int]int)
	for _, car := range cars {
		counts[car.CurrentSplineID]++
	}
	return counts
}

func findShortestPathWeighted(splines []Spline, startSplineID, endSplineID int, vehicleCounts map[int]int) ([]int, float32, bool) {
	if len(splines) == 0 {
		return nil, 0, false
	}
	indexByID := buildSplineIndexByID(splines)
	startIdx, okStart := indexByID[startSplineID]
	endIdx, okEnd := indexByID[endSplineID]
	if !okStart || !okEnd {
		return nil, 0, false
	}

	startsByNode := buildStartsByNode(splines)

	const inf = float32(1e30)
	dist := make([]float32, len(splines))
	prev := make([]int, len(splines))
	visited := make([]bool, len(splines))
	for i := range dist {
		dist[i] = inf
		prev[i] = -1
	}
	dist[startIdx] = segmentTravelCost(splines[startIdx], vehicleCounts)

	for i := 0; i < len(splines); i++ {
		u := -1
		best := inf
		for j := range splines {
			if !visited[j] && dist[j] < best {
				best = dist[j]
				u = j
			}
		}
		if u < 0 {
			break
		}
		if u == endIdx {
			break
		}
		visited[u] = true
		nextNodes := expandWithCoupledNeighbors(startsByNode[pointKey(splines[u].P3)], splines, indexByID)
		for _, v := range nextNodes {
			alt := dist[u] + segmentTravelCost(splines[v], vehicleCounts)
			if alt < dist[v] {
				dist[v] = alt
				prev[v] = u
			}
		}
	}

	if dist[endIdx] >= inf/2 {
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
		pathIDs = append(pathIDs, splines[idx].ID)
	}
	return pathIDs, dist[endIdx], true
}

func chooseNextSplineOnBestPath(splines []Spline, currentSplineID, destinationSplineID int, vehicleCounts map[int]int) (int, bool) {
	currentSpline, ok := findSplineByID(splines, currentSplineID)
	if !ok {
		return 0, false
	}
	startsByNode := buildStartsByNode(splines)
	candidateIndices := startsByNode[pointKey(currentSpline.P3)]
	if len(candidateIndices) == 0 {
		return 0, false
	}

	bestSplineID := 0
	bestCost := float32(1e30)
	found := false
	for _, idx := range candidateIndices {
		candidate := splines[idx]
		_, cost, ok := findShortestPathWeighted(splines, candidate.ID, destinationSplineID, vehicleCounts)
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

// expandWithCoupledNeighbors takes a slice of spline indices (direct neighbours
// reachable by a physical P3→P0 connection) and appends the indices of any
// splines that are coupled with those direct neighbours, without cascading
// further (i.e. only one level of coupling is added).
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
	return s.Length * float32(1+vehicleCounts[s.ID])
}

func sampleSplineAtDistance(s Spline, distance float32) (rl.Vector2, rl.Vector2) {
	if s.Length <= 0 {
		return s.P0, rl.NewVector2(1, 0)
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
			pos := rl.GetSplinePointBezierCubic(s.P0, s.P1, s.P2, s.P3, t)
			tangent := bezierDerivative(s, t)
			return pos, normalize(tangent)
		}
	}
	return s.P3, normalize(vecSub(s.P3, s.P2))
}

func bezierDerivative(s Spline, t float32) rl.Vector2 {
	u := 1 - t
	term0 := vecScale(vecSub(s.P1, s.P0), 3*u*u)
	term1 := vecScale(vecSub(s.P2, s.P1), 6*u*t)
	term2 := vecScale(vecSub(s.P3, s.P2), 3*t*t)
	return vecAdd(vecAdd(term0, term1), term2)
}

func buildSplineIndexByID(splines []Spline) map[int]int {
	m := make(map[int]int, len(splines))
	for i, s := range splines {
		m[s.ID] = i
	}
	return m
}

func findSplineByID(splines []Spline, id int) (Spline, bool) {
	for _, s := range splines {
		if s.ID == id {
			return s, true
		}
	}
	return Spline{}, false
}

func findRouteID(routes []Route, startSplineID, endSplineID int) int {
	for _, route := range routes {
		if route.StartSplineID == startSplineID && route.EndSplineID == endSplineID {
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

func pickSplineFilePath(save bool) (string, error) {
	home, _ := os.UserHomeDir()
	startDir := home
	if startDir == "" {
		startDir = "."
	}
	defaultPath := filepath.Join(startDir, "splines.json")

	type picker struct {
		name string
		run  func(bool, string) (string, error)
	}

	pickers := []picker{
		{name: "zenity", run: pickFileWithZenity},
		{name: "yad", run: pickFileWithYad},
		{name: "kdialog", run: pickFileWithKDialog},
	}

	var missing []string
	for _, picker := range pickers {
		if _, err := exec.LookPath(picker.name); err != nil {
			missing = append(missing, picker.name)
			continue
		}
		path, err := picker.run(save, defaultPath)
		if err != nil {
			return "", err
		}
		return normalizePickedPath(path, save), nil
	}

	return "", fmt.Errorf("no supported file dialog found; install zenity, yad, or kdialog")
}

func pickFileWithZenity(save bool, defaultPath string) (string, error) {
	args := []string{"--file-selection", "--title=Select spline file"}
	if save {
		args = append(args, "--save", "--confirm-overwrite")
	}
	args = append(args, "--filename="+defaultPath, "--file-filter=Spline JSON | *.json")
	return runDialogCommand("zenity", args...)
}

func pickFileWithYad(save bool, defaultPath string) (string, error) {
	args := []string{"--file", "--title=Select spline file"}
	if save {
		args = append(args, "--save", "--confirm-overwrite")
	}
	args = append(args, "--filename="+defaultPath, "--file-filter=*.json")
	return runDialogCommand("yad", args...)
}

func pickFileWithKDialog(save bool, defaultPath string) (string, error) {
	filter := "JSON files (*.json)"
	if save {
		return runDialogCommand("kdialog", "--getsavefilename", defaultPath, filter)
	}
	return runDialogCommand("kdialog", "--getopenfilename", defaultPath, filter)
}

func runDialogCommand(name string, args ...string) (string, error) {
	cmd := exec.Command(name, args...)
	output, err := cmd.Output()
	if err != nil {
		if exitErr, ok := err.(*exec.ExitError); ok {
			code := exitErr.ExitCode()
			if code == 1 {
				return "", nil
			}
			stderr := strings.TrimSpace(string(exitErr.Stderr))
			if stderr != "" {
				return "", fmt.Errorf("%s: %s", name, stderr)
			}
			return "", fmt.Errorf("%s exited with code %d", name, code)
		}
		return "", err
	}
	return strings.TrimSpace(string(output)), nil
}

func normalizePickedPath(path string, save bool) string {
	path = strings.TrimSpace(path)
	path = strings.TrimPrefix(path, "file://")
	if path == "" {
		return ""
	}
	if save && filepath.Ext(path) == "" {
		path += ".json"
	}
	return path
}

func saveSplineFile(splines []Spline, routes []Route, cars []Car, lights []TrafficLight, cycles []TrafficCycle, path string) error {
	saved := SavedSplineFile{
		Splines:       make([]SavedSpline, 0, len(splines)),
		Routes:        make([]SavedRoute, 0, len(routes)),
		Cars:          make([]SavedCar, 0, len(cars)),
		TrafficLights: make([]SavedTrafficLight, 0, len(lights)),
		TrafficCycles: make([]SavedTrafficCycle, 0, len(cycles)),
	}
	for _, spline := range splines {
		saved.Splines = append(saved.Splines, SavedSpline{
			ID:             spline.ID,
			Priority:       spline.Priority,
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
	for _, route := range routes {
		saved.Routes = append(saved.Routes, SavedRoute{
			ID:             route.ID,
			StartSplineID:  route.StartSplineID,
			EndSplineID:    route.EndSplineID,
			PathIDs:        route.PathIDs,
			SpawnPerMinute: route.SpawnPerMinute,
			ColorIndex:     route.ColorIndex,
		})
	}
	for _, car := range cars {
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
		})
	}

	for _, l := range lights {
		saved.TrafficLights = append(saved.TrafficLights, SavedTrafficLight{
			ID:           l.ID,
			SplineID:     l.SplineID,
			DistOnSpline: l.DistOnSpline,
			WorldPosX:    l.WorldPos.X,
			WorldPosY:    l.WorldPos.Y,
			CycleID:      l.CycleID,
		})
	}
	for _, c := range cycles {
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

func loadSplineFile(path string) ([]Spline, []Route, []Car, []TrafficLight, []TrafficCycle, int, int, int, int, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, nil, nil, nil, nil, 1, 1, 1, 1, err
	}

	var saved SavedSplineFile
	if err := json.Unmarshal(data, &saved); err != nil {
		return nil, nil, nil, nil, nil, 1, 1, 1, 1, err
	}

	loadedSplines := make([]Spline, 0, len(saved.Splines))
	maxSplineID := 0
	for _, entry := range saved.Splines {
		spline := newSpline(entry.ID, entry.P0, entry.P1, entry.P2, entry.P3)
		spline.Priority = entry.Priority
		spline.HardCoupledIDs = append([]int(nil), entry.HardCoupledIDs...)
		spline.SoftCoupledIDs = append([]int(nil), entry.SoftCoupledIDs...)
		spline.SpeedLimitKmh = entry.SpeedLimitKmh
		spline.LanePreference = entry.LanePreference
		if spline.SpeedLimitKmh > 0 {
			spline.CurveSpeedMPS = buildCurveSpeedProfile(&spline)
		}
		loadedSplines = append(loadedSplines, spline)
		if entry.ID > maxSplineID {
			maxSplineID = entry.ID
		}
	}

	loadedRoutes := make([]Route, 0, len(saved.Routes))
	maxRouteID := 0
	for _, entry := range saved.Routes {
		route := Route{
			ID:             entry.ID,
			StartSplineID:  entry.StartSplineID,
			EndSplineID:    entry.EndSplineID,
			PathIDs:        entry.PathIDs,
			SpawnPerMinute: entry.SpawnPerMinute,
			ColorIndex:     entry.ColorIndex,
			Color:          routePaletteColor(entry.ColorIndex),
		}
		if entry.ID > maxRouteID {
			maxRouteID = entry.ID
		}
		loadedRoutes = append(loadedRoutes, route)
	}

	loadedCars := make([]Car, 0, len(saved.Cars))
	routeColorByID := make(map[int]rl.Color, len(loadedRoutes))
	for _, r := range loadedRoutes {
		routeColorByID[r.ID] = r.Color
	}
	for _, entry := range saved.Cars {
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

			DesiredLaneSplineID: -1,
			PreferenceCooldown:  rand.Float32() * preferenceChangeCooldownS,
		}
		if car.CurveSpeedMultiplier == 0 {
			car.CurveSpeedMultiplier = randRange(0.8, 1.2)
		}
		loadedCars = append(loadedCars, car)
	}

	loadedLights := make([]TrafficLight, 0, len(saved.TrafficLights))
	maxLightID := 0
	for _, entry := range saved.TrafficLights {
		loadedLights = append(loadedLights, TrafficLight{
			ID:           entry.ID,
			SplineID:     entry.SplineID,
			DistOnSpline: entry.DistOnSpline,
			WorldPos:     rl.NewVector2(entry.WorldPosX, entry.WorldPosY),
			CycleID:      entry.CycleID,
		})
		if entry.ID > maxLightID {
			maxLightID = entry.ID
		}
	}

	loadedCycles := make([]TrafficCycle, 0, len(saved.TrafficCycles))
	maxCycleID := 0
	for _, entry := range saved.TrafficCycles {
		phases := make([]TrafficPhase, len(entry.Phases))
		for i, p := range entry.Phases {
			clrDur := p.ClearanceDurationSecs
			if clrDur <= 0 {
				// backward compat: fall back to cycle-level yellow_duration_secs if present
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
		loadedCycles = append(loadedCycles, TrafficCycle{
			ID:         entry.ID,
			LightIDs:   nil, // not saved; rebuilt from lights
			Phases:     phases,
			Timer:      0,
			PhaseIndex: 0,
			Enabled:    entry.Enabled,
		})
		if entry.ID > maxCycleID {
			maxCycleID = entry.ID
		}
	}
	// Rebuild LightIDs from the loaded lights
	cycleIdx := make(map[int]int, len(loadedCycles))
	for i, c := range loadedCycles {
		cycleIdx[c.ID] = i
	}
	for _, l := range loadedLights {
		if l.CycleID >= 0 {
			if i, ok := cycleIdx[l.CycleID]; ok {
				loadedCycles[i].LightIDs = append(loadedCycles[i].LightIDs, l.ID)
			}
		}
	}

	return loadedSplines, loadedRoutes, loadedCars, loadedLights, loadedCycles,
		maxSplineID + 1, maxRouteID + 1, maxLightID + 1, maxCycleID + 1, nil
}

func isCtrlDown() bool {
	return rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl)
}

func pointKey(v rl.Vector2) string {
	qx := int(math.Round(float64(v.X * 100)))
	qy := int(math.Round(float64(v.Y * 100)))
	return fmt.Sprintf("%d:%d", qx, qy)
}

func pointInRect(p rl.Vector2, r rl.Rectangle) bool {
	return p.X >= r.X && p.X <= r.X+r.Width && p.Y >= r.Y && p.Y <= r.Y+r.Height
}

func sliderValueFromMouse(mouseX float32, rect rl.Rectangle, minValue, maxValue float32) float32 {
	t := clampf((mouseX-rect.X)/rect.Width, 0, 1)
	value := minValue + t*(maxValue-minValue)
	return float32(math.Round(float64(value*2)) / 2)
}

func randomizedSpawnDelay(spawnPerMinute float32) float32 {
	if spawnPerMinute <= 0 {
		return float32(math.MaxFloat32)
	}
	lambda := float64(spawnPerMinute) / 60.0
	u := math.Max(rand.Float64(), 1e-5)
	return float32(-math.Log(u) / lambda)
}

var routePalette = []rl.Color{
	rl.NewColor(224, 94, 94, 255),   // Red
	rl.NewColor(76, 150, 230, 255),  // Blue
	rl.NewColor(99, 190, 123, 255),  // Green
	rl.NewColor(225, 169, 76, 255),  // Orange
	rl.NewColor(154, 108, 224, 255), // Purple
	rl.NewColor(76, 191, 188, 255),  // Cyan
	rl.NewColor(213, 104, 171, 255), // Pink
	rl.NewColor(218, 205, 60, 255),  // Yellow
	rl.NewColor(140, 205, 70, 255),  // Lime
	rl.NewColor(98, 118, 228, 255),  // Indigo
	rl.NewColor(230, 112, 82, 255),  // Coral
	rl.NewColor(62, 178, 152, 255),  // Teal
	rl.NewColor(178, 132, 228, 255), // Lavender
	rl.NewColor(225, 82, 128, 255),  // Rose
	rl.NewColor(228, 158, 48, 255),  // Amber
}

func routePaletteColor(idx int) rl.Color {
	n := len(routePalette)
	if n == 0 {
		return rl.NewColor(90, 90, 90, 255)
	}
	return routePalette[((idx%n)+n)%n]
}

func pickNextColorIndex(routes []Route) int {
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

// collisionRadius returns the radius of each of the two hitbox circles.
// The circles are sized so they protrude exactly 0.5 m beyond the car's sides:
//
//	r = W/2 + 0.5
func collisionRadius(car Car) float32 {
	return car.Width/2 + 0.5
}

// circleOffset returns how far each hitbox circle centre is placed from the
// car's centre along its heading. Combined with collisionRadius this makes the
// circles protrude exactly 1.0 m beyond the car's front and back:
//
//	offset = L/2 + 1.0 − r
func circleOffset(car Car) float32 {
	return car.Length/2 + 1.0 - collisionRadius(car)
}

func headingAngleDegrees(a, b rl.Vector2) float32 {
	da := normalize(a)
	db := normalize(b)
	d := clampf(dot(da, db), -1, 1)
	return float32(math.Acos(float64(d)) * 180 / math.Pi)
}

func signedAngleDegrees(from, to rl.Vector2) float32 {
	nFrom := normalize(from)
	nTo := normalize(to)
	return float32(math.Atan2(float64(cross2D(nFrom, nTo)), float64(dot(nFrom, nTo))) * 180 / math.Pi)
}

func cross2D(a, b rl.Vector2) float32 {
	return a.X*b.Y - a.Y*b.X
}

func vectorLengthSq(v rl.Vector2) float32 {
	return v.X*v.X + v.Y*v.Y
}

func pixelsToWorld(zoom, pixels float32) float32 {
	return pixels / zoom
}

func vecAdd(a, b rl.Vector2) rl.Vector2 {
	return rl.NewVector2(a.X+b.X, a.Y+b.Y)
}

func vecSub(a, b rl.Vector2) rl.Vector2 {
	return rl.NewVector2(a.X-b.X, a.Y-b.Y)
}

func vecScale(v rl.Vector2, s float32) rl.Vector2 {
	return rl.NewVector2(v.X*s, v.Y*s)
}

func dot(a, b rl.Vector2) float32 {
	return a.X*b.X + a.Y*b.Y
}

func distSq(a, b rl.Vector2) float32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}

func normalize(v rl.Vector2) rl.Vector2 {
	lenSq := v.X*v.X + v.Y*v.Y
	if lenSq <= 1e-9 {
		return rl.NewVector2(1, 0)
	}
	inv := 1 / float32(math.Sqrt(float64(lenSq)))
	return rl.NewVector2(v.X*inv, v.Y*inv)
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
