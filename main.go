package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"path/filepath"
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
	maxZoom float32 = 12.0

	curveSamples = 64
	hoverSamples = 48
	simSamples   = 96

	handlePixels float32 = 6.0
	linePixels   float32 = 3.0
	hoverPixels  float32 = 10.0
	snapPixels   float32 = 14.0

	spawnSliderMaxPerMinute float32 = 60.0

	predictionHorizonSeconds float32 = 4.0
	predictionStepSeconds    float32 = 0.15
	blameAngleThresholdDeg   float32 = 45.0
	brakeDecelMultiplier     float32 = 5.0
)

const (
	accelEscapeLookaheadSecs  float32 = 2.0
	accelEscapeTestShortSecs  float32 = 0.25
	accelEscapeTestMediumSecs float32 = 0.5
	accelEscapeTestLongSecs   float32 = 1.0
)

type Spline struct {
	ID       int
	Priority bool

	P0 rl.Vector2
	P1 rl.Vector2
	P2 rl.Vector2
	P3 rl.Vector2

	Length      float32
	SpeedFactor float32
	Samples     [simSamples + 1]rl.Vector2
	CumLen      [simSamples + 1]float32
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

	DraggingSlider bool
}

type Car struct {
	RouteID int

	CurrentSplineID     int
	DestinationSplineID int
	DistanceOnSpline    float32
	Speed               float32
	MaxSpeed            float32
	Accel               float32
	Length              float32
	Width               float32
	Color               rl.Color
	Braking             bool
}

type TrajectorySample struct {
	Time     float32
	Position rl.Vector2
	Heading  rl.Vector2
	Priority bool
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
}

type DebugBlameLink struct {
	FromCarIndex int
	ToCarIndex   int
}

type SavedSplineFile struct {
	Splines []SavedSpline `json:"splines"`
}

type SavedSpline struct {
	ID       int        `json:"id"`
	Priority bool       `json:"priority"`
	P0       rl.Vector2 `json:"p0"`
	P1       rl.Vector2 `json:"p1"`
	P2       rl.Vector2 `json:"p2"`
	P3       rl.Vector2 `json:"p3"`
}

func main() {
	rand.Seed(time.Now().UnixNano())

	rl.SetConfigFlags(rl.FlagWindowResizable)
	rl.InitWindow(initialWidth, initialHeight, "raylib-go traffic spline editor")
	defer rl.CloseWindow()
	rl.SetTargetFPS(144)

	camera := rl.NewCamera2D(
		rl.NewVector2(float32(rl.GetScreenWidth())/2, float32(rl.GetScreenHeight())/2),
		rl.NewVector2(0, 0),
		0,
		1,
	)

	splines := make([]Spline, 0, 128)
	routes := make([]Route, 0, 32)
	cars := make([]Car, 0, 256)

	mode := ModeEdit
	stage := StageIdle
	draft := newDraft()
	routePanel := RoutePanel{}
	routeStartSplineID := -1
	debugMode := false
	nextSplineID := 1
	nextRouteID := 1

	noticeText := ""
	noticeTimer := float32(0)

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
			default:
				mode = ModeEdit
			}
			stage = StageIdle
			draft = newDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
		}
		if rl.IsKeyPressed(rl.KeyE) {
			mode = ModeEdit
			stage = StageIdle
			draft = newDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
		}
		if rl.IsKeyPressed(rl.KeyR) {
			mode = ModeRoute
			stage = StageIdle
			draft = newDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
		}
		if rl.IsKeyPressed(rl.KeyP) {
			mode = ModePriority
			stage = StageIdle
			draft = newDraft()
			routePanel = RoutePanel{}
			routeStartSplineID = -1
		}
		if rl.IsKeyPressed(rl.KeyD) {
			debugMode = !debugMode
		}
		if isCtrlDown() && rl.IsKeyPressed(rl.KeyS) {
			path, err := pickSplineFilePath(true)
			if err != nil {
				noticeText = fmt.Sprintf("Save failed: %v", err)
				noticeTimer = 3.0
			} else if path != "" {
				if err := saveSplineFile(splines, path); err != nil {
					noticeText = fmt.Sprintf("Save failed: %v", err)
				} else {
					noticeText = fmt.Sprintf("Saved %d splines to %s", len(splines), path)
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
				loadedSplines, loadedNextSplineID, err := loadSplineFile(path)
				if err != nil {
					noticeText = fmt.Sprintf("Load failed: %v", err)
					noticeTimer = 3.0
				} else {
					splines = loadedSplines
					routes = routes[:0]
					cars = cars[:0]
					stage = StageIdle
					draft = newDraft()
					routePanel = RoutePanel{}
					routeStartSplineID = -1
					nextSplineID = loadedNextSplineID
					noticeText = fmt.Sprintf("Loaded %d splines from %s", len(splines), path)
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

		hoverRadius := pixelsToWorld(camera.Zoom, hoverPixels)
		snapRadius := pixelsToWorld(camera.Zoom, snapPixels)
		handleRadius := pixelsToWorld(camera.Zoom, handlePixels)
		baseThickness := pixelsToWorld(camera.Zoom, linePixels)

		hoveredSpline := findHoveredSpline(splines, mouseWorld, hoverRadius)
		hoveredEnd := findNearbyEnd(splines, mouseWorld, snapRadius)
		hoveredStart := findNearbyStart(splines, mouseWorld, snapRadius)

		vehicleCounts := buildVehicleCounts(cars)
		routes = updateRouteVisuals(routes, splines, vehicleCounts)
		brakingDecisions, debugBlameLinks := computeBrakingDecisions(cars, splines, vehicleCounts)
		cars = updateCars(cars, routes, splines, vehicleCounts, brakingDecisions, dt)
		routes, cars = updateRouteSpawning(routes, cars, dt)

		if routePanel.Open {
			var applied bool
			routePanel, routes, cars, applied = updateRoutePanel(routePanel, routes, cars, &nextRouteID)
			if applied {
				routeStartSplineID = -1
			}
		}

		if !routePanel.Open {
			switch mode {
			case ModeEdit:
				var topologyChanged bool
				stage, draft, splines, nextSplineID, topologyChanged = handleEditMode(stage, draft, splines, hoveredSpline, hoveredEnd, hoveredStart, mouseWorld, nextSplineID)
				if topologyChanged {
					routes = refreshRoutes(routes, splines)
					cars = cars[:0]
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
			}
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode2D(camera)
		drawGrid(camera)
		drawAxes(camera)

		for _, route := range routes {
			if !route.Valid {
				continue
			}
			drawRoute(route, splines, pixelsToWorld(camera.Zoom, 2.0))
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

			preview, hasPreview := buildPreview(stage, draft, mouseWorld, hoveredStart, splines)
			if hasPreview {
				drawDraft(stage, draft, mouseWorld, camera.Zoom)
				drawSpline(preview, pixelsToWorld(camera.Zoom, 4), rl.NewColor(214, 76, 76, 255))
			}
		}

		if mode == ModeRoute {
			drawRoutePicking(routeStartSplineID, routePanel, hoveredStart, hoveredEnd, splines, vehicleCounts, camera.Zoom)
		}

		drawCars(cars, splines)
		if debugMode {
			drawDebugBlameLinks(debugBlameLinks, cars, splines, camera.Zoom)
		}
		rl.EndMode2D()

		drawHud(mode, stage, draft, hoveredSpline, routeStartSplineID, debugMode, camera.Zoom, len(splines), len(routes), len(cars))
		if routePanel.Open {
			drawRoutePanel(routePanel, routes)
		}
		if noticeText != "" {
			drawNotice(noticeText)
		}
		rl.DrawFPS(int32(rl.GetScreenWidth()-90), 10)
		rl.EndDrawing()
	}
}

func handleEditMode(stage Stage, draft Draft, splines []Spline, hoveredSpline int, hoveredEnd EndHit, hoveredStart EndHit, mouseWorld rl.Vector2, nextSplineID int) (Stage, Draft, []Spline, int, bool) {
	topologyChanged := false

	if rl.IsMouseButtonPressed(rl.MouseButtonRight) {
		switch stage {
		case StageIdle:
			if hoveredSpline >= 0 {
				splines = append(splines[:hoveredSpline], splines[hoveredSpline+1:]...)
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
	if existingRouteID >= 0 {
		if idx := findRouteIndexByID(routes, existingRouteID); idx >= 0 {
			spawnPerMinute = routes[idx].SpawnPerMinute
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

func updateRoutePanel(panel RoutePanel, routes []Route, cars []Car, nextRouteID *int) (RoutePanel, []Route, []Car, bool) {
	panelRect := rl.NewRectangle(float32(rl.GetScreenWidth())-360, 18, 340, 174)
	sliderRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+82, panelRect.Width-36, 22)
	applyRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+126, 120, 32)
	cancelRect := rl.NewRectangle(panelRect.X+202, panelRect.Y+126, 120, 32)
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
			routes[idx].Color = colorForDestination(routes[idx].EndSplineID)
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
		Color:          colorForDestination(panel.EndSplineID),
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
		routes[i].Color = colorForDestination(routes[i].EndSplineID)
		if routes[i].SpawnPerMinute > 0 && routes[i].NextSpawnIn <= 0 {
			routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
		}
	}
	return routes
}

func updateRouteSpawning(routes []Route, cars []Car, dt float32) ([]Route, []Car) {
	for i := range routes {
		if !routes[i].Valid || routes[i].SpawnPerMinute <= 0 {
			continue
		}
		routes[i].NextSpawnIn -= dt
		if routes[i].NextSpawnIn > 0 {
			continue
		}
		car := spawnCar(routes[i])
		cars = append(cars, car)
		routes[i].NextSpawnIn = randomizedSpawnDelay(routes[i].SpawnPerMinute)
	}
	return routes, cars
}

func spawnCar(route Route) Car {
	return Car{
		RouteID:             route.ID,
		CurrentSplineID:     route.StartSplineID,
		DestinationSplineID: route.EndSplineID,
		DistanceOnSpline:    0,
		Speed:               randRange(0, 10),
		MaxSpeed:            randRange(70, 135),
		Accel:               randRange(24, 48),
		Length:              randRange(16, 22),
		Width:               randRange(9, 11),
		Color:               route.Color,
		Braking:             false,
	}
}

func computeBrakingDecisions(cars []Car, splines []Spline, vehicleCounts map[int]int) ([]bool, []DebugBlameLink) {
	flags := make([]bool, len(cars))
	initialBlame := make([]bool, len(cars))
	tentativeLinks := make([]DebugBlameLink, 0)
	if len(cars) < 2 {
		return flags, tentativeLinks
	}

	predictions := make([][]TrajectorySample, len(cars))
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
			blameI, blameJ := determineBlame(collision, cars[i], cars[j])
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

	debugLinks := make([]DebugBlameLink, 0, len(tentativeLinks))
	for _, link := range tentativeLinks {
		if link.FromCarIndex >= 0 && link.FromCarIndex < len(flags) && flags[link.FromCarIndex] {
			debugLinks = append(debugLinks, link)
		}
	}

	return flags, debugLinks
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
		accelEscapeTestLongSecs,
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
		if hasBlamedConflictWithPrediction(carIndex, testCar, testPrediction, cars, predictions) {
			return true
		}
	}

	return false
}

func hasBlamedConflictWithPrediction(carIndex int, testCar Car, testPrediction []TrajectorySample, cars []Car, predictions [][]TrajectorySample) bool {
	for otherIndex, otherCar := range cars {
		if otherIndex == carIndex || otherIndex >= len(predictions) || len(predictions[otherIndex]) == 0 {
			continue
		}

		collision, ok := predictCollision(testPrediction, predictions[otherIndex], testCar, otherCar)
		if !ok {
			continue
		}

		blameTestCar, _ := determineBlame(collision, testCar, otherCar)
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
		})

		if stepIndex == steps || !active || speed <= 0.01 {
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

			move -= remaining
			simCar.DistanceOnSpline = currentSpline.Length

			if simCar.CurrentSplineID == simCar.DestinationSplineID {
				active = false
				break
			}

			nextSplineID, ok := chooseNextSplineOnBestPath(splines, simCar.CurrentSplineID, simCar.DestinationSplineID, vehicleCounts)
			if !ok {
				active = false
				break
			}
			simCar.CurrentSplineID = nextSplineID
			simCar.DistanceOnSpline = 0
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

	collisionDistance := collisionRadius(carA) + collisionRadius(carB)
	collisionDistanceSq := collisionDistance * collisionDistance

	for i := 0; i < count; i++ {
		if distSq(aSamples[i].Position, bSamples[i].Position) > collisionDistanceSq {
			continue
		}
		prevIndex := i - 1
		if prevIndex < 0 {
			prevIndex = 0
		}
		return CollisionPrediction{
			Time:      aSamples[i].Time,
			PosA:      aSamples[i].Position,
			PosB:      bSamples[i].Position,
			PrevPosA:  aSamples[prevIndex].Position,
			PrevPosB:  bSamples[prevIndex].Position,
			HeadingA:  aSamples[i].Heading,
			HeadingB:  bSamples[i].Heading,
			PriorityA: aSamples[i].Priority,
			PriorityB: bSamples[i].Priority,
		}, true
	}

	return CollisionPrediction{}, false
}

func determineBlame(collision CollisionPrediction, carA, carB Car) (bool, bool) {
	if collision.PriorityA != collision.PriorityB {
		if collision.PriorityA {
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

func updateCars(cars []Car, routes []Route, splines []Spline, vehicleCounts map[int]int, brakingDecisions []bool, dt float32) []Car {
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
		car.Braking = shouldBrake

		for {
			currentSpline, ok := findSplineByID(splines, car.CurrentSplineID)
			if !ok {
				break
			}

			targetSpeed := car.MaxSpeed * currentSpline.SpeedFactor
			if car.Braking {
				targetSpeed = 0
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

			car.DistanceOnSpline -= currentSpline.Length
			if car.CurrentSplineID == car.DestinationSplineID {
				break
			}

			nextSplineID, ok := chooseNextSplineOnBestPath(splines, car.CurrentSplineID, car.DestinationSplineID, vehicleCounts)
			if !ok {
				break
			}
			car.CurrentSplineID = nextSplineID
		}
	}
	return alive
}

func drawCars(cars []Car, splines []Spline) {
	if len(cars) == 0 {
		return
	}
	splineIndexByID := buildSplineIndexByID(splines)
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
			rl.DrawCircleV(pos, maxf(car.Width*0.22, 2), rl.NewColor(220, 50, 50, 255))
		}
	}
}

func drawDebugBlameLinks(links []DebugBlameLink, cars []Car, splines []Spline, zoom float32) {
	if len(links) == 0 || len(cars) == 0 {
		return
	}

	splineIndexByID := buildSplineIndexByID(splines)
	lineColor := rl.NewColor(220, 50, 50, 220)
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

func drawRoute(route Route, splines []Spline, thickness float32) {
	indexByID := buildSplineIndexByID(splines)
	color := rl.NewColor(route.Color.R, route.Color.G, route.Color.B, 90)
	for _, pathID := range route.PathIDs {
		idx, ok := indexByID[pathID]
		if !ok {
			continue
		}
		drawSpline(splines[idx], thickness, color)
	}
	if start, ok := findSplineByID(splines, route.StartSplineID); ok {
		drawEndpoint(start.P0, 5, route.Color)
	}
	if end, ok := findSplineByID(splines, route.EndSplineID); ok {
		drawEndpoint(end.P3, 6, color)
	}
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
				previewRoute := Route{PathIDs: pathIDs, Color: colorForDestination(hoveredEnd.SplineID), Valid: true}
				drawRoute(previewRoute, splines, pixelsToWorld(zoom, 4))
			}
		}
	}
	_ = routePanel
}

func drawRoutePanel(panel RoutePanel, routes []Route) {
	panelRect := rl.NewRectangle(float32(rl.GetScreenWidth())-360, 18, 340, 174)
	sliderRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+82, panelRect.Width-36, 22)
	applyRect := rl.NewRectangle(panelRect.X+18, panelRect.Y+126, 120, 32)
	cancelRect := rl.NewRectangle(panelRect.X+202, panelRect.Y+126, 120, 32)

	bg := rl.NewColor(248, 248, 250, 245)
	outline := rl.NewColor(210, 210, 215, 255)
	text := rl.NewColor(30, 30, 35, 255)
	muted := rl.NewColor(90, 90, 100, 255)
	accent := rl.NewColor(70, 110, 220, 255)
	button := rl.NewColor(235, 236, 240, 255)

	rl.DrawRectangle(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), bg)
	rl.DrawRectangleLines(int32(panelRect.X), int32(panelRect.Y), int32(panelRect.Width), int32(panelRect.Height), outline)
	title := "Create route"
	if panel.ExistingRouteID >= 0 {
		title = "Edit existing route"
	}
	rl.DrawText(title, int32(panelRect.X+18), int32(panelRect.Y+16), 22, text)
	rl.DrawText(fmt.Sprintf("Start spline #%d → end spline #%d", panel.StartSplineID, panel.EndSplineID), int32(panelRect.X+18), int32(panelRect.Y+44), 18, muted)
	meanSeconds := float32(0)
	if panel.SpawnPerMinute > 0 {
		meanSeconds = 60 / panel.SpawnPerMinute
	}
	rl.DrawText(fmt.Sprintf("Average spawn frequency: %.1f cars/min  (mean %.1fs)", panel.SpawnPerMinute, meanSeconds), int32(panelRect.X+18), int32(panelRect.Y+64), 18, text)

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
	rl.DrawText(applyLabel, int32(applyRect.X+34), int32(applyRect.Y+7), 20, text)
	rl.DrawText("Close", int32(cancelRect.X+34), int32(cancelRect.Y+7), 20, text)
	rl.DrawText(fmt.Sprintf("Current weighted cost: %.0f", panel.PathLength), int32(panelRect.X+18), int32(panelRect.Y+158), 16, muted)
	_ = routes
}

func drawNotice(text string) {
	width := int32(18 + len(text)*9)
	x := int32((int32(rl.GetScreenWidth()) - width) / 2)
	y := int32(rl.GetScreenHeight() - 54)
	bg := rl.NewColor(40, 44, 52, 230)
	rl.DrawRectangle(x, y, width, 34, bg)
	rl.DrawText(text, x+14, y+8, 18, rl.White)
}

func drawHud(mode EditorMode, stage Stage, draft Draft, hoveredSpline int, routeStartSplineID int, debugMode bool, zoom float32, splineCount, routeCount, carCount int) {
	panel := rl.NewColor(248, 248, 250, 240)
	outline := rl.NewColor(210, 210, 215, 255)
	text := rl.NewColor(30, 30, 35, 255)
	muted := rl.NewColor(90, 90, 100, 255)

	rl.DrawRectangle(12, 12, 980, 156, panel)
	rl.DrawRectangleLines(12, 12, 980, 156, outline)

	modeText := "Edit splines"
	statusText := stageLabel(stage, draft)
	switch mode {
	case ModeRoute:
		modeText = "Route mode"
		if routeStartSplineID >= 0 {
			statusText = fmt.Sprintf("Pick a destination end for start spline #%d", routeStartSplineID)
		} else {
			statusText = "Pick a route origin on a spline start"
		}
	case ModePriority:
		modeText = "Priority paint"
		statusText = "Left click marks hovered spline as priority, right click clears it"
	}

	hoverText := "none"
	if hoveredSpline >= 0 {
		hoverText = fmt.Sprintf("#%d", hoveredSpline)
	}
	debugText := "off"
	if debugMode {
		debugText = "on"
	}

	rl.DrawText("Traffic spline editor", 24, 24, 22, text)
	rl.DrawText("E: edit | R: route | P: priority | D: debug | Ctrl+S: save as | Ctrl+O: open | Tab: cycle modes", 24, 54, 18, muted)
	rl.DrawText("Priority splines are painted purple. Debug draws blame lines from braking cars to feared collision cars.", 24, 78, 18, muted)
	rl.DrawText(fmt.Sprintf("Mode: %s   Status: %s", modeText, statusText), 24, 108, 18, text)
	rl.DrawText(fmt.Sprintf("Splines: %d   Routes: %d   Cars: %d   Hovered spline: %s   Debug: %s   Zoom: %.2fx", splineCount, routeCount, carCount, hoverText, debugText, zoom), 24, 132, 18, text)
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
	s.SpeedFactor = splineSpeedFactor(*s)
}

func splineSpeedFactor(s Spline) float32 {
	startTangent := normalize(vecSub(s.P1, s.P0))
	endTangent := normalize(vecSub(s.P3, s.P2))
	d := clampf(dot(startTangent, endTangent), -1, 1)
	angle := float32(math.Acos(float64(d)))
	turnPenalty := angle / float32(math.Pi)
	return 1.0 - 0.55*turnPenalty
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

	major := chooseGridSpacing(camera.Zoom)
	minor := major / 5

	minorColor := rl.NewColor(236, 236, 239, 255)
	majorColor := rl.NewColor(212, 212, 218, 255)

	for x := float32(math.Floor(float64(minX/minor))) * minor; x <= maxX; x += minor {
		rl.DrawLineV(rl.NewVector2(x, minY), rl.NewVector2(x, maxY), minorColor)
	}
	for y := float32(math.Floor(float64(minY/minor))) * minor; y <= maxY; y += minor {
		rl.DrawLineV(rl.NewVector2(minX, y), rl.NewVector2(maxX, y), minorColor)
	}

	for x := float32(math.Floor(float64(minX/major))) * major; x <= maxX; x += major {
		rl.DrawLineV(rl.NewVector2(x, minY), rl.NewVector2(x, maxY), majorColor)
	}
	for y := float32(math.Floor(float64(minY/major))) * major; y <= maxY; y += major {
		rl.DrawLineV(rl.NewVector2(minX, y), rl.NewVector2(maxX, y), majorColor)
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
		nextNodes := startsByNode[pointKey(splines[u].P3)]
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

func saveSplineFile(splines []Spline, path string) error {
	saved := SavedSplineFile{Splines: make([]SavedSpline, 0, len(splines))}
	for _, spline := range splines {
		saved.Splines = append(saved.Splines, SavedSpline{
			ID:       spline.ID,
			Priority: spline.Priority,
			P0:       spline.P0,
			P1:       spline.P1,
			P2:       spline.P2,
			P3:       spline.P3,
		})
	}

	data, err := json.MarshalIndent(saved, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}

func loadSplineFile(path string) ([]Spline, int, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, 1, err
	}

	var saved SavedSplineFile
	if err := json.Unmarshal(data, &saved); err != nil {
		return nil, 1, err
	}

	loaded := make([]Spline, 0, len(saved.Splines))
	maxID := 0
	for _, entry := range saved.Splines {
		spline := newSpline(entry.ID, entry.P0, entry.P1, entry.P2, entry.P3)
		spline.Priority = entry.Priority
		loaded = append(loaded, spline)
		if entry.ID > maxID {
			maxID = entry.ID
		}
	}

	return loaded, maxID + 1, nil
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

func colorForDestination(destinationSplineID int) rl.Color {
	palette := []rl.Color{
		rl.NewColor(224, 94, 94, 255),
		rl.NewColor(76, 150, 230, 255),
		rl.NewColor(99, 190, 123, 255),
		rl.NewColor(225, 169, 76, 255),
		rl.NewColor(154, 108, 224, 255),
		rl.NewColor(76, 191, 188, 255),
		rl.NewColor(213, 104, 171, 255),
	}
	if destinationSplineID < 0 {
		return rl.NewColor(90, 90, 90, 255)
	}
	return palette[destinationSplineID%len(palette)]
}

func collisionRadius(car Car) float32 {
	return 0.5 * float32(math.Sqrt(float64(car.Length*car.Length+car.Width*car.Width)))
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
