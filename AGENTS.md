# AGENTS.md

Orientation for coding agents working on this repository. The user-facing
overview is in `README.md`; this file documents the things that are not obvious
from reading the code for the first time.

## Layout

- `main.go` — single-file Raylib editor / runtime (≈6k lines). Build tag
  `!darwin`. All UI, input, camera, toolbars, saving dialogs, route panels.
- `main_darwin.go` — stub that exits with an error on macOS. macOS is not
  supported; don't try to make it build there.
- `sim/` — headless simulation package. Can be consumed without Raylib.
  - `sim.go` — all Go-side simulation logic. Still the single file for the
    sim core (don't split it unless explicitly asked).
  - `vehicle_models.go` — loads `assets/cars.json` + `assets/buses.json` via
    `go:embed` and exposes `VehicleModel` / `RandomVehicleModel`.
  - `assets/cars.json`, `assets/buses.json` — authored vehicle catalogues.
    Bundled at compile time. See "Vehicle models" below.
  - `braking.h` / `braking.c` — C port of the braking/conflict-resolution
    pipeline (OpenMP-parallel). This is the hot path.
  - `braking_cgo.go` — cgo glue. Marshals `RoadGraph` + `[]Car` into C structs
    and calls `compute_braking_decisions`. Caches the marshalled graph across
    frames by topology hash (`brakingGraphTopologyKey`); only cars / segment
    costs are re-marshalled each frame.
  - `braking_test.go` — synthetic + stress tests against the C pipeline.
  - `pedestrian_test.go`, `turn_signal_test.go`, `vehicle_models_test.go` —
    focused tests for pedestrians, indicators, and the model catalogue.
- `internal/filepicker/` — platform-specific native file dialog wrappers
  (zenity/kdialog/yad on Linux, PowerShell on Windows).
- `examples/` — curated example scenarios (loaded via the editor).
- `tests/` — scenario JSONs used by `TestWorldStepCStress` and manual repros.
  Not all of these are asserted on — treat them as fixtures.
- `tile_map.py` — standalone helper script, not part of the build.

## Build / run / test

- Go 1.22.2. Raylib must be installed on the system (CI only runs `go test` /
  cross-`go build`; it does not run the binary).
- cgo is **required** — the braking kernel is in C and is compiled with
  `-Ofast -march=native -fopenmp -flto` (see `braking_cgo.go` preamble).
  You need a C compiler with OpenMP (`libgomp`) on the build host.
- Run:  `go run .`
- Test: `go test ./...`
- The stress test `TestWorldStepCStress` runs 800 frames of a highway scenario
  at 5× spawn rate; it's the most realistic load exercise.

## High-level data flow

`World = TopologyState + RuntimeState`. Splines/routes/lights/cycles are the
*topology*; cars and lane-change bridge splines are the *runtime*. They are
split because topology is stable across many frames and the C side caches it.

Per frame, `(*World).Step(dt)` does, in order:

1. `BuildVehicleCounts` → `newRoadGraphFromTopology` (uses the cached
   permanent topology; see below). Note: `BuildVehicleCounts` ignores
   `CarControlExternal`, so the player proxy does not contribute to route-cost
   congestion weighting yet.
2. `UpdateRouteVisualsWithGraph` — only every 0.5s, recomputes each route's
   precomputed path via the graph's per-destination Dijkstra tree cache.
3. `computeLaneChanges` — runs against the full `world.Cars` snapshot so AI
   cars can treat external cars as occupancy, but it skips mutating
   `CarControlExternal` cars. May append temporary bridge splines to
   `LaneChangeSplines` and mutate each AI car's `LaneChanging` /
   `AfterSplineID`.
4. Rebuild `allGraph` = permanent topology **+** lane-change bridges. This is
   the graph passed to the braking kernel and pathfinding for the rest of the
   step.
5. In parallel (two goroutines):
   - `computeBrakingDecisionsC` → (brake flags, hold-speed flags, debug links)
     for the full car slice, including external cars.
   - `computeFollowingSpeedCaps` → per-car soft speed cap from the car ahead,
     also for the full car slice.
6. Split by `Car.ControlMode`. Project the read-side outputs back onto the AI
   subset only, then `updateCars` integrates only those AI cars forward,
   applies speed caps, handles spline transitions, bus-stop dwell. Parallel
   fast path + sequential transition resolver.
7. `gcLaneChangeSplines` drops bridges nobody references.
8. `updateRouteSpawning` — Poisson-ish spawns (`randomizedSpawnDelay`). Spawn
   blocking sees external cars too, so the player proxy occupies space for
   spawn checks.
9. Merge AI cars + external cars back into `world.Cars`.
10. `UpdateTrafficCycles` — advances light phases.
11. Pedestrian pipeline: `computePedestrianCrossings` →
    `buildStoppingPedestrianLightsByPath` → `buildPedestrianBlockedSplineDists`
    → `updatePedestrians`. Crossings + blocked map feed step 6's car speed
    caps (red-light analogue); lights + crossings feed pedestrian movement
    caps. Cars used for pedestrian yielding are the post-`updateCars`
    positions (sim + external), so pedestrians react to the frame's actual
    car state.
12. `assignCarTurnSignals` — writes `Car.TurnSignal` for AI cars based on
    lane-change state / desired lane / map-authored turn links. Skips
    external cars so the player proxy keeps whatever the controller wrote.

Everything else (debug blame links, profile counters) is book-keeping.

## Concepts that surprise people

### NodeKey — "exact" endpoint matching

Spline connectivity is built by bucketing endpoints on a **quantized** 2D key
(`NodeKey = [2]int32`), quantized to 1/100 units (`nodeKeyFromVec2`). Two
splines are considered connected iff their endpoint keys match exactly after
quantization. There is no tolerance beyond that — splines that *visually*
meet but don't snap to the same key will not route to each other. The editor
is responsible for snapping during placement.

### Route tree caching

`RoadGraph.routeCache` stores one Dijkstra tree per `(destinationID,
VehicleKind)` keyed pair. Trees are built on the *reverse* graph
(`reverseNeighbors`), rooted at the destination, so every spline gets
`cost-to-destination` and `nextHop`. This makes per-car "which spline should I
go to next" an O(1) lookup. The C kernel builds the same trees independently
in `build_route_tree` (parallel over distinct destinations).

### Permanent-graph topology cache

`World.permanentRoadGraphTopology()` caches the graph topology keyed by the
**slice backing address + element count + content hash**
(`permanentGraphSplineAddr` / `splineTopologyKey`). The slice address is part
of the key on purpose: if someone replaces `w.Splines` with a new slice, the
cache is invalidated even if contents happen to hash the same. Lane-change
bridges are *not* part of this cache — they're merged on top via
`buildMergedRoadGraphFromTopology`.

### Coupled splines (lanes)

- `HardCoupledIDs` — splines that represent parallel lanes the vehicle can
  change into via a bridge. Used by `findBetterPreferenceLaneCandidates` and by
  the "forced" lane change when the only route to the destination goes through
  a sibling lane.
- `SoftCoupledIDs` — also considered for preference-based lane changes but
  with different semantics (e.g. lanes that are *allowed* merges but not
  structural lane pairs).
- `LanePreference` — lower is better. A car tries to migrate toward lower
  preference numbers on its current cooldown timer. `0` means "no preference".
- `BusOnly` splines are skipped by car pathfinding (`is_spline_usable` /
  `isSplineUsableForVehicle`).

### Bus routes / stops

Buses don't drive straight to `Route.EndSplineID`; they chain through
`Route.BusStops` in order. `Car.DestinationSplineID` is updated to the
*current* target (`CurrentRouteTarget`). Stopping at a stop is triggered by
`shouldBeginBusStopDwell`; dwell timer is in `Car.BusStopTimer`. When it
hits 0, `resumeBusRouteAfterStop` bumps `NextBusStopIndex` and re-targets.

### Lane change as a temporary Bezier bridge

`buildLaneChangeBridge` synthesises a new cubic spline with:
- `P0 = car's current spline-sampled position`
- `P1 = P0 + half_dist * current_heading`
- `P3 = point on destination lane half_dist after the closest projection`
- `P2 = P3 - half_dist * destination_heading`

where `half_dist = speed * laneChangeHalfSecs`. The bridge is appended to
`World.LaneChangeSplines`, the car's `CurrentSplineID` is set to the bridge,
and `AfterSplineID` / `AfterSplineDist` tell it where to land. Bridges get
GC'd once no car references them.

### Braking pipeline (the heavy one)

Implemented in `braking.c`; the Go reference implementation in
`computeBrakingDecisions` is kept around but not called (replaced by
`computeBrakingDecisionsC` in `Step`). Both produce identical outputs by
design. Stages:

1. **Base prediction** — for each car, simulate a 3 s / 0.15 s trajectory
   along its planned route (`predict_car_trajectory`). Produces a list of
   `TrajectorySample { time, position, heading, priority, spline_id }`.
2. **Spatial grid + broad phase** — insert each car's start position into a
   spatial hash (`sgrid_*`), cell size = median of per-car `reach`. For each
   pair of cars inside the same 3×3 cell neighbourhood, check a cheap
   `closing-rate-scaled` distance test then AABB overlap.
3. **Narrow phase** — `predict_collision` does multi-circle-offset hitbox
   checks at each time step (body + trailer). Returns the first time step a
   circle-pair touches.
4. **Blame** — `determine_blame` decides which car should yield based on
   priority splines, head-on angle (> `BLAME_ANGLE_THRESH = 45°` → left/right
   geometric rule), or rear-ender rule for shallow-angle collisions.
5. **Stationary exoneration** — if the blamed car would *still* collide even
   if it stopped dead, it isn't actually the cause of the conflict. Its blame
   is cleared. This prevents everyone in a queue from braking for everyone
   else.
6. **Brake probe** — for each blamed car, re-predict with `speed +=
   accel*short_dt`; if the conflict disappears when the car goes *faster*,
   it should accelerate to clear instead of braking.
7. **Hold probe** — for non-braking cars, check whether accelerating a bit
   would cause a new conflict. If so, mark `holdSpeed = true` (don't
   accelerate but don't brake either).
8. **Deadlock release** — detect short blame cycles (length ≤ 4) in the
   blame graph. For each cycle, the car with the lowest index gets both
   brake and hold flags cleared. This breaks symmetric yields at
   intersections where everybody is waiting for everybody else.

### Stationary predictions shared across threads

In the C pipeline, stationary predictions (`speed = 0` trajectories) are
computed on-demand and shared between threads via a per-car `stat_lock`
word: 0 = free, 1 = computing, 2 = done. The winner of the CAS computes it;
losers spin until it hits 2. See `ensure_stationary_pred`. Don't change this
to a regular mutex — it's in the hot path and contention is rare.

### Why cars jitter when you change splines

`RoadGraph.segmentCosts` depends on `vehicleCounts` (a congestion penalty).
If you rebuild the graph every frame the costs move, which moves the
route tree, which can flip `nextHop` for some cars. The route-visuals update
is intentionally throttled to 0.5 s to damp this.

### Curve speed limit

Each spline precomputes a `CurveSpeedMPS` table sampled every
`curveSpeedIntervalM` (10 m) from `curveSpeedAtArcLen` =
`sqrt(maxLateralAccelMPS2 / curvature)`. `Car.CurveSpeedMultiplier` lets
individual cars lean into curves harder than the nominal limit (e.g. sports
cars). Speed caps are min-combined with the posted `SpeedLimitKmh`.

### Wheelbase / hitch model

Cars have a *rear reference point* (`RearPosition`). The position used for
trajectory-sampling is the "front pivot" at `P0 + rightNormal *
LateralOffset` on the spline; the rear is dragged behind at distance
`Length * Car.WheelbaseFrac()`. Each `VehicleModel` declares its own
`front_pivot_frac` and `rear_pivot_frac` (cars typically ~0.18–0.22 from
the front and back; vans/buses run cab-forward at ~0.10–0.13), and the
spawn step copies them onto the `Car` struct so each vehicle pivots
according to its make. `defaultFrontPivotFrac` / `defaultRearPivotFrac`
(0.20 / 0.80) cover model-less cases like the player proxy. Trailers
hitch to the car's rear point with the same wheelbase fraction. This is
why the prediction code carries `simRearPos` and `simTrailerRearPos` as
mutable state separate from the spline-distance position.

### Pedestrians

- `PedestrianPath` is a straight segment P0→P1 with fixed
  `PedestrianPathWidthM` (4 m). Pedestrians walk along it in one of two
  directions (`Forward` = P0→P1 or P1→P0). `ped.Distance` always increases
  monotonically as a pedestrian progresses, regardless of direction — the
  canonical "path-x from P0" is `ped.Distance` for forward walkers and
  `path.Length - ped.Distance` for backward walkers.
- Right-of-path preferred lateral offset is computed in
  `preferredPedestrianOffset`. The path's Normal is CCW rotation of Dir
  (`(-dir.Y, dir.X)`), which in Y-down screen coords is the *visual right*
  side. Forward pedestrians get `+preferredOffset`, backward get
  `-preferredOffset` — both end up on their own right-hand side of travel.
- `computePedestrianCrossings(paths, splines)` finds every point where a
  pedestrian path meets a car spline, using each spline's sampled polyline
  as the geometry. Spline segments are treated as half-open
  (`splineT ∈ [0, 1)`, except the final segment keeps its endpoint) so
  intersections exactly on a shared sample don't duplicate.
- Pedestrian-car interaction has two *independent* channels:
  - **`buildPedestrianBlockedSplineDists`** — per-spline list of distances
    where an approaching pedestrian forces cars to yield (red-light
    analogue, reusing `computePedestrianCrossingSpeedCap` which mirrors the
    traffic-light speed cap). Pedestrians waiting at a red pedestrian light
    that sits between them and the crossing are *excluded* here so cars
    don't stop for someone who isn't coming.
  - **`computePedestrianMovementCaps`** — per-pedestrian max `ped.Distance`
    this step. Populated by (a) crossings currently occupied by a moving
    car (stopped cars are treated as yielding to break deadlocks), and
    (b) red pedestrian-path lights matching the ped's direction. A final
    spacing pass (`propagatePedestrianQueueSpacing`) sorts
    same-path-same-direction groups by progress and steps each follower's
    cap down by `pedestrianQueueSpacingM` so a waiting queue doesn't clip
    into itself.
- Pedestrian spawn/despawn lives at the dead-ends of the pedestrian graph.
  `pedestrianSpawnIntervalS = 32 s`. Junction routing uses a quadratic
  Bézier turn connector (`buildPedestrianTurn`).

### Pedestrian-path traffic lights

Same `TrafficLight` struct as for cars, discriminated by `SplineID <= 0`
(spline IDs always start at 1). When `IsPedestrianLight()` is true:

- `PedestrianPathIndex` is the path's index in `World.PedestrianPaths`.
- `DistOnPath` is arc length from P0.
- `PedestrianForward` picks which walking direction is affected. Placing
  the light on the right side of the path (in the editor) yields
  `PedestrianForward = true` (stops P0→P1 walkers).

A single cycle can mix car-side and ped-side lights — they're filtered
apart by `IsPedestrianLight()` in `buildStoppingTrafficLightsBySpline` vs
`buildStoppingPedestrianLightsByPath`. When a pedestrian path is deleted,
`removePedestrianLightsForPath` in `main.go` drops the lights that point at
it and decrements higher indices so references stay valid.

### Turn signals

`Car.TurnSignal` (`TurnSignalNone | TurnSignalLeft | TurnSignalRight`) is
written by `computeCarTurnSignal` each step, with trigger precedence:

1. **Active lane change** — project `bridge.P3 − bridge.P0` onto the bridge
   start-tangent right-normal.
2. **Desired lane** (`DesiredLaneSplineID >= 0`) — project the nearest
   point on the desired spline relative to the car's current pose.
3. **Map-authored turn links** — `Spline.LeftTurnLinkIDs` /
   `RightTurnLinkIDs` on the car's current spline. If the routed
   next-spline-on-best-path matches one of the lists, indicate that way.

`turnSignalDeadbandM = 0.15` on the geometric triggers suppresses flicker
on straight sibling splines. External-controlled cars are skipped by
`assignCarTurnSignals` so the player's controller owns its indicators.

**Blink ticking is display-side.** `main.go#turnSignalBlinkOnForCar`
derives a per-car half-period and phase offset from `car.ID` using two
coprime-ish moduli so adjacent IDs don't lock into the same rhythm; the
phase uses `rl.GetTime()` so indicators keep blinking while the sim is
paused. Don't move this into the sim — it's not deterministic-dependent
and the renderer already has a clock.

### Turn-link authoring tool

In Rules mode, `ToolTurnLink` (hotkey **N**) implements a couple-tool-like
two-click flow. First click picks the source spline, second click on the
target: left mouse → right-turn link, right mouse → left-turn link. The
same button again toggles the link off; switching directions replaces the
opposite one. `handleTurnLinkMode` + `toggleTurnLink` in `main.go`.
`drawTurnLinkMode` renders arrows from the source 3/4-point to the target
start (amber = right, blue = left). `reverseSpline` clears turn-link lists
because a reversed spline inverts the flow they describe.

### Vehicle models

Spawning copies physical parameters from an authored `VehicleModel`
instead of rolling new random numbers. Catalogues live in
`sim/assets/cars.json` and `sim/assets/buses.json`, bundled via
`go:embed`. Each entry has an `id`, display name, dimensions, max speed,
accel, curve-speed multiplier, and an optional `trailer` sub-object
(pickup-with-trailer for cars, articulated bus for buses).

`spawnVehicle` calls `RandomVehicleModel(route.VehicleKind)` and writes
`Car.ModelID` on the resulting car. `SavedCar.ModelID` preserves it
across save/load. Display code can key textures/meshes off `ModelID` via
`LookupVehicleModel`. If the embedded JSON fails to parse (reported by
`VehicleModelsError`), spawning falls back to `fallbackVehicleModel`
values so the sim still runs.

### Driving mode / player proxy

The human-driven car in `main.go` is **not** a `sim.Car`. Driving mode keeps a
screen-centered gameplay car with its own physics and camera behavior, then
updates a spline-bound proxy in `sim` via `World.UpdatePlayerProxy`.

That proxy:
- lives in `world.Cars`
- has `ControlMode = CarControlExternal`
- is mirrored in `World.HasPlayerProxy` / `PlayerProxyCar` /
  `PlayerProxyAttach` for debug rendering and attachment state
- is inserted / updated after `editorState.commit(&world)` in `main.go`, so it
  does not get overwritten by editor-state synchronization

Read-side AI logic should see the proxy. Write-side AI logic must not move it.
That split is enforced in `World.Step` by running lane-change / braking /
following against the full car slice, then updating only the AI subset.

The proxy fitter lives in `sim/player_proxy.go`. Important details:
- fitting is spline-centric with hysteresis over the previous attachment
- the player center is shifted forward to the sim's front-pivot reference
  before projecting, otherwise the proxy lags behind the rendered player car
- the selected driving-mode destination is currently stored for future path
  prediction work, but it is **not** yet used to bias proxy spline fitting

## Save format

JSON, defined by `SavedSplineFile` / `SavedSpline` / `SavedRoute` /
`SavedCar` / `SavedTrafficLight` / `SavedTrafficCycle` /
`SavedPedestrianPath`. Note: older files may omit newer fields (e.g.
`bus_only`, `lane_preference`, `vehicle_kind`, `model_id`,
`left_turn_link_ids`, `right_turn_link_ids`, the
`pedestrian_path_index` / `dist_on_path` / `pedestrian_forward` trio on
traffic lights); `LoadWorld` should stay tolerant of missing fields
(`omitempty` on all of them). If you add a field, default it in
`LoadWorld` — don't require it in existing saves. The example and test
JSONs under `examples/` and `tests/` are committed and used by humans;
don't regenerate them unless asked.

Runtime-only state is intentionally excluded from saves:
- `LaneChangeSplines`
- debug/profile fields
- the player proxy / any `CarControlExternal` cars

## Gotchas / rules

- **Do not add macOS support.** The whole thing is stubbed out for Darwin
  on purpose (Raylib + cgo + OpenMP story). `main_darwin.go` is the entire
  macOS build.
- **Do not rewrite the C braking kernel in Go for "consistency".** The Go
  reference (`computeBrakingDecisions`) still exists for debugging; the C
  one is what `Step` actually calls. If you change semantics, change both
  and add a parity test, or remove the Go one explicitly.
- **Car indices are unstable across `Step`.** `updateCars` returns an
  `indexRemap` that `Step` uses to fix up `DebugBlameLinks` / etc. If you
  stash a car index between frames, remap it. Prefer `Car.ID` for
  cross-frame identity.
- **Slice identity matters for the topology cache.** Don't replace
  `w.Splines` with `w.Splines[:0]` + `append(...)` expecting the cache to
  invalidate if the slice header happens to stay the same; look at how
  `ResetTransientState` vs `LoadWorld` handle this.
- **`NodeKey` snapping is the source of truth for connectivity.** If an
  agent is tempted to add a float tolerance to `nodeKeyFromVec2`, don't —
  quantization is the whole contract. Fix the editor-side snap instead.
- **Float32 throughout.** The sim uses `float32` everywhere (math.Float32
  helpers, custom `sqrtf`, etc.). Don't silently widen to `float64` in hot
  paths.
- **Parallelism.** Go side uses `parallelFor` (GOMAXPROCS workers). C side
  uses OpenMP. Per-thread scratch buffers are allocated once per
  `compute_braking_decisions` call — merged after the parallel section.
  Don't add `#pragma omp critical` in hot inner loops; use atomics or
  per-thread accumulators like the existing code.

## Profiling

`F3` toggles the overlay (see comment in `main.go`). Fields correspond to
the `*Profile` structs in `sim.go` (`BrakingProfile`, `FollowingProfile`,
`UpdateCarsProfile`). The C kernel timings use `omp_get_wtime`; the Go
timings use `time.Since`. `profile.KernelMS` is the in-C total;
`profile.MarshalMS` + `profile.UnmarshalMS` are the cgo boundary cost.
When tuning, look at `BrakingProfile.ConflictScanMS` vs `BrakeProbeMS` vs
`HoldProbeMS` first — those are the three parallel regions.
