# Traffic Sim

`traffic_sim` is a small Go traffic simulator with an interactive road editor. It is aimed at people who want to sketch road layouts quickly, define traffic flows, and watch cars and buses move through the network without building a full game or transport stack first.

The entire project has been vibecoded in a week, but is pretty good nonetheless.

The repository has two parts:

- a desktop editor/sandbox in `main.go` built with Raylib
- a reusable simulation core in [`sim`](./sim)

## What It Does

This project lets you:

- draw road networks as connected cubic Bezier splines
- add car routes and bus lines
- mark priority lanes, lane couplings, lane preferences, and speed limits
- place traffic lights and define multi-phase traffic cycles
- save/load scenarios as JSON
- run the simulation live and inspect how vehicles behave

There are example scenarios in [`examples`](./examples) and larger test maps in [`tests`](./tests).

## Quick Start

Requirements:

- Go 1.22+
- Raylib support on your machine so `raylib-go` can build
- `zenity`, `yad`, or `kdialog` if you want the built-in file picker for `Ctrl+S` / `Ctrl+O`

Run the editor:

```bash
go run .
```

A good first step is to launch the app and load one of the example maps from `examples/`.

## Basic Workflow

The editor is split into four modes: Draw, Rules, Route, and Traffic.

### Draw Scenarios

Use Draw mode to create the road geometry.

- `E` selects the spline tool: click to place `P0`, `P1`, `P2`, and `P3`
- `Q` selects the quadratic helper tool: place start, middle control, and end; the editor converts it to a cubic spline internally
- `C` cuts an existing spline into two
- `X` reverses spline direction

Road connectivity comes from matching endpoints, so the normal workflow is to keep snapping new splines onto existing spline ends. The editor also supports continuation, axis snapping, and mirrored handles to make lane drawing easier.

### Add Driving Rules

Use Rules mode to define how traffic should treat each lane.

- `P`: mark a spline as priority
- `L`: couple two splines so vehicles can lane-change between them
- `S`: apply a custom speed limit
- `V`: assign lane preference numbers

These settings are not just visual metadata. They directly affect route choice, speed targets, overtaking, and lane changes.

### Add Routes

Use Route mode to create demand.

- `R`: create car routes by picking a spline start endpoint and a spline end endpoint
- `B`: create bus routes the same way, then add bus stops in travel order
- right-click on a spline in bus mode to toggle whether it is bus-only

Each route has a spawn rate, and the sim keeps spawning vehicles onto valid routes over time.

### Add Traffic Lights

Use Traffic mode (`T`) to place traffic lights on splines, group them into a traffic cycle, and define the green/clearance phases for that cycle.

### Save and Load

- `Ctrl+S` saves the current scenario to JSON
- `Ctrl+O` loads a saved scenario

Scenario files contain splines, routes, active cars, traffic lights, and traffic cycles.

## How The Simulation Works

At a high level:

1. Roads are stored as sampled cubic splines with cached length, tangents, and curvature.
2. A road graph is built from spline endpoints that meet exactly.
3. Routes use weighted shortest-path search over that graph.
4. `World.Step(dt)` advances traffic, lane changes, traffic lights, and route spawning.

Vehicle behavior is driven by several layers:

- pathfinding uses travel time plus a simple congestion penalty
- car routes avoid bus-only splines, while bus routes may use them
- target speed is limited by the vehicle max speed, spline speed limit, curve speed, traffic lights, bus stops, and the car ahead
- conflict handling uses short-horizon collision prediction and priority rules
- lane changes are modeled as temporary bridging splines between coupled lanes

The result is not a full microscopic traffic research simulator, but it is detailed enough to explore merges, overtakes, bus stops, intersections, roundabouts, and signal timing on hand-drawn maps.

## Scenario Files

Saved scenarios are JSON. The main top-level sections are:

- `splines`
- `routes`
- `cars`
- `traffic_lights`
- `traffic_cycles`

The easiest way to understand the format is to save a small map from the editor and inspect the JSON, or start from a file in [`examples`](./examples).

There is also a small helper script for duplicating a scenario into a grid:

```bash
python3 tile_map.py examples/roundabout.json tiled.json --cols 3 --rows 3 --spacing 800
```

## Building On Top Of `sim`

The simulation core is not tied to Raylib. The `sim` package exposes the core types and update logic, including:

- `World`
- `Spline`
- `Route`
- `TrafficLight`
- `TrafficCycle`
- `RoadGraph`

That means you can use this repository as more than just an editor. If you want to build another tool on top of it, `sim` already gives you scenario loading/saving, route computation, graph queries, spline sampling, traffic-light state updates, and per-step world simulation.

The current desktop app is one consumer of that package. Another project could just as well use `sim` headlessly, attach a different renderer, or wrap it in custom gameplay or planning logic.
