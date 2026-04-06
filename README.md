# Simple traffic simulation

`simple-traffic-simulation` is small Go traffic simulator with an interactive road editor. Cars are bound to splines

## What it does

Draw splines, set up a route, and cars be spawning and going

You can mark lanes as priority

Program traffic lights

Cars can change lanes

Bus lines even

## Quick start

- Go 1.22+
- Raylib support on your machine so `raylib-go` can build
- Linux: `zenity`, `yad`, or `kdialog` if you want the built-in file picker for `Ctrl+S` / `Ctrl+O`
- Windows: PowerShell for the built-in file picker
- macOS is not supported

```bash
go run .
```

A good first step is to launch the app, load the turbo roundabout example and stare at it for 5 minutes

Then press the info button and info textfields will contain some information. The spline editor is fairly advanced so will take you some time to figure it out

## How The Simulation Works

At a high level:

1. Roads are stored as sampled cubic splines with cached length, tangents, and curvature.
2. A road graph is built from spline endpoints that meet exactly.
3. Routes use weighted shortest-path search over that graph.
4. `World.Step(dt)` advances traffic, lane changes, traffic lights, and route spawning.

Vehicle behaviour is driven by several layers:

- pathfinding uses travel time plus a simple congestion penalty
- car routes avoid bus-only splines, while bus routes may use them
- target speed is limited by the vehicle max speed, spline speed limit, curve speed, traffic lights, bus stops, and the car ahead
- conflict handling uses short-horizon collision prediction and priority rules
- lane changes are modeled as temporary bridging splines between coupled lanes

The result is not a full microscopic traffic research simulator, but it is detailed enough to explore merges, overtakes, bus stops, intersections, roundabouts, and signal timing on hand-drawn maps.


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
