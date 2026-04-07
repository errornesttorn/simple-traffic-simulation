package sim

/*
#cgo CFLAGS: -Ofast -march=native -fopenmp -flto
#cgo LDFLAGS: -fopenmp -lm
#include "braking.h"
#include <stdlib.h>
*/
import "C"
import (
	"unsafe"
)

func computeBrakingDecisionsC(cars []Car, graph *RoadGraph, debugSelectedCar int, debugSelectedCarMode int) ([]bool, []bool, []DebugBlameLink, []DebugBlameLink, []DebugBlameLink, BrakingProfile) {
	N := len(cars)
	flags := make([]bool, N)
	holdSpeed := make([]bool, N)
	var profile BrakingProfile
	profile.Cars = N

	if N < 2 {
		return flags, holdSpeed, nil, nil, nil, profile
	}

	var cAllocs []unsafe.Pointer
	freeLater := func(p unsafe.Pointer) unsafe.Pointer {
		cAllocs = append(cAllocs, p)
		return p
	}
	defer func() {
		for i := len(cAllocs) - 1; i >= 0; i-- {
			C.free(cAllocs[i])
		}
	}()

	// ── Marshal cars ────────────────────────────────────────
	cCarsPtr := (*C.CCar)(freeLater(C.malloc(C.size_t(N) * C.size_t(unsafe.Sizeof(C.CCar{})))))
	cCars := unsafe.Slice(cCarsPtr, N)

	// ── Pre-populate route trees ────────────────────────────
	type rtKey struct {
		dest int
		vk   VehicleKind
	}
	rtMap := make(map[rtKey]int)
	var rtSlice []routeTreeEntry

	for i := range cars {
		k := rtKey{cars[i].DestinationSplineID, cars[i].VehicleKind}
		if _, ok := rtMap[k]; !ok {
			tree, ok := graph.routeTree(k.dest, k.vk)
			if ok {
				rtMap[k] = len(rtSlice)
				rtSlice = append(rtSlice, tree)
			} else {
				empty := routeTreeEntry{
					costs:   make([]float32, len(graph.splines)),
					nextHop: make([]int, len(graph.splines)),
				}
				for j := range empty.costs {
					empty.costs[j] = 1e30
					empty.nextHop[j] = -1
				}
				rtMap[k] = len(rtSlice)
				rtSlice = append(rtSlice, empty)
			}
		}
	}

	for i := range cars {
		c := &cars[i]
		cc := &cCars[i]
		cc.current_spline_id = C.int(c.CurrentSplineID)
		cc.destination_spline_id = C.int(c.DestinationSplineID)
		cc.prev_spline_ids[0] = C.int(c.PrevSplineIDs[0])
		cc.prev_spline_ids[1] = C.int(c.PrevSplineIDs[1])
		cc.distance_on_spline = C.float(c.DistanceOnSpline)
		cc.rear_position = C.CVec2{C.float(c.RearPosition.X), C.float(c.RearPosition.Y)}
		cc.lateral_offset = C.float(c.LateralOffset)
		cc.speed = C.float(c.Speed)
		cc.max_speed = C.float(c.MaxSpeed)
		cc.accel = C.float(c.Accel)
		cc.length = C.float(c.Length)
		cc.width = C.float(c.Width)
		cc.vehicle_kind = C.int(c.VehicleKind)
		cc.lane_changing = boolToInt(c.LaneChanging)
		cc.lane_change_spline_id = C.int(c.LaneChangeSplineID)
		cc.after_spline_id = C.int(c.AfterSplineID)
		cc.after_spline_dist = C.float(c.AfterSplineDist)
		cc.has_trailer = boolToInt(c.Trailer.HasTrailer)
		cc.trailer_length = C.float(c.Trailer.Length)
		cc.trailer_width = C.float(c.Trailer.Width)
		cc.trailer_rear_position = C.CVec2{C.float(c.Trailer.RearPosition.X), C.float(c.Trailer.RearPosition.Y)}
		cc.route_id = C.int(c.RouteID)
		k := rtKey{c.DestinationSplineID, c.VehicleKind}
		cc.route_tree_index = C.int(rtMap[k])
	}

	// ── Marshal graph ───────────────────────────────────────
	numSplines := len(graph.splines)
	maxID := 0
	for _, sp := range graph.splines {
		if sp.ID > maxID {
			maxID = sp.ID
		}
	}

	indexCount := maxID + 1
	if indexCount < 1 {
		indexCount = 1
	}
	indexByIDPtr := (*C.int)(freeLater(C.malloc(C.size_t(indexCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
	indexByID := unsafe.Slice(indexByIDPtr, indexCount)
	for i := range indexByID {
		indexByID[i] = -1
	}
	for i, sp := range graph.splines {
		if sp.ID >= 0 && sp.ID <= maxID {
			indexByID[sp.ID] = C.int(i)
		}
	}

	splineCount := numSplines
	if splineCount < 1 {
		splineCount = 1
	}
	cSplinesPtr := (*C.CSpline)(freeLater(C.malloc(C.size_t(splineCount) * C.size_t(unsafe.Sizeof(C.CSpline{})))))
	cSplines := unsafe.Slice(cSplinesPtr, splineCount)
	var hardCoupledFlat []C.int
	for i, sp := range graph.splines {
		cs := &cSplines[i]
		cs.id = C.int(sp.ID)
		cs.priority = boolToInt(sp.Priority)
		cs.bus_only = boolToInt(sp.BusOnly)
		cs.p0 = C.CVec2{C.float(sp.P0.X), C.float(sp.P0.Y)}
		cs.p1 = C.CVec2{C.float(sp.P1.X), C.float(sp.P1.Y)}
		cs.p2 = C.CVec2{C.float(sp.P2.X), C.float(sp.P2.Y)}
		cs.p3 = C.CVec2{C.float(sp.P3.X), C.float(sp.P3.Y)}
		cs.length = C.float(sp.Length)
		cs.speed_factor = C.float(sp.SpeedFactor)
		for j := 0; j < simSamples+1; j++ {
			cs.samples[j] = C.CVec2{C.float(sp.Samples[j].X), C.float(sp.Samples[j].Y)}
			cs.cum_len[j] = C.float(sp.CumLen[j])
		}
		if len(sp.SampleTangents) == simSamples+1 {
			for j := 0; j < simSamples+1; j++ {
				cs.sample_tangents[j] = C.CVec2{C.float(sp.SampleTangents[j].X), C.float(sp.SampleTangents[j].Y)}
			}
		}
		if len(sp.SampleCurv) == simSamples+1 {
			for j := 0; j < simSamples+1; j++ {
				cs.sample_curv[j] = C.float(sp.SampleCurv[j])
			}
		}
		cs.hard_coupled_offset = C.int(len(hardCoupledFlat))
		cs.hard_coupled_count = C.int(len(sp.HardCoupledIDs))
		for _, hcid := range sp.HardCoupledIDs {
			hardCoupledFlat = append(hardCoupledFlat, C.int(hcid))
		}
	}
	if len(hardCoupledFlat) == 0 {
		hardCoupledFlat = append(hardCoupledFlat, 0)
	}
	hardCoupledPtr := (*C.int)(freeLater(C.malloc(C.size_t(len(hardCoupledFlat)) * C.size_t(unsafe.Sizeof(C.int(0))))))
	hardCoupled := unsafe.Slice(hardCoupledPtr, len(hardCoupledFlat))
	copy(hardCoupled, hardCoupledFlat)

	sbnCount := numSplines
	if sbnCount < 1 {
		sbnCount = 1
	}
	sbnOffsetsPtr := (*C.int)(freeLater(C.malloc(C.size_t(sbnCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
	sbnCountsPtr := (*C.int)(freeLater(C.malloc(C.size_t(sbnCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
	sbnOffsets := unsafe.Slice(sbnOffsetsPtr, sbnCount)
	sbnCounts := unsafe.Slice(sbnCountsPtr, sbnCount)
	var sbnData []C.int
	for i, sp := range graph.splines {
		key := nodeKeyFromVec2(sp.P3)
		neighbors := graph.startsByNode[key]
		sbnOffsets[i] = C.int(len(sbnData))
		sbnCounts[i] = C.int(len(neighbors))
		for _, idx := range neighbors {
			sbnData = append(sbnData, C.int(idx))
		}
	}
	if len(sbnData) == 0 {
		sbnData = append(sbnData, 0)
	}
	sbnDataPtr := (*C.int)(freeLater(C.malloc(C.size_t(len(sbnData)) * C.size_t(unsafe.Sizeof(C.int(0))))))
	sbnDataC := unsafe.Slice(sbnDataPtr, len(sbnData))
	copy(sbnDataC, sbnData)

	cRouteTreesSize := C.size_t(len(rtSlice)) * C.size_t(unsafe.Sizeof(C.CRouteTree{}))
	if cRouteTreesSize == 0 {
		cRouteTreesSize = C.size_t(unsafe.Sizeof(C.CRouteTree{}))
	}
	cRouteTreesPtr := (*C.CRouteTree)(freeLater(C.malloc(cRouteTreesSize)))
	cRouteTrees := unsafe.Slice(cRouteTreesPtr, max(len(rtSlice), 1))

	treeValueCount := numSplines
	if treeValueCount < 1 {
		treeValueCount = 1
	}
	for i, rt := range rtSlice {
		costsPtr := (*C.float)(freeLater(C.malloc(C.size_t(treeValueCount) * C.size_t(unsafe.Sizeof(C.float(0))))))
		nextHopPtr := (*C.int)(freeLater(C.malloc(C.size_t(treeValueCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
		costs := unsafe.Slice(costsPtr, treeValueCount)
		nextHop := unsafe.Slice(nextHopPtr, treeValueCount)
		for j := 0; j < numSplines && j < len(rt.costs); j++ {
			costs[j] = C.float(rt.costs[j])
			nextHop[j] = C.int(rt.nextHop[j])
		}
		cRouteTrees[i].costs = costsPtr
		cRouteTrees[i].next_hop = nextHopPtr
	}
	if len(rtSlice) == 0 {
		dummyPtr := (*C.float)(freeLater(C.malloc(C.size_t(unsafe.Sizeof(C.float(0))))))
		dummyIPtr := (*C.int)(freeLater(C.malloc(C.size_t(unsafe.Sizeof(C.int(0))))))
		cRouteTrees[0].costs = dummyPtr
		cRouteTrees[0].next_hop = dummyIPtr
	}

	// ── Output buffers (C-allocated to avoid Go pointer issues) ──
	linkCap := N * 8
	brakeFlags := (*C.int)(freeLater(C.calloc(C.size_t(N), C.size_t(unsafe.Sizeof(C.int(0))))))
	holdFlags := (*C.int)(freeLater(C.calloc(C.size_t(N), C.size_t(unsafe.Sizeof(C.int(0))))))
	debugLinksC := (*C.CBlameLink)(freeLater(C.malloc(C.size_t(linkCap) * C.size_t(unsafe.Sizeof(C.CBlameLink{})))))
	holdLinksC := (*C.CBlameLink)(freeLater(C.malloc(C.size_t(linkCap) * C.size_t(unsafe.Sizeof(C.CBlameLink{})))))
	candidateLinksC := (*C.CBlameLink)(freeLater(C.malloc(C.size_t(linkCap) * C.size_t(unsafe.Sizeof(C.CBlameLink{})))))

	// ── Build context (C-allocated) ─────────────────────────
	ctx := (*C.CBrakingCtx)(freeLater(C.malloc(C.size_t(unsafe.Sizeof(C.CBrakingCtx{})))))
	ctx.cars = cCarsPtr
	ctx.num_cars = C.int(N)
	ctx.graph.splines = cSplinesPtr
	ctx.graph.num_splines = C.int(numSplines)
	ctx.graph.index_by_id = indexByIDPtr
	ctx.graph.max_spline_id = C.int(maxID)
	ctx.graph.sbn_offsets = sbnOffsetsPtr
	ctx.graph.sbn_counts = sbnCountsPtr
	ctx.graph.sbn_data = sbnDataPtr
	ctx.graph.hard_coupled_ids = hardCoupledPtr
	ctx.route_trees = cRouteTreesPtr
	ctx.num_route_trees = C.int(max(len(rtSlice), 1))
	ctx.debug_selected_car = C.int(debugSelectedCar)
	ctx.debug_selected_car_mode = C.int(debugSelectedCarMode)
	ctx.brake_flags = brakeFlags
	ctx.hold_flags = holdFlags
	ctx.debug_links = debugLinksC
	ctx.debug_links_count = 0
	ctx.debug_links_cap = C.int(linkCap)
	ctx.hold_links = holdLinksC
	ctx.hold_links_count = 0
	ctx.hold_links_cap = C.int(linkCap)
	ctx.candidate_links = candidateLinksC
	ctx.candidate_links_count = 0
	ctx.candidate_links_cap = C.int(linkCap)

	C.compute_braking_decisions(ctx)

	// ── Unmarshal results ───────────────────────────────────
	bfSlice := unsafe.Slice(brakeFlags, N)
	hfSlice := unsafe.Slice(holdFlags, N)
	for i := 0; i < N; i++ {
		flags[i] = bfSlice[i] != 0
		holdSpeed[i] = hfSlice[i] != 0
	}

	dlSlice := unsafe.Slice(debugLinksC, int(ctx.debug_links_count))
	debugOut := make([]DebugBlameLink, int(ctx.debug_links_count))
	for i := range debugOut {
		debugOut[i] = DebugBlameLink{FromCarIndex: int(dlSlice[i].from), ToCarIndex: int(dlSlice[i].to)}
	}
	hlSlice := unsafe.Slice(holdLinksC, int(ctx.hold_links_count))
	holdOut := make([]DebugBlameLink, int(ctx.hold_links_count))
	for i := range holdOut {
		holdOut[i] = DebugBlameLink{FromCarIndex: int(hlSlice[i].from), ToCarIndex: int(hlSlice[i].to)}
	}
	clSlice := unsafe.Slice(candidateLinksC, int(ctx.candidate_links_count))
	candOut := make([]DebugBlameLink, int(ctx.candidate_links_count))
	for i := range candOut {
		candOut[i] = DebugBlameLink{FromCarIndex: int(clSlice[i].from), ToCarIndex: int(clSlice[i].to)}
	}

	cp := ctx.profile
	profile = BrakingProfile{
		Cars:                      int(cp.cars),
		BasePredictMS:             float64(cp.base_predict_ms),
		ConflictScanMS:            float64(cp.conflict_scan_ms),
		BrakeProbeMS:              float64(cp.brake_probe_ms),
		HoldProbeMS:               float64(cp.hold_probe_ms),
		FinalizeMS:                float64(cp.finalize_ms),
		BasePredictions:           int(cp.base_predictions),
		StationaryPredictions:     int(cp.stationary_predictions),
		EscapePredictions:         int(cp.escape_predictions),
		FasterPredictions:         int(cp.faster_predictions),
		TotalPredictions:          int(cp.total_predictions),
		TotalPredictionSamples:    int(cp.total_prediction_samples),
		PrimaryPairCandidates:     int(cp.primary_pair_candidates),
		PrimaryBroadPhasePairs:    int(cp.primary_broad_phase_pairs),
		PrimaryCollisionChecks:    int(cp.primary_collision_checks),
		PrimaryCollisionHits:      int(cp.primary_collision_hits),
		StationaryCollisionChecks: int(cp.stationary_collision_checks),
		StationaryCollisionHits:   int(cp.stationary_collision_hits),
		EscapeCollisionChecks:     int(cp.escape_collision_checks),
		EscapeCollisionHits:       int(cp.escape_collision_hits),
		HoldCollisionChecks:       int(cp.hold_collision_checks),
		HoldCollisionHits:         int(cp.hold_collision_hits),
		InitiallyBlamedCars:       int(cp.initially_blamed_cars),
		BrakingCars:               int(cp.braking_cars),
		HoldCars:                  int(cp.hold_cars),
	}

	return flags, holdSpeed, debugOut, holdOut, candOut, profile
}

func boolToInt(b bool) C.int {
	if b {
		return 1
	}
	return 0
}
