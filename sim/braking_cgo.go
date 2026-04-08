package sim

/*
#cgo CFLAGS: -Ofast -march=native -fopenmp -flto
#cgo LDFLAGS: -fopenmp -lm
#include "braking.h"
#include <stdlib.h>
*/
import "C"
import (
	"sync"
	"time"
	"unsafe"
)

type brakingCGraphCache struct {
	key            uint64
	numSplines     int
	maxID          int
	cSplinesPtr    *C.CSpline
	indexByIDPtr   *C.int
	sbnOffsetsPtr  *C.int
	sbnCountsPtr   *C.int
	sbnDataPtr     *C.int
	revOffsetsPtr  *C.int
	revCountsPtr   *C.int
	revDataPtr     *C.int
	hardCoupledPtr *C.int
}

func (c *brakingCGraphCache) free() {
	if c == nil {
		return
	}
	if c.cSplinesPtr != nil {
		C.free(unsafe.Pointer(c.cSplinesPtr))
	}
	if c.indexByIDPtr != nil {
		C.free(unsafe.Pointer(c.indexByIDPtr))
	}
	if c.sbnOffsetsPtr != nil {
		C.free(unsafe.Pointer(c.sbnOffsetsPtr))
	}
	if c.sbnCountsPtr != nil {
		C.free(unsafe.Pointer(c.sbnCountsPtr))
	}
	if c.sbnDataPtr != nil {
		C.free(unsafe.Pointer(c.sbnDataPtr))
	}
	if c.revOffsetsPtr != nil {
		C.free(unsafe.Pointer(c.revOffsetsPtr))
	}
	if c.revCountsPtr != nil {
		C.free(unsafe.Pointer(c.revCountsPtr))
	}
	if c.revDataPtr != nil {
		C.free(unsafe.Pointer(c.revDataPtr))
	}
	if c.hardCoupledPtr != nil {
		C.free(unsafe.Pointer(c.hardCoupledPtr))
	}
}

var brakingGraphMarshalCache struct {
	mu    sync.Mutex
	graph *brakingCGraphCache
}

func brakingGraphTopologyKey(graph *RoadGraph) uint64 {
	const offset = 1469598103934665603
	h := uint64(offset)
	h = hashIntBits(h, len(graph.splines))
	for _, sp := range graph.splines {
		h = hashIntBits(h, sp.ID)
		h = hashIntBits(h, boolToGoInt(sp.Priority))
		h = hashIntBits(h, boolToGoInt(sp.BusOnly))
		h = hashFloat32Bits(h, sp.P0.X)
		h = hashFloat32Bits(h, sp.P0.Y)
		h = hashFloat32Bits(h, sp.P1.X)
		h = hashFloat32Bits(h, sp.P1.Y)
		h = hashFloat32Bits(h, sp.P2.X)
		h = hashFloat32Bits(h, sp.P2.Y)
		h = hashFloat32Bits(h, sp.P3.X)
		h = hashFloat32Bits(h, sp.P3.Y)
		h = hashIntBits(h, len(sp.HardCoupledIDs))
		for _, coupledID := range sp.HardCoupledIDs {
			h = hashIntBits(h, coupledID)
		}
	}
	return h
}

func buildBrakingCGraphCache(graph *RoadGraph) *brakingCGraphCache {
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
	indexByIDPtr := (*C.int)(C.malloc(C.size_t(indexCount) * C.size_t(unsafe.Sizeof(C.int(0)))))
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
	cSplinesPtr := (*C.CSpline)(C.malloc(C.size_t(splineCount) * C.size_t(unsafe.Sizeof(C.CSpline{}))))
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
	hardCoupledPtr := (*C.int)(C.malloc(C.size_t(len(hardCoupledFlat)) * C.size_t(unsafe.Sizeof(C.int(0)))))
	hardCoupled := unsafe.Slice(hardCoupledPtr, len(hardCoupledFlat))
	copy(hardCoupled, hardCoupledFlat)

	sbnCount := numSplines
	if sbnCount < 1 {
		sbnCount = 1
	}
	sbnOffsetsPtr := (*C.int)(C.malloc(C.size_t(sbnCount) * C.size_t(unsafe.Sizeof(C.int(0)))))
	sbnCountsPtr := (*C.int)(C.malloc(C.size_t(sbnCount) * C.size_t(unsafe.Sizeof(C.int(0)))))
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
	sbnDataPtr := (*C.int)(C.malloc(C.size_t(len(sbnData)) * C.size_t(unsafe.Sizeof(C.int(0)))))
	sbnDataC := unsafe.Slice(sbnDataPtr, len(sbnData))
	copy(sbnDataC, sbnData)

	revCount := numSplines
	if revCount < 1 {
		revCount = 1
	}
	revOffsetsPtr := (*C.int)(C.malloc(C.size_t(revCount) * C.size_t(unsafe.Sizeof(C.int(0)))))
	revCountsPtr := (*C.int)(C.malloc(C.size_t(revCount) * C.size_t(unsafe.Sizeof(C.int(0)))))
	revOffsets := unsafe.Slice(revOffsetsPtr, revCount)
	revCounts := unsafe.Slice(revCountsPtr, revCount)
	var revData []C.int
	for i := range graph.reverseNeighbors {
		neighbors := graph.reverseNeighbors[i]
		revOffsets[i] = C.int(len(revData))
		revCounts[i] = C.int(len(neighbors))
		for _, idx := range neighbors {
			revData = append(revData, C.int(idx))
		}
	}
	if len(revData) == 0 {
		revData = append(revData, 0)
	}
	revDataPtr := (*C.int)(C.malloc(C.size_t(len(revData)) * C.size_t(unsafe.Sizeof(C.int(0)))))
	revDataC := unsafe.Slice(revDataPtr, len(revData))
	copy(revDataC, revData)

	return &brakingCGraphCache{
		key:            brakingGraphTopologyKey(graph),
		numSplines:     numSplines,
		maxID:          maxID,
		cSplinesPtr:    cSplinesPtr,
		indexByIDPtr:   indexByIDPtr,
		sbnOffsetsPtr:  sbnOffsetsPtr,
		sbnCountsPtr:   sbnCountsPtr,
		sbnDataPtr:     sbnDataPtr,
		revOffsetsPtr:  revOffsetsPtr,
		revCountsPtr:   revCountsPtr,
		revDataPtr:     revDataPtr,
		hardCoupledPtr: hardCoupledPtr,
	}
}

func cachedBrakingCGraph(graph *RoadGraph) *brakingCGraphCache {
	key := brakingGraphTopologyKey(graph)
	if brakingGraphMarshalCache.graph != nil && brakingGraphMarshalCache.graph.key == key {
		return brakingGraphMarshalCache.graph
	}
	next := buildBrakingCGraphCache(graph)
	if brakingGraphMarshalCache.graph != nil {
		brakingGraphMarshalCache.graph.free()
	}
	brakingGraphMarshalCache.graph = next
	return next
}

func computeBrakingDecisionsC(cars []Car, graph *RoadGraph, debugSelectedCar int, debugSelectedCarMode int) ([]bool, []bool, []DebugBlameLink, []DebugBlameLink, []DebugBlameLink, BrakingProfile) {
	brakingGraphMarshalCache.mu.Lock()
	defer brakingGraphMarshalCache.mu.Unlock()

	start := time.Now()
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
	var rtKeys []rtKey

	for i := range cars {
		k := rtKey{cars[i].DestinationSplineID, cars[i].VehicleKind}
		if _, ok := rtMap[k]; !ok {
			rtMap[k] = len(rtKeys)
			rtKeys = append(rtKeys, k)
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

	// ── Marshal graph (cached across frames) ────────────────
	graphCache := cachedBrakingCGraph(graph)
	numSplines := graphCache.numSplines
	maxID := graphCache.maxID

	segmentCostCount := numSplines
	if segmentCostCount < 1 {
		segmentCostCount = 1
	}
	segmentCostsPtr := (*C.float)(freeLater(C.malloc(C.size_t(segmentCostCount) * C.size_t(unsafe.Sizeof(C.float(0))))))
	segmentCosts := unsafe.Slice(segmentCostsPtr, segmentCostCount)
	for i := 0; i < numSplines && i < len(graph.segmentCosts); i++ {
		segmentCosts[i] = C.float(graph.segmentCosts[i])
	}

	routeTreeReqCount := len(rtKeys)
	if routeTreeReqCount < 1 {
		routeTreeReqCount = 1
	}
	routeTreeDestIDsPtr := (*C.int)(freeLater(C.malloc(C.size_t(routeTreeReqCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
	routeTreeVehicleKindsPtr := (*C.int)(freeLater(C.malloc(C.size_t(routeTreeReqCount) * C.size_t(unsafe.Sizeof(C.int(0))))))
	routeTreeDestIDs := unsafe.Slice(routeTreeDestIDsPtr, routeTreeReqCount)
	routeTreeVehicleKinds := unsafe.Slice(routeTreeVehicleKindsPtr, routeTreeReqCount)
	for i, k := range rtKeys {
		routeTreeDestIDs[i] = C.int(k.dest)
		routeTreeVehicleKinds[i] = C.int(k.vk)
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
	ctx.graph.splines = graphCache.cSplinesPtr
	ctx.graph.num_splines = C.int(numSplines)
	ctx.graph.index_by_id = graphCache.indexByIDPtr
	ctx.graph.max_spline_id = C.int(maxID)
	ctx.graph.sbn_offsets = graphCache.sbnOffsetsPtr
	ctx.graph.sbn_counts = graphCache.sbnCountsPtr
	ctx.graph.sbn_data = graphCache.sbnDataPtr
	ctx.graph.rev_offsets = graphCache.revOffsetsPtr
	ctx.graph.rev_counts = graphCache.revCountsPtr
	ctx.graph.rev_data = graphCache.revDataPtr
	ctx.graph.hard_coupled_ids = graphCache.hardCoupledPtr
	ctx.graph.segment_costs = segmentCostsPtr
	ctx.route_tree_dest_ids = routeTreeDestIDsPtr
	ctx.route_tree_vehicle_kinds = routeTreeVehicleKindsPtr
	ctx.num_route_trees = C.int(len(rtKeys))
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

	marshalMS := sinceMS(start)
	C.compute_braking_decisions(ctx)
	unmarshalStart := time.Now()

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
		MarshalMS:                 marshalMS,
		RouteTreeMS:               float64(cp.route_tree_ms),
		MarshalSetupMS:            float64(cp.marshal_setup_ms),
		BasePredictMS:             float64(cp.base_predict_ms),
		ConflictScanMS:            float64(cp.conflict_scan_ms),
		BrakeProbeMS:              float64(cp.brake_probe_ms),
		HoldProbeMS:               float64(cp.hold_probe_ms),
		FinalizeMS:                float64(cp.finalize_ms),
		KernelMS:                  float64(cp.total_ms),
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
	profile.UnmarshalMS = sinceMS(unmarshalStart)

	return flags, holdSpeed, debugOut, holdOut, candOut, profile
}

func boolToInt(b bool) C.int {
	if b {
		return 1
	}
	return 0
}

func boolToGoInt(b bool) int {
	if b {
		return 1
	}
	return 0
}
