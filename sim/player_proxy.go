package sim

type PlayerProxyAttachment struct {
	Valid            bool
	CurrentSplineID  int
	DistanceOnSpline float32
	LateralOffset    float32
	Confidence       float32
	PrevSplineIDs    [2]int
}

type PlayerProxyFitInput struct {
	Position Vec2
	Heading  Vec2
	Speed    float32

	Length              float32
	Width               float32
	CarID               int
	Color               Color
	DestinationSplineID int

	Previous PlayerProxyAttachment
}

type PlayerProxyFitResult struct {
	Car        Car
	Attachment PlayerProxyAttachment
	Score      float32
}

type proxyProjection struct {
	splineID      int
	distance      float32
	position      Vec2
	tangent       Vec2
	worldDistance float32
	lateralOffset float32
}

const (
	proxyCandidateSame = iota
	proxyCandidateCoupled
	proxyCandidateNext
	proxyCandidateNearby
)

func playerProxyReferencePosition(input PlayerProxyFitInput) Vec2 {
	ref := input.Position
	if input.Length <= 0 || vectorLengthSq(input.Heading) <= 1e-9 {
		return ref
	}
	forward := normalize(input.Heading)
	frontOffset := input.Length * wheelbaseFrac * 0.5
	return vecAdd(ref, vecScale(forward, frontOffset))
}

func projectPointToSpline(s *Spline, pos Vec2) proxyProjection {
	if s == nil {
		return proxyProjection{}
	}

	bestDistSq := float32(1e30)
	bestDistance := float32(0)
	for i := 1; i <= simSamples; i++ {
		a := s.Samples[i-1]
		b := s.Samples[i]
		ab := vecSub(b, a)
		denom := dot(ab, ab)
		t := float32(0)
		if denom > 1e-9 {
			t = clampf(dot(vecSub(pos, a), ab)/denom, 0, 1)
		}
		p := vecAdd(a, vecScale(ab, t))
		dSq := distSq(pos, p)
		if dSq < bestDistSq {
			bestDistSq = dSq
			bestDistance = s.CumLen[i-1] + (s.CumLen[i]-s.CumLen[i-1])*t
		}
	}

	bestDistance = clampf(bestDistance, 0, s.Length)
	splinePos, tangent, _ := sampleSplineStateAtDistance(s, bestDistance, nil)
	rightNormal := Vec2{X: tangent.Y, Y: -tangent.X}
	lateralOffset := dot(vecSub(pos, splinePos), rightNormal)
	return proxyProjection{
		splineID:      s.ID,
		distance:      bestDistance,
		position:      splinePos,
		tangent:       tangent,
		worldDistance: sqrtf(distSq(pos, splinePos)),
		lateralOffset: lateralOffset,
	}
}

func headingPenaltyForProjection(heading Vec2, speed float32, tangent Vec2) float32 {
	if vectorLengthSq(heading) <= 1e-9 {
		return 0
	}
	headingWeight := clampf(maxf(absf(speed), 2.0)/12.0, 0.2, 1.0)
	angleDeg := headingAngleDegrees(heading, tangent)
	penalty := angleDeg * 0.12 * headingWeight
	if angleDeg > 85 {
		penalty += 20 * headingWeight
	}
	return penalty
}

func addProxyCandidate(categories map[int]int, splineID int, category int) {
	if splineID <= 0 {
		return
	}
	prev, ok := categories[splineID]
	if !ok || category < prev {
		categories[splineID] = category
	}
}

func buildProxyCandidateSet(splines []Spline, input PlayerProxyFitInput) map[int]int {
	categories := make(map[int]int, 16)
	indexByID := buildSplineIndexByID(splines)
	startsByNode := BuildStartsByNode(splines)
	referencePos := playerProxyReferencePosition(input)

	if input.Previous.Valid {
		addProxyCandidate(categories, input.Previous.CurrentSplineID, proxyCandidateSame)
		if prevIdx, ok := indexByID[input.Previous.CurrentSplineID]; ok {
			prevSpline := &splines[prevIdx]
			for _, coupledID := range prevSpline.HardCoupledIDs {
				addProxyCandidate(categories, coupledID, proxyCandidateCoupled)
			}
			for _, coupledID := range prevSpline.SoftCoupledIDs {
				addProxyCandidate(categories, coupledID, proxyCandidateCoupled)
			}
			for _, nextIdx := range startsByNode[nodeKeyFromVec2(prevSpline.P3)] {
				nextSpline := &splines[nextIdx]
				addProxyCandidate(categories, nextSpline.ID, proxyCandidateNext)
				for _, coupledID := range nextSpline.HardCoupledIDs {
					addProxyCandidate(categories, coupledID, proxyCandidateNext)
				}
			}
		}
	}

	nearbyRadius := maxf(12, absf(input.Speed)*0.9+8)
	for i := range splines {
		proj := projectPointToSpline(&splines[i], referencePos)
		if proj.worldDistance <= nearbyRadius {
			addProxyCandidate(categories, splines[i].ID, proxyCandidateNearby)
		}
	}

	if len(categories) == 0 {
		for i := range splines {
			addProxyCandidate(categories, splines[i].ID, proxyCandidateNearby)
		}
	}

	return categories
}

func proxyCategoryPenalty(category int) float32 {
	switch category {
	case proxyCandidateSame:
		return 0
	case proxyCandidateCoupled:
		return 2.5
	case proxyCandidateNext:
		return 4.0
	default:
		return 9.0
	}
}

func proxySwitchThreshold(category int, nearEnd bool) float32 {
	switch category {
	case proxyCandidateNext:
		if nearEnd {
			return 1.5
		}
		return 3.0
	case proxyCandidateCoupled:
		return 1.5
	default:
		return 6.0
	}
}

func proxyScoreProjection(proj proxyProjection, category int, input PlayerProxyFitInput, splines []Spline, indexByID map[int]int) float32 {
	score := proj.worldDistance*6 + absf(proj.lateralOffset)*1.5
	score += headingPenaltyForProjection(input.Heading, input.Speed, proj.tangent)
	score += proxyCategoryPenalty(category)

	if !input.Previous.Valid {
		return score
	}

	prevIdx, prevOK := indexByID[input.Previous.CurrentSplineID]
	if !prevOK {
		return score
	}
	prevSpline := &splines[prevIdx]
	nearEnd := prevSpline.Length-input.Previous.DistanceOnSpline <= maxf(10, absf(input.Speed)*0.75+4)

	if proj.splineID == input.Previous.CurrentSplineID {
		delta := proj.distance - input.Previous.DistanceOnSpline
		if input.Speed >= 2 && delta < -6 {
			score += absf(delta+6)*2.5 + 8
		}
		if input.Speed <= -2 && delta > 6 {
			score += absf(delta-6)*2.5 + 8
		}
		return score
	}

	if nearEnd && category == proxyCandidateNext {
		score -= 4
	}

	for _, id := range prevSpline.HardCoupledIDs {
		if id == proj.splineID {
			score -= 1
			break
		}
	}

	return score
}

func buildPlayerProxyCar(input PlayerProxyFitInput, attachment PlayerProxyAttachment, splines []Spline) (Car, bool) {
	spline, ok := findSplinePtrByID(splines, attachment.CurrentSplineID)
	if !ok {
		return Car{}, false
	}

	splinePos, tangent, _ := sampleSplineStateAtDistance(spline, attachment.DistanceOnSpline, nil)
	rightNormal := Vec2{X: tangent.Y, Y: -tangent.X}
	frontPos := vecAdd(splinePos, vecScale(rightNormal, attachment.LateralOffset))
	bodyHeading := tangent
	if vectorLengthSq(input.Heading) > 1e-9 {
		if dot(normalize(input.Heading), tangent) >= 0.5 {
			bodyHeading = normalize(input.Heading)
		}
	}

	length := input.Length
	if length <= 0 {
		length = 4.6
	}
	width := input.Width
	if width <= 0 {
		width = 1.9
	}

	car := Car{
		ID:                  input.CarID,
		RouteID:             -1,
		ControlMode:         CarControlExternal,
		CurrentSplineID:     attachment.CurrentSplineID,
		DestinationSplineID: attachment.CurrentSplineID,
		PrevSplineIDs:       attachment.PrevSplineIDs,
		DistanceOnSpline:    attachment.DistanceOnSpline,
		RearPosition:        vecSub(frontPos, vecScale(bodyHeading, length*wheelbaseFrac)),
		LateralOffset:       attachment.LateralOffset,
		Speed:               absf(input.Speed),
		MaxSpeed:            maxf(absf(input.Speed), 1),
		Accel:               0,
		Length:              length,
		Width:               width,
		Color:               input.Color,
		LaneChangeSplineID:  -1,
		AfterSplineID:       -1,
		DesiredLaneSplineID: -1,
		VehicleKind:         VehicleCar,
	}
	if input.DestinationSplineID > 0 {
		car.DestinationSplineID = input.DestinationSplineID
	}
	return car, true
}

func FitPlayerProxyCar(input PlayerProxyFitInput, splines []Spline) (PlayerProxyFitResult, bool) {
	if len(splines) == 0 {
		return PlayerProxyFitResult{}, false
	}

	indexByID := buildSplineIndexByID(splines)
	referencePos := playerProxyReferencePosition(input)
	candidateSet := buildProxyCandidateSet(splines, input)

	bestScore := float32(1e30)
	bestCategory := proxyCandidateNearby
	bestAttachment := PlayerProxyAttachment{}
	prevScore := float32(1e30)
	prevFound := false

	for splineID, category := range candidateSet {
		idx, ok := indexByID[splineID]
		if !ok {
			continue
		}
		proj := projectPointToSpline(&splines[idx], referencePos)
		score := proxyScoreProjection(proj, category, input, splines, indexByID)

		attachment := PlayerProxyAttachment{
			Valid:            true,
			CurrentSplineID:  splineID,
			DistanceOnSpline: proj.distance,
			LateralOffset:    proj.lateralOffset,
			Confidence:       1 / (1 + score*0.1),
			PrevSplineIDs:    input.Previous.PrevSplineIDs,
		}
		if input.Previous.Valid && input.Previous.CurrentSplineID != splineID {
			attachment.PrevSplineIDs[1] = input.Previous.PrevSplineIDs[0]
			attachment.PrevSplineIDs[0] = input.Previous.CurrentSplineID
		}

		if input.Previous.Valid && splineID == input.Previous.CurrentSplineID {
			prevScore = score
			prevFound = true
		}

		if score < bestScore {
			bestScore = score
			bestCategory = category
			bestAttachment = attachment
		}
	}

	if !bestAttachment.Valid {
		return PlayerProxyFitResult{}, false
	}

	if input.Previous.Valid && prevFound && bestAttachment.CurrentSplineID != input.Previous.CurrentSplineID {
		nearEnd := false
		if prevIdx, ok := indexByID[input.Previous.CurrentSplineID]; ok {
			prevSpline := &splines[prevIdx]
			nearEnd = prevSpline.Length-input.Previous.DistanceOnSpline <= maxf(10, absf(input.Speed)*0.75+4)
		}
		if prevScore <= bestScore+proxySwitchThreshold(bestCategory, nearEnd) {
			bestAttachment = input.Previous
			bestAttachment.Confidence = 1 / (1 + prevScore*0.1)
			bestScore = prevScore
		}
	}

	car, ok := buildPlayerProxyCar(input, bestAttachment, splines)
	if !ok {
		return PlayerProxyFitResult{}, false
	}

	return PlayerProxyFitResult{
		Car:        car,
		Attachment: bestAttachment,
		Score:      bestScore,
	}, true
}

func upsertCarByID(cars []Car, car Car) []Car {
	for i := range cars {
		if cars[i].ID == car.ID {
			cars[i] = car
			return cars
		}
	}
	return append(cars, car)
}

func removeCarByID(cars []Car, carID int) []Car {
	if carID <= 0 {
		return cars
	}
	for i := range cars {
		if cars[i].ID == carID {
			copy(cars[i:], cars[i+1:])
			return cars[:len(cars)-1]
		}
	}
	return cars
}

func (w *World) ClearPlayerProxy() {
	if w == nil {
		return
	}
	w.Cars = removeCarByID(w.Cars, w.PlayerProxyCar.ID)
	w.HasPlayerProxy = false
	w.PlayerProxyCar = Car{}
	w.PlayerProxyAttach = PlayerProxyAttachment{}
}

func (w *World) UpdatePlayerProxy(input PlayerProxyFitInput) bool {
	if w == nil {
		return false
	}
	input.Previous = w.PlayerProxyAttach
	result, ok := FitPlayerProxyCar(input, w.Splines)
	if !ok {
		w.ClearPlayerProxy()
		return false
	}
	w.HasPlayerProxy = true
	w.PlayerProxyCar = result.Car
	w.PlayerProxyAttach = result.Attachment
	w.Cars = upsertCarByID(w.Cars, result.Car)
	return true
}
