package sim

import "testing"

func straightSpline(id int, x0, y0, x1, y1 float32) Spline {
	dx := x1 - x0
	dy := y1 - y0
	return NewSpline(
		id,
		NewVec2(x0, y0),
		NewVec2(x0+dx/3, y0+dy/3),
		NewVec2(x0+2*dx/3, y0+2*dy/3),
		NewVec2(x1, y1),
	)
}

func TestFitPlayerProxyCarChoosesCoupledLane(t *testing.T) {
	left := straightSpline(1, 0, 0, 40, 0)
	right := straightSpline(2, 0, 4, 40, 4)
	left.HardCoupledIDs = []int{2}
	right.HardCoupledIDs = []int{1}
	splines := []Spline{left, right}

	result, ok := FitPlayerProxyCar(PlayerProxyFitInput{
		Position: NewVec2(20, 3.4),
		Heading:  NewVec2(1, 0),
		Speed:    8,
		Length:   4.6,
		Width:    1.9,
		Previous: PlayerProxyAttachment{
			Valid:            true,
			CurrentSplineID:  1,
			DistanceOnSpline: 20,
		},
	}, splines)
	if !ok {
		t.Fatal("expected proxy fit result")
	}
	if result.Attachment.CurrentSplineID != 2 {
		t.Fatalf("expected lane change to spline 2, got spline %d", result.Attachment.CurrentSplineID)
	}
}

func TestFitPlayerProxyCarChoosesIntersectionBranchByHeading(t *testing.T) {
	incoming := straightSpline(1, 0, 0, 20, 0)
	straight := straightSpline(2, 20, 0, 40, 0)
	turn := straightSpline(3, 20, 0, 20, -20)
	splines := []Spline{incoming, straight, turn}

	result, ok := FitPlayerProxyCar(PlayerProxyFitInput{
		Position: NewVec2(20, -6),
		Heading:  NewVec2(0, -1),
		Speed:    7,
		Length:   4.6,
		Width:    1.9,
		Previous: PlayerProxyAttachment{
			Valid:            true,
			CurrentSplineID:  1,
			DistanceOnSpline: 18,
		},
	}, splines)
	if !ok {
		t.Fatal("expected proxy fit result")
	}
	if result.Attachment.CurrentSplineID != 3 {
		t.Fatalf("expected turn spline 3, got spline %d", result.Attachment.CurrentSplineID)
	}
}

func TestFitPlayerProxyCarAlignsProxyCenterWithPlayerCenter(t *testing.T) {
	splines := []Spline{
		straightSpline(1, 0, 0, 40, 0),
	}

	input := PlayerProxyFitInput{
		Position: NewVec2(20, 0),
		Heading:  NewVec2(1, 0),
		Speed:    8,
		Length:   4.6,
		Width:    1.9,
	}
	result, ok := FitPlayerProxyCar(input, splines)
	if !ok {
		t.Fatal("expected proxy fit result")
	}

	splinePos, tangent := SampleSplineAtDistance(splines[0], result.Car.DistanceOnSpline)
	frontPos := vecAdd(splinePos, vecScale(Vec2{X: tangent.Y, Y: -tangent.X}, result.Car.LateralOffset))
	center := vecScale(vecAdd(frontPos, result.Car.RearPosition), 0.5)
	if sqrtf(distSq(center, input.Position)) > 0.25 {
		t.Fatalf("expected proxy center near player center, got center=(%.3f, %.3f) player=(%.3f, %.3f)", center.X, center.Y, input.Position.X, input.Position.Y)
	}
}
