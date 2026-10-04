package calib

import "math"

// segment is a contiguous half-open frame range [Start, End).
type segment struct {
	Start, End int
}

// splitSegments divides n frames into k contiguous, roughly equal segments.
func splitSegments(n, k int) []segment {
	segs := make([]segment, 0, k)
	for i := 0; i < k; i++ {
		s := i * n / k
		e := (i + 1) * n / k
		if e > s {
			segs = append(segs, segment{Start: s, End: e})
		}
	}
	return segs
}

// segmentError dead-reckons through one segment starting from the true pose
// at the segment start and returns the end-pose error against ground truth.
// Outlier frames are skipped (their encoder data is garbage for any params).
func (d Dataset) segmentError(frames []Frame, p Params, inlier []bool, seg segment) float64 {
	sub := frames[seg.Start:seg.End]
	var skip []bool
	if inlier != nil {
		skip = make([]bool, len(sub))
		for i := range sub {
			skip[i] = !inlier[seg.Start+i]
		}
	}
	start := Pose{X: frames[seg.Start].X, Y: frames[seg.Start].Y, Theta: frames[seg.Start].Theta}
	traj := d.Integrate(start, sub, p, skip)
	end := frames[seg.End-1]
	return d.PoseError(traj[len(traj)-1], Pose{X: end.X, Y: end.Y, Theta: end.Theta})
}

// evalSegments returns the RMS end-pose error of dead-reckoning each segment.
func (d Dataset) evalSegments(frames []Frame, p Params, inlier []bool, segs []segment) float64 {
	if len(segs) == 0 {
		return 0
	}
	var sum float64
	for _, s := range segs {
		e := d.segmentError(frames, p, inlier, s)
		sum += e * e
	}
	return math.Sqrt(sum / float64(len(segs)))
}

// excitation summarizes how informative a set of frames is for calibration.
type excitation struct {
	Frames   int
	Distance float64 // total |ds| (m)
	Rotation float64 // total |dθ| (rad)
}

func measureExcitation(frames []Frame, use []bool) excitation {
	var ex excitation
	for i := 1; i < len(frames); i++ {
		if use != nil && !use[i] {
			continue
		}
		ds, dth := TruthIncrement(frames[i-1], frames[i])
		ex.Frames++
		ex.Distance += math.Abs(ds)
		ex.Rotation += math.Abs(dth)
	}
	return ex
}
