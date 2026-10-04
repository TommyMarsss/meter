package calib

import "math"

// WrapAngle normalizes an angle to [-π, π].
func WrapAngle(a float64) float64 { return math.Remainder(a, 2*math.Pi) }

// Increment applies the kinematic model: encoder ticks of one frame ->
// body-frame (ds, dθ) increment under correction factors p.
func (d Dataset) Increment(f Frame, p Params) (ds, dtheta float64) {
	c := d.TickScale()
	dL := p.KL * c * f.TicksL
	dR := p.KR * c * f.TicksR
	ds = (dL + dR) / 2
	dtheta = (dR - dL) / (p.KB * d.WheelBase)
	return ds, dtheta
}

// TruthIncrement derives the real (ds, dθ) increment between two consecutive
// ground-truth poses. ds is signed by the heading direction so backward
// motion is handled correctly.
func TruthIncrement(prev, cur Frame) (ds, dtheta float64) {
	dx, dy := cur.X-prev.X, cur.Y-prev.Y
	dtheta = WrapAngle(cur.Theta - prev.Theta)
	dist := math.Hypot(dx, dy)
	headingMid := prev.Theta + dtheta/2
	if dx*math.Cos(headingMid)+dy*math.Sin(headingMid) < 0 {
		dist = -dist
	}
	return dist, dtheta
}

// Integrate dead-reckons a trajectory from start over frames[1:] using
// midpoint (exact-arc) integration. Frames with skip[i] == true (rejected
// outliers) contribute no increment — the robot is treated as holding still
// for that sample, which is the best available guess when the encoders lied.
func (d Dataset) Integrate(start Pose, frames []Frame, p Params, skip []bool) []Pose {
	out := make([]Pose, len(frames))
	out[0] = start
	theta := start.Theta
	x, y := start.X, start.Y
	for i := 1; i < len(frames); i++ {
		if skip != nil && skip[i] {
			out[i] = Pose{X: x, Y: y, Theta: theta}
			continue
		}
		ds, dth := d.Increment(frames[i], p)
		x += ds * math.Cos(theta+dth/2)
		y += ds * math.Sin(theta+dth/2)
		theta = WrapAngle(theta + dth)
		out[i] = Pose{X: x, Y: y, Theta: theta}
	}
	return out
}

// PoseError is a combined position+heading error metric (meters-equivalent).
// Heading error is converted to a lateral offset at one wheelbase.
func (d Dataset) PoseError(a, b Pose) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y) + d.WheelBase*math.Abs(WrapAngle(a.Theta-b.Theta))
}
