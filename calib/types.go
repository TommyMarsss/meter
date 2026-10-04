// Package calib implements odometry calibration for a differential-drive
// robot: least-squares fitting of systematic wheel-radius / wheelbase
// correction factors, robust outlier-frame rejection, and hold-out
// validation against overfitting. Standard library only.
package calib

import "math"

// Frame is one sample of the calibration log. TicksL/TicksR are the encoder
// tick increments between the previous frame and this one (frame 0 carries
// zeros). X/Y/Theta is the ground-truth pose at this frame.
type Frame struct {
	TicksL float64 `json:"tl"`
	TicksR float64 `json:"tr"`
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	Theta  float64 `json:"theta"`
}

// Dataset is a calibration log plus the nominal (factory) robot parameters.
type Dataset struct {
	TicksPerRev float64 `json:"ticks_per_rev"`
	WheelRadius float64 `json:"wheel_radius"` // nominal radius, meters
	WheelBase   float64 `json:"wheel_base"`   // nominal track width, meters
	Frames      []Frame `json:"frames"`
}

// Pose is a 2D robot pose.
type Pose struct {
	X, Y, Theta float64
}

// Params are the systematic correction factors estimated by calibration.
// A value of 1.0 means "matches the nominal parameter exactly".
//
//	dL = KL * c * ticksL        (c = nominal meters-per-tick)
//	dR = KR * c * ticksR
//	ds = (dL + dR) / 2
//	dθ = (dR - dL) / (KB * wheelBase)
//
// KL/KR absorb the effective wheel-radius ratio (diameter difference, tire
// compression); KB absorbs the effective wheelbase error.
type Params struct {
	KL, KR, KB float64
}

// Nominal returns the identity correction (all factors 1).
func Nominal() Params { return Params{KL: 1, KR: 1, KB: 1} }

// Config tunes the calibration pipeline.
type Config struct {
	// Outlier rejection
	OutlierSigma    float64 // MAD-robust threshold multiplier
	OutlierFloor    float64 // absolute residual floor (m) below which nothing is flagged
	MaxOutlierFrac  float64 // hard cap on the fraction of frames that may be dropped
	MaxOutlierIters int     // fit/reject iterations

	// Validation / overfitting detection
	Segments       int     // contiguous segments the log is split into (odd=train, even=validation)
	MinImprovement float64 // required relative validation-error reduction, e.g. 0.10 = 10%
	MinFrames      int     // minimum number of training frames
	MinRotation    float64 // minimum total |dθ| (rad) in training data for KB identifiability
	MinDistance    float64 // minimum total path length (m) in training data
	StabilityTol   float64 // max allowed param drift between two halves of the training set
}

// DefaultConfig returns conservative defaults.
func DefaultConfig() Config {
	return Config{
		OutlierSigma:    8.0,
		OutlierFloor:    0.02,
		MaxOutlierFrac:  0.25,
		MaxOutlierIters: 6,
		Segments:        5,
		MinImprovement:  0.10,
		MinFrames:       50,
		MinRotation:     2.0,
		MinDistance:     2.0,
		StabilityTol:    0.03,
	}
}

// Status is the calibration verdict.
type Status int

const (
	// StatusOK: calibrated parameters reduced the held-out error reliably.
	StatusOK Status = iota
	// StatusOverfit: parameters fit the training data but did not reduce the
	// validation error — the correction is not trustworthy.
	StatusOverfit
	// StatusInsufficientData: not enough frames / motion excitation to
	// identify the parameters at all.
	StatusInsufficientData
)

func (s Status) String() string {
	switch s {
	case StatusOK:
		return "OK"
	case StatusOverfit:
		return "OVERFIT / UNRELIABLE"
	case StatusInsufficientData:
		return "INSUFFICIENT DATA"
	}
	return "UNKNOWN"
}

// TickScale returns nominal meters per encoder tick.
func (d Dataset) TickScale() float64 {
	return 2 * math.Pi * d.WheelRadius / d.TicksPerRev
}
