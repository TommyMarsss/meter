package calib

import (
	"math"
	"math/rand"
)

// SimConfig configures the synthetic-log generator used by tests and the
// demo mode. The generator plays the physical causality in the correct
// direction: the robot executes a true motion, and encoder ticks are then
// derived from the *true* (distorted) wheel parameters plus noise — so a
// correct calibration must recover KL/KR/KB.
type SimConfig struct {
	Seed       int64
	KL, KR, KB float64 // true systematic factors to be recovered
	NoiseStd   float64 // encoder noise, ticks (1σ)
	SlipCount  int     // number of slip/collision frames to inject
	SlipMag    float64 // slip magnitude, ticks
	Primitives int     // number of motion primitives in the path
	DT         float64 // sample period, seconds
}

// DefaultSimConfig is a realistic scenario: ~3% wheel-radius mismatch, 6%
// wheelbase error, moderate noise, a handful of slip events.
func DefaultSimConfig() SimConfig {
	return SimConfig{
		Seed:       42,
		KL:         1.030,
		KR:         0.975,
		KB:         1.060,
		NoiseStd:   2.0,
		SlipCount:  10,
		SlipMag:    1500,
		Primitives: 14,
		DT:         0.05,
	}
}

// NominalRobot returns plausible nominal hardware parameters.
func NominalRobot() Dataset {
	return Dataset{TicksPerRev: 4096, WheelRadius: 0.05, WheelBase: 0.32}
}

// Simulate generates a synthetic calibration log. It returns the dataset and
// the indices of the injected slip frames (ground truth for tests).
//
// The path mixes straights, gentle arcs, and deliberately aggressive
// high-curvature turns (radius down to ~1.5 wheelbases) plus in-place
// pivots, so tests can verify that legitimate hard turns are never mistaken
// for slip.
func Simulate(cfg SimConfig) (Dataset, []int) {
	rng := rand.New(rand.NewSource(cfg.Seed))
	d := NominalRobot()
	c := d.TickScale()
	bTrue := cfg.KB * d.WheelBase

	type prim struct {
		v, w float64 // linear / angular velocity
		n    int     // frames
	}
	var prims []prim
	for i := 0; i < cfg.Primitives; i++ {
		switch i % 4 {
		case 0: // straight
			prims = append(prims, prim{v: 0.25 + 0.25*rng.Float64(), n: 30 + rng.Intn(50)})
		case 1: // gentle arc left
			prims = append(prims, prim{v: 0.3, w: 0.3 + 0.3*rng.Float64(), n: 25 + rng.Intn(40)})
		case 2: // gentle arc right
			prims = append(prims, prim{v: 0.3, w: -(0.3 + 0.3*rng.Float64()), n: 25 + rng.Intn(40)})
		case 3: // aggressive: pivot or very tight arc (curvature radius ~1.5 wheelbases)
			if rng.Intn(2) == 0 {
				w := 1.0 + 0.4*rng.Float64()
				if rng.Intn(2) == 0 {
					w = -w
				}
				prims = append(prims, prim{v: 0, w: w, n: 15 + rng.Intn(20)})
			} else {
				w := 1.6 + 0.6*rng.Float64() // v/w ≈ 0.16 m ≈ 0.5·wheelBase radius... tight
				if rng.Intn(2) == 0 {
					w = -w
				}
				prims = append(prims, prim{v: 0.25, w: w, n: 15 + rng.Intn(20)})
			}
		}
	}

	// True motion -> truth poses; encoder ticks from distorted params.
	d.Frames = append(d.Frames, Frame{X: 0, Y: 0, Theta: 0})
	x, y, th := 0.0, 0.0, 0.0
	for _, pr := range prims {
		for k := 0; k < pr.n; k++ {
			ds := pr.v * cfg.DT
			dth := pr.w * cfg.DT
			// true wheel distances
			dL := ds - dth*bTrue/2
			dR := ds + dth*bTrue/2
			// encoder ticks: wheels with effective radii KL,KR
			nL := dL/(cfg.KL*c) + cfg.NoiseStd*rng.NormFloat64()
			nR := dR/(cfg.KR*c) + cfg.NoiseStd*rng.NormFloat64()
			// truth pose integrates the true motion
			x += ds * math.Cos(th+dth/2)
			y += ds * math.Sin(th+dth/2)
			th = WrapAngle(th + dth)
			d.Frames = append(d.Frames, Frame{TicksL: nL, TicksR: nR, X: x, Y: y, Theta: th})
		}
	}

	// Inject slip/collision frames: encoders report a large spurious jump
	// (wheel spun freely / robot got stuck) while the truth keeps its true
	// course. Truth poses are untouched — that is exactly what a motion-
	// capture system would record.
	var slipIdx []int
	if cfg.SlipCount > 0 && len(d.Frames) > 2 {
		perm := rng.Perm(len(d.Frames) - 1)
		for i := 0; i < cfg.SlipCount; i++ {
			idx := perm[i] + 1
			sign := 1.0
			if rng.Intn(2) == 0 {
				sign = -1
			}
			mag := cfg.SlipMag * (0.6 + 0.8*rng.Float64())
			if rng.Intn(2) == 0 {
				d.Frames[idx].TicksL += sign * mag
			} else {
				d.Frames[idx].TicksR += sign * mag
			}
			slipIdx = append(slipIdx, idx)
		}
	}
	return d, slipIdx
}
