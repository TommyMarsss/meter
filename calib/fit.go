package calib

import (
	"math"
	"sort"
)

// fitResult is the output of one least-squares fit.
type fitResult struct {
	Params
	// RotIdentifiable is false when the data contains too little turning
	// motion to identify KB; KB is then left at 1.
	RotIdentifiable bool
}

// fit estimates KL, KR, KB from the frames selected by use (use[i]==true),
// by ordinary least squares against the ground-truth increments.
//
// Translation:  dsT = a·nL + b·nR          (2x2 normal equations, a=c·KL/2, b=c·KR/2)
// Rotation:     dθT = λ·u, u=(KR·nR−KL·nL)·c/wheelBase   (1-D LS, KB=1/λ)
//
// Splitting it this way keeps every solve closed-form; no matrix library is
// needed. When the robot never turns (nL≈nR for all frames) the 2x2 system
// is rank-deficient and we fall back to a single shared scale factor.
func fit(d Dataset, frames []Frame, use []bool) fitResult {
	c := d.TickScale()
	var sLL, sRR, sLR, sLs, sRs float64
	for i := 1; i < len(frames); i++ {
		if use != nil && !use[i] {
			continue
		}
		dsT, _ := TruthIncrement(frames[i-1], frames[i])
		nL, nR := frames[i].TicksL, frames[i].TicksR
		sLL += nL * nL
		sRR += nR * nR
		sLR += nL * nR
		sLs += nL * dsT
		sRs += nR * dsT
	}
	var kL, kR float64
	det := sLL*sRR - sLR*sLR
	if det > 1e-9*sLL*sRR && det > 0 {
		a := (sLs*sRR - sRs*sLR) / det // = c·KL/2
		b := (sLL*sRs - sLR*sLs) / det // = c·KR/2
		kL, kR = 2*a/c, 2*b/c
	} else {
		// Rank-deficient (straight-line-only data): fit one shared scale.
		var sNN, sNs float64
		for i := 1; i < len(frames); i++ {
			if use != nil && !use[i] {
				continue
			}
			dsT, _ := TruthIncrement(frames[i-1], frames[i])
			n := frames[i].TicksL + frames[i].TicksR
			sNN += n * n
			sNs += n * dsT
		}
		a := 0.0
		if sNN > 0 {
			a = sNs / sNN
		}
		kL, kR = 2*a/c, 2*a/c
	}

	// Rotation: dθT = (1/KB)·u with u = (kR·nR − kL·nL)·c/wheelBase.
	var su2, sud, suAbs float64
	for i := 1; i < len(frames); i++ {
		if use != nil && !use[i] {
			continue
		}
		_, dthT := TruthIncrement(frames[i-1], frames[i])
		u := (kR*frames[i].TicksR - kL*frames[i].TicksL) * c / d.WheelBase
		su2 += u * u
		sud += u * dthT
		suAbs += math.Abs(u)
	}
	kB, ident := 1.0, false
	if su2 > 1e-12 {
		if lambda := sud / su2; lambda > 0 {
			kB = 1 / lambda
			ident = true
		}
	}
	return fitResult{Params: Params{KL: kL, KR: kR, KB: kB}, RotIdentifiable: ident}
}

// residuals returns, per frame i>0, the model-vs-truth increment mismatch
// under params p: hypot(Δds, wheelBase·Δdθ). Frame 0 has residual 0.
func residuals(d Dataset, frames []Frame, p Params) []float64 {
	r := make([]float64, len(frames))
	for i := 1; i < len(frames); i++ {
		dsT, dthT := TruthIncrement(frames[i-1], frames[i])
		dsP, dthP := d.Increment(frames[i], p)
		r[i] = math.Hypot(dsT-dsP, d.WheelBase*WrapAngle(dthT-dthP))
	}
	return r
}

func median(v []float64) float64 {
	if len(v) == 0 {
		return 0
	}
	s := make([]float64, len(v))
	copy(s, v)
	sort.Float64s(s)
	if len(s)%2 == 1 {
		return s[len(s)/2]
	}
	return (s[len(s)/2-1] + s[len(s)/2]) / 2
}
