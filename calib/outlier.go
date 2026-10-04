package calib

import "sort"

// detectOutliers iteratively fits the model and rejects frames whose
// model-vs-truth residual is inconsistent with the robust noise level.
//
// Why this does not eat legitimate high-curvature turns: the kinematic model
// explains turns *exactly* (a sharp arc is just a large dθ that the model
// reproduces from the encoder ticks), so a normal turn frame has a residual
// at the noise level. A slip/collision frame is one where the encoders and
// the truth disagree in a way no choice of (KL,KR,KB) can explain — its
// residual is large regardless of the fitted parameters. We therefore
// threshold the *residual*, never the curvature.
//
// The threshold is median + OutlierSigma·1.4826·MAD (MAD = median absolute
// deviation), a robust σ estimate that the outliers themselves cannot
// inflate, floored at OutlierFloor so ultra-clean logs don't get shaved.
// The total rejected fraction is hard-capped at MaxOutlierFrac.
//
// Returns the inlier mask (true = keep) and the params fitted on the
// surviving inliers.
func detectOutliers(d Dataset, frames []Frame, cfg Config) ([]bool, []float64, Params) {
	n := len(frames)
	use := make([]bool, n)
	for i := range use {
		use[i] = true
	}
	maxOut := int(cfg.MaxOutlierFrac * float64(n))
	var p fitResult
	var res []float64
	for iter := 0; iter < cfg.MaxOutlierIters; iter++ {
		p = fit(d, frames, use)
		res = residuals(d, frames, p.Params)

		// Robust scale over currently-kept frames.
		var kept []float64
		for i := 1; i < n; i++ {
			if use[i] {
				kept = append(kept, res[i])
			}
		}
		med := median(kept)
		var dev []float64
		for _, v := range kept {
			dev = append(dev, abs(v-med))
		}
		sigma := 1.4826 * median(dev)
		thr := med + cfg.OutlierSigma*sigma
		if thr < cfg.OutlierFloor {
			thr = cfg.OutlierFloor
		}

		var flagged []int
		for i := 1; i < n; i++ {
			if use[i] && res[i] > thr {
				flagged = append(flagged, i)
			}
		}
		if len(flagged) == 0 {
			break
		}
		if len(flagged) > maxOut {
			// Keep only the worst offenders: never mass-delete data.
			sort.Slice(flagged, func(a, b int) bool { return res[flagged[a]] > res[flagged[b]] })
			flagged = flagged[:maxOut]
		}
		for _, i := range flagged {
			use[i] = false
		}
	}
	p = fit(d, frames, use)
	res = residuals(d, frames, p.Params)
	return use, res, p.Params
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
