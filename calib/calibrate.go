package calib

import (
	"fmt"
	"math"
)

// Result is the full output of a calibration run.
type Result struct {
	Params          Params   // calibrated correction factors
	Status          Status   // overall verdict
	Reasons         []string // human-readable findings / failure causes
	Inlier          []bool   // per-frame mask: false = rejected outlier frame
	Residuals       []float64
	OutlierIdx      []int
	Excitation      excitation // measured on the training inliers
	RotIdentifiable bool

	// Error metrics (RMS end-pose error per segment, meters-equivalent).
	TrainBefore, TrainAfter float64
	ValBefore, ValAfter     float64
	ValImprovement          float64 // relative, (before-after)/before

	// Parameter stability: fits on the two halves of the training set.
	StabFirst, StabSecond Params
	StabOK                bool

	// Trajectories for the report: full-sequence dead reckoning.
	RawTraj []Pose // nominal params, all frames
	CalTraj []Pose // calibrated params, outliers skipped

	TrainSegs, ValSegs []segment
}

// Calibrate runs the full pipeline:
//  1. robust outlier rejection (iterative MAD-thresholded refit)
//  2. train/validation split over contiguous segments
//  3. least-squares fit of (KL, KR, KB) on training inliers
//  4. data-sufficiency and stability checks
//  5. hold-out validation: the correction must reduce the validation error
//     by at least cfg.MinImprovement, otherwise the run is reported as
//     overfit/unreliable instead of emitting plausible-looking junk.
func Calibrate(d Dataset, cfg Config) Result {
	frames := d.Frames
	res := Result{}

	// Step 1: global robust outlier detection.
	inlier, residuals, _ := detectOutliers(d, frames, cfg)
	res.Inlier = inlier
	res.Residuals = residuals
	for i := 1; i < len(frames); i++ {
		if !inlier[i] {
			res.OutlierIdx = append(res.OutlierIdx, i)
		}
	}
	res.Reasons = append(res.Reasons,
		fmt.Sprintf("rejected %d/%d frames as outliers (slip/collision)", len(res.OutlierIdx), len(frames)))

	// Step 2: segment split — even segments train, odd segments validate.
	segs := splitSegments(len(frames), cfg.Segments)
	trainMask := make([]bool, len(frames))
	for i, s := range segs {
		train := i%2 == 0
		for j := s.Start; j < s.End; j++ {
			trainMask[j] = train
		}
		if train {
			res.TrainSegs = append(res.TrainSegs, s)
		} else {
			res.ValSegs = append(res.ValSegs, s)
		}
	}
	fitMask := make([]bool, len(frames))
	for i := range frames {
		fitMask[i] = trainMask[i] && inlier[i]
	}

	// Step 3: sufficiency checks on the training inliers.
	ex := measureExcitation(frames, fitMask)
	res.Excitation = ex
	sufficient := true
	if ex.Frames < cfg.MinFrames {
		res.Reasons = append(res.Reasons,
			fmt.Sprintf("only %d usable training frames (need >= %d)", ex.Frames, cfg.MinFrames))
		sufficient = false
	}
	if ex.Distance < cfg.MinDistance {
		res.Reasons = append(res.Reasons,
			fmt.Sprintf("training path too short: %.2f m (need >= %.2f m)", ex.Distance, cfg.MinDistance))
		sufficient = false
	}
	if ex.Rotation < cfg.MinRotation {
		res.Reasons = append(res.Reasons,
			fmt.Sprintf("insufficient turning motion: %.2f rad total (need >= %.2f rad) — wheelbase factor KB not identifiable", ex.Rotation, cfg.MinRotation))
		sufficient = false
	}

	// Step 4: fit on training inliers.
	fr := fit(d, frames, fitMask)
	res.Params = fr.Params
	res.RotIdentifiable = fr.RotIdentifiable
	if !fr.RotIdentifiable {
		res.Reasons = append(res.Reasons, "rotation excitation too weak: KB left at 1.0")
	}

	// Parameter sanity: correction factors beyond ±50% are not a plausible
	// systematic error — indicates bad data or a modelling problem.
	for _, kv := range []struct {
		name string
		v    float64
	}{{"KL", fr.KL}, {"KR", fr.KR}, {"KB", fr.KB}} {
		if math.Abs(kv.v-1) > 0.5 {
			res.Reasons = append(res.Reasons,
				fmt.Sprintf("%s=%.3f is outside physical sanity bounds [0.5, 1.5]", kv.name, kv.v))
			sufficient = false
		}
	}

	// Step 5: stability — refit on each half of the training data.
	halfMask1, halfMask2 := splitMaskAlternating(fitMask)
	f1 := fit(d, frames, halfMask1)
	f2 := fit(d, frames, halfMask2)
	res.StabFirst, res.StabSecond = f1.Params, f2.Params
	res.StabOK = true
	for _, d := range [][2]float64{{f1.KL, f2.KL}, {f1.KR, f2.KR}, {f1.KB, f2.KB}} {
		if math.Abs(d[0]-d[1]) > cfg.StabilityTol {
			res.StabOK = false
		}
	}
	if !res.StabOK {
		res.Reasons = append(res.Reasons, fmt.Sprintf(
			"parameter instability between training halves: (%.4f,%.4f,%.4f) vs (%.4f,%.4f,%.4f)",
			f1.KL, f1.KR, f1.KB, f2.KL, f2.KR, f2.KB))
	}

	// Step 6: error metrics before/after on train and validation segments.
	nom := Nominal()
	res.TrainBefore = d.evalSegments(frames, nom, inlier, res.TrainSegs)
	res.TrainAfter = d.evalSegments(frames, res.Params, inlier, res.TrainSegs)
	res.ValBefore = d.evalSegments(frames, nom, inlier, res.ValSegs)
	res.ValAfter = d.evalSegments(frames, res.Params, inlier, res.ValSegs)
	if res.ValBefore > 1e-9 {
		res.ValImprovement = (res.ValBefore - res.ValAfter) / res.ValBefore
	}

	// Step 7: verdict.
	switch {
	case !sufficient:
		res.Status = StatusInsufficientData
		res.Reasons = append(res.Reasons, "verdict: data insufficient — parameters NOT reliable")
	case res.ValBefore < 1e-6:
		// Nominal odometry already matches truth; nothing to calibrate.
		res.Status = StatusOK
		res.Reasons = append(res.Reasons, "nominal parameters already match ground truth; no correction needed")
	case res.ValImprovement < cfg.MinImprovement:
		res.Status = StatusOverfit
		res.Reasons = append(res.Reasons, fmt.Sprintf(
			"verdict: validation error improved only %.1f%% (required >= %.0f%%; before %.4f, after %.4f) — calibration REJECTED as overfit/unreliable",
			res.ValImprovement*100, cfg.MinImprovement*100, res.ValBefore, res.ValAfter))
	default:
		res.Status = StatusOK
		res.Reasons = append(res.Reasons, fmt.Sprintf(
			"verdict: validation error reduced %.1f%% (%.4f -> %.4f)", res.ValImprovement*100, res.ValBefore, res.ValAfter))
	}

	// Trajectories for the report.
	start := Pose{X: frames[0].X, Y: frames[0].Y, Theta: frames[0].Theta}
	res.RawTraj = d.Integrate(start, frames, nom, nil)
	skip := make([]bool, len(frames))
	for i := range frames {
		skip[i] = !inlier[i]
	}
	res.CalTraj = d.Integrate(start, frames, res.Params, skip)
	return res
}

// splitMaskAlternating splits the true entries of mask into two halves by
// index order (first half / second half of the log).
func splitMaskAlternating(mask []bool) ([]bool, []bool) {
	var idx []int
	for i, v := range mask {
		if v {
			idx = append(idx, i)
		}
	}
	a := make([]bool, len(mask))
	b := make([]bool, len(mask))
	for k, i := range idx {
		if k < len(idx)/2 {
			a[i] = true
		} else {
			b[i] = true
		}
	}
	return a, b
}
