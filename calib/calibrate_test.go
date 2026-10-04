package calib

import (
	"math"
	"testing"
)

// TestCalibrationConverges: with known systematic errors and no slip, the
// fitted correction factors must recover the true values, and the run must
// validate as OK with a large error reduction.
func TestCalibrationConverges(t *testing.T) {
	cfg := DefaultSimConfig()
	cfg.SlipCount = 0
	d, _ := Simulate(cfg)

	res := Calibrate(d, DefaultConfig())

	for _, tc := range []struct {
		name string
		got  float64
		want float64
	}{
		{"KL", res.Params.KL, cfg.KL},
		{"KR", res.Params.KR, cfg.KR},
		{"KB", res.Params.KB, cfg.KB},
	} {
		if math.Abs(tc.got-tc.want) > 0.01 {
			t.Errorf("%s: got %.4f, want %.4f (±0.01)", tc.name, tc.got, tc.want)
		}
	}
	if res.Status != StatusOK {
		t.Errorf("status = %v, want OK; reasons: %v", res.Status, res.Reasons)
	}
	if res.ValAfter >= res.ValBefore {
		t.Errorf("validation error not reduced: before %.4f after %.4f", res.ValBefore, res.ValAfter)
	}
	if res.ValImprovement < 0.5 {
		t.Errorf("validation improvement %.1f%% too low for a real systematic error", res.ValImprovement*100)
	}
	if len(res.OutlierIdx) != 0 {
		t.Errorf("clean log: %d frames wrongly rejected: %v", len(res.OutlierIdx), res.OutlierIdx)
	}
}

// TestOutlierRejection: injected slip frames must be rejected, and the
// aggressive high-curvature/pivot frames in the path must survive.
func TestOutlierRejection(t *testing.T) {
	cfg := DefaultSimConfig()
	d, slipIdx := Simulate(cfg)
	slipSet := map[int]bool{}
	for _, i := range slipIdx {
		slipSet[i] = true
	}

	res := Calibrate(d, DefaultConfig())

	// Every rejected frame must be a true slip frame (no false positives —
	// in particular none of the hard-turn frames).
	for _, i := range res.OutlierIdx {
		if !slipSet[i] {
			t.Errorf("frame %d wrongly rejected (false positive)", i)
		}
	}
	// And it must catch the overwhelming majority of slips.
	caught := 0
	for _, i := range slipIdx {
		if !res.Inlier[i] {
			caught++
		}
	}
	if float64(caught) < 0.8*float64(len(slipIdx)) {
		t.Errorf("caught %d/%d slip frames, want >= 80%%", caught, len(slipIdx))
	}
	// Parameters must still converge despite the contamination.
	if math.Abs(res.Params.KL-cfg.KL) > 0.01 ||
		math.Abs(res.Params.KR-cfg.KR) > 0.01 ||
		math.Abs(res.Params.KB-cfg.KB) > 0.01 {
		t.Errorf("params polluted by outliers: got (%.4f, %.4f, %.4f), want (%.3f, %.3f, %.3f)",
			res.Params.KL, res.Params.KR, res.Params.KB, cfg.KL, cfg.KR, cfg.KB)
	}
	if res.Status != StatusOK {
		t.Errorf("status = %v, want OK; reasons: %v", res.Status, res.Reasons)
	}
}

// TestHighCurvatureFramesSurvive: a log consisting ONLY of hard turns and
// pivots, with zero slip, must produce zero rejections.
func TestHighCurvatureFramesSurvive(t *testing.T) {
	cfg := DefaultSimConfig()
	cfg.SlipCount = 0
	cfg.Primitives = 12
	d, _ := Simulate(cfg)
	// Force every primitive to be a turn by checking residuals directly:
	// the generator already alternates arcs/pivots; here we just assert the
	// detector finds nothing to reject.
	inlier, _, _ := detectOutliers(d, d.Frames, DefaultConfig())
	for i := 1; i < len(d.Frames); i++ {
		if !inlier[i] {
			t.Errorf("frame %d (normal turn data) wrongly rejected", i)
		}
	}
}

// TestOverfitDetected: when the nominal parameters are already exact (only
// random noise, no systematic error), any "correction" fits noise. The
// pipeline must refuse to bless it.
func TestOverfitDetected(t *testing.T) {
	cfg := DefaultSimConfig()
	cfg.KL, cfg.KR, cfg.KB = 1, 1, 1
	cfg.SlipCount = 0
	d, _ := Simulate(cfg)

	res := Calibrate(d, DefaultConfig())
	if res.Status == StatusOK && res.ValImprovement >= 0.10 {
		t.Errorf("noise-only log was reported as a successful calibration: %+v", res.Params)
	}
	if res.Status == StatusOverfit {
		t.Logf("overfit correctly detected: %v", res.Reasons[len(res.Reasons)-1])
	}
}

// TestInsufficientData: a tiny log must be flagged, not "calibrated".
func TestInsufficientData(t *testing.T) {
	cfg := DefaultSimConfig()
	cfg.SlipCount = 0
	d, _ := Simulate(cfg)
	d.Frames = d.Frames[:30] // keep only a fragment
	if len(d.Frames) >= DefaultConfig().MinFrames {
		t.Fatalf("test setup: got %d frames, need < %d", len(d.Frames), DefaultConfig().MinFrames)
	}
	res := Calibrate(d, DefaultConfig())
	if res.Status != StatusInsufficientData {
		t.Errorf("status = %v, want INSUFFICIENT DATA", res.Status)
	}
}

// TestStraightLineOnlyRotationUnidentifiable: with no turning motion at all,
// KB must stay at 1 and the fit must not blow up. Note: on pure
// straight-line data KL and KR are individually non-identifiable (nL and nR
// are collinear), so the fit falls back to a shared scale — we assert the
// *combined* translation scale is correct instead of the individual values.
func TestStraightLineOnly(t *testing.T) {
	cfg := DefaultSimConfig()
	cfg.SlipCount = 0
	d, _ := Simulate(cfg)
	// Rebuild a straight-only log.
	frames := []Frame{{}}
	x := 0.0
	c := d.TickScale()
	for i := 0; i < 200; i++ {
		ds := 0.015
		x += ds
		frames = append(frames, Frame{
			TicksL: ds / (cfg.KL * c),
			TicksR: ds / (cfg.KR * c),
			X:      x,
		})
	}
	d.Frames = frames
	res := Calibrate(d, DefaultConfig())
	if res.RotIdentifiable {
		t.Errorf("rotation reported identifiable on straight-line data")
	}
	if res.Params.KB != 1.0 {
		t.Errorf("KB = %.4f, want 1.0 (unidentifiable)", res.Params.KB)
	}
	// Shared-scale fallback: both factors equal, and equal to the harmonic
	// combination 2·KL·KR/(KL+KR) that straight-line motion can identify.
	if res.Params.KL != res.Params.KR {
		t.Errorf("expected shared scale fallback, got KL=%.4f KR=%.4f", res.Params.KL, res.Params.KR)
	}
	want := 2 * cfg.KL * cfg.KR / (cfg.KL + cfg.KR)
	if math.Abs(res.Params.KL-want) > 0.01 {
		t.Errorf("combined scale = %.4f, want %.4f", res.Params.KL, want)
	}
}

// TestValidationSplitSanity ensures the segment split produces both train
// and validation segments.
func TestValidationSplitSanity(t *testing.T) {
	segs := splitSegments(100, 5)
	if len(segs) != 5 {
		t.Fatalf("got %d segments, want 5", len(segs))
	}
	if segs[0].Start != 0 || segs[4].End != 100 {
		t.Errorf("segments do not cover the log: %+v", segs)
	}
	for i := 1; i < len(segs); i++ {
		if segs[i].Start != segs[i-1].End {
			t.Errorf("gap between segments %d and %d", i-1, i)
		}
	}
}
