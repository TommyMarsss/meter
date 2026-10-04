// Command meter-calib runs differential-drive odometry calibration and
// emits a single self-contained HTML report.
//
//	meter-calib -input log.json -out report.html   # calibrate from a log
//	meter-calib -demo -out report.html             # synthetic demo log
//
// Input JSON format:
//
//	{
//	  "ticks_per_rev": 4096, "wheel_radius": 0.05, "wheel_base": 0.32,
//	  "frames": [{"tl": 12, "tr": 13, "x": 0.0, "y": 0.0, "theta": 0.0}, ...]
//	}
//
// tl/tr are encoder tick increments since the previous frame; x/y/theta is
// the ground-truth pose (e.g. from motion capture) at that frame.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"

	"github.com/TommyMarsss/meter/calib"
	"github.com/TommyMarsss/meter/report"
)

func main() {
	input := flag.String("input", "", "calibration log JSON (omit with -demo)")
	out := flag.String("out", "calibration_report.html", "output HTML report path")
	demo := flag.Bool("demo", false, "generate a synthetic demo log and calibrate it")
	seed := flag.Int64("seed", 42, "random seed for -demo")
	dump := flag.String("dump", "", "optional: also write the (demo) log JSON here")
	flag.Parse()

	var d calib.Dataset
	switch {
	case *demo || *input == "":
		cfg := calib.DefaultSimConfig()
		cfg.Seed = *seed
		var slips []int
		d, slips = calib.Simulate(cfg)
		fmt.Printf("demo log: %d frames, true factors KL=%.3f KR=%.3f KB=%.3f, %d injected slip frames\n",
			len(d.Frames), cfg.KL, cfg.KR, cfg.KB, len(slips))
	default:
		f, err := os.Open(*input)
		if err != nil {
			fatal(err)
		}
		defer f.Close()
		if err := json.NewDecoder(f).Decode(&d); err != nil {
			fatal(fmt.Errorf("parse %s: %w", *input, err))
		}
		if len(d.Frames) < 2 {
			fatal(fmt.Errorf("%s: need at least 2 frames", *input))
		}
	}

	if *dump != "" {
		f, err := os.Create(*dump)
		if err != nil {
			fatal(err)
		}
		enc := json.NewEncoder(f)
		enc.SetIndent("", "  ")
		if err := enc.Encode(d); err != nil {
			fatal(err)
		}
		f.Close()
		fmt.Printf("log written to %s\n", *dump)
	}

	res := calib.Calibrate(d, calib.DefaultConfig())

	fmt.Println("--- calibration result ---")
	fmt.Printf("status: %s\n", res.Status)
	fmt.Printf("KL=%.4f KR=%.4f KB=%.4f (1.0 = nominal)\n", res.Params.KL, res.Params.KR, res.Params.KB)
	fmt.Printf("outliers rejected: %d frames\n", len(res.OutlierIdx))
	fmt.Printf("train error: %.4f -> %.4f m\n", res.TrainBefore, res.TrainAfter)
	fmt.Printf("validation error: %.4f -> %.4f m (%.1f%% improvement)\n",
		res.ValBefore, res.ValAfter, res.ValImprovement*100)
	for _, r := range res.Reasons {
		fmt.Println("  -", r)
	}

	f, err := os.Create(*out)
	if err != nil {
		fatal(err)
	}
	defer f.Close()
	if err := report.Write(f, d, res); err != nil {
		fatal(err)
	}
	fmt.Printf("report written to %s\n", *out)

	if res.Status != calib.StatusOK {
		os.Exit(2) // non-zero: calibration not trustworthy (useful in CI)
	}
}

func fatal(err error) {
	fmt.Fprintln(os.Stderr, "error:", err)
	os.Exit(1)
}
