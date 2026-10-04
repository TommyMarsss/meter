package main

import (
	"math"
	"testing"
)

var testNom = Nominal{TicksPerRev: 1024, WheelRadius: 0.05, WheelBase: 0.30}

// biasedTruth 返回相对标称参数有已知偏差的"真实"参数：
// 右轮半径 +2%，轮距 −4%。
func biasedTruth() Calib {
	nom := NominalCalib(testNom)
	return Calib{
		ScaleL:    nom.ScaleL,
		ScaleR:    nom.ScaleR * 1.02,
		WheelBase: testNom.WheelBase * 0.96,
	}
}

func closeRel(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if want == 0 {
		t.Fatalf("%s: 期望值为 0，无法做相对误差判断", name)
	}
	if rel := math.Abs(got-want) / math.Abs(want); rel > tol {
		t.Errorf("%s = %.6f, 期望 %.6f (相对误差 %.2f%% > %.2f%%)",
			name, got, want, rel*100, tol*100)
	}
}

// TestLin2 验证二元最小二乘求解器。
func TestLin2(t *testing.T) {
	// y = 3*x1 - 2*x2
	var x1, x2, y []float64
	for i := 0; i < 100; i++ {
		a := float64(i) * 0.1
		b := math.Sin(float64(i))
		x1 = append(x1, a)
		x2 = append(x2, b)
		y = append(y, 3*a-2*b)
	}
	a, b, _, ok := lin2(x1, x2, y)
	if !ok {
		t.Fatal("lin2 报告病态，但数据激励充分")
	}
	closeRel(t, "a", a, 3, 1e-9)
	closeRel(t, "b", b, -2, 1e-9)

	// 共线数据应报告病态。
	x2c := make([]float64, len(x1))
	copy(x2c, x1)
	_, _, _, ok = lin2(x1, x2c, y)
	if ok {
		t.Error("共线数据应被判定为病态")
	}
}

// TestCalibrateConverges 已知轮径比/轮距偏差下，标定结果应收敛到真值附近。
func TestCalibrateConverges(t *testing.T) {
	truth := biasedTruth()
	frames, _ := Synth(SynthConfig{
		Seed: 7, NumFrames: 800,
		TrueScaleL: truth.ScaleL, TrueScaleR: truth.ScaleR, TrueBase: truth.WheelBase,
		Noise: 0.3, TurnEvery: 60,
	})
	res, err := Calibrate(frames)
	if err != nil {
		t.Fatalf("标定失败: %v", err)
	}
	if len(res.Outliers) != 0 {
		t.Errorf("无异常帧的数据不应有剔除，实际剔除 %v", res.Outliers)
	}
	closeRel(t, "轮半径比 rR/rL", res.Calib.RadiusRatio(), truth.RadiusRatio(), 0.005)
	closeRel(t, "有效轮距", res.Calib.WheelBase, truth.WheelBase, 0.01)
	closeRel(t, "左轮尺度", res.Calib.ScaleL, truth.ScaleL, 0.005)
	closeRel(t, "右轮尺度", res.Calib.ScaleR, truth.ScaleR, 0.005)
}

// TestOutlierRejection 异常帧应被剔除，且正常高曲率转弯帧不被误伤。
func TestOutlierRejection(t *testing.T) {
	truth := biasedTruth()
	outliers := []int{50, 123, 200, 317, 455, 590, 701, 760}
	isOutlier := map[int]bool{}
	for _, k := range outliers {
		isOutlier[k] = true
	}
	frames, _ := Synth(SynthConfig{
		Seed: 11, NumFrames: 800,
		TrueScaleL: truth.ScaleL, TrueScaleR: truth.ScaleR, TrueBase: truth.WheelBase,
		Noise: 0.3, TurnEvery: 60, Outliers: outliers,
	})
	res, err := Calibrate(frames)
	if err != nil {
		t.Fatalf("标定失败: %v", err)
	}
	got := map[int]bool{}
	for _, k := range res.Outliers {
		got[k] = true
	}
	// 全部真实异常帧必须被剔除。
	for _, k := range outliers {
		if !got[k] {
			t.Errorf("异常帧 %d 未被剔除", k)
		}
	}
	// 高曲率转弯帧（i%60<8）中的正常帧不得被误剔。
	for i := 1; i < len(frames); i++ {
		if i%60 < 8 && !isOutlier[i] && got[i] {
			t.Errorf("正常高曲率转弯帧 %d 被误剔除", i)
		}
	}
	// 误剔总数应很少（允许个别边界帧）。
	if len(res.Outliers) > len(outliers)+3 {
		t.Errorf("剔除帧数 %d 远超真实异常数 %d", len(res.Outliers), len(outliers))
	}
	// 剔除异常后参数仍应收敛。
	closeRel(t, "轮半径比 rR/rL", res.Calib.RadiusRatio(), truth.RadiusRatio(), 0.005)
	closeRel(t, "有效轮距", res.Calib.WheelBase, truth.WheelBase, 0.01)
}

// TestValidationPassesOnGoodData 正常数据上过拟合验证应通过，
// 且校正后误差显著下降。
func TestValidationPassesOnGoodData(t *testing.T) {
	truth := biasedTruth()
	frames, _ := Synth(SynthConfig{
		Seed: 3, NumFrames: 800,
		TrueScaleL: truth.ScaleL, TrueScaleR: truth.ScaleR, TrueBase: truth.WheelBase,
		Noise: 0.3, TurnEvery: 60,
		Outliers: []int{100, 300, 500, 700},
	})
	v, res, _, _ := Validate(frames, testNom, 0.6)
	if !v.OK {
		t.Errorf("良好数据上验证应通过，失败原因: %v", v.Reasons)
	}
	if v.ValCal.Final >= v.ValRaw.Final {
		t.Errorf("验证段终点误差未降低: %.4f → %.4f", v.ValRaw.Final, v.ValCal.Final)
	}
	closeRel(t, "轮半径比", res.Calib.RadiusRatio(), truth.RadiusRatio(), 0.01)
}

// TestOverfitDetected 训练段全程直行（无转弯激励）时，
// 轮距与左右轮尺度不可辨识，必须报告标定失败/数据不足，
// 而不是输出一组看似合理的参数。
func TestOverfitDetected(t *testing.T) {
	truth := biasedTruth()
	frames, _ := Synth(SynthConfig{
		Seed: 5, NumFrames: 800,
		TrueScaleL: truth.ScaleL, TrueScaleR: truth.ScaleR, TrueBase: truth.WheelBase,
		Noise: 0.3, TurnEvery: 60,
		StraightUntil: 500, // 训练段（前 60% = 480 帧）全为直行
	})
	v, _, _, _ := Validate(frames, testNom, 0.6)
	if v.OK {
		t.Error("训练段无旋转激励时验证应判定为不可靠")
	}
	if len(v.Reasons) == 0 {
		t.Error("应给出失败原因")
	}
	t.Logf("检测到的失败原因: %v", v.Reasons)
}

// TestPureStraightDetected 全程直行数据必须被拒绝。
func TestPureStraightDetected(t *testing.T) {
	truth := biasedTruth()
	frames, _ := Synth(SynthConfig{
		Seed: 9, NumFrames: 400,
		TrueScaleL: truth.ScaleL, TrueScaleR: truth.ScaleR, TrueBase: truth.WheelBase,
		Noise: 0.3, Straight: true,
	})
	_, err := Calibrate(frames)
	if err == nil {
		t.Error("纯直行数据应报告激励不足")
	}
}

// TestTooFewFrames 帧数不足应报错。
func TestTooFewFrames(t *testing.T) {
	if _, err := Calibrate(make([]Frame, 5)); err == nil {
		t.Error("帧数过少应报错")
	}
}

// TestWrapAngle 角度归一化。
func TestWrapAngle(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{0, 0}, {math.Pi, math.Pi}, {3 * math.Pi, math.Pi},
		{-3 * math.Pi / 2, math.Pi / 2}, {5 * math.Pi / 2, math.Pi / 2},
	}
	for _, c := range cases {
		if got := wrapAngle(c.in); math.Abs(got-c.want) > 1e-12 {
			t.Errorf("wrapAngle(%f) = %f, 期望 %f", c.in, got, c.want)
		}
	}
}

// TestGTIncrementsArcCorrection 验证弦长→弧长修正：
// 半圆轨迹上弧长应大于弦长。
func TestGTIncrementsArcCorrection(t *testing.T) {
	prev := Frame{X: 1, Y: 0, Theta: math.Pi / 2}
	cur := Frame{X: -1, Y: 0, Theta: -math.Pi / 2}
	arc, dth := gtIncrements(prev, cur)
	if math.Abs(dth+math.Pi) > 1e-9 && math.Abs(dth-math.Pi) > 1e-9 {
		t.Errorf("半圆转向差应为 ±π，实际 %f", dth)
	}
	// 弦长 2，弧长 π。
	if math.Abs(arc-math.Pi) > 1e-9 {
		t.Errorf("弧长修正错误: 得到 %f，期望 π", arc)
	}
}
