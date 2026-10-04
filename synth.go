package main

import (
	"math"
	"math/rand"
)

// SynthConfig 描述一段合成标定数据。
type SynthConfig struct {
	Seed          int64
	NumFrames     int
	TrueScaleL    float64 // 真实左轮每 tick 位移
	TrueScaleR    float64 // 真实右轮每 tick 位移
	TrueBase      float64 // 真实轮距
	Noise         float64 // 编码器量化/噪声标准差（tick）
	Outliers      []int   // 指定为异常帧的下标（打滑/碰撞）
	TurnEvery     int     // 每隔多少帧插入一段高曲率转弯（0 表示不插入）
	Straight      bool    // true 时全程直行（用于激励不足/过拟合测试）
	StraightUntil int     // 该下标之前强制直行（模拟训练段缺少转弯激励）
}

// Synth 生成一条含直线、弧线、（可选）高曲率转弯的合成轨迹，
// 返回帧序列与真实参数对应的 Calib。运动指令先以 (v, ω) 给出，
// 真值位姿用真实参数积分；编码器读数 = 真实轮位移/真实尺度 + 噪声。
// 异常帧处把编码器读数随机打乱（模拟打滑空转或碰撞堵转），
// 使其与真值运动脱节。
func Synth(cfg SynthConfig) ([]Frame, Calib) {
	rng := rand.New(rand.NewSource(cfg.Seed))
	trueCal := Calib{ScaleL: cfg.TrueScaleL, ScaleR: cfg.TrueScaleR, WheelBase: cfg.TrueBase}

	outSet := map[int]bool{}
	for _, k := range cfg.Outliers {
		outSet[k] = true
	}

	frames := make([]Frame, cfg.NumFrames)
	p := Pose{}
	frames[0] = Frame{X: p.X, Y: p.Y, Theta: p.Theta}
	// 指令序列：基础为缓变圆弧，周期性插入高曲率转弯。
	for i := 1; i < cfg.NumFrames; i++ {
		var v, w float64
		if cfg.Straight || i < cfg.StraightUntil {
			v, w = 0.05, 0
		} else {
			v = 0.05 + 0.02*math.Sin(float64(i)*0.05)
			w = 0.15 * math.Sin(float64(i)*0.03)
			if cfg.TurnEvery > 0 && i%cfg.TurnEvery < 8 {
				// 高曲率转弯段（正常数据，不应被剔除）
				w = 1.2
				if (i/cfg.TurnEvery)%2 == 0 {
					w = -1.2
				}
			}
		}
		// 真实轮位移
		dsR := v + w*cfg.TrueBase/2
		dsL := v - w*cfg.TrueBase/2
		tR := dsR / cfg.TrueScaleR
		tL := dsL / cfg.TrueScaleL
		// 真值位姿推进（用真实参数，无噪声）
		p = step(p, trueCal, tL, tR)
		// 编码器读数 = 真实 tick + 噪声
		nL := tL + rng.NormFloat64()*cfg.Noise
		nR := tR + rng.NormFloat64()*cfg.Noise
		if outSet[i] {
			// 打滑/碰撞：读数与真实运动脱节
			if rng.Intn(2) == 0 {
				// 打滑空转：读数暴涨
				nL *= 3 + rng.Float64()*4
				nR *= 3 + rng.Float64()*4
			} else {
				// 碰撞堵转：读数接近零而真值在动
				nL *= 0.05
				nR *= 0.05
			}
		}
		frames[i] = Frame{TicksL: nL, TicksR: nR, X: p.X, Y: p.Y, Theta: p.Theta}
	}
	return frames, trueCal
}
