package main

import "math"

// Pose 是机器人平面位姿（x, y 单位米，theta 单位弧度）。
type Pose struct {
	X, Y, Theta float64
}

// Frame 是一帧标定数据：相对上一帧的编码器增量（tick 数）与当前真实位姿。
type Frame struct {
	TicksL float64 `json:"ticksL"`
	TicksR float64 `json:"ticksR"`
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	Theta  float64 `json:"theta"`
}

// Nominal 是机器人出厂/标称参数。
type Nominal struct {
	TicksPerRev float64 `json:"ticksPerRev"` // 每转编码器线数
	WheelRadius float64 `json:"wheelRadius"` // 标称轮半径（左右轮相同）
	WheelBase   float64 `json:"wheelBase"`   // 标称轮距
}

// Calib 是标定得到的有效运动学参数。
type Calib struct {
	ScaleL    float64 // 左轮每 tick 有效位移 (m/tick)
	ScaleR    float64 // 右轮每 tick 有效位移 (m/tick)
	WheelBase float64 // 有效轮距 (m)
}

// NominalCalib 由标称参数构造未校正的 Calib。
func NominalCalib(n Nominal) Calib {
	s := 2 * math.Pi * n.WheelRadius / n.TicksPerRev
	return Calib{ScaleL: s, ScaleR: s, WheelBase: n.WheelBase}
}

// RadiusRatio 返回有效轮半径比 rR/rL（等于每 tick 位移比）。
func (c Calib) RadiusRatio() float64 { return c.ScaleR / c.ScaleL }

// gtPose 取第 i 帧的真实位姿。
func gtPose(f Frame) Pose { return Pose{f.X, f.Y, f.Theta} }

// wrapAngle 把角度归一化到 (-pi, pi]。
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// gtIncrements 由相邻两帧真值计算真实运动增量：
// 平移采用弧长（对弦长做曲率修正），旋转为朝向差。
func gtIncrements(prev, cur Frame) (arc, dtheta float64) {
	dx := cur.X - prev.X
	dy := cur.Y - prev.Y
	chord := math.Hypot(dx, dy)
	dtheta = wrapAngle(cur.Theta - prev.Theta)
	// 弦长 -> 弧长修正：arc = chord * (dθ/2) / sin(dθ/2)
	half := dtheta / 2
	if math.Abs(half) < 1e-9 {
		arc = chord
	} else {
		arc = chord * half / math.Sin(half)
	}
	return arc, dtheta
}

// step 用给定参数把位姿 p 推进一帧（精确圆弧模型）。
// 等曲率运动下位移为弦：|chord| = d·sin(dθ/2)/(dθ/2)，方向为中点朝向。
// 与 gtIncrements 的弦长→弧长修正互为逆运算，保证模型自洽。
func step(p Pose, c Calib, ticksL, ticksR float64) Pose {
	dsL := c.ScaleL * ticksL
	dsR := c.ScaleR * ticksR
	d := (dsL + dsR) / 2
	dth := (dsR - dsL) / c.WheelBase
	mid := p.Theta + dth/2
	chord := d
	if half := dth / 2; math.Abs(half) > 1e-9 {
		chord = d * math.Sin(half) / half
	}
	return Pose{
		X:     p.X + chord*math.Cos(mid),
		Y:     p.Y + chord*math.Sin(mid),
		Theta: wrapAngle(p.Theta + dth),
	}
}

// Integrate 从 start 出发，用参数 c 对 frames 的编码器数据积分，返回逐帧位姿
// （长度与 frames 相同，poses[0] = start）。skip 非 nil 时跳过被剔除帧
// （该帧位姿保持上一帧不变）。
func Integrate(frames []Frame, start Pose, c Calib, skip []bool) []Pose {
	poses := make([]Pose, len(frames))
	p := start
	for i := range frames {
		if i > 0 && (skip == nil || !skip[i]) {
			p = step(p, c, frames[i].TicksL, frames[i].TicksR)
		}
		poses[i] = p
	}
	return poses
}

// TrajectoryError 计算里程计轨迹相对真值的误差统计。
type TrajectoryError struct {
	Final   float64 // 终点位置误差 (m)
	RMSE    float64 // 逐帧位置误差均方根 (m)
	MaxErr  float64 // 最大位置误差 (m)
	Heading float64 // 终点朝向误差 (rad)
}

// EvalTrajectory 对比积分轨迹与真值。
func EvalTrajectory(frames []Frame, poses []Pose) TrajectoryError {
	var e TrajectoryError
	n := len(frames)
	if n == 0 {
		return e
	}
	var sumSq float64
	for i := range frames {
		d := math.Hypot(poses[i].X-frames[i].X, poses[i].Y-frames[i].Y)
		sumSq += d * d
		if d > e.MaxErr {
			e.MaxErr = d
		}
	}
	e.RMSE = math.Sqrt(sumSq / float64(n))
	last := n - 1
	e.Final = math.Hypot(poses[last].X-frames[last].X, poses[last].Y-frames[last].Y)
	e.Heading = math.Abs(wrapAngle(poses[last].Theta - frames[last].Theta))
	return e
}
