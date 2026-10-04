package main

import (
	"errors"
	"math"
	"sort"
)

// 标定模型（对每帧 i）：
//   平移: arc_i  = (sR·nR_i + sL·nL_i) / 2
//   旋转: dθ_i   = (sR·nR_i − sL·nL_i) / B
// 其中 nL/nR 为编码器 tick 增量，sL/sR 为每 tick 有效位移，B 为有效轮距。
// 两个方程分别对 (sR/2, sL/2) 与 (sR/B, sL/B) 是线性的，
// 因此可化为两个独立的二元最小二乘问题，用正规方程闭式求解。

// lin2 解二元最小二乘 min Σ (y − a·x1 − b·x2)²，返回 a、b 与设计矩阵
// 列相关系数（用于激励不足/病态检测）。列近似共线时 ok=false。
func lin2(x1, x2, y []float64) (a, b, corr float64, ok bool) {
	var s11, s12, s22, s1y, s2y float64
	for i := range y {
		s11 += x1[i] * x1[i]
		s12 += x1[i] * x2[i]
		s22 += x2[i] * x2[i]
		s1y += x1[i] * y[i]
		s2y += x2[i] * y[i]
	}
	if s11 <= 0 || s22 <= 0 {
		return 0, 0, 1, false
	}
	// 列相关系数：|corr|→1 表示两列共线（如全程直行时 nR≈nL），参数不可辨识。
	corr = s12 / math.Sqrt(s11*s22)
	if math.Abs(corr) > 0.995 {
		return 0, 0, corr, false
	}
	det := s11*s22 - s12*s12
	a = (s1y*s22 - s2y*s12) / det
	b = (s2y*s11 - s1y*s12) / det
	return a, b, corr, true
}

// median 返回切片中位数（会复制，不改原切片）。
func median(v []float64) float64 {
	if len(v) == 0 {
		return 0
	}
	s := make([]float64, len(v))
	copy(s, v)
	sort.Float64s(s)
	m := len(s) / 2
	if len(s)%2 == 1 {
		return s[m]
	}
	return (s[m-1] + s[m]) / 2
}

// madSigma 用中位绝对偏差（MAD）估计鲁棒标准差。
// 对少数巨大异常值不敏感，避免异常帧撑大阈值而漏检。
func madSigma(res []float64) float64 {
	med := median(res)
	dev := make([]float64, len(res))
	for i, r := range res {
		dev[i] = math.Abs(r - med)
	}
	s := 1.4826 * median(dev)
	if s < 1e-12 {
		s = 1e-12 // 全零残差（完美数据）时避免除零
	}
	return s
}

// CalibResult 保存一次标定的完整结果。
type CalibResult struct {
	Calib      Calib
	Outliers   []int   // 被剔除帧的下标（相对传入的 frames）
	Iterations int     // 鲁棒迭代轮数
	TransCorr  float64 // 平移方程列相关系数（激励质量）
	RotCorr    float64 // 旋转方程列相关系数
	Used       int     // 参与最终拟合的帧数
}

const (
	maxRejectFrac  = 0.2  // 最多剔除 20% 帧，超过则视为数据整体异常
	rejectGate     = 3.5  // 归一化残差门限（≈3.5σ，MAD 鲁棒尺度下）
	modelTol       = 0.02 // 模型/参数相对容差，防止大运动量正常帧被误剔
	maxRobustIters = 10
)

var errTooManyOutliers = errors.New("异常帧比例超过上限，数据整体不可用")

// fitLS 在给定保留掩码上做一次最小二乘拟合。
// keep[i]=false 的帧不参与。返回标定参数与各方程列相关系数。
func fitLS(frames []Frame, keep []bool) (CalibResult, error) {
	var nR, nL, arc, dth []float64
	for i := 1; i < len(frames); i++ {
		if !keep[i] {
			continue
		}
		a, t := gtIncrements(frames[i-1], frames[i])
		nR = append(nR, frames[i].TicksR)
		nL = append(nL, frames[i].TicksL)
		arc = append(arc, a)
		dth = append(dth, t)
	}
	var res CalibResult
	res.Used = len(arc)
	if len(arc) < 8 {
		return res, errors.New("有效帧数不足（<8），无法拟合")
	}
	// 平移方程: arc = (sR/2)·nR + (sL/2)·nL
	aR, aL, tc, okT := lin2(nR, nL, arc)
	res.TransCorr = tc
	// 旋转方程: dθ = (sR/B)·nR + (−sL/B)·nL
	negNL := make([]float64, len(nL))
	for i, v := range nL {
		negNL[i] = -v
	}
	cR, cL, rc, okR := lin2(nR, negNL, dth)
	res.RotCorr = rc
	if !okT && !okR {
		return res, errors.New("平移与旋转方程均激励不足（轨迹近似纯直行或纯旋转），参数不可辨识")
	}
	if !okT {
		return res, errors.New("平移方程激励不足：左右轮 tick 序列近似共线，无法分离左右轮尺度")
	}
	sR := 2 * aR
	sL := 2 * aL
	if sL <= 0 || sR <= 0 {
		return res, errors.New("拟合得到非物理的轮尺度（≤0），数据异常")
	}
	if !okR {
		return res, errors.New("旋转方程激励不足：轨迹缺少转弯，轮距不可辨识")
	}
	// 有效轮距：B = sR/cR，亦应 = sL/cL，取两者均值并保留一致性信息。
	b1 := sR / cR
	b2 := sL / cL
	B := (b1 + b2) / 2
	if B <= 0 || math.Abs(b1-b2)/B > 0.5 {
		return res, errors.New("由左右轮分别估计的轮距不一致，模型与数据不符")
	}
	res.Calib = Calib{ScaleL: sL, ScaleR: sR, WheelBase: B}
	return res, nil
}

// residuals 计算每帧在当前参数下的归一化残差（取平移/旋转两通道最大值）。
// keep 掩码只用于估计鲁棒噪声尺度（MAD 与中位数），u 对全部帧计算，
// 使被剔除帧在拟合改善后有机会重新纳入（re-entry）。
// 门限由两部分组成：MAD 鲁棒噪声尺度（绝对项）+ 帧运动量 × modelTol（相对项）。
// 相对项吸收参数残差在大运动量帧（如高曲率转弯）上的放大效应，
// 使正常转弯帧不被误判，而打滑/碰撞帧的残差远超该门限。
func residuals(frames []Frame, c Calib, keep []bool) []float64 {
	n := len(frames)
	arcRes := make([]float64, n)
	rotRes := make([]float64, n)
	arcMag := make([]float64, n)
	rotMag := make([]float64, n)
	var kArcRes, kRotRes []float64
	for i := 1; i < n; i++ {
		arc, dth := gtIncrements(frames[i-1], frames[i])
		dsL := c.ScaleL * frames[i].TicksL
		dsR := c.ScaleR * frames[i].TicksR
		arcRes[i] = (dsL+dsR)/2 - arc
		rotRes[i] = (dsR-dsL)/c.WheelBase - dth
		arcMag[i] = math.Abs(arc)
		rotMag[i] = math.Abs(dth)
		if keep[i] {
			kArcRes = append(kArcRes, arcRes[i])
			kRotRes = append(kRotRes, rotRes[i])
		}
	}
	sd := madSigma(kArcRes)
	st := madSigma(kRotRes)
	medA, medT := median(kArcRes), median(kRotRes)
	u := make([]float64, n)
	for i := 1; i < n; i++ {
		ua := math.Abs(arcRes[i]-medA) / (sd + modelTol*arcMag[i])
		ut := math.Abs(rotRes[i]-medT) / (st + modelTol*rotMag[i])
		u[i] = math.Max(ua, ut)
	}
	return u
}

// Calibrate 对 frames 做鲁棒标定：迭代执行
//  1. 在当前保留集上做最小二乘拟合；
//  2. 用 MAD 鲁棒尺度计算归一化残差，剔除超过门限的帧；
//     直到剔除集合收敛或达到迭代上限。
//
// 异常帧（打滑/碰撞）的本质是编码器增量与真值增量脱节，残差远超噪声水平；
// 正常高曲率转弯帧仍满足运动学模型，残差与直线帧同量级，因此不会被误剔。
func Calibrate(frames []Frame) (CalibResult, error) {
	var res CalibResult
	if len(frames) < 10 {
		return res, errors.New("帧数过少（<10）")
	}
	keep := make([]bool, len(frames))
	for i := range keep {
		keep[i] = true
	}
	keep[0] = false // 第 0 帧无增量
	maxReject := int(math.Ceil(maxRejectFrac * float64(len(frames))))

	rejected := map[int]bool{}
	for iter := 0; iter < maxRobustIters; iter++ {
		r, err := fitLS(frames, keep)
		if err != nil {
			return res, err
		}
		res = r
		res.Iterations = iter + 1
		u := residuals(frames, res.Calib, keep)
		// 每轮基于全部帧重算剔除集（允许被误剔的帧在拟合改善后回入），
		// 直到剔除集收敛。拟合只在保留帧上进行。
		newRejected := map[int]bool{}
		for i := 1; i < len(frames); i++ {
			if u[i] > rejectGate {
				newRejected[i] = true
			}
		}
		if len(newRejected) > maxReject {
			return res, errTooManyOutliers
		}
		same := len(newRejected) == len(rejected)
		if same {
			for k := range newRejected {
				if !rejected[k] {
					same = false
					break
				}
			}
		}
		rejected = newRejected
		for i := range keep {
			keep[i] = !rejected[i]
		}
		keep[0] = false
		if same {
			break
		}
	}
	// 用最终剔除集合再拟合一次作为输出参数。
	final, err := fitLS(frames, keep)
	if err != nil {
		return res, err
	}
	final.Iterations = res.Iterations
	for k := range rejected {
		final.Outliers = append(final.Outliers, k)
	}
	sort.Ints(final.Outliers)
	return final, nil
}
