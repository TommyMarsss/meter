package main

import (
	"fmt"
	"math"
)

// Verdict 是过拟合/可靠性验证的结论。
type Verdict struct {
	OK       bool     `json:"ok"`
	Reasons  []string `json:"reasons"`
	TrainRaw TrajectoryError
	TrainCal TrajectoryError
	ValRaw   TrajectoryError
	ValCal   TrajectoryError
	TrainN   int
	ValN     int
	Outliers int
}

// splitContiguous 把帧序列按下标切成训练段 [0,cut) 与验证段 [cut,n)。
// 采用连续切分而非随机抽样：验证段是一条完整的未知轨迹，
// 能真实检验参数对未见过运动的泛化能力。
func splitContiguous(frames []Frame, trainFrac float64) (train, val []Frame, cut int) {
	cut = int(math.Round(trainFrac * float64(len(frames))))
	if cut < 10 {
		cut = 10
	}
	if len(frames)-cut < 10 {
		cut = len(frames) - 10
	}
	return frames[:cut], frames[cut:], cut
}

// Validate 用 trainFrac 比例的前段数据标定，后段数据验证，检测过拟合。
//
// 判定逻辑（任一不满足即判定标定不可靠）：
//  1. 训练段拟合本身成功（激励充分、异常帧比例可接受）；
//  2. 验证段上校正后误差必须显著低于校正前（终点误差与 RMSE 均不升高）；
//  3. 训练段上校正有效（排除"本来就没误差可降"的假象）。
func Validate(frames []Frame, nom Nominal, trainFrac float64) (Verdict, CalibResult, []Frame, []Frame) {
	var v Verdict
	train, val, _ := splitContiguous(frames, trainFrac)
	v.TrainN, v.ValN = len(train), len(val)

	res, err := Calibrate(train)
	if err != nil {
		v.Reasons = append(v.Reasons, fmt.Sprintf("训练段标定失败: %v", err))
		return v, res, train, val
	}
	v.Outliers = len(res.Outliers)

	nomCal := NominalCalib(nom)

	// 训练段拟合优度（剔除异常帧后积分）。
	trainSkip := make([]bool, len(train))
	for _, k := range res.Outliers {
		trainSkip[k] = true
	}
	v.TrainRaw = EvalTrajectory(train, Integrate(train, gtPose(train[0]), nomCal, trainSkip))
	v.TrainCal = EvalTrajectory(train, Integrate(train, gtPose(train[0]), res.Calib, trainSkip))

	// 验证段：从该段真值起点出发，仅使用编码器数据积分。
	// 验证段的异常帧同样按训练得到的门限剔除，避免污染评估。
	valSkip := detectOnly(val, res.Calib)
	v.ValRaw = EvalTrajectory(val, Integrate(val, gtPose(val[0]), nomCal, valSkip))
	v.ValCal = EvalTrajectory(val, Integrate(val, gtPose(val[0]), res.Calib, valSkip))

	// 判据 1：训练段必须有明显改善（否则标定无意义）。
	if v.TrainCal.Final > 0.8*v.TrainRaw.Final && v.TrainCal.RMSE > 0.8*v.TrainRaw.RMSE {
		v.Reasons = append(v.Reasons, "训练段上校正未带来明显改善，系统误差可能不显著")
	}
	// 判据 2：验证段误差不得升高（允许 10% 容差吸收噪声涨落）。
	if v.ValCal.Final > 1.1*v.ValRaw.Final {
		v.Reasons = append(v.Reasons, fmt.Sprintf(
			"验证段终点误差升高: 校正前 %.4f m → 校正后 %.4f m（疑似过拟合或数据不足）",
			v.ValRaw.Final, v.ValCal.Final))
	}
	if v.ValCal.RMSE > 1.1*v.ValRaw.RMSE {
		v.Reasons = append(v.Reasons, fmt.Sprintf(
			"验证段RMSE升高: 校正前 %.4f m → 校正后 %.4f m（疑似过拟合或数据不足）",
			v.ValRaw.RMSE, v.ValCal.RMSE))
	}
	// 判据 3：验证段应有统计意义上的改善（至少降低 20%）。
	improved := v.ValCal.Final < 0.8*v.ValRaw.Final || v.ValCal.RMSE < 0.8*v.ValRaw.RMSE
	if !improved {
		v.Reasons = append(v.Reasons, "验证段上校正改善不显著（<20%），参数泛化能力不足")
	}
	v.OK = len(v.Reasons) == 0
	return v, res, train, val
}

// detectOnly 用已标定参数对另一段数据做异常帧检测（不重新拟合），
// 返回剔除掩码。用于验证段评估时剔除打滑/碰撞帧。
func detectOnly(frames []Frame, c Calib) []bool {
	skip := make([]bool, len(frames))
	keep := make([]bool, len(frames))
	for i := range keep {
		keep[i] = true
	}
	keep[0] = false
	u := residuals(frames, c, keep)
	for i := 1; i < len(frames); i++ {
		if u[i] > rejectGate {
			skip[i] = true
		}
	}
	return skip
}
