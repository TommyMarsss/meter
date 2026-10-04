package main

import (
	"encoding/json"
	"io"
	"math"
	"strings"
)

// ReportData 是写入 HTML 的全部数据。
type ReportData struct {
	Nominal  Nominal         `json:"nominal"`
	Raw      Calib           `json:"raw"`
	Calib    Calib           `json:"calib"`
	Verdict  Verdict         `json:"verdict"`
	GT       []Pose          `json:"gt"`
	RawTraj  []Pose          `json:"rawTraj"`
	CalTraj  []Pose          `json:"calTraj"`
	Outliers []int           `json:"outliers"` // 全序列下标
	TrainCut int             `json:"trainCut"` // 训练/验证分界帧
	ErrRaw   []float64       `json:"errRaw"`   // 逐帧位置误差
	ErrCal   []float64       `json:"errCal"`
	FullRaw  TrajectoryError `json:"fullRaw"`
	FullCal  TrajectoryError `json:"fullCal"`
}

// BuildReport 汇总全序列对比数据。
func BuildReport(frames []Frame, nom Nominal, res CalibResult, v Verdict, trainCut int, valSkipFull []bool) ReportData {
	nomCal := NominalCalib(nom)
	skip := make([]bool, len(frames))
	for _, k := range res.Outliers {
		skip[k] = true
	}
	for i, s := range valSkipFull {
		if s {
			skip[i] = true
		}
	}
	rawTraj := Integrate(frames, gtPose(frames[0]), nomCal, skip)
	// 标定失败时 res.Calib 为零值，无法积分；退化为标称参数，
	// 保证报告仍可生成（结论栏会显示失败原因）。
	cal := res.Calib
	if cal.WheelBase <= 0 || cal.ScaleL <= 0 || cal.ScaleR <= 0 {
		cal = nomCal
	}
	calTraj := Integrate(frames, gtPose(frames[0]), cal, skip)

	d := ReportData{
		Nominal:  nom,
		Raw:      nomCal,
		Calib:    cal,
		Verdict:  v,
		Outliers: []int{},
		TrainCut: trainCut,
		FullRaw:  EvalTrajectory(frames, rawTraj),
		FullCal:  EvalTrajectory(frames, calTraj),
	}
	for _, f := range frames {
		d.GT = append(d.GT, gtPose(f))
	}
	d.RawTraj = rawTraj
	d.CalTraj = calTraj
	outSet := map[int]bool{}
	for _, k := range res.Outliers {
		outSet[k] = true
	}
	for i, s := range valSkipFull {
		if s {
			outSet[i] = true
		}
	}
	for i := range frames {
		if outSet[i] {
			d.Outliers = append(d.Outliers, i)
		}
		d.ErrRaw = append(d.ErrRaw, dist(rawTraj[i], d.GT[i]))
		d.ErrCal = append(d.ErrCal, dist(calTraj[i], d.GT[i]))
	}
	return d
}

func dist(a, b Pose) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}

// WriteHTML 把报告写成单一静态 HTML 文件。
func WriteHTML(w io.Writer, d ReportData) error {
	j, err := json.Marshal(d)
	if err != nil {
		return err
	}
	page := strings.Replace(htmlTemplate, "/*__DATA__*/", string(j), 1)
	_, err = io.WriteString(w, page)
	return err
}
