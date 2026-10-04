package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math/rand"
	"os"
)

// Input 是标定输入文件（JSON）的格式。
type Input struct {
	Nominal Nominal `json:"nominal"`
	Frames  []Frame `json:"frames"`
}

func main() {
	input := flag.String("input", "", "标定数据 JSON 文件（为空则使用内置合成演示数据）")
	out := flag.String("out", "calibration_report.html", "输出 HTML 报告路径")
	trainFrac := flag.Float64("train", 0.6, "训练集比例（其余为验证集）")
	seed := flag.Int64("seed", 42, "演示数据随机种子")
	flag.Parse()

	var in Input
	if *input == "" {
		in = demoInput(*seed)
		fmt.Println("未指定 -input，使用内置合成演示数据。")
	} else {
		data, err := os.ReadFile(*input)
		if err != nil {
			fatal("读取输入文件失败", err)
		}
		if err := json.Unmarshal(data, &in); err != nil {
			fatal("解析输入 JSON 失败", err)
		}
	}
	if len(in.Frames) < 20 {
		fatal("帧数不足", fmt.Errorf("需要至少 20 帧，实际 %d 帧", len(in.Frames)))
	}

	verdict, res, _, _ := Validate(in.Frames, in.Nominal, *trainFrac)

	fmt.Println("========== 标定结果 ==========")
	nom := NominalCalib(in.Nominal)
	if res.Calib.WheelBase > 0 && res.Calib.ScaleL > 0 {
		fmt.Printf("有效轮半径比 rR/rL : %.6f (标称 1.0)\n", res.Calib.RadiusRatio())
		fmt.Printf("左轮尺度 ScaleL    : %.8f m/tick\n", res.Calib.ScaleL)
		fmt.Printf("右轮尺度 ScaleR    : %.8f m/tick\n", res.Calib.ScaleR)
		fmt.Printf("有效轮距 WheelBase : %.4f m (标称 %.4f m, 偏差 %+.2f%%)\n",
			res.Calib.WheelBase, nom.WheelBase,
			(res.Calib.WheelBase/nom.WheelBase-1)*100)
	} else {
		fmt.Println("标定失败：未得到有效参数（详见下方验证结论）。")
	}
	fmt.Printf("剔除异常帧         : %d 个 %v\n", len(res.Outliers), res.Outliers)
	fmt.Println("========== 过拟合验证 ========")
	fmt.Printf("训练段: 终点误差 %.4f → %.4f m, RMSE %.4f → %.4f m\n",
		verdict.TrainRaw.Final, verdict.TrainCal.Final,
		verdict.TrainRaw.RMSE, verdict.TrainCal.RMSE)
	fmt.Printf("验证段: 终点误差 %.4f → %.4f m, RMSE %.4f → %.4f m\n",
		verdict.ValRaw.Final, verdict.ValCal.Final,
		verdict.ValRaw.RMSE, verdict.ValCal.RMSE)
	if verdict.OK {
		fmt.Println("结论: 标定有效，参数在验证集上稳定降低误差。")
	} else {
		fmt.Println("结论: 标定不可靠！")
		for _, r := range verdict.Reasons {
			fmt.Println("  - " + r)
		}
	}

	// 全序列报告数据：验证段的异常帧也标记出来。
	_, val, cut := splitContiguous(in.Frames, *trainFrac)
	valSkipFull := make([]bool, len(in.Frames))
	if res.Calib.WheelBase > 0 {
		vs := detectOnly(val, res.Calib)
		for i, s := range vs {
			if s {
				valSkipFull[cut+i] = true
			}
		}
	}
	rep := BuildReport(in.Frames, in.Nominal, res, verdict, cut, valSkipFull)

	f, err := os.Create(*out)
	if err != nil {
		fatal("创建输出文件失败", err)
	}
	defer f.Close()
	if err := WriteHTML(f, rep); err != nil {
		fatal("生成 HTML 失败", err)
	}
	fmt.Printf("报告已写入 %s\n", *out)
	if !verdict.OK {
		os.Exit(2)
	}
}

func fatal(msg string, err error) {
	fmt.Fprintf(os.Stderr, "%s: %v\n", msg, err)
	os.Exit(1)
}

// demoInput 生成带已知系统偏差与异常帧的演示数据。
func demoInput(seed int64) Input {
	nom := Nominal{TicksPerRev: 1024, WheelRadius: 0.05, WheelBase: 0.30}
	nomCal := NominalCalib(nom)
	// 真实参数：右轮半径大 2%，轮距小 4%。
	trueCal := Calib{
		ScaleL:    nomCal.ScaleL * 1.0,
		ScaleR:    nomCal.ScaleR * 1.02,
		WheelBase: nom.WheelBase * 0.96,
	}
	// 随机挑 8 个异常帧（打滑/碰撞）。
	rng := rand.New(rand.NewSource(seed + 1))
	outSet := map[int]bool{}
	for len(outSet) < 8 {
		k := 20 + rng.Intn(760)
		if k%60 >= 8 { // 避开高曲率转弯段，确保测试区分度
			outSet[k] = true
		}
	}
	outliers := []int{}
	for k := range outSet {
		outliers = append(outliers, k)
	}
	frames, _ := Synth(SynthConfig{
		Seed:       seed,
		NumFrames:  800,
		TrueScaleL: trueCal.ScaleL,
		TrueScaleR: trueCal.ScaleR,
		TrueBase:   trueCal.WheelBase,
		Noise:      0.3,
		Outliers:   outliers,
		TurnEvery:  60,
	})
	return Input{Nominal: nom, Frames: frames}
}
