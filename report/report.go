// Package report renders a calibration result as a single self-contained
// static HTML file: all data and logic are inlined, no server, no external
// libraries — plain HTML/CSS/JavaScript with Canvas 2D.
package report

import (
	"encoding/json"
	"fmt"
	"html/template"
	"io"

	"github.com/TommyMarsss/meter/calib"
)

// data is the JSON payload embedded in the page.
type data struct {
	Status     string     `json:"status"`
	StatusOK   bool       `json:"statusOK"`
	Reasons    []string   `json:"reasons"`
	Params     [3]float64 `json:"params"` // KL, KR, KB calibrated
	StabFirst  [3]float64 `json:"stabFirst"`
	StabSecond [3]float64 `json:"stabSecond"`

	Metrics struct {
		TrainBefore, TrainAfter float64
		ValBefore, ValAfter     float64
		ValImprovement          float64
		Outliers, Frames        int
		Distance, Rotation      float64
	} `json:"metrics"`

	Truth     []calib.Pose `json:"truth"`
	Raw       []calib.Pose `json:"raw"`
	Cal       []calib.Pose `json:"cal"`
	Outliers  []int        `json:"outliers"`
	Residuals []float64    `json:"residuals"`
}

// Write renders the report for dataset d and calibration result res.
func Write(w io.Writer, d calib.Dataset, res calib.Result) error {
	truth := make([]calib.Pose, len(d.Frames))
	for i, f := range d.Frames {
		truth[i] = calib.Pose{X: f.X, Y: f.Y, Theta: f.Theta}
	}
	var p data
	p.Status = res.Status.String()
	p.StatusOK = res.Status == calib.StatusOK
	p.Reasons = res.Reasons
	p.Params = [3]float64{res.Params.KL, res.Params.KR, res.Params.KB}
	p.StabFirst = [3]float64{res.StabFirst.KL, res.StabFirst.KR, res.StabFirst.KB}
	p.StabSecond = [3]float64{res.StabSecond.KL, res.StabSecond.KR, res.StabSecond.KB}
	p.Metrics.TrainBefore = res.TrainBefore
	p.Metrics.TrainAfter = res.TrainAfter
	p.Metrics.ValBefore = res.ValBefore
	p.Metrics.ValAfter = res.ValAfter
	p.Metrics.ValImprovement = res.ValImprovement
	p.Metrics.Outliers = len(res.OutlierIdx)
	p.Metrics.Frames = len(d.Frames)
	p.Metrics.Distance = res.Excitation.Distance
	p.Metrics.Rotation = res.Excitation.Rotation
	p.Truth = truth
	p.Raw = res.RawTraj
	p.Cal = res.CalTraj
	p.Outliers = res.OutlierIdx
	p.Residuals = res.Residuals

	j, err := json.Marshal(p)
	if err != nil {
		return fmt.Errorf("marshal report data: %w", err)
	}
	return page.Execute(w, struct{ JSON template.JS }{JSON: template.JS(j)})
}

var page = template.Must(template.New("report").Parse(pageHTML))

const pageHTML = `<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>里程计标定报告</title>
<style>
  :root { color-scheme: light; }
  body { font-family: -apple-system, "Helvetica Neue", "PingFang SC", sans-serif;
         margin: 0; background: #f4f5f7; color: #1c1e21; }
  header { padding: 20px 28px 8px; }
  h1 { font-size: 20px; margin: 0 0 6px; }
  .badge { display: inline-block; padding: 4px 12px; border-radius: 12px;
           font-weight: 600; font-size: 13px; color: #fff; }
  .ok { background: #1a9e54; } .bad { background: #d23b3b; }
  main { display: grid; grid-template-columns: 1fr 340px; gap: 16px; padding: 12px 28px 28px; }
  .card { background: #fff; border: 1px solid #e2e4e9; border-radius: 10px; padding: 14px 16px; }
  .card h2 { font-size: 14px; margin: 0 0 10px; color: #444; }
  canvas { width: 100%; display: block; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th, td { text-align: right; padding: 4px 8px; border-bottom: 1px solid #eef0f3; }
  th:first-child, td:first-child { text-align: left; }
  th { color: #777; font-weight: 500; }
  .legend label { display: inline-flex; align-items: center; gap: 5px; margin-right: 14px;
                  font-size: 13px; cursor: pointer; }
  .sw { width: 14px; height: 4px; border-radius: 2px; display: inline-block; }
  ul { margin: 6px 0 0; padding-left: 18px; font-size: 13px; }
  li { margin-bottom: 3px; }
  .warn { color: #b26a00; } .fail { color: #d23b3b; } .good { color: #1a9e54; }
  #traj { height: 520px; } #resid { height: 140px; }
  .col { display: flex; flex-direction: column; gap: 16px; }
</style>
</head>
<body>
<header>
  <h1>差速驱动里程计标定报告 <span id="badge" class="badge"></span></h1>
  <div style="font-size:13px;color:#666">校正模型：dL=K<sub>L</sub>·c·n<sub>L</sub>，dR=K<sub>R</sub>·c·n<sub>R</sub>，dθ=(dR−dL)/(K<sub>B</sub>·b)</div>
</header>
<main>
  <div class="col">
    <div class="card">
      <h2>轨迹对比（真值 / 校正前 / 校正后，红圈 = 被剔除的异常帧）</h2>
      <div class="legend">
        <label><input type="checkbox" id="ckTruth" checked><span class="sw" style="background:#222"></span>真值</label>
        <label><input type="checkbox" id="ckRaw" checked><span class="sw" style="background:#d23b3b"></span>校正前（标称参数）</label>
        <label><input type="checkbox" id="ckCal" checked><span class="sw" style="background:#1a6fd4"></span>校正后（标定参数）</label>
        <label><input type="checkbox" id="ckOut" checked><span class="sw" style="background:#ff9500"></span>异常帧</label>
      </div>
      <canvas id="traj"></canvas>
    </div>
    <div class="card">
      <h2>逐帧残差（模型预测 vs 真值增量，橙线 = 鲁棒阈值，红点 = 已剔除）</h2>
      <canvas id="resid"></canvas>
    </div>
  </div>
  <div class="col">
    <div class="card">
      <h2>标定参数（1.0 = 与标称一致）</h2>
      <table>
        <tr><th>参数</th><th>标定值</th><th>前半段</th><th>后半段</th></tr>
        <tr><td>K<sub>L</sub> 左轮半径比</td><td id="p0"></td><td id="s00"></td><td id="s10"></td></tr>
        <tr><td>K<sub>R</sub> 右轮半径比</td><td id="p1"></td><td id="s01"></td><td id="s11"></td></tr>
        <tr><td>K<sub>B</sub> 轮距比</td><td id="p2"></td><td id="s02"></td><td id="s12"></td></tr>
      </table>
    </div>
    <div class="card">
      <h2>误差指标（段末位姿误差 RMS，米）</h2>
      <table>
        <tr><th></th><th>校正前</th><th>校正后</th></tr>
        <tr><td>训练段</td><td id="tb"></td><td id="ta"></td></tr>
        <tr><td>验证段</td><td id="vb"></td><td id="va"></td></tr>
        <tr><td>验证集改善</td><td colspan="2" id="imp"></td></tr>
      </table>
    </div>
    <div class="card">
      <h2>数据概况</h2>
      <table>
        <tr><td>总帧数</td><td id="nf"></td></tr>
        <tr><td>剔除异常帧</td><td id="no"></td></tr>
        <tr><td>训练里程</td><td id="dist"></td></tr>
        <tr><td>训练总转角</td><td id="rot"></td></tr>
      </table>
    </div>
    <div class="card">
      <h2>判定依据</h2>
      <ul id="reasons"></ul>
    </div>
  </div>
</main>
<script>
const D = {{.JSON}};

// ---- fill tables ----
const badge = document.getElementById('badge');
badge.textContent = D.status;
badge.className = 'badge ' + (D.statusOK ? 'ok' : 'bad');
D.params.forEach((v,i)=>document.getElementById('p'+i).textContent=v.toFixed(4));
D.stabFirst.forEach((v,i)=>document.getElementById('s0'+i).textContent=v.toFixed(4));
D.stabSecond.forEach((v,i)=>document.getElementById('s1'+i).textContent=v.toFixed(4));
const M=D.metrics;
document.getElementById('tb').textContent=M.TrainBefore.toFixed(4);
document.getElementById('ta').textContent=M.TrainAfter.toFixed(4);
document.getElementById('vb').textContent=M.ValBefore.toFixed(4);
document.getElementById('va').textContent=M.ValAfter.toFixed(4);
const imp=document.getElementById('imp');
imp.textContent=(M.ValImprovement*100).toFixed(1)+'%';
imp.className=M.ValImprovement>=0.1?'good':'fail';
document.getElementById('nf').textContent=M.Frames;
document.getElementById('no').textContent=M.Outliers;
document.getElementById('dist').textContent=M.Distance.toFixed(2)+' m';
document.getElementById('rot').textContent=M.Rotation.toFixed(2)+' rad';
const ul=document.getElementById('reasons');
D.reasons.forEach(r=>{const li=document.createElement('li');li.textContent=r;
  if(/REJECT|insufficient|NOT reliable|instability/i.test(r))li.className='fail';
  else if(/reduced|no correction/i.test(r))li.className='good';
  ul.appendChild(li);});

// ---- trajectory canvas ----
const trajCv=document.getElementById('traj');
function fitCanvas(cv){
  const r=cv.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  cv.width=r.width*dpr; cv.height=r.height*dpr;
  return [cv.getContext('2d'), dpr];
}
function bounds(){
  let x0=1e18,y0=1e18,x1=-1e18,y1=-1e18;
  const acc=p=>{x0=Math.min(x0,p.X);x1=Math.max(x1,p.X);y0=Math.min(y0,p.Y);y1=Math.max(y1,p.Y);};
  D.truth.forEach(acc); D.raw.forEach(acc); D.cal.forEach(acc);
  const mx=(x1-x0)*0.06+1e-6, my=(y1-y0)*0.06+1e-6;
  return [x0-mx,x1+mx,y0-my,y1+my];
}
function drawTraj(){
  const [ctx,dpr]=fitCanvas(trajCv);
  const W=trajCv.width,H=trajCv.height;
  ctx.clearRect(0,0,W,H);
  const [x0,x1,y0,y1]=bounds();
  const s=Math.min(W/(x1-x0),H/(y1-y0));
  const ox=(W-s*(x1-x0))/2, oy=(H-s*(y1-y0))/2;
  const X=p=>ox+(p.X-x0)*s, Y=p=>H-(oy+(p.Y-y0)*s);
  const path=(pts,color,lw)=>{
    ctx.strokeStyle=color;ctx.lineWidth=lw*dpr;ctx.beginPath();
    pts.forEach((p,i)=>i?ctx.lineTo(X(p),Y(p)):ctx.moveTo(X(p),Y(p)));
    ctx.stroke();
  };
  if(document.getElementById('ckRaw').checked) path(D.raw,'#d23b3b',1.2);
  if(document.getElementById('ckCal').checked) path(D.cal,'#1a6fd4',1.6);
  if(document.getElementById('ckTruth').checked) path(D.truth,'#222',1.8);
  // start marker
  ctx.fillStyle='#1a9e54';ctx.beginPath();
  ctx.arc(X(D.truth[0]),Y(D.truth[0]),5*dpr,0,7);ctx.fill();
  // outliers highlighted at their ground-truth positions
  if(document.getElementById('ckOut').checked){
    ctx.strokeStyle='#ff9500';ctx.lineWidth=2*dpr;
    D.outliers.forEach(i=>{const p=D.truth[i];
      ctx.beginPath();ctx.arc(X(p),Y(p),6*dpr,0,7);ctx.stroke();
      ctx.beginPath();ctx.moveTo(X(p)-4*dpr,Y(p)-4*dpr);ctx.lineTo(X(p)+4*dpr,Y(p)+4*dpr);
      ctx.moveTo(X(p)+4*dpr,Y(p)-4*dpr);ctx.lineTo(X(p)-4*dpr,Y(p)+4*dpr);ctx.stroke();});
  }
}
// ---- residual canvas ----
function drawResid(){
  const cv=document.getElementById('resid');
  const [ctx,dpr]=fitCanvas(cv);
  const W=cv.width,H=cv.height;
  ctx.clearRect(0,0,W,H);
  const r=D.residuals, n=r.length;
  const outSet=new Set(D.outliers);
  // robust display scale: 99th percentile of inlier residuals
  const inl=r.filter((_,i)=>!outSet.has(i)).slice(1).sort((a,b)=>a-b);
  const p99=inl.length?inl[Math.min(inl.length-1,Math.floor(inl.length*0.99))]:1;
  const maxV=Math.max(p99*1.5,1e-6);
  const bw=Math.max(1,W/n);
  for(let i=1;i<n;i++){
    const h=Math.min(r[i],maxV)/maxV*(H-8*dpr);
    ctx.fillStyle=outSet.has(i)?'#ff9500':'#9db7d4';
    ctx.fillRect(i/n*W,H-h,bw*0.8,h);
  }
  // threshold line (median + 8·1.4826·MAD over inliers)
  const med=inl.length?inl[Math.floor(inl.length/2)]:0;
  const dev=inl.map(v=>Math.abs(v-med)).sort((a,b)=>a-b);
  const mad=dev.length?dev[Math.floor(dev.length/2)]:0;
  const thr=Math.max(med+8*1.4826*mad,0.02);
  const ty=H-Math.min(thr,maxV)/maxV*(H-8*dpr);
  ctx.strokeStyle='#ff9500';ctx.setLineDash([6,4]);ctx.beginPath();
  ctx.moveTo(0,ty);ctx.lineTo(W,ty);ctx.stroke();ctx.setLineDash([]);
}
function drawAll(){drawTraj();drawResid();}
['ckTruth','ckRaw','ckCal','ckOut'].forEach(id=>
  document.getElementById(id).addEventListener('change',drawAll));
window.addEventListener('resize',drawAll);
drawAll();
</script>
</body>
</html>`
