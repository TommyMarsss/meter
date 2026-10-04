package main

// htmlTemplate 是单文件报告的模板，/*__DATA__*/ 处注入 JSON 数据。
// 前端只用原生 HTML/CSS/JavaScript + Canvas，无任何第三方库。
const htmlTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>差速机器人里程计标定报告</title>
<style>
  body { font-family: -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif;
         margin: 0; background: #f5f6f8; color: #222; }
  header { background: #1f2937; color: #fff; padding: 16px 24px; }
  header h1 { margin: 0; font-size: 20px; }
  main { max-width: 1100px; margin: 0 auto; padding: 20px; }
  .card { background: #fff; border-radius: 8px; padding: 16px 20px;
          margin-bottom: 20px; box-shadow: 0 1px 3px rgba(0,0,0,.08); }
  .badge { display: inline-block; padding: 4px 14px; border-radius: 12px;
           font-weight: 600; font-size: 14px; }
  .ok { background: #d1fae5; color: #065f46; }
  .fail { background: #fee2e2; color: #991b1b; }
  table { border-collapse: collapse; width: 100%; font-size: 14px; }
  th, td { padding: 6px 10px; border-bottom: 1px solid #eee; text-align: right; }
  th:first-child, td:first-child { text-align: left; }
  th { color: #666; font-weight: 600; }
  canvas { width: 100%; border: 1px solid #e5e7eb; border-radius: 6px; background: #fff; }
  .legend span { display: inline-block; margin-right: 18px; font-size: 13px; }
  .sw { display: inline-block; width: 12px; height: 12px; border-radius: 2px;
        margin-right: 5px; vertical-align: -1px; }
  ul.reasons { color: #991b1b; margin: 8px 0 0; padding-left: 20px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  h2 { font-size: 16px; margin: 4px 0 12px; }
  .mono { font-family: ui-monospace, Menlo, monospace; }
  #outliers { font-size: 13px; color: #555; max-height: 120px; overflow-y: auto; }
</style>
</head>
<body>
<header><h1>差速驱动机器人里程计标定报告</h1></header>
<main>
  <div class="card" id="verdict"></div>
  <div class="grid">
    <div class="card"><h2>标定参数</h2><div id="params"></div></div>
    <div class="card"><h2>误差指标（全序列）</h2><div id="metrics"></div></div>
  </div>
  <div class="card">
    <h2>轨迹对比（X-Y）</h2>
    <div class="legend">
      <span><span class="sw" style="background:#059669"></span>真值</span>
      <span><span class="sw" style="background:#dc2626"></span>校正前</span>
      <span><span class="sw" style="background:#2563eb"></span>校正后</span>
      <span><span class="sw" style="background:#f59e0b"></span>剔除的异常帧</span>
      <span><span class="sw" style="background:#9ca3af"></span>训练/验证分界</span>
    </div>
    <canvas id="traj" width="1060" height="640"></canvas>
  </div>
  <div class="card">
    <h2>逐帧位置误差</h2>
    <canvas id="err" width="1060" height="300"></canvas>
  </div>
  <div class="card"><h2>被剔除的异常帧</h2><div id="outliers"></div></div>
</main>
<script>
const D = /*__DATA__*/;

function fmt(x, n) { return Number(x).toFixed(n === undefined ? 4 : n); }

// ---- 结论 ----
(function () {
  const v = D.verdict;
  let h = '<span class="badge ' + (v.ok ? 'ok' : 'fail') + '">' +
          (v.ok ? '标定有效' : '标定不可靠') + '</span>';
  h += '<p style="margin:10px 0 0;font-size:14px">训练帧数 ' + v.TrainN +
       '，验证帧数 ' + v.ValN + '，训练段剔除异常帧 ' + v.Outliers + ' 个。</p>';
  if (!v.ok && v.reasons.length) {
    h += '<ul class="reasons">' + v.reasons.map(r => '<li>' + r + '</li>').join('') + '</ul>';
  }
  document.getElementById('verdict').innerHTML = h;
})();

// ---- 参数表 ----
(function () {
  const n = D.nominal, r = D.raw, c = D.calib;
  const rows = [
    ['左轮每 tick 位移 (m)', r.ScaleL, c.ScaleL],
    ['右轮每 tick 位移 (m)', r.ScaleR, c.ScaleR],
    ['轮半径比 rR/rL', r.ScaleR / r.ScaleL, c.ScaleR / c.ScaleL],
    ['有效轮距 (m)', r.WheelBase, c.WheelBase],
  ];
  let h = '<table><tr><th>参数</th><th>标称值</th><th>标定值</th><th>偏差</th></tr>';
  for (const [name, a, b] of rows) {
    const dev = (b - a) / a * 100;
    h += '<tr><td>' + name + '</td><td class="mono">' + fmt(a, 6) +
         '</td><td class="mono">' + fmt(b, 6) + '</td><td class="mono">' +
         (dev >= 0 ? '+' : '') + fmt(dev, 2) + '%</td></tr>';
  }
  h += '</table><p style="font-size:13px;color:#666">标称轮半径 ' + n.WheelRadius +
       ' m，每转 ' + n.TicksPerRev + ' 线，标称轮距 ' + n.WheelBase + ' m。</p>';
  document.getElementById('params').innerHTML = h;
})();

// ---- 指标表 ----
(function () {
  function row(name, a, b) {
    const pct = a > 0 ? fmt((a - b) / a * 100, 1) + '%' : '—';
    return '<tr><td>' + name + '</td><td class="mono">' + fmt(a) +
           ' m</td><td class="mono">' + fmt(b) + ' m</td><td class="mono">' + pct + '</td></tr>';
  }
  let h = '<table><tr><th>指标</th><th>校正前</th><th>校正后</th><th>改善</th></tr>';
  h += row('终点位置误差', D.fullRaw.Final, D.fullCal.Final);
  h += row('位置误差 RMSE', D.fullRaw.RMSE, D.fullCal.RMSE);
  h += row('最大位置误差', D.fullRaw.MaxErr, D.fullCal.MaxErr);
  h += '</table>';
  h += '<h2 style="margin-top:14px">验证段（泛化）</h2><table>' +
       '<tr><th>指标</th><th>校正前</th><th>校正后</th><th>改善</th></tr>' +
       row('终点位置误差', D.verdict.ValRaw.Final, D.verdict.ValCal.Final) +
       row('位置误差 RMSE', D.verdict.ValRaw.RMSE, D.verdict.ValCal.RMSE) +
       '</table>';
  document.getElementById('metrics').innerHTML = h;
})();

// ---- 轨迹图 ----
(function () {
  const cv = document.getElementById('traj'), ctx = cv.getContext('2d');
  const all = D.gt.concat(D.rawTraj, D.calTraj);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const p of all) {
    minX = Math.min(minX, p.X); maxX = Math.max(maxX, p.X);
    minY = Math.min(minY, p.Y); maxY = Math.max(maxY, p.Y);
  }
  const pad = 30, W = cv.width, H = cv.height;
  const span = Math.max(maxX - minX, maxY - minY, 1e-6);
  const sc = Math.min((W - 2 * pad) / span, (H - 2 * pad) / span);
  const ox = pad + ((W - 2 * pad) - (maxX - minX) * sc) / 2;
  const oy = pad + ((H - 2 * pad) - (maxY - minY) * sc) / 2;
  const X = p => ox + (p.X - minX) * sc;
  const Y = p => H - oy - (p.Y - minY) * sc;

  function path(pts, color, width) {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    pts.forEach((p, i) => i ? ctx.lineTo(X(p), Y(p)) : ctx.moveTo(X(p), Y(p)));
    ctx.stroke();
  }
  path(D.rawTraj, '#dc2626', 1.5);
  path(D.calTraj, '#2563eb', 1.5);
  path(D.gt, '#059669', 2);
  // 训练/验证分界
  const cut = D.gt[D.trainCut];
  if (cut) {
    ctx.strokeStyle = '#9ca3af'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.arc(X(cut), Y(cut), 10, 0, 2 * Math.PI); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#6b7280'; ctx.font = '12px sans-serif';
    ctx.fillText('训练|验证', X(cut) + 12, Y(cut) - 12);
  }
  // 异常帧高亮
  ctx.fillStyle = '#f59e0b';
  for (const i of D.outliers) {
    const p = D.gt[i];
    ctx.beginPath(); ctx.arc(X(p), Y(p), 5, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = '#b45309'; ctx.lineWidth = 1.5; ctx.stroke();
  }
})();

// ---- 误差曲线 ----
(function () {
  const cv = document.getElementById('err'), ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height, pad = 36;
  const n = D.errRaw.length;
  let maxE = 1e-9;
  for (let i = 0; i < n; i++) maxE = Math.max(maxE, D.errRaw[i], D.errCal[i]);
  const X = i => pad + (W - 2 * pad) * i / (n - 1);
  const Y = e => H - pad - (H - 2 * pad) * e / maxE;
  ctx.strokeStyle = '#d1d5db'; ctx.fillStyle = '#6b7280'; ctx.font = '11px sans-serif';
  for (let g = 0; g <= 4; g++) {
    const e = maxE * g / 4, y = Y(e);
    ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(W - pad, y); ctx.stroke();
    ctx.fillText(e.toFixed(3) + ' m', 2, y + 3);
  }
  // 训练/验证分界线
  const cx = X(D.trainCut);
  ctx.strokeStyle = '#9ca3af'; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(cx, pad); ctx.lineTo(cx, H - pad); ctx.stroke();
  ctx.setLineDash([]);
  function curve(arr, color) {
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath();
    arr.forEach((e, i) => i ? ctx.lineTo(X(i), Y(e)) : ctx.moveTo(X(i), Y(e)));
    ctx.stroke();
  }
  curve(D.errRaw, '#dc2626');
  curve(D.errCal, '#2563eb');
  ctx.fillStyle = '#dc2626'; ctx.fillText('校正前', W - 130, pad + 4);
  ctx.fillStyle = '#2563eb'; ctx.fillText('校正后', W - 70, pad + 4);
})();

// ---- 异常帧列表 ----
document.getElementById('outliers').textContent =
  D.outliers.length ? '帧下标: ' + D.outliers.join(', ') : '未检测到异常帧。';
</script>
</body>
</html>
`
