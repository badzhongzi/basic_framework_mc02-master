"""把 sim_result.npz 画成一个离线 HTML（内联 SVG），不依赖 matplotlib。

这个环境里 matplotlib 一 import 就原生崩溃（见 USAGE.md 第 9 节），所以
sim.py 的 MAKE_PLOT 常年关着，图只能这样看。本脚本只用 numpy + 标准库。

用法:
    D:/Minforge/envs/robotics/python.exe -u plot_npz.py
    # 然后用浏览器打开 sim_result.html

每个关节三幅图: 跟踪(q_ref vs q)、真实误差、力矩(tau/tau_ff/dtau)。
顶部的时间段按钮会同时作用于所有图和下面的表格。
"""
import json
from pathlib import Path

import numpy as np

# HTML 模板。__PAYLOAD__ 会被替换成数据的 JSON。
# 配色取自 dataviz 的参考调色板，前三个分类槽（蓝/橙/青），深浅两套都是
# 按各自表面重新取的步进值，不是把浅色模式直接反转。
TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>机械臂 MPC 仿真结果</title>
<style>
.viz-root{
  color-scheme: light;
  --surface-1:#fcfcfb; --page:#f9f9f7;
  --text-primary:#0b0b0b; --text-secondary:#52514e; --muted:#898781;
  --grid:#e1e0d9; --axis:#c3c2b7; --border:rgba(11,11,11,0.10);
  --series-1:#2a78d6; --series-2:#eb6834; --series-3:#1baf7a;
}
@media (prefers-color-scheme: dark){
  :root:where(:not([data-theme="light"])) .viz-root{
    color-scheme: dark;
    --surface-1:#1a1a19; --page:#0d0d0d;
    --text-primary:#f0efec; --text-secondary:#c3c2b7; --muted:#898781;
    --grid:#2c2c2a; --axis:#383835; --border:rgba(255,255,255,0.10);
    --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70;
  }
}
:root[data-theme="dark"] .viz-root{
  color-scheme: dark;
  --surface-1:#1a1a19; --page:#0d0d0d;
  --text-primary:#f0efec; --text-secondary:#c3c2b7; --muted:#898781;
  --grid:#2c2c2a; --axis:#383835; --border:rgba(255,255,255,0.10);
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70;
}
*{box-sizing:border-box}
body{margin:0;font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
.viz-root{background:var(--page);color:var(--text-primary);padding:20px 24px 40px}
h1{font-size:19px;font-weight:600;margin:0 0 4px}
.sub{color:var(--text-secondary);margin:0 0 18px;font-size:13px}
.bar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:18px}
.bar .sp{flex:1}
button{font:inherit;padding:6px 12px;border-radius:8px;cursor:pointer;
  border:1px solid var(--border);background:var(--surface-1);color:var(--text-primary)}
button[aria-pressed="true"]{border-color:var(--series-1);color:var(--series-1);font-weight:600}
.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:8px 10px 2px}
.card h3{margin:0 0 2px;font-size:12px;font-weight:600;color:var(--text-secondary)}
.lg{display:flex;gap:12px;flex-wrap:wrap;font-size:11px;color:var(--text-secondary);margin:0 0 2px}
.lg span{display:inline-flex;align-items:center;gap:4px}
.lg i{width:12px;height:2px;border-radius:1px;display:inline-block}
svg{display:block;width:100%;height:auto;overflow:visible}
.gl{stroke:var(--grid);stroke-width:1}
.ax{stroke:var(--axis);stroke-width:1}
.tk{fill:var(--muted);font-size:10px;font-variant-numeric:tabular-nums}
.ln{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
/* 约束边界（±DTAU_MAX、零线）比网格重一点：它是数据的一部分，不是网格 */
.lim{stroke:var(--text-secondary);stroke-width:1;opacity:.45}
.ev{stroke:var(--muted);stroke-width:1;opacity:.5}
.cx{stroke:var(--text-secondary);stroke-width:1;opacity:.6}
.dot{stroke:var(--surface-1);stroke-width:2}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:13px}
th,td{text-align:right;padding:5px 9px;border-bottom:1px solid var(--border)}
th:first-child,td:first-child{text-align:left}
th{color:var(--text-secondary);font-weight:600}
.tip{position:fixed;pointer-events:none;z-index:9;background:var(--surface-1);
  border:1px solid var(--border);border-radius:8px;padding:7px 9px;font-size:12px;
  box-shadow:0 4px 14px rgba(0,0,0,.14);display:none;min-width:150px}
.tip .tt{color:var(--text-secondary);font-size:11px;margin-bottom:4px}
.tip .rw{display:flex;align-items:center;gap:6px;justify-content:space-between}
.tip .rw i{width:10px;height:2px;border-radius:1px;flex:none}
.tip .rw b{font-variant-numeric:tabular-nums}
.tip .rw span{color:var(--text-secondary);font-size:11px}
details{margin-top:22px}
summary{cursor:pointer;color:var(--text-secondary);font-size:13px;margin-bottom:10px}
.note{color:var(--muted);font-size:12px;margin:8px 0 0}
</style>
</head>
<body><div class="viz-root">
<h1>机械臂 MPC 闭环仿真</h1>
<p class="sub" id="sub"></p>
<div class="bar" id="bar"></div>
<div id="charts"></div>
<details open>
  <summary>表格视图（当前时间段，全分辨率统计）</summary>
  <div id="tbl"></div>
</details>
<details>
  <summary>残差模型与参数</summary>
  <div id="meta"></div>
</details>
<p class="note" id="foot"></p>
</div>
<div class="tip" id="tip"></div>
<script>
const D = __PAYLOAD__;
</script>
<script>
__SCRIPT__
</script>
</body></html>
"""

SCRIPT = r"""
const SVGNS = "http://www.w3.org/2000/svg";
const COL = ["var(--series-1)", "var(--series-2)", "var(--series-3)"];
const W = 360, H = 112, M = {l: 44, r: 10, t: 8, b: 20};
let cur = 0;                      // 当前时间段预设

const el = (tag, attrs) => {
  const n = document.createElementNS(SVGNS, tag);
  for (const k in attrs) n.setAttribute(k, attrs[k]);
  return n;
};
const fmt = (v) => {
  const a = Math.abs(v);
  if (a === 0) return "0";
  if (a < 0.001) return v.toExponential(1);
  if (a < 1) return v.toFixed(4);
  if (a < 100) return v.toFixed(3);
  return v.toFixed(1);
};
// y 轴刻度取整到好看的数
function ticks(lo, hi, n) {
  if (!(hi > lo)) { hi = lo + 1; }
  const raw = (hi - lo) / n;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => s >= raw) || mag * 10;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) {
    out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
  }
  return out;
}
// 只保留落在当前时间窗里的点，两端各多留一个，免得曲线被截断
function slice(s, lo, hi) {
  const t = s.t, y = s.y, ot = [], oy = [];
  for (let i = 0; i < t.length; i++) {
    if (t[i] >= lo && t[i] <= hi) { ot.push(t[i]); oy.push(y[i]); }
  }
  return {name: s.name, t: ot, y: oy};
}

// 画一幅图。series 是 [{name,t,y}]，extra 是额外的水平参考线
function panel(parent, title, series, lo, hi, extra) {
  const card = document.createElement("div");
  card.className = "card";
  const h = document.createElement("h3");
  h.textContent = title;
  card.appendChild(h);
  if (series.length >= 2) {                 // 单条曲线不需要图例，标题已经说明
    const lg = document.createElement("div");
    lg.className = "lg";
    series.forEach((s, i) => {
      const sp = document.createElement("span");
      const ic = document.createElement("i");
      ic.style.background = COL[i % 3];
      sp.appendChild(ic);
      sp.appendChild(document.createTextNode(s.name));
      lg.appendChild(sp);
    });
    card.appendChild(lg);
  }
  const svg = el("svg", {viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none"});
  let ymin = Infinity, ymax = -Infinity;
  series.forEach(s => s.y.forEach(v => { if (v < ymin) ymin = v; if (v > ymax) ymax = v; }));
  (extra || []).forEach(v => { if (v < ymin) ymin = v; if (v > ymax) ymax = v; });
  if (!isFinite(ymin)) { ymin = 0; ymax = 1; }
  if (ymax - ymin < 1e-9) { ymax += 0.5; ymin -= 0.5; }
  const pad = (ymax - ymin) * 0.08;
  ymin -= pad; ymax += pad;
  const X = v => M.l + (v - lo) / (hi - lo || 1) * (W - M.l - M.r);
  const Y = v => H - M.b - (v - ymin) / (ymax - ymin) * (H - M.t - M.b);

  ticks(ymin, ymax, 3).forEach(v => {
    svg.appendChild(el("line", {class: "gl", x1: M.l, x2: W - M.r, y1: Y(v), y2: Y(v)}));
    const tx = el("text", {class: "tk", x: M.l - 5, y: Y(v) + 3, "text-anchor": "end"});
    tx.textContent = fmt(v);
    svg.appendChild(tx);
  });
  // 事件时刻的竖线：负载突变和碰撞冲击
  [[D.meta.payload_t, "负载"], [D.meta.impact_t, "冲击"]].forEach(([tv]) => {
    if (tv > lo && tv < hi) {
      svg.appendChild(el("line", {class: "ev", x1: X(tv), x2: X(tv), y1: M.t, y2: H - M.b}));
    }
  });
  (extra || []).forEach(v => {
    svg.appendChild(el("line", {class: "lim", x1: M.l, x2: W - M.r, y1: Y(v), y2: Y(v),
                                "stroke-dasharray": "0"}));
  });
  svg.appendChild(el("line", {class: "ax", x1: M.l, x2: W - M.r, y1: H - M.b, y2: H - M.b}));
  // 一条轴只用一种刻度样式：窗口短于 2 s 时统一带一位小数，否则统一整数
  const xd = (hi - lo) < 2 ? 1 : 0;
  ticks(lo, hi, 4).forEach(v => {
    if (v < lo || v > hi) return;
    const tx = el("text", {class: "tk", x: X(v), y: H - M.b + 13, "text-anchor": "middle"});
    tx.textContent = v.toFixed(xd);
    svg.appendChild(tx);
  });
  series.forEach((s, i) => {
    if (!s.t.length) return;
    let dd = "";
    for (let k = 0; k < s.t.length; k++) dd += (k ? "L" : "M") + X(s.t[k]).toFixed(1) + " " + Y(s.y[k]).toFixed(1);
    svg.appendChild(el("path", {class: "ln", d: dd, stroke: COL[i % 3]}));
  });
  card.appendChild(svg);
  parent.appendChild(card);
  hookHover(svg, series, lo, hi, X, Y, title);
  return card;
}

// 十字准线 + 一个 tooltip 列出该时刻所有曲线的值。
// 读者只要瞄时间，不用去压那条 2px 的线；键盘聚焦给同样的信息。
const tip = document.getElementById("tip");
function hookHover(svg, series, lo, hi, X, Y, title) {
  const cx = el("line", {class: "cx", y1: M.t, y2: H - M.b, x1: 0, x2: 0, opacity: 0});
  svg.appendChild(cx);
  const dots = series.map((s, i) => {
    const c = el("circle", {class: "dot", r: 4, fill: COL[i % 3], opacity: 0});
    svg.appendChild(c);
    return c;
  });
  svg.setAttribute("tabindex", "0");
  const hide = () => {
    tip.style.display = "none";
    cx.setAttribute("opacity", 0);
    dots.forEach(c => c.setAttribute("opacity", 0));
  };
  const show = (ev) => {
    const bb = svg.getBoundingClientRect();
    const px = (ev.clientX - bb.left) / bb.width * W;
    const tv = lo + (px - M.l) / (W - M.l - M.r) * (hi - lo);
    if (tv < lo || tv > hi) { hide(); return; }
    cx.setAttribute("x1", X(tv)); cx.setAttribute("x2", X(tv));
    cx.setAttribute("opacity", 1);
    tip.textContent = "";
    const hd = document.createElement("div");
    hd.className = "tt";
    hd.textContent = title + "   t = " + tv.toFixed(3) + " s";
    tip.appendChild(hd);
    series.forEach((s, i) => {
      if (!s.t.length) { dots[i].setAttribute("opacity", 0); return; }
      let k = 0, best = Infinity;
      for (let m = 0; m < s.t.length; m++) {
        const dd = Math.abs(s.t[m] - tv);
        if (dd < best) { best = dd; k = m; }
      }
      dots[i].setAttribute("cx", X(s.t[k]));
      dots[i].setAttribute("cy", Y(s.y[k]));
      dots[i].setAttribute("opacity", 1);
      const rw = document.createElement("div");
      rw.className = "rw";
      const ic = document.createElement("i");
      ic.style.background = COL[i % 3];
      const nm = document.createElement("span");
      nm.textContent = s.name;                 // 名字当数据处理，用 textContent
      const vb = document.createElement("b");
      vb.textContent = fmt(s.y[k]);
      const left = document.createElement("span");
      left.style.display = "inline-flex";
      left.style.alignItems = "center";
      left.style.gap = "6px";
      left.appendChild(ic); left.appendChild(nm);
      rw.appendChild(left); rw.appendChild(vb);
      tip.appendChild(rw);
    });
    tip.style.display = "block";
    const tw = tip.offsetWidth, th = tip.offsetHeight;
    let lx = ev.clientX + 14, ly = ev.clientY - th / 2;
    if (lx + tw > innerWidth - 8) lx = ev.clientX - tw - 14;
    if (ly < 8) ly = 8;
    if (ly + th > innerHeight - 8) ly = innerHeight - th - 8;
    tip.style.left = lx + "px";
    tip.style.top = ly + "px";
  };
  svg.addEventListener("pointermove", show);
  svg.addEventListener("pointerleave", hide);
  svg.addEventListener("blur", hide);
  svg.addEventListener("focus", (e) => {
    const bb = svg.getBoundingClientRect();
    show({clientX: bb.left + bb.width / 2, clientY: bb.top + bb.height / 2});
  });
}

function table(k) {
  const rows = D.stats[k];
  const head = ["关节", "max|e| rad", "max|tau| N·m", "max|ff|", "max|fb|",
                "max|扰动|", "DTAU_MAX", "迭代均值", "迭代最大"];
  const t = document.createElement("table");
  const tr = document.createElement("tr");
  head.forEach(h => { const th = document.createElement("th"); th.textContent = h; tr.appendChild(th); });
  t.appendChild(tr);
  rows.forEach((r, j) => {
    const x = document.createElement("tr");
    [`J${j}`, fmt(r.e), fmt(r.tau), fmt(r.ff), fmt(r.fb), fmt(r.dist),
     fmt(D.meta.dtau_max[j]), r.it_mean.toFixed(1), String(r.it_max)]
      .forEach(v => { const td = document.createElement("td"); td.textContent = v; x.appendChild(td); });
    t.appendChild(x);
  });
  const box = document.getElementById("tbl");
  box.textContent = "";
  box.appendChild(t);
}

function metaTable() {
  const m = D.meta;
  const t = document.createElement("table");
  const tr = document.createElement("tr");
  ["关节", "M_eff 设计", "K_pos", "K_vel", "DTAU_MAX"].forEach(h => {
    const th = document.createElement("th"); th.textContent = h; tr.appendChild(th);
  });
  t.appendChild(tr);
  for (let j = 0; j < D.nj; j++) {
    const x = document.createElement("tr");
    [`J${j}`, fmt(m.m_design[j]), fmt(m.k_pos[j]), fmt(m.k_vel[j]), fmt(m.dtau_max[j])]
      .forEach(v => { const td = document.createElement("td"); td.textContent = v; x.appendChild(td); });
    t.appendChild(x);
  }
  const box = document.getElementById("meta");
  box.textContent = "";
  const p = document.createElement("p");
  p.className = "note";
  p.textContent = `Q_POS=${m.q_pos} Q_VEL=${m.q_vel} R=${m.r_weight}`
    + ` · N=${m.n} · BASE_DT=${(m.base_dt * 1e3).toFixed(1)}ms · MPC_DT=${(m.mpc_dt * 1e3).toFixed(1)}ms`;
  box.appendChild(p);
  box.appendChild(t);
}

function render() {
  const pr = D.presets[cur];
  const root = document.getElementById("charts");
  root.textContent = "";
  for (let j = 0; j < D.nj; j++) {
    const g = document.createElement("div");
    g.className = "grid";
    const p = D.panels[j];
    panel(g, `J${j} 跟踪 [rad]`, p.track.map(s => slice(s, pr.lo, pr.hi)), pr.lo, pr.hi, null);
    panel(g, `J${j} 真实误差 [rad]`, p.err.map(s => slice(s, pr.lo, pr.hi)), pr.lo, pr.hi, [0]);
    panel(g, `J${j} 力矩 [N·m]`, p.torque.map(s => slice(s, pr.lo, pr.hi)), pr.lo, pr.hi,
          [p.dtau_max, -p.dtau_max]);
    root.appendChild(g);
  }
  table(cur);
}

function init() {
  const m = D.meta;
  document.getElementById("sub").textContent =
    `${D.nj} 轴 · ${m.n_steps} 步 · ${m.n_solves} 次求解 · 迭代 均值 `
    + `${m.it_mean_all.toFixed(1)} / 最大 ${m.it_max_all}`;
  const bar = document.getElementById("bar");
  const btns = [];
  D.presets.forEach((p, i) => {
    const b = document.createElement("button");
    b.textContent = p.name;
    b.setAttribute("aria-pressed", i === cur ? "true" : "false");
    b.onclick = () => {
      cur = i;
      btns.forEach((x, k) => x.setAttribute("aria-pressed", k === i ? "true" : "false"));
      render();
    };
    bar.appendChild(b); btns.push(b);
  });
  const sp = document.createElement("span");
  sp.className = "sp";
  bar.appendChild(sp);
  const th = document.createElement("button");
  const setTheme = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    th.textContent = dark ? "浅色" : "深色";
  };
  th.onclick = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    document.documentElement.setAttribute("data-theme", dark ? "light" : "dark");
    setTheme();
  };
  setTheme();
  bar.appendChild(th);
  document.getElementById("foot").textContent =
    "竖线 = 负载突变和碰撞冲击时刻；力矩图上的两条横线 = 该关节的 ±DTAU_MAX。"
    + " 曲线按 min/max 分桶抽样（保留尖峰），表格里的统计用全分辨率数据算。"
    + " q_ref 和 q、tau 和 tau_ff 几乎重合是正常的（跟踪好、前馈占主项），"
    + " 具体数值看 tooltip 或表格。";
  metaTable();
  render();
}
init();
"""

SCRIPT_DIR = Path(__file__).resolve().parent
NPZ = SCRIPT_DIR / "sim_result.npz"
OUT = SCRIPT_DIR / "sim_result.html"

# 每条曲线最多画这么多点。用 min/max 分桶抽样而不是等间隔取点：
# 等间隔会把噪声和冲击瞬态的尖峰整段漏掉，min/max 保留每个桶的上下包络。
MAX_POINTS = 1600


def decimate(t, y, n_out):
    """min/max 分桶抽样。返回 (t_out, y_out)，长度约 n_out。"""
    n = len(t)
    if n <= n_out:
        return t, y
    n_buckets = max(1, n_out // 2)
    edges = np.linspace(0, n, n_buckets + 1).astype(int)
    ts, ys = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        seg = y[a:b]
        i_lo = a + int(np.argmin(seg))
        i_hi = a + int(np.argmax(seg))
        for i in sorted((i_lo, i_hi)):
            ts.append(t[i])
            ys.append(y[i])
    return np.asarray(ts), np.asarray(ys)


def r5(a):
    """压缩输出体积：5 位有效数字足够画图，省掉一半文件大小。"""
    return [float(f"{v:.5g}") for v in np.asarray(a, dtype=float)]


def main():
    if not NPZ.exists():
        raise SystemExit(f"找不到 {NPZ}，先跑 sim.py")
    d = np.load(NPZ)
    t = d["t"]
    nj = int(d["n_joints"])
    need = ("e_true", "tau_dist", "t_traj", "payload_t", "impact_t")
    missing = [k for k in need if k not in d.files]
    if missing:
        raise SystemExit(
            f"{NPZ.name} 缺字段 {missing}：这是旧版 sim.py 存的，重跑一次 sim.py")

    t_traj = float(d["t_traj"])
    payload_t = float(d["payload_t"])
    impact_t = float(d["impact_t"])
    t_end = float(t[-1])
    dtau_max = np.asarray(d["dtau_max"], dtype=float)

    # 时间段预设。全程放第一个，其余按仿真里的事件分段。
    presets = [
        ("全程", 0.0, t_end),
        ("轨迹段", 0.0, t_traj),
        ("静止段", t_traj, payload_t),
        (f"负载段 (+{float(d['payload_mass']):g}kg)", payload_t, impact_t),
        ("冲击段", impact_t, t_end),
    ]

    # 每个关节三幅图的曲线。抽样只为画图；下面表格里的统计用全分辨率算。
    panels = []
    for j in range(nj):
        tr = []
        for name, arr in (("q_ref", d["q_ref"][:, j]), ("q", d["q"][:, j])):
            td, yd = decimate(t, arr, MAX_POINTS)
            tr.append({"name": name, "t": r5(td), "y": r5(yd)})
        te, ye = decimate(t, d["e_true"][:, j], MAX_POINTS)
        err = [{"name": "e_true", "t": r5(te), "y": r5(ye)}]
        tq = []
        for name, arr in (("tau", d["tau"][:, j]), ("tau_ff", d["tau_ff"][:, j]),
                          ("dtau", d["dtau"][:, j])):
            td, yd = decimate(t, arr, MAX_POINTS)
            tq.append({"name": name, "t": r5(td), "y": r5(yd)})
        panels.append({"track": tr, "err": err, "torque": tq,
                       "dtau_max": float(dtau_max[j])})

    # 每个预设段、每个关节的统计，用全分辨率数据算，表格直接读
    sj = np.asarray(d["solve_joint"], dtype=int)
    si = np.asarray(d["solve_iter"], dtype=float)
    st = np.asarray(d["solve_step"], dtype=int)
    base_dt = float(d["base_dt"])
    stats = []
    for _, lo, hi in presets:
        m = (t >= lo) & (t < hi) if hi < t_end else (t >= lo)
        sm = (st * base_dt >= lo) & (st * base_dt < hi)
        rows = []
        for j in range(nj):
            it_j = si[sm & (sj == j)]
            rows.append({
                "e": float(np.abs(d["e_true"][m, j]).max()) if m.any() else 0.0,
                "tau": float(np.abs(d["tau"][m, j]).max()) if m.any() else 0.0,
                "ff": float(np.abs(d["tau_ff"][m, j]).max()) if m.any() else 0.0,
                "fb": float(np.abs(d["dtau"][m, j]).max()) if m.any() else 0.0,
                "dist": float(np.abs(d["tau_dist"][m, j]).max()) if m.any() else 0.0,
                "it_mean": float(it_j.mean()) if it_j.size else 0.0,
                "it_max": int(it_j.max()) if it_j.size else 0,
            })
        stats.append(rows)

    payload = {
        "nj": nj,
        "presets": [{"name": n, "lo": lo, "hi": hi} for n, lo, hi in presets],
        "panels": panels,
        "stats": stats,
        "meta": {
            "q_pos": float(d["q_pos"]), "q_vel": float(d["q_vel"]),
            "r_weight": float(d["r_weight"]),
            "base_dt": base_dt, "mpc_dt": float(d["mpc_dt"]),
            "n": int(d["N"]),
            "m_design": r5(d["M_design"]), "k_pos": r5(d["K_pos"]),
            "k_vel": r5(d["K_vel"]), "dtau_max": r5(dtau_max),
            "t_traj": t_traj, "payload_t": payload_t, "impact_t": impact_t,
            "impact_duration": float(d["impact_duration"]),
            "payload_mass": float(d["payload_mass"]),
            "n_steps": int(len(t)), "n_solves": int(si.size),
            "it_max_all": int(si.max()), "it_mean_all": float(si.mean()),
        },
    }
    html = (TEMPLATE
            .replace("__SCRIPT__", SCRIPT)
            .replace("__PAYLOAD__", json.dumps(payload, ensure_ascii=False)))
    OUT.write_text(html, encoding="utf-8", newline="\n")
    size_kb = OUT.stat().st_size / 1024.0
    print(f"写入 {OUT}  ({size_kb:.0f} KB)")
    print(f"  {nj} 个关节 x 3 幅图, 原始 {len(t)} 步抽样到每条曲线 <= {MAX_POINTS} 点")
    print("  浏览器直接打开即可；顶部按钮切时间段，右上角切深浅色")


if __name__ == "__main__":
    main()
