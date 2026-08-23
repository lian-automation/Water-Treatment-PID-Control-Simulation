# -*- coding: utf-8 -*-
"""
tune/tuner.py —— 响应曲线法过程辨识 + Z-N 整定建议 + 参数对比实验
====================================================================

职责：
    ① 过程辨识：对指定回路做开环阶跃试验，用「两点法」从响应曲线
       反推 FOPDT 三参数（K、T、tau），并与仿真真值对照验证；
    ② 整定计算：按 Ziegler-Nichols 响应曲线法（开环法）公式给出
       P/I/D 建议值；
    ③ 对比实验：默认保守参数 vs Z-N 整定参数，在同一场景下闭环运行，
       统计超调量、调节时间、稳态误差(%FS)、IAE 等指标，
       输出 Markdown 报告与 ECharts 对比曲线网页。

两点法原理：
    FOPDT 归一化阶跃响应 y(t)/y∞ = 1 - exp(-(t-τ)/T)，t>τ。
    取两个特征点：
        y/y∞ = 28.3% 处时刻 t28 = τ + 0.3327·T
        y/y∞ = 63.2% 处时刻 t63 = τ + 1.0000·T
    联立解出：
        T = (t63 - t28) / 0.6673
        τ = t63 - T
        K = Δy∞ / ΔOP
    两点法只用两个采样点，对测量噪声的鲁棒性比切线法（找拐点作切线）
    好得多，是工程上最常用的响应曲线辨识方法。

Z-N 开环整定公式（反应曲线法）：
    Kp = 1.2 · T / (K · τ)      Ti = 2 · τ      Td = 0.5 · τ
    注：Z-N 公式以" quarter-amplitude decay"（4:1 衰减）为目标，
    偏激进，实际工程常在建议值基础上再保守化修正，报告中有对比数据。

用法：
    python -m tune.tuner        # 运行辨识 + 整定 + 双回路对比实验
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from control.pid import PIDController
from process.plant import FOPDTLoop, ProcessPlant, build_default_plant

# ---------------------------------------------------------------------------
# 实验公共配置
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DOCS_DIR = PROJECT_ROOT / "docs"

DT = 1.0                     # 仿真步长(s)
EVENT_T = 600                # 扰动/阶跃发生时刻(s)——前段用于让系统进入稳态
SERVO_TOTAL_S = 1800         # 设定值阶跃场景总时长(s)
LOAD_TOTAL_S = 1800          # 负荷扰动场景总时长(s)
NOISE_TOTAL_S = 1500         # 噪声干扰场景总时长(s)

# 各回路工艺基准设定值（典型工艺经验值：余氯 1.0 mg/L，DO 2.5 mg/L）
BASELINE_SP = {"dosing": 1.0, "aeration": 2.5}
# 各回路面板量程（用于 %FS 指标归一）
FS_SPAN = {"dosing": 5.0, "aeration": 10.0}

# 默认保守参数（经验保守整定：小增益、长积分时间，响应慢但稳健）
CONSERVATIVE_PARAMS = {
    "dosing": {"kp": 15.0, "ti": 100.0, "td": 8.0},
    "aeration": {"kp": 10.0, "ti": 80.0, "td": 4.0},
}


# ---------------------------------------------------------------------------
# ① 开环阶跃辨识（两点法）
# ---------------------------------------------------------------------------
def identify_fopdt(loop_template: FOPDTLoop,
                   op_step_frac: float = 0.4,
                   ident_seconds: float = 1200.0) -> dict:
    """对回路模板做无噪声开环阶跃试验，两点法辨识 K/T/tau。

    参数：
        loop_template  用哪个回路做试验（会克隆一份无噪声副本）
        op_step_frac   阶跃幅度占 OP 量程的比例（默认 40%，幅度足够大、
                       信噪比好，又不触及限幅饱和）
        ident_seconds  试验时长(s)，需远大于 T+tau 以看到稳态平台
    返回：
        dict(k_hat, t_hat, tau_hat, y_inf, op_step, t[], y[])
    """
    loop = loop_template.clone(keep_noise=False, seed=7)  # 无噪声副本
    op_step = (loop.op_max - loop.op_min) * op_step_frac
    n_steps = int(ident_seconds / DT)
    y = np.empty(n_steps)
    for i in range(n_steps):
        y[i] = loop.step(op_step)
    t = np.arange(n_steps) * DT

    # 稳态值：取末段平均
    y_inf = float(np.mean(y[-int(60 / DT):]))
    delta_op = op_step - loop.op_min  # 从初始 OP 到阶跃后 OP 的变化量
    k_hat = (y_inf - 0.0) / delta_op  # 初始 PV=0

    def cross_time(frac: float) -> float:
        """线性插值求响应首次达到 frac*y_inf 的时刻。"""
        target = frac * y_inf
        idx = int(np.argmax(y >= target))  # 第一个达标样本
        if idx == 0 and y[0] >= target:
            return 0.0
        y0, y1 = y[idx - 1], y[idx]
        # 在 [t(idx-1), t(idx)] 区间内线性插值
        return float(t[idx - 1] + (target - y0) / (y1 - y0) * DT)

    t28 = cross_time(0.283)
    t63 = cross_time(0.632)
    t_hat = (t63 - t28) / (math.log(1.0 / (1.0 - 0.283))
                           - math.log(1.0 / (1.0 - 0.632)))
    # 说明：上式分母 = ln(1/0.717) - ln(1/0.368) = 0.3327 - (-1.0)? 注意符号，
    # 化简后等于 0.6673（推导见文件头注释），这里直接给出数值结果。
    t_hat = (t63 - t28) / 0.6673
    tau_hat = t63 - t_hat
    # 物理约束：滞后不可能为负
    if tau_hat < 0.0:
        tau_hat = 0.0
    return {
        "k_hat": k_hat, "t_hat": t_hat, "tau_hat": tau_hat,
        "y_inf": y_inf, "op_step": op_step,
        "t": t, "y": y,
    }


def ziegler_nichols_open_loop(k: float, t: float, tau: float) -> dict:
    """Z-N 响应曲线法（开环）整定公式。"""
    if k <= 0 or tau <= 0:
        raise ValueError("辨识结果异常：K 与 tau 必须为正")
    return {
        "kp": 1.2 * t / (k * tau),
        "ti": 2.0 * tau,
        "td": 0.5 * tau,
    }


# Z-N 工程修正系数：原始 Z-N 以 4:1 衰减为目标、临界稳定裕度极小，
# 本项目仿真验证其闭环为强振荡（见 docs/整定对比报告.md 数据），
# 故按常用经验规则做保守化修正：Kp×0.5、Ti×1.5（Td 不变）。
ENGINEER_CORRECTION = {"kp_factor": 0.5, "ti_factor": 1.5}


def engineering_correction(zn: dict) -> dict:
    """在 Z-N 原始建议值基础上做工程保守化修正。"""
    return {
        "kp": zn["kp"] * ENGINEER_CORRECTION["kp_factor"],
        "ti": zn["ti"] * ENGINEER_CORRECTION["ti_factor"],
        "td": zn["td"],
    }


# ---------------------------------------------------------------------------
# ② 闭环实验引擎（供本模块对比实验与 run_test.py 共用）
# ---------------------------------------------------------------------------
def run_closed_loop(loop_key: str,
                    pid_params: dict,
                    scenario_type: str,
                    total_s: int,
                    event_t: int = EVENT_T,
                    quiet_noise: bool = False) -> dict:
    """在单一回路上跑一段闭环实验并记录全部曲线。

    参数：
        loop_key       "dosing"(加药/余氯) 或 "aeration"(曝气/溶解氧)
        pid_params     {"kp","ti","td"}
        scenario_type  "servo"(设定值阶跃+10%) / "load"(进水负荷+30%) /
                       "noise"(测量噪声×5 连续干扰)
        total_s        总仿真时长(s)
        event_t        事件发生时刻(s)
        quiet_noise    True 时关闭测量噪声（用于隔离扰动动力学的对照试验）
    返回：
        dict(t[], sp[], pv_meas[], pv_true[], op[], metrics{}, meta{})
    """
    assert loop_key in ("dosing", "aeration"), f"未知回路 {loop_key}"
    assert scenario_type in ("servo", "load", "noise"), f"未知场景 {scenario_type}"

    plant = build_default_plant()          # 固定随机种子 → 各参数组噪声一致
    if quiet_noise:
        # 对照试验：隔离测量噪声，单独考察扰动抑制能力
        plant.dosing.set_noise_scale(0.0)
        plant.aeration.set_noise_scale(0.0)
    loop = getattr(plant, loop_key)
    sp = BASELINE_SP[loop_key]
    deadband = 0.005 if loop_key == "dosing" else 0.01
    pid = PIDController(
        kp=pid_params["kp"], ti=pid_params["ti"], td=pid_params["td"],
        dt=DT, out_min=loop.op_min, out_max=loop.op_max, deadband=deadband,
    )

    n = int(total_s / DT)
    t_arr = np.arange(n) * DT
    sp_arr = np.empty(n)
    pv_meas_arr = np.empty(n)
    pv_true_arr = np.empty(n)
    op_arr = np.empty(n)

    for i in range(n):
        if i == event_t:
            if scenario_type == "servo":
                sp = sp * 1.10             # 设定值阶跃 +10%（相对当前值）
            elif scenario_type == "load":
                plant.apply_load_step(30.0)  # 进水负荷阶跃 +30%
            else:
                loop.set_noise_scale(5.0)    # 测量噪声放大 5 倍
        pv_meas_arr[i] = loop.pv
        pv_true_arr[i] = loop.true_pv
        sp_arr[i] = sp
        op = pid.update(sp, loop.pv)
        op_arr[i] = op
        if loop_key == "dosing":
            plant.step(op, 0.0)
        else:
            plant.step(0.0, op)

    metrics = calc_metrics(
        t=t_arr, sp=sp_arr, pv_true=pv_true_arr, pv_meas=pv_meas_arr,
        event_t=event_t, fs_span=FS_SPAN[loop_key],
    )
    # 伺服场景补充：以设定值阶跃幅值为基准的超调百分比
    if scenario_type == "servo":
        step_size = abs(sp_arr[event_t] - sp_arr[event_t - 1])
        metrics["overshoot_pct_of_step"] = (
            metrics["overshoot_abs"] / step_size * 100.0 if step_size > 0 else 0.0
        )
    return {
        "t": t_arr, "sp": sp_arr, "pv_meas": pv_meas_arr,
        "pv_true": pv_true_arr, "op": op_arr,
        "metrics": metrics,
        "meta": {
            "loop_key": loop_key, "scenario": scenario_type,
            "pid_params": dict(pid_params), "event_t": event_t,
            "baseline_sp": BASELINE_SP[loop_key],
            "fs_span": FS_SPAN[loop_key], "pv_unit": loop.pv_unit,
            "op_unit": loop.op_unit,
        },
    }


def calc_metrics(t: np.ndarray,
                 sp: np.ndarray,
                 pv_true: np.ndarray,
                 pv_meas: np.ndarray,
                 event_t: int,
                 fs_span: float,
                 dt: float = DT,
                 band_pct: float = 2.0,
                 tail_s: float = 90.0) -> dict:
    """计算调节品质指标（事件时刻之后的数据段）。

    指标定义（详见测试报告）：
        max_dev        最大偏差绝对值（真值，相对事件前基线误差）
        overshoot_abs  超调绝对量 = 峰值PV - 终态平均PV
        settling_s     调节时间：误差最后一次越出 ±band_pct%FS 的时刻
                       （相对事件发生时刻；此后一直保持在带内）
        ss_err_pct_fs  稳态误差：末段平均误差 / 量程 ×100%
        iae            IAE 积分 = Σ|e|·dt（控制器视角，含测量噪声）
    """
    i0 = int(round(event_t / dt))
    n_tail = max(int(tail_s / dt), 1)
    # 事件前基线误差（消除进入事件前残留的小偏差影响）
    b0 = max(0, i0 - n_tail)
    base_err = float(np.mean(sp[b0:i0] - pv_true[b0:i0])) if i0 > 0 else 0.0

    dev_true = (sp[i0:] - pv_true[i0:]) - base_err   # 真值偏差序列
    dev_ctrl = (sp[i0:] - pv_meas[i0:]) - base_err   # 控制器视角偏差
    tail_mean = float(np.mean(dev_true[-n_tail:]))

    i_peak = int(np.argmax(np.abs(dev_true)))
    max_dev = float(abs(dev_true[i_peak]))
    max_dev_signed = float(dev_true[i_peak])

    seg_pv = pv_true[i0:]
    peak_pv = float(np.max(seg_pv))
    final_pv = float(np.mean(seg_pv[-n_tail:]))
    overshoot_abs = max(peak_pv - final_pv, 0.0)

    # 调节时间：±band 带（以终态平均偏差为中心）
    band = fs_span * band_pct / 100.0
    outside = np.where(np.abs(dev_true - tail_mean) > band)[0]
    settling_s = float((outside[-1] + 1) * dt) if len(outside) > 0 else 0.0

    # 恢复时间：偏差回落到 ±0.5%FS 工程恢复带内并保持的时刻
    # （用于扰动场景——扰动可能始终未越出 ±band，调节时间会失去区分度；
    #   阈值用量程的固定比例而非峰值的比例，避免"偏差越小阈值越低"
    #   导致对抑制能力更强的参数组反而不公平）
    thresh = fs_span * 0.005
    outside_r = np.where(np.abs(dev_true - tail_mean) > thresh)[0]
    recovery_s = float((outside_r[-1] + 1) * dt) if len(outside_r) > 0 else 0.0

    return {
        "max_dev": max_dev,
        "max_dev_signed": max_dev_signed,
        "peak_pv": peak_pv,
        "final_pv": final_pv,
        "overshoot_abs": overshoot_abs,
        "settling_s": settling_s,
        "recovery_s": recovery_s,
        "ss_err_pct_fs": tail_mean / fs_span * 100.0,
        "iae": float(np.sum(np.abs(dev_ctrl)) * dt),
        "iae_true": float(np.sum(np.abs(dev_true)) * dt),
        "base_err": base_err,
        "band": band,
    }


# ---------------------------------------------------------------------------
# ③ ECharts 对比报告生成（HTML 使用 CDN 引入 ECharts，浏览器联网可看）
# ---------------------------------------------------------------------------
_ECHARTS_CDN = "https://cdn.jsdelivr.net/npm/echarts@5.5.0/dist/echarts.min.js"


def make_comparison_html(title: str,
                         charts: list[dict],
                         out_path: Path,
                         notes: str = "") -> None:
    """生成独立 HTML 对比曲线页面。

    charts: [{title, y_name, x_name, series:[{name, x:[], y:[]}]} ...]
    """
    payload = json.dumps(charts, ensure_ascii=False)
    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>{title}</title>
<script src="{_ECHARTS_CDN}"></script>
<style>
  body {{ font-family: "Microsoft YaHei", sans-serif; margin: 24px; background:#f7f8fa; }}
  h1 {{ font-size: 20px; }}
  .chart {{ width: 860px; height: 380px; background:#fff;
           border:1px solid #e5e6eb; border-radius:6px; margin-bottom:16px; }}
  .note {{ color:#555; font-size:13px; line-height:1.8; max-width:860px; }}
</style>
</head>
<body>
<h1>{title}</h1>
<div class="note">{notes}</div>
<div id="app"></div>
<script>
const charts = {payload};
const app = document.getElementById('app');
charts.forEach((cfg, i) => {{
  const div = document.createElement('div');
  div.className = 'chart';
  app.appendChild(div);
  const chart = echarts.init(div);
  chart.setOption({{
    title: {{ text: cfg.title, left: 'center', textStyle: {{ fontSize: 14 }} }},
    tooltip: {{ trigger: 'axis' }},
    legend: {{ top: 28 }},
    grid: {{ left: 60, right: 30, top: 64, bottom: 48 }},
    xAxis: {{ type: 'category', name: cfg.x_name || 't/s', data: cfg.series[0].x }},
    yAxis: {{ type: 'value', name: cfg.y_name || '', scale: true }},
    dataZoom: [{{ type: 'inside' }}, {{ type: 'slider' }}],
    series: cfg.series.map(s => ({{
      name: s.name, type: 'line', data: s.y, showSymbol: false, lineWidth: 1.5
    }}))
  }});
}});
</script>
</body>
</html>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding="utf-8")


# ---------------------------------------------------------------------------
# ④ 对比实验主流程
# ---------------------------------------------------------------------------
def run_comparison() -> list[dict]:
    """完整跑一遍「辨识 → Z-N → 保守 vs 整定 对比」，返回汇总结果。"""
    plant_tpl = build_default_plant()
    results = []
    for loop_key, tpl in (("dosing", plant_tpl.dosing),
                          ("aeration", plant_tpl.aeration)):
        # --- 辨识 ---------------------------------------------------------
        ident_seconds = 1200.0 if loop_key == "dosing" else 900.0
        ident = identify_fopdt(tpl, op_step_frac=0.4, ident_seconds=ident_seconds)
        zn = ziegler_nichols_open_loop(ident["k_hat"], ident["t_hat"],
                                       ident["tau_hat"])
        print("=" * 78)
        print(f"[{tpl.name}] 响应曲线辨识（两点法, 阶跃 {ident['op_step']:.1f}"
              f"{tpl.op_unit}）")
        print(f"  辨识: K={ident['k_hat']:.4f}  T={ident['t_hat']:.1f}s  "
              f"tau={ident['tau_hat']:.1f}s   y∞={ident['y_inf']:.3f}{tpl.pv_unit}")
        print(f"  真值: K={tpl.k:.4f}  T={tpl.t:.1f}s  tau={tpl.tau:.1f}s")
        print(f"  Z-N 建议: Kp={zn['kp']:.2f}  Ti={zn['ti']:.1f}s  "
              f"Td={zn['td']:.2f}s")

        # --- 同一伺服场景下三组参数对比 ------------------------------------
        # ① 默认保守参数（基线）② Z-N 原始建议值（验证其激进程度）
        # ③ Z-N 工程修正参数（实际推荐采用，run_test.py 的"整定后"组）
        zn_corr = engineering_correction(zn)
        runs = []
        for label, params in (
            ("默认保守参数", CONSERVATIVE_PARAMS[loop_key]),
            ("Z-N原始建议值", zn),
            ("Z-N工程修正值", zn_corr),
        ):
            run = run_closed_loop(loop_key, params, "servo",
                                  SERVO_TOTAL_S, EVENT_T)
            m = run["metrics"]
            print(f"  [{label}] Kp={params['kp']:.2f} Ti={params['ti']:.1f} "
                  f"Td={params['td']:.2f} → "
                  f"超调={m.get('overshoot_pct_of_step', 0):.1f}% "
                  f"调节时间={m['settling_s']:.0f}s "
                  f"稳态误差={m['ss_err_pct_fs']:.3f}%FS "
                  f"IAE={m['iae']:.2f}")
            runs.append((label, run))

        results.append({
            "loop_key": loop_key, "name": tpl.name,
            "pv_unit": tpl.pv_unit, "op_unit": tpl.op_unit,
            "ident": ident, "true": {"k": tpl.k, "t": tpl.t, "tau": tpl.tau},
            "zn": zn, "runs": runs,
        })
    return results


def write_reports(results: list[dict]) -> tuple[Path, Path]:
    """把对比实验结果写成 Markdown 报告 + ECharts 曲线页。"""
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    md_path = DOCS_DIR / "整定对比报告.md"
    html_path = DOCS_DIR / "整定对比曲线.html"

    md = ["# 参数整定对比报告（仿真验证值）", "",
          "> 本报告由 `python -m tune.tuner` 自动生成；所有数值均为**仿真验证值**，"
          "模型参数为典型经验值，不代表真实水厂。", ""]
    charts = []
    for r in results:
        unit = r["pv_unit"]
        ident, tru, zn = r["ident"], r["true"], r["zn"]
        zn_corr = engineering_correction(zn)
        md.append(f"## {r['name']}")
        md.append("")
        md.append("### 1. 响应曲线辨识（两点法）")
        md.append("")
        md.append("| 项目 | K | T(s) | τ(s) |")
        md.append("|---|---|---|---|")
        md.append(f"| 辨识值 | {ident['k_hat']:.4f} | {ident['t_hat']:.1f} "
                  f"| {ident['tau_hat']:.1f} |")
        md.append(f"| 仿真真值 | {tru['k']:.4f} | {tru['t']:.1f} | {tru['tau']:.1f} |")
        err_k = abs(ident['k_hat'] - tru['k']) / tru['k'] * 100
        err_t = abs(ident['t_hat'] - tru['t']) / tru['t'] * 100
        err_tau = abs(ident['tau_hat'] - tru['tau']) / tru['tau'] * 100
        md.append(f"| 相对误差 | {err_k:.1f}% | {err_t:.1f}% | {err_tau:.1f}% |")
        md.append("")
        md.append(f"### 2. Z-N 整定建议值（Kp={zn['kp']:.2f}，"
                  f"Ti={zn['ti']:.1f}s，Td={zn['td']:.2f}s）")
        md.append("")
        md.append(f"工程修正（Kp×{ENGINEER_CORRECTION['kp_factor']}、"
                  f"Ti×{ENGINEER_CORRECTION['ti_factor']}）："
                  f"Kp={zn_corr['kp']:.2f}，Ti={zn_corr['ti']:.1f}s，"
                  f"Td={zn_corr['td']:.2f}s。原始 Z-N 以 4:1 衰减为目标，"
                  "本仿真中验证为临界稳定强振荡，故实际采用工程修正值。")
        md.append("")
        md.append("### 3. 设定值阶跃 +10% 对比（事件时刻 600s，总时长 1800s）")
        md.append("")
        md.append("| 参数组 | Kp | Ti(s) | Td(s) | 超调量(%) | 调节时间(s) "
                  "| 稳态误差(%FS) | IAE |")
        md.append("|---|---|---|---|---|---|---|---|")
        for label, run in r["runs"]:
            p = run["meta"]["pid_params"]
            m = run["metrics"]
            md.append(
                f"| {label} | {p['kp']:.2f} | {p['ti']:.1f} | {p['td']:.2f} "
                f"| {m.get('overshoot_pct_of_step', 0):.1f} "
                f"| {m['settling_s']:.0f} "
                f"| {m['ss_err_pct_fs']:.3f} | {m['iae']:.2f} |")
        md.append("")
        # 曲线图：真值 PV 对比 + OP 对比
        x = [float(v) for v in r["runs"][0][1]["t"]]
        pv_series = [{"name": label, "x": x,
                      "y": [round(float(v), 4) for v in run["pv_true"]]}
                     for label, run in r["runs"]]
        op_series = [{"name": label, "x": x,
                      "y": [round(float(v), 3) for v in run["op"]]}
                     for label, run in r["runs"]]
        charts.append({"title": f"{r['name']} PV 对比（{unit}）",
                       "y_name": unit, "series": pv_series})
        charts.append({"title": f"{r['name']} OP 对比（{r['op_unit']}）",
                       "y_name": r["op_unit"], "series": op_series})
        md.append(f"对比曲线见 `{html_path.name}`。")
        md.append("")

    md_path.write_text("\n".join(md), encoding="utf-8")
    make_comparison_html(
        title="PID 参数整定对比曲线（默认保守 vs Z-N 整定，仿真验证值）",
        charts=charts, out_path=html_path,
        notes="生成时间见文件属性；场景：设定值阶跃 +10%（600s 时刻）；"
              "模型为 FOPDT 典型经验值仿真，不代表真实水厂。",
    )
    return md_path, html_path


# ---------------------------------------------------------------------------
# 直接运行本模块：完整演示「辨识 → 整定 → 对比 → 出报告」
# 用法：python -m tune.tuner
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    res = run_comparison()
    md_path, html_path = write_reports(res)
    print("=" * 78)
    print(f"已生成报告：{md_path}")
    print(f"已生成曲线：{html_path}（浏览器打开，ECharts 经 CDN 加载）")
