# -*- coding: utf-8 -*-
"""
run_test.py —— 三大标准场景测试与报告生成（核心产出脚本）
====================================================================

职责：
    对两个控制回路（加药/余氯、曝气/溶解氧），用两组参数
    （①默认保守参数 ②Z-N 工程修正整定参数）各跑一遍三个标准场景：

      场景① 设定值阶跃 +10%        （伺服跟随能力）
      场景② 进水负荷阶跃 +30%      （抗扰动能力/调节性能）
      场景③ 测量噪声连续干扰 ×5    （噪声鲁棒性）

    自动统计调节品质指标并生成：
      docs/测试报告.md   —— 指标对比表格 + 结论（Markdown）
      docs/测试曲线.html —— ECharts 对比曲线（浏览器打开查看）

指标定义（详见 docs/系统设计说明书.md）：
    超调量%          峰值超出终态的量 / 设定值阶跃幅值（伺服场景）
    最大偏差 %FS     事件后最大偏差 / 面板量程 ×100（扰动场景）
    调节时间(s)      误差最后一次越出 ±2%FS 的时刻（相对事件发生时刻）
    稳态误差 %FS     事件后末段 90s 平均误差 / 量程 ×100
    IAE              Σ|e|·dt，控制器视角积分绝对误差（含测量噪声）

用法：
    python run_test.py                # 全部场景 + 报告生成
"""

from __future__ import annotations

import time
from datetime import datetime

import numpy as np

from process.plant import build_default_plant
from tune.tuner import (CONSERVATIVE_PARAMS, DOCS_DIR,
                        ENGINEER_CORRECTION, EVENT_T, FS_SPAN,
                        LOAD_TOTAL_S, NOISE_TOTAL_S, SERVO_TOTAL_S,
                        engineering_correction, identify_fopdt,
                        make_comparison_html, run_closed_loop,
                        ziegler_nichols_open_loop)

REPORT_MD = DOCS_DIR / "测试报告.md"
REPORT_HTML = DOCS_DIR / "测试曲线.html"

SCENARIOS = [
    ("servo", "场景① 设定值阶跃 +10%", SERVO_TOTAL_S),
    ("load", "场景② 进水负荷阶跃 +30%", LOAD_TOTAL_S),
    ("noise", "场景③ 测量噪声连续干扰 ×5", NOISE_TOTAL_S),
]


def noise_std_of(pv: np.ndarray, t: np.ndarray, t_from: float, t_to: float) -> float:
    """取指定时间段的真值 PV 波动标准差（衡量噪声水平）。"""
    mask = (t >= t_from) & (t <= t_to)
    return float(np.std(pv[mask]))


def fmt_row(cells: list[str]) -> str:
    return "| " + " | ".join(cells) + " |"


def main() -> None:
    t_start = time.time()
    plant_tpl = build_default_plant()
    charts: list[dict] = []
    md: list[str] = []

    md.append("# PID 控制回路场景测试报告")
    md.append("")
    md.append(f"> 自动生成于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
              f"（`python run_test.py`）。所有指标均为**仿真验证值**；"
              "过程模型参数为水处理行业**典型经验值**，仅用于求职作品集"
              "技术验证，不代表任何真实水厂的实际性能。")
    md.append("")

    # ------------------------------------------------------------------
    # 1. 测试对象与参数组
    # ------------------------------------------------------------------
    md.append("## 1. 测试对象与参数组")
    md.append("")
    md.append(fmt_row(["回路", "被控量/操作量", "模型 K/T/τ(真值)",
                       "辨识结果(两点法)", "保守参数 Kp/Ti/Td",
                       "整定参数(Z-N工程修正值) Kp/Ti/Td"]))
    md.append("|---|---|---|---|---|---|")

    tuned_params: dict[str, dict] = {}
    ident_info: dict[str, dict] = {}

    for key in ("dosing", "aeration"):
        tpl = getattr(plant_tpl, key)
        ident_seconds = 1200.0 if key == "dosing" else 900.0
        ident = identify_fopdt(tpl, op_step_frac=0.4,
                               ident_seconds=ident_seconds)
        zn = ziegler_nichols_open_loop(ident["k_hat"], ident["t_hat"],
                                       ident["tau_hat"])
        corr = engineering_correction(zn)
        tuned_params[key] = corr
        ident_info[key] = {"ident": ident, "zn": zn}
        cons = CONSERVATIVE_PARAMS[key]
        md.append(fmt_row([
            tpl.name, f"{tpl.pv_unit}/{tpl.op_unit}",
            f"{tpl.k:.3f} / {tpl.t:.0f}s / {tpl.tau:.0f}s",
            f"{ident['k_hat']:.3f} / {ident['t_hat']:.0f}s / "
            f"{ident['tau_hat']:.0f}s",
            f"{cons['kp']:.1f} / {cons['ti']:.0f} / {cons['td']:.1f}",
            f"{corr['kp']:.1f} / {corr['ti']:.1f} / {corr['td']:.2f}",
        ]))
    md.append("")
    md.append(f"整定参数为 Z-N 响应曲线法建议值经工程修正"
              f"（Kp×{ENGINEER_CORRECTION['kp_factor']}、"
              f"Ti×{ENGINEER_CORRECTION['ti_factor']}）；"
              "原始 Z-N 建议值的激进表现见 `docs/整定对比报告.md`。")
    md.append("")

    # ------------------------------------------------------------------
    # 2~4. 三大场景
    # ------------------------------------------------------------------
    conclusions: list[str] = []
    section_no = 2
    for scene_key, scene_title, total_s in SCENARIOS:
        md.append(f"## {section_no}. {scene_title}")
        md.append("")
        section_no += 1

        if scene_key == "servo":
            md.append("指标口径：超调量%=（峰值PV−终态PV）/设定阶跃幅值；"
                      "其余同上。事件时刻 600s。")
        elif scene_key == "load":
            md.append("指标口径：最大偏差%FS=事件后最大偏差/面板量程；"
                      "恢复时间=偏差回落到 ±0.5%FS 工程恢复带内并保持的时刻；"
                      "本场景**关闭测量噪声**以隔离扰动动力学（噪声影响见"
                      "场景③）。负荷扰动同时注入两回路，事件时刻 600s。")
        else:
            md.append("指标口径：PV波动=测量真值标准差（事件前 300~600s "
                      "vs 事件后稳态段）；OP动作强度=输出标准差（反映执行器"
                      "磨损）；IAE(真值)=按无噪声真值误差积分。噪声倍乘系数"
                      "由 1 放大到 5，事件时刻 600s。**注**：×5 强噪声下 "
                      "IAE(真值) 由噪声激励主导，组间差异主要反映"
                      "「高增益回路会把测量噪声经执行器转化为真实工艺波动」"
                      "的机理，而非调节能力优劣；工程对策是加强微分滤波或"
                      "切除微分作用。")
        md.append("")

        for key in ("dosing", "aeration"):
            tpl = getattr(plant_tpl, key)
            fs = FS_SPAN[key]
            unit = tpl.pv_unit
            runs = {}
            for label, params in (
                ("默认保守参数", CONSERVATIVE_PARAMS[key]),
                ("Z-N工程修正参数", tuned_params[key]),
            ):
                runs[label] = run_closed_loop(
                    key, params, scene_key, total_s, EVENT_T,
                    quiet_noise=(scene_key == "load"),
                )

            md.append(f"### {tpl.name}")
            md.append("")
            if scene_key == "servo":
                md.append(fmt_row(["参数组", "Kp", "超调量(%)", "调节时间(s)",
                                   "稳态误差(%FS)", "IAE",
                                   f"最大偏差({unit})"]))
            elif scene_key == "load":
                md.append(fmt_row(["参数组", "Kp", "最大偏差(%FS)",
                                   "恢复时间(s)", "稳态误差(%FS)", "IAE",
                                   f"终态PV({unit})"]))
            else:
                md.append(fmt_row(["参数组", "Kp",
                                   "PV波动 前→后", "波动放大倍数",
                                   "OP动作强度", "IAE(真值)",
                                   "最大偏差(%FS)"]))
            md.append("|---|---|---|---|---|---|---|")

            for label, run in runs.items():
                m = run["metrics"]
                p = run["meta"]["pid_params"]
                op_std = float(np.std(run["op"][EVENT_T:]))
                if scene_key == "servo":
                    row = [label,
                           f"{p['kp']:.1f}",
                           f"{m.get('overshoot_pct_of_step', 0):.1f}",
                           f"{m['settling_s']:.0f}",
                           f"{m['ss_err_pct_fs']:.3f}",
                           f"{m['iae']:.2f}",
                           f"{m['max_dev']:.3f}"]
                elif scene_key == "load":
                    row = [label,
                           f"{p['kp']:.1f}",
                           f"{abs(m['max_dev_signed']) / fs * 100:.2f}",
                           f"{m['recovery_s']:.0f}",
                           f"{m['ss_err_pct_fs']:.3f}",
                           f"{m['iae']:.2f}",
                           f"{m['final_pv']:.3f}"]
                else:
                    std_before = noise_std_of(run["pv_true"], run["t"],
                                              300, 590)
                    std_after = noise_std_of(run["pv_true"], run["t"],
                                             900, total_s - 10)
                    row = [label,
                           f"{p['kp']:.1f}",
                           f"{std_before:.4f} → {std_after:.4f}",
                           f"×{std_after / max(std_before, 1e-9):.1f}",
                           f"{op_std:.2f} {tpl.op_unit}",
                           f"{m['iae_true']:.2f}",
                           f"{abs(m['max_dev_signed']) / fs * 100:.1f}"]
                md.append(fmt_row(row))
            md.append("")

            # 对比曲线（真值 PV + SP）
            base_run = runs["Z-N工程修正参数"]
            x = [float(v) for v in base_run["t"]]
            series = [{"name": "SP", "x": x,
                       "y": [round(float(v), 3) for v in base_run["sp"]]}]
            for label, run in runs.items():
                series.append({
                    "name": label,
                    "x": x,
                    "y": [round(float(v), 4) for v in run["pv_true"]],
                })
            charts.append({
                "title": f"{scene_title}｜{tpl.name}（{unit}）",
                "y_name": unit, "series": series,
            })

            # 结论素材
            mc = runs["Z-N工程修正参数"]["metrics"]
            mm = runs["默认保守参数"]["metrics"]
            if scene_key == "servo":
                gain = (mm.get("overshoot_pct_of_step") -
                        mc.get("overshoot_pct_of_step"))
                conclusions.append(
                    f"{scene_title}·{tpl.name}：整定后超调 "
                    f"{mm.get('overshoot_pct_of_step', 0):.1f}%→"
                    f"{mc.get('overshoot_pct_of_step', 0):.1f}%"
                    f"（{'降低' if gain > 0 else '增加'} {abs(gain):.1f} 个百分点），"
                    f"IAE {mm['iae']:.1f}→{mc['iae']:.1f}。")
            elif scene_key == "load":
                d_cons = abs(mm["max_dev_signed"]) / fs * 100
                d_tune = abs(mc["max_dev_signed"]) / fs * 100
                conclusions.append(
                    f"{scene_title}·{tpl.name}：整定后最大偏差 "
                    f"{d_cons:.2f}%FS→{d_tune:.2f}%FS，"
                    f"恢复时间 {mm['recovery_s']:.0f}s→{mc['recovery_s']:.0f}s。")
            else:
                op_std_m = float(np.std(runs["默认保守参数"]["op"][EVENT_T:]))
                op_std_c = float(np.std(runs["Z-N工程修正参数"]["op"][EVENT_T:]))
                conclusions.append(
                    f"{scene_title}·{tpl.name}：整定后 IAE(真值) "
                    f"{mm['iae_true']:.1f}→{mc['iae_true']:.1f}，OP 动作强度 "
                    f"{op_std_m:.2f}→{op_std_c:.2f}{tpl.op_unit}"
                    "（强整定对噪声更敏感，须配合不完全微分+限幅抑制）。")

    # ------------------------------------------------------------------
    # 5. 综合结论
    # ------------------------------------------------------------------
    md.append(f"## {section_no}. 综合结论（仿真验证值）")
    md.append("")
    for c in conclusions:
        md.append(f"- {c}")
    md.append("")
    md.append("**总体评价**：Z-N 工程修正参数在伺服跟随与扰动恢复速度上"
              "明显优于经验保守参数；代价是对测量噪声更敏感（OP 动作更频繁），"
              "因此工程实施必须配合「不完全微分 + 微分限幅」抑制噪声放大——"
              "这正是本项目控制器实现的三个关键点之一。")
    md.append("")
    elapsed = time.time() - t_start
    md.append("---")
    md.append(f"*测试耗时 {elapsed:.1f}s；曲线交互查看：`docs/测试曲线.html`"
              "（ECharts 经 CDN 加载，需联网）。*")

    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_MD.write_text("\n".join(md), encoding="utf-8")
    make_comparison_html(
        title="三大标准场景 PID 对比曲线（仿真验证值）",
        charts=charts, out_path=REPORT_HTML,
        notes="每组曲线含 SP（虚线）、默认保守参数与 Z-N 工程修正参数的 "
              "PV 真值响应；可用缩放条回看细节。",
    )

    print("=" * 72)
    print("场景测试完成，已生成：")
    print(f"  报告：{REPORT_MD}")
    print(f"  曲线：{REPORT_HTML}")
    print("=" * 72)
    for c in conclusions:
        print(f"  · {c}")


if __name__ == "__main__":
    main()
