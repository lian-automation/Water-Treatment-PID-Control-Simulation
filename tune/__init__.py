# -*- coding: utf-8 -*-
"""tune 包 —— 参数整定与对比实验子包。

对外暴露（惰性导入）：
    identify_fopdt           两点法开环阶跃辨识
    ziegler_nichols_open_loop Z-N 响应曲线法整定公式
    run_closed_loop          单回路闭环实验引擎（供对比实验与场景测试共用）
    calc_metrics             调节品质指标计算
    run_comparison           完整对比实验主流程
    write_reports            生成整定对比报告（MD + ECharts HTML）
"""

__all__ = [
    "identify_fopdt", "ziegler_nichols_open_loop", "engineering_correction",
    "run_closed_loop", "calc_metrics", "run_comparison", "write_reports",
    "CONSERVATIVE_PARAMS", "BASELINE_SP", "FS_SPAN",
]


def __getattr__(name: str):
    """PEP 562 惰性导出：首次访问时才真正导入子模块。"""
    if name in __all__:
        from . import tuner
        return getattr(tuner, name)
    raise AttributeError(f"tune 包中没有属性：{name}")
