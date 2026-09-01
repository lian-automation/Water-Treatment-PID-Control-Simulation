# -*- coding: utf-8 -*-
"""control 包 —— 控制器子包。

对外暴露（惰性导入）：
    PIDController 增量式 PID 控制器（输出限幅/增量钳位抗积分饱和/
                  测量死区/手自动无扰切换/微分先行/参数在线修改）
"""

__all__ = ["PIDController"]


def __getattr__(name: str):
    """PEP 562 惰性导出：首次访问时才真正导入子模块。"""
    if name == "PIDController":
        from .pid import PIDController
        return PIDController
    raise AttributeError(f"control 包中没有属性：{name}")
