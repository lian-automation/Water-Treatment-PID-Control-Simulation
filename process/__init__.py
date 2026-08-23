# -*- coding: utf-8 -*-
"""process 包 —— 过程对象模型子包。

对外暴露（惰性导入）：
    FOPDTLoop            单回路 FOPDT（一阶惯性+纯滞后）仿真对象
    ProcessPlant         双回路水处理过程厂（加药回路 + 曝气回路）
    build_default_plant  使用典型经验值参数快速构建默认过程厂
"""

__all__ = ["FOPDTLoop", "ProcessPlant", "build_default_plant"]


def __getattr__(name: str):
    """PEP 562 惰性导出：首次访问时才真正导入子模块。"""
    if name == "FOPDTLoop":
        from .plant import FOPDTLoop
        return FOPDTLoop
    if name == "ProcessPlant":
        from .plant import ProcessPlant
        return ProcessPlant
    if name == "build_default_plant":
        from .plant import build_default_plant
        return build_default_plant
    raise AttributeError(f"process 包中没有属性：{name}")
