# -*- coding: utf-8 -*-
"""plc_link 包 —— Modbus/TCP 联调子包。

对外暴露（惰性导入）：
    PLCRuntime   从站内部运行时：过程仿真 + PID 控制 + 寄存器同步

服务启动入口为 modbus_server.main()（python -m plc_link.modbus_server），
主站自测入口为 modbus_client_test.main()（python -m plc_link.modbus_client_test）。
"""

__all__ = ["PLCRuntime"]


def __getattr__(name: str):
    """PEP 562 惰性导出。"""
    if name == "PLCRuntime":
        from .modbus_server import PLCRuntime
        return PLCRuntime
    raise AttributeError(f"plc_link 包中没有属性：{name}")
