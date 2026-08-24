# -*- coding: utf-8 -*-
"""
plc_link/common.py —— 寄存器映射/报警逻辑的单一事实来源
====================================================================

职责：
    把「保持寄存器地址、定标、报警位、Unit 规划、回路配置、越限报警
    判定」这些被三方共同使用的定义收敛到唯一模块：
        从站(modbus_server) / 主站自测(modbus_client_test) / 看板(app)
    避免多处复制导致的失同步风险（代码评审发现的重复代码问题）。

依赖约束：
    本模块【不得】import pymodbus —— 保证单机模式看板可以零成本导入；
    pymodbus 相关类型（如带写保护的数据块）放在 modbus_server 中实现。
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# 保持寄存器地址（0 基址）。SP/PV/OP 均为 ×100 定标（0.01 工程单位）
# ---------------------------------------------------------------------------
HR_SP = 0     # 设定值（主站写）
HR_PV = 1     # 测量值（从站写，只读）
HR_OP = 2     # 输出值（从站写，只读）
HR_MODE = 3   # 1=自动 0=手动（主站写）
HR_ALM = 4    # 报警码位域（从站写，只读）
HR_HB = 5     # 心跳计数（从站写，只读）
N_REGS = 100  # 每个 Unit 的保持寄存器区长度

# 相对块基址的只读偏移：外部 Modbus 写请求落入这些位置将被数据块拒绝
READONLY_OFFSETS = (HR_PV, HR_OP, HR_ALM, HR_HB)

FC_HOLDING = 3  # 保持寄存器对应功能码（pymodbus 内部寻址用）

# ---------------------------------------------------------------------------
# 报警位定义与边沿提取
# ---------------------------------------------------------------------------
ALM_BIT_HIGH = 1   # bit0：PV 高限
ALM_BIT_LOW = 2    # bit1：PV 低限
ALM_BIT_DEV = 4    # bit2：SP/PV 偏差

ALM_ITEMS = ((ALM_BIT_HIGH, "高限"), (ALM_BIT_LOW, "低限"),
             (ALM_BIT_DEV, "偏差"))


def alarm_edges(prev_code: int, new_code: int) -> list[tuple[int, str, bool]]:
    """比较前后两拍报警码，返回报警边沿列表。

    返回元素为 (位掩码, 名称, 是否激活)；供报警流水记录使用，
    取代原先在从站/看板各写一份的边沿检测代码。
    """
    edges: list[tuple[int, str, bool]] = []
    for bit, name in ALM_ITEMS:
        was = bool(prev_code & bit)
        now = bool(new_code & bit)
        if now and not was:
            edges.append((bit, name, True))
        elif was and not now:
            edges.append((bit, name, False))
    return edges


class AlarmMonitor:
    """越限延时确认报警判定器。

    连续 confirm 个周期越限才置对应位，任一拍恢复立即清零计数——
    防止瞬间越限/测量抖动引起报警闪发。建议传入真值 PV 判定
    （调用方自行决定），进一步隔离测量噪声影响。
    """

    def __init__(self, high: float, low: float, dev: float,
                 confirm: int = 5) -> None:
        """
        参数：
            high/low/dev  高限、低限、SP-PV 偏差带（工程单位）
            confirm       延时确认周期数（默认 5 拍）
        """
        self.high = float(high)
        self.low = float(low)
        self.dev = float(dev)
        self.confirm = max(int(confirm), 1)
        self._cnt_high = 0
        self._cnt_low = 0
        self._cnt_dev = 0

    def evaluate(self, pv: float, sp: float) -> int:
        """输入本拍 PV 与 SP，返回当前报警码位域。"""
        if pv > self.high:
            self._cnt_high += 1
        else:
            self._cnt_high = 0
        if pv < self.low:
            self._cnt_low += 1
        else:
            self._cnt_low = 0
        if abs(pv - sp) > self.dev:
            self._cnt_dev += 1
        else:
            self._cnt_dev = 0

        code = 0
        if self._cnt_high >= self.confirm:
            code |= ALM_BIT_HIGH
        if self._cnt_low >= self.confirm:
            code |= ALM_BIT_LOW
        if self._cnt_dev >= self.confirm:
            code |= ALM_BIT_DEV
        return code


# ---------------------------------------------------------------------------
# Unit 规划、回路名称与回路配置（工艺基准/整定参数均为典型经验值）
# ---------------------------------------------------------------------------
UNIT_DOSING = 1
UNIT_AERATION = 2
UNIT_OF_LOOP = {"dosing": UNIT_DOSING, "aeration": UNIT_AERATION}
LOOP_NAMES = {"dosing": "加药回路(余氯)", "aeration": "曝气回路(溶解氧)"}

LOOP_CONFIG = {
    "dosing": {
        "unit": UNIT_DOSING, "name": "加药回路-出水余氯",
        "sp0": 1.0,                              # 初始设定值 mg/L
        "kp": 31.0, "ti": 87.0, "td": 14.5,      # Z-N 工程修正参数（见整定报告）
        "deadband": 0.005,
        "high": 4.5, "low": 0.2, "dev": 0.5,     # 报警限 mg/L
    },
    "aeration": {
        "unit": UNIT_AERATION, "name": "曝气回路-溶解氧",
        "sp0": 2.5,
        "kp": 32.1, "ti": 42.0, "td": 7.0,
        "deadband": 0.01,
        "high": 8.0, "low": 1.0, "dev": 0.6,
    },
}


# ---------------------------------------------------------------------------
# 定标换算
# ---------------------------------------------------------------------------
def scale_to_reg(value: float) -> int:
    """工程值 -> 寄存器整数（×100 定标）。"""
    return int(round(value * 100))


def scale_from_reg(reg: int) -> float:
    """寄存器整数 -> 工程值。"""
    return reg / 100.0
