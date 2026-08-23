# -*- coding: utf-8 -*-
"""
plc_link/modbus_server.py —— Modbus/TCP 从站（模拟 PLC）
====================================================================

职责：
    在没有真实 PLC 的条件下，用 pymodbus 搭建一个 Modbus/TCP 从站，
    内部跑「过程模型 + PID 控制器」仿真，并把运行数据映射到保持
    寄存器（Holding Register, HR），供上位机/看板/自测脚本读写。

从站规划（两个 Unit ID = 两套控制回路，寄存器布局完全相同）：
    Unit 1 —— 加药回路（余氯控制）
    Unit 2 —— 曝气回路（溶解氧控制）

保持寄存器地址映射（0 基址；SP/PV/OP 均为 ×100 定标，即 0.01 工程单位）：
    HR0  SP   设定值     读/写   ×100（如 1.05 mg/L -> 105）
    HR1  PV   测量值     只读    ×100
    HR2  OP   输出值     只读    ×100（加药泵 %；风机 0.01Hz）
    HR3  MODE 自/手动    读/写   1=自动 0=手动
    HR4  ALM  报警码     只读    bit0=PV高限 bit1=PV低限 bit2=偏差报警
    HR5  HB   心跳       只读    每周期 +1，mod 65536

线程模型：
    主线程：StartTcpServer 阻塞服务（pymodbus 处理 Modbus/TCP 请求）；
    后台 daemon 线程：按控制周期(默认 1s)推进仿真并刷新寄存器。
    说明：pymodbus 数据区为普通内存对象，本仿真中"单写多读"的并发访问
    在 CPython GIL 保护下足够安全；真实工程应使用带锁的数据块。

用法：
    python -m plc_link.modbus_server                    # 默认 127.0.0.1:5020
    python -m plc_link.modbus_server --port 15020 --interval 1.0
验证：
    另开一个终端运行 python -m plc_link.modbus_client_test --port 5020
"""

from __future__ import annotations

import argparse
import threading
import time

from pymodbus.datastore import (ModbusSequentialDataBlock,
                                ModbusServerContext,
                                ModbusSlaveContext)
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.server import StartTcpServer

from control.pid import PIDController
from process.plant import build_default_plant

# ---------------------------------------------------------------------------
# 寄存器地址与报警位定义
# ---------------------------------------------------------------------------
HR_SP = 0        # 设定值（×100）
HR_PV = 1        # 测量值（×100）
HR_OP = 2        # 输出值（×100）
HR_MODE = 3      # 1=自动 0=手动
HR_ALM = 4       # 报警码
HR_HB = 5        # 心跳
N_REGS = 100     # 每个 slave 保持寄存器区长度

FC_HOLDING = 3           # 保持寄存器对应的功能码（pymodbus 内部寻址用）
ALM_BIT_HIGH = 1         # bit0：PV 高限
ALM_BIT_LOW = 2          # bit1：PV 低限
ALM_BIT_DEV = 4          # bit2：SP/PV 偏差报警

# 各回路配置：工艺基准、整定参数（Z-N 工程修正值，见 docs/整定对比报告.md）、
# 报警限值——均为典型经验值，仅用于仿真验证。
LOOP_CONFIG = {
    "dosing": {
        "unit": 1, "name": "加药回路-出水余氯",
        "sp0": 1.0,                              # 初始设定值 mg/L
        "kp": 31.0, "ti": 87.0, "td": 14.5,
        "deadband": 0.005,
        "high": 4.5, "low": 0.2, "dev": 0.5,     # 报警限 mg/L
    },
    "aeration": {
        "unit": 2, "name": "曝气回路-溶解氧",
        "sp0": 2.5, "kp": 32.1, "ti": 42.0, "td": 7.0,
        "deadband": 0.01,
        "high": 8.0, "low": 1.0, "dev": 0.6,
    },
}


class PLCRuntime:
    """单个回路的运行时：过程模型 + PID + 报警判断 + 寄存器同步状态。

    tick() 每个控制周期调用一次，完成「采样 → 运算 → 输出 → 上送」。
    """

    def __init__(self, key: str) -> None:
        cfg = LOOP_CONFIG[key]
        self.key = key
        self.cfg = cfg
        self.plant = build_default_plant()
        self.loop = getattr(self.plant, key)          # 本回路过程对象
        other = "aeration" if key == "dosing" else "dosing"
        self.other_loop = getattr(self.plant, other)  # 另一回路（同步推进保持活性）
        self.pid = PIDController(
            kp=cfg["kp"], ti=cfg["ti"], td=cfg["td"],
            dt=1.0, out_min=self.loop.op_min, out_max=self.loop.op_max,
            deadband=cfg["deadband"],
        )
        self.sp = float(cfg["sp0"])
        self.mode_auto = True
        self.manual_op = float(self.loop.op_min)
        self.heartbeat = 0
        self.alarm_code = 0
        # 报警延时确认计数器（连续 5 个周期越限才报警，防瞬间越限误报）
        self._alm_high_t = 0
        self._alm_low_t = 0
        self._alm_dev_t = 0

    # ---------------- 上位机接口（经寄存器或看板调用） ----------------
    def set_sp(self, sp: float) -> None:
        """下发设定值（工程单位）。"""
        if sp < 0:
            raise ValueError("设定值不能为负")
        self.sp = float(sp)

    def set_mode(self, auto: bool) -> None:
        """手/自动切换（无扰：控制器内部已做跟踪）。"""
        if auto and not self.mode_auto:
            self.pid.set_auto()
            self.mode_auto = True
        elif not auto and self.mode_auto:
            current_op = self.pid.get_state()["output"]
            self.pid.set_manual(current_op)
            self.manual_op = current_op
            self.mode_auto = False

    def write_manual(self, op: float) -> None:
        """手动模式下的软手操。"""
        if self.mode_auto:
            raise RuntimeError("自动模式下禁止软手操，请先切手动")
        self.pid.write_manual(op)
        self.manual_op = op

    def change_tuning(self, kp: float, ti: float, td: float) -> None:
        """在线修改 P/I/D 参数（无需重启）。"""
        self.pid.change_tuning(kp, ti, td)

    # ------------------------------------------------------------------
    def _eval_alarm(self) -> int:
        """报警判断：连续 5 个周期越限才置位（延时确认消抖）。

        用真值而非带噪声测量值判断，避免噪声抖动引起报警闪发。
        """
        pv = self.loop.true_pv
        code = 0
        if pv > self.cfg["high"]:
            self._alm_high_t += 1
        else:
            self._alm_high_t = 0
        if pv < self.cfg["low"]:
            self._alm_low_t += 1
        else:
            self._alm_low_t = 0
        if abs(pv - self.sp) > self.cfg["dev"]:
            self._alm_dev_t += 1
        else:
            self._alm_dev_t = 0
        if self._alm_high_t >= 5:
            code |= ALM_BIT_HIGH
        if self._alm_low_t >= 5:
            code |= ALM_BIT_LOW
        if self._alm_dev_t >= 5:
            code |= ALM_BIT_DEV
        return code

    def tick(self) -> dict:
        """推进一个控制周期，返回本回路的测量值与输出值。"""
        pv = self.loop.pv
        # 手动模式下 update() 内部只做积分跟踪并返回手操值；
        # 自动模式执行完整增量式 PID。
        op = self.pid.update(self.sp, pv)
        if not self.mode_auto:
            op = self.manual_op
        if self.key == "dosing":
            out = self.plant.step(op, 0.0)
        else:
            out = self.plant.step(0.0, op)
        # 让另一回路也推进一步（自然衰减），保持两回路时钟一致
        if self.key == "dosing":
            self.plant.aeration.step(0.0)
        else:
            self.plant.dosing.step(0.0)
        self.heartbeat = (self.heartbeat + 1) % 65536
        self.alarm_code = self._eval_alarm()
        return {"pv": out[f"{self.key}_pv"], "op": op}


def scale_to_reg(value: float) -> int:
    """工程值 -> 寄存器整数（×100 定标）。"""
    return int(round(value * 100))


def scale_from_reg(reg: int) -> float:
    """寄存器整数 -> 工程值。"""
    return reg / 100.0


def build_context() -> tuple[ModbusServerContext, dict[int, ModbusSlaveContext]]:
    """构建 pymodbus 数据区：Unit1=加药、Unit2=曝气。

    zero_mode=True 保证寄存器地址与客户端地址一致（HR0 <-> 地址 0），
    这是本项目在 pymodbus 3.6.x 下实测验证过的约定。
    """
    slaves: dict[int, ModbusSlaveContext] = {}
    for unit in (1, 2):
        block = ModbusSequentialDataBlock(0, [0] * N_REGS)
        slaves[unit] = ModbusSlaveContext(hr=block, zero_mode=True)
    ctx = ModbusServerContext(slaves=slaves, single=False)
    return ctx, slaves


def sync_regs_to_runtime(slave: ModbusSlaveContext, rt: PLCRuntime) -> None:
    """从站数据区 -> 运行时：读取上位机写入的 SP 与手/自动模式。"""
    sp_reg = slave.getValues(FC_HOLDING, HR_SP, count=1)[0]
    mode_reg = slave.getValues(FC_HOLDING, HR_MODE, count=1)[0]
    rt.set_sp(scale_from_reg(sp_reg))
    want_auto = bool(mode_reg == 1)
    if want_auto != rt.mode_auto:
        rt.set_mode(want_auto)


def sync_runtime_to_regs(slave: ModbusSlaveContext, rt: PLCRuntime) -> None:
    """运行时 -> 从站数据区：刷新 PV / OP / 报警码 / 心跳。"""
    state = rt.pid.get_state()
    slave.setValues(FC_HOLDING, HR_PV, [scale_to_reg(rt.loop.pv)])
    slave.setValues(FC_HOLDING, HR_OP, [scale_to_reg(state["output"])])
    slave.setValues(FC_HOLDING, HR_ALM, [rt.alarm_code])
    slave.setValues(FC_HOLDING, HR_HB, [rt.heartbeat])


def main() -> None:
    """入口：解析参数 → 建数据区 → 起仿真线程 → 阻塞运行从站服务。"""
    parser = argparse.ArgumentParser(
        description="水处理双回路 PID 仿真 Modbus/TCP 从站")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址")
    parser.add_argument("--port", type=int, default=5020, help="监听端口")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="控制周期(秒)，默认 1s 实时运行")
    args = parser.parse_args()

    ctx, slaves = build_context()

    # 两个回路的运行时
    runtimes = {key: PLCRuntime(key) for key in ("dosing", "aeration")}
    unit_map = {LOOP_CONFIG[key]["unit"]: (key, runtimes[key])
                for key in runtimes}

    # 初始化各从站的 SP / MODE 寄存器初值
    for key, rt in runtimes.items():
        slave = slaves[rt.cfg["unit"]]
        slave.setValues(FC_HOLDING, HR_SP, [scale_to_reg(rt.sp)])
        slave.setValues(FC_HOLDING, HR_MODE, [1])  # 默认自动

    def sim_loop() -> None:
        """后台仿真线程：每 interval 秒完成一次「读寄存器→运算→写寄存器」。"""
        next_t = time.time()
        while True:
            for unit, (key, rt) in unit_map.items():
                slave = slaves[unit]
                sync_regs_to_runtime(slave, rt)      # 先收上位机给定
                rt.tick()                            # 推进仿真与控制
                sync_runtime_to_regs(slave, rt)      # 再刷新只读数据
            next_t += args.interval
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.time()                 # 处理器慢时重新对齐节拍

    th = threading.Thread(target=sim_loop, name="plc-sim", daemon=True)
    th.start()

    identity = ModbusDeviceIdentification()
    identity.VendorName = "水处理PID仿真"
    identity.ProductCode = "PID-SIM-2L"
    identity.VendorUrl = "https://localhost"
    identity.ProductName = "加药/曝气回路 Modbus 从站模拟"
    identity.ModelName = "FOPDT-PID"
    identity.MajorMinorRevision = "1.0"

    print("=" * 64)
    print(f"Modbus/TCP 从站已启动：{args.host}:{args.port}")
    print("  Unit 1 = 加药回路(余氯)   Unit 2 = 曝气回路(溶解氧)")
    print("  寄存器：HR0=SP×100 HR1=PV×100 HR2=OP×100 "
          "HR3=自/手动 HR4=报警码 HR5=心跳")
    print("  Ctrl+C 退出")
    print("=" * 64)
    try:
        StartTcpServer(context=ctx, address=(args.host, args.port),
                       identity=identity)
    except KeyboardInterrupt:
        print("\n从站已停止")


if __name__ == "__main__":
    main()
