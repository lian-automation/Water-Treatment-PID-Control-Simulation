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

保持寄存器地址映射与只读保护约定（定义见 plc_link/common.py）：
    HR0  SP   设定值     读/写   ×100（如 1.05 mg/L -> 105）
    HR1  PV   测量值     只读    ×100，外部写被数据块拒绝
    HR2  OP   输出值     只读    ×100，外部写被数据块拒绝
    HR3  MODE 自/手动    读/写   1=自动 0=手动
    HR4  ALM  报警码     只读    bit0高限/bit1低限/bit2偏差，外部写被拒
    HR5  HB   心跳       只读    每周期 +1 mod 65536，外部写被拒

线程模型：
    主线程：StartTcpServer 阻塞服务；
    后台 daemon 线程：按控制周期(默认 1s)「读给定→运算→刷新遥测」。
    说明：pymodbus 数据区为普通内存对象，本仿真"单写多读"并发在
    CPython GIL 下足够安全；真实工程应使用带锁的数据块。

用法：
    python -m plc_link.modbus_server                    # 默认 127.0.0.1:5020
    python -m plc_link.modbus_server --port 15020 --interval 1.0
验证：
    另开终端运行 python -m plc_link.modbus_client_test --port 5020
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
from plc_link.common import (AlarmMonitor, FC_HOLDING, HR_ALM, HR_HB,
                             HR_MODE, HR_OP, HR_PV, HR_SP, LOOP_CONFIG,
                             N_REGS, READONLY_OFFSETS, scale_from_reg,
                             scale_to_reg)
from process.plant import build_default_plant

# 兼容旧引用：报警位常量仍可从本模块导入（单一来源在 common）
__all__ = ["PLCRuntime", "build_context", "main"]


class GuardedDataBlock(ModbusSequentialDataBlock):
    """带只读区保护的保持寄存器数据块。

    外部 Modbus 写请求若落入只读偏移（PV/OP/报警码/心跳），直接静默
    忽略——真实工程应返回"非法地址/写失败"异常响应并留审计日志；
    从站内部遥测刷新走 write_internal()，绕过保护直写存储区。
    """

    def __init__(self, address: int, values: list,
                 readonly_offsets: tuple = ()) -> None:
        super().__init__(address, values)
        self.readonly_offsets = tuple(readonly_offsets)

    def _overlaps_readonly(self, address: int, count: int) -> bool:
        """判断 [address, address+count) 是否触及任一只读寄存器。"""
        start = address - self.address          # 转为块内相对偏移
        end = start + max(int(count), 1)
        return any(start <= off < end for off in self.readonly_offsets)

    # pymodbus 处理主站写请求时调用此方法（0x06/0x10 功能码路径）
    def setValues(self, address: int, values) -> None:
        count = len(values) if hasattr(values, "__len__") else 1
        if self._overlaps_readonly(address, count):
            return                              # 静默忽略非法写
        super().setValues(address, values)

    def write_internal(self, address: int, value: int) -> None:
        """从站内部遥测写入：不受只读保护限制。"""
        idx = address - self.address
        if 0 <= idx < len(self.values):
            self.values[idx] = int(value) % 65536


class PLCRuntime:
    """单个回路的运行时：过程模型 + PID + 报警判定 + 寄存器同步状态。

    tick() 每个控制周期调用一次，完成「采样 → 运算 → 输出 → 上送」。
    注意：plant.step() 每次会同时推进两个回路各一步（共享同一仿真
    时钟），因此本方法不再对另一回路额外步进——否则会造成非受控
    回路时钟双倍快跑（代码评审发现并修复的缺陷）。
    """

    def __init__(self, key: str) -> None:
        cfg = LOOP_CONFIG[key]
        self.key = key
        self.cfg = cfg
        self.plant = build_default_plant()
        self.loop = getattr(self.plant, key)          # 本回路过程对象
        other = "aeration" if key == "dosing" else "dosing"
        self.other_loop = getattr(self.plant, other)  # 另一回路引用（诊断用）
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
        # 越限延时确认报警器（连续 5 拍越限才置位，逻辑单点在 common）
        self.alarm_monitor = AlarmMonitor(cfg["high"], cfg["low"],
                                          cfg["dev"], confirm=5)

    # ---------------- 上位机接口 ----------------
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
    def tick(self) -> dict:
        """推进一个控制周期，返回本回路的测量值与输出值。

        手动模式下 update() 内部只做积分跟踪并原样返回手操值；
        自动模式执行完整增量式 PID——两种模式都无需再补一次输出赋值。
        """
        pv = self.loop.pv
        op = self.pid.update(self.sp, pv)
        if self.key == "dosing":
            out = self.plant.step(op, 0.0)
        else:
            out = self.plant.step(0.0, op)
        self.heartbeat = (self.heartbeat + 1) % 65536
        # 报警判定用真值 PV，隔离测量噪声（消抖逻辑在 common.AlarmMonitor）
        self.alarm_code = self.alarm_monitor.evaluate(
            self.loop.true_pv, self.sp)
        return {"pv": out[f"{self.key}_pv"], "op": op}


def build_context() -> tuple[ModbusServerContext,
                             dict[int, ModbusSlaveContext],
                             dict[int, GuardedDataBlock]]:
    """构建 pymodbus 数据区：Unit1=加药、Unit2=曝气。

    返回 (context, slaves, blocks)：blocks 是底层数据块的直接引用，
    供从站内部绕过只读保护刷新 PV/OP/报警码/心跳。
    zero_mode=True 保证寄存器地址与客户端地址一致（HR0 <-> 地址 0），
    为本项目在 pymodbus 3.6.x 下实测验证过的约定。
    """
    slaves: dict[int, ModbusSlaveContext] = {}
    blocks: dict[int, GuardedDataBlock] = {}
    for unit in (1, 2):
        block = GuardedDataBlock(0, [0] * N_REGS,
                                 readonly_offsets=READONLY_OFFSETS)
        blocks[unit] = block
        slaves[unit] = ModbusSlaveContext(hr=block, zero_mode=True)
    ctx = ModbusServerContext(slaves=slaves, single=False)
    return ctx, slaves, blocks


def sync_regs_to_runtime(slave: ModbusSlaveContext, rt: PLCRuntime) -> None:
    """从站数据区 -> 运行时：读取上位机写入的 SP 与手/自动模式。"""
    sp_reg = slave.getValues(FC_HOLDING, HR_SP, count=1)[0]
    mode_reg = slave.getValues(FC_HOLDING, HR_MODE, count=1)[0]
    rt.set_sp(scale_from_reg(sp_reg))
    want_auto = bool(mode_reg == 1)
    if want_auto != rt.mode_auto:
        rt.set_mode(want_auto)


def sync_runtime_to_regs(block: GuardedDataBlock, rt: PLCRuntime) -> None:
    """运行时 -> 从站数据区：经内部通道刷新 PV / OP / 报警码 / 心跳。

    使用 write_internal 绕过只读保护（这些寄存器对外部主站是只读的）。
    """
    state = rt.pid.get_state()
    block.write_internal(HR_PV, scale_to_reg(rt.loop.pv))
    block.write_internal(HR_OP, scale_to_reg(state["output"]))
    block.write_internal(HR_ALM, rt.alarm_code)
    block.write_internal(HR_HB, rt.heartbeat)


def main() -> None:
    """入口：解析参数 → 建数据区 → 起仿真线程 → 阻塞运行从站服务。"""
    parser = argparse.ArgumentParser(
        description="水处理双回路 PID 仿真 Modbus/TCP 从站")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址")
    parser.add_argument("--port", type=int, default=5020, help="监听端口")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="控制周期(秒)，默认 1s 实时运行")
    args = parser.parse_args()

    ctx, slaves, blocks = build_context()

    # 两个回路的运行时
    runtimes = {key: PLCRuntime(key) for key in ("dosing", "aeration")}
    unit_map = {LOOP_CONFIG[key]["unit"]: (key, runtimes[key])
                for key in runtimes}

    # 初始化各从站的 SP / MODE 寄存器初值（两者均可写，走公共入口即可）
    for key, rt in runtimes.items():
        slave = slaves[rt.cfg["unit"]]
        slave.setValues(FC_HOLDING, HR_SP, [scale_to_reg(rt.sp)])
        slave.setValues(FC_HOLDING, HR_MODE, [1])  # 默认自动

    def sim_loop() -> None:
        """后台仿真线程：每 interval 秒完成一次「收给定→运算→刷遥测」。"""
        next_t = time.time()
        while True:
            for unit, (key, rt) in unit_map.items():
                sync_regs_to_runtime(slaves[unit], rt)   # 先收上位机给定
                rt.tick()                                # 推进仿真与控制
                sync_runtime_to_regs(blocks[unit], rt)   # 内部通道刷遥测
            next_t += args.interval
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.time()                     # 慢机重对齐节拍

    th = threading.Thread(target=sim_loop, name="plc-sim", daemon=True)
    th.start()

    identity = ModbusDeviceIdentification()
    identity.VendorName = "水处理PID仿真"
    identity.ProductCode = "PID-SIM-2L"
    identity.VendorUrl = "https://localhost"
    identity.ProductName = "加药/曝气回路 Modbus 从站模拟"
    identity.ModelName = "FOPDT-PID"
    identity.MajorMinorRevision = "1.1"

    print("=" * 64)
    print(f"Modbus/TCP 从站已启动：{args.host}:{args.port}")
    print("  Unit 1 = 加药回路(余氯)   Unit 2 = 曝气回路(溶解氧)")
    print("  寄存器：HR0=SP×100 HR1=PV×100 HR2=OP×100 "
          "HR3=自/手动 HR4=报警码 HR5=心跳")
    print("  只读保护：HR1/HR2/HR4/HR5 的外部写请求被数据块拒绝")
    print("  Ctrl+C 退出")
    print("=" * 64)
    try:
        StartTcpServer(context=ctx, address=(args.host, args.port),
                       identity=identity)
    except KeyboardInterrupt:
        print("\n从站已停止")


if __name__ == "__main__":
    main()
