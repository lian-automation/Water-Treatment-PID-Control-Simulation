# -*- coding: utf-8 -*-
"""
plc_link/modbus_client_test.py —— Modbus/TCP 主站自测脚本
====================================================================

职责：
    作为 Modbus/TCP 主站（模拟上位机/DCS 通信模块）对从站做全项目
    通信验收测试，逐项检查并打印 PASS/FAIL，任一项失败则退出码为 1。

测试项：
    1) 连接从站（Unit1=加药、Unit2=曝气）；
    2) 读 HR0~HR5，检查心跳在增长（从站仿真线程存活）；
    3) PV/OP 寄存器数值在合理范围内（0.01 定标）；
    4) 写 HR3=0（切手动）→ 回读校验；再写 HR3=1（切回自动）；
    5) 写 HR0（SP 下调 30% 阶跃）→ 回读校验（原值在判据完成后恢复）；
    6) SP 下调阶跃后控制器输出 OP 必须即时下降（"闭环在调"判据）；
    7) 只读写保护离线验证：HR1/HR2/HR4/HR5 外部写被数据块拒绝，
       内部遥测通道正常，批量写触及只读区时整段拒绝。
    注意 4)/5) 的先后次序不可颠倒：SP 阶跃必须发生在自动模式下且
    其后无手动保持——若阶跃写后紧跟切手动，从站下一拍落在手动
    "输出保持"里，比例增量被无扰切换吞掉（e1 在手动期间照常前移），
    OP 判据只能看到积分回拉（实机实测 Δ=+2.09 假FAIL，教训已固化
    为本时序约定）。

"闭环在调"判据说明（判据修订记录）：
    旧判据为"SP 下发 +10% 后等 8s，PV 向新 SP 靠拢"。加药回路
    τ=30s、T=120s，8s 观察窗落在过程纯滞后死区内，PV 尚未启动
    响应，判据实际在比较两个独立噪声采样（σ=0.02）到 SP 的距离，
    通过与否接近掷硬币——修订前对真实从站实测 3 次运行 2 次加药
    FAIL、1 次双回路 FAIL，而 60s 轨迹探针证明回路本身在调
    （PV 1.06→1.72），问题在判据窗口不在控制回路。
    新判据改验控制器输出 OP 对 SP 阶跃的响应：
      * 即时性：增量式 PID 的比例增量 dP = Kp×(e[k]-e[k-1]) 与 SP
        阶跃同拍生效，不经过过程滞后（PV 要等 τ+T 才动，OP 当拍
        就动），观察窗 3s + 3 次采样即可确定性地覆盖；
      * 幅度确定性：SP 下调 30% 时比例增量 ≈ -Kp×30%×SP ≈ -9.3%
        （加药）/ -24 Hz（曝气）。阶跃取下调 30% 而非 ±10%，是因
        为冷启动时控制器处于深饱和/大误差工况（曝气 OP 顶格 50Hz，
        加药 OP≈32% 爬坡中），积分项以 Kp/Ti×e 的速率把 OP 往回
        拉，±10% 小阶跃的 OP 响应会在观察窗内被积分回拉吞掉
        （离线仿真实测 Δ 仅 -0.6~-1.6，与阈值同量级）；
      * 抗噪声：OP 单拍抖动来自 Kp 对测量噪声的放大（单拍 σ≈1.5~
        2.5），判据取基准/响应窗口各 3 次采样（间隔 1s）的均值差，
        阈值 OP_DELTA_MIN=2.5 为执行器量化步长（0.01，寄存器
        1 LSB）的 250 倍；离线压测（冷启动/预热 × 双回路 × 400 次，
        调度抖动 ±0.5 拍 + 随机噪声种子）最坏组最小响应 -3.9、
        0 次误判；实机冷启动连跑 5 次全过（记录见 docs/验收清单 5C）；
      * 方向性：SP 下调，OP 均值必须下降 ≥ 阈值——排除噪声单向
        上偏造成的假通过；若从站控制线程停摆（OP 冻结），Δ≈0 必判
        FAIL，判据保有甄别力。

用法：
    先启动从站：python -m plc_link.modbus_server --port 5020
    再运行本测试：python -m plc_link.modbus_client_test --port 5020
"""

from __future__ import annotations

import argparse
import sys
import time

from pymodbus.client import ModbusTcpClient

# 寄存器定义与从站同源：单一事实来源在 plc_link/common.py（评审修复项）
from plc_link.common import (HR_ALM, HR_HB, HR_MANUAL, HR_MODE, HR_OP,
                             HR_PV, HR_SP, LOOP_NAMES, N_REGS,
                             READONLY_OFFSETS)

UNIT_NAMES = {1: LOOP_NAMES["dosing"], 2: LOOP_NAMES["aeration"]}

# ---- "闭环在调"OP 响应判据参数（判据修订，依据见模块 docstring）----
SP_STEP_FRAC = 0.30    # SP 下调阶跃幅度（30%）：保证冷启动深饱和工况下
                       # OP 响应幅度（≥Kp×30%×SP）远大于积分回拉与噪声
OP_BASE_SAMPLES = 3    # 基准窗口采样数（SP 阶跃前，间隔 1s=控制周期）
OP_RESP_SAMPLES = 3    # 响应窗口采样数（阶跃沉降后，间隔 1s）
OP_SETTLE_S = 3.0      # 阶跃后沉降等待（覆盖从站 2~3 个控制周期）
OP_DELTA_MIN = 2.5     # OP 均值下降阈值（工程单位 %/Hz；寄存器 250 LSB，
                       # 为执行器量化步长 0.01 的 250 倍）

_results: list[tuple[str, bool, str]] = []


def check(item: str, ok: bool, detail: str = "") -> None:
    """记录并打印一条测试结果。"""
    _results.append((item, ok, detail))
    mark = "PASS" if ok else "FAIL"
    print(f"[{mark}] {item}" + (f"  ({detail})" if detail else ""))


def read_regs(client: ModbusTcpClient, unit: int,
              retries: int = 3) -> list[int] | None:
    """读 Unit 的 HR0~HR5；偶发通信失败自动重试，最终失败返回 None。

    说明：工业现场总线上偶发超时/异常响应是常态，主站程序必须重试，
    这也是本自测脚本模拟真实上位机的写法。
    """
    for attempt in range(retries):
        rr = client.read_holding_registers(HR_SP, count=6, slave=unit)
        if not rr.isError():
            return list(rr.registers)
        time.sleep(0.4)
    return None


def write_reg(client: ModbusTcpClient, unit: int,
              address: int, value: int, retries: int = 3) -> bool:
    """写单个保持寄存器，带重试。"""
    for attempt in range(retries):
        w = client.write_register(address, value, slave=unit)
        if not w.isError():
            time.sleep(0.2)   # 请求间稍作间隔，模拟真实主站节流
            return True
        time.sleep(0.4)
    return False


def guard_offline_check() -> None:
    """只读写保护离线单元验证：直接构造 GuardedDataBlock，不依赖网络。

    覆盖四个语义：
      ① 外部写只读寄存器（HR1）被拒绝；
      ② 从站内部通道刷新只读寄存器正常生效；
      ③ 可写区（HR0/HR3）外部写照常放行；
      ④ 批量写一旦触及任一只读寄存器则整段拒绝（防"部分写入"污染）。
    """
    # 延迟导入：避免在仅做网络测试时拉起整个从站模块
    from plc_link.modbus_server import GuardedDataBlock

    print("-" * 64)
    print("只读寄存器写保护：离线构造 GuardedDataBlock 验证")
    print("-" * 64)
    block = GuardedDataBlock(0, [0] * N_REGS,
                             readonly_offsets=READONLY_OFFSETS)

    block.setValues(HR_PV, [999])
    check("只读保护: 外部写 HR1(PV) 被拒绝",
          block.getValues(HR_PV, 1)[0] == 0, "写入后仍为初值 0")

    block.write_internal(HR_PV, 12345)
    check("只读保护: 内部遥测刷新 HR1 生效",
          block.getValues(HR_PV, 1)[0] == 12345, "0 -> 12345")

    block.setValues(HR_SP, [100])
    block.setValues(HR_MODE, [1])
    block.setValues(HR_MANUAL, [3500])   # HR8 手操值：可写区（评审修复新增）
    check("只读保护: 可写区 HR0/HR3/HR8 放行",
          block.getValues(HR_SP, 1)[0] == 100
          and block.getValues(HR_MODE, 1)[0] == 1
          and block.getValues(HR_MANUAL, 1)[0] == 3500,
          "SP=100 MODE=1 MVAL=3500")

    block.setValues(HR_MODE, [1, 7, 8])   # HR3~HR5，触及只读的 HR4/HR5
    rejected = (block.getValues(HR_MODE, 1)[0] == 1
                and block.getValues(HR_ALM, 1)[0] == 0
                and block.getValues(HR_HB, 1)[0] == 0)
    check("只读保护: 批量写触及只读区整段拒绝", rejected,
          "HR3 保持原值、HR4/HR5 未被写入")


def test_unit(client: ModbusTcpClient, unit: int) -> None:
    """对单个从站单元执行全部测试项。"""
    name = UNIT_NAMES[unit]
    print("-" * 64)
    print(f"开始测试 Unit {unit}（{name}）")
    print("-" * 64)

    regs0 = read_regs(client, unit)
    check(f"{name}: 读 HR0~HR5", regs0 is not None,
          f"HR={regs0}" if regs0 else "读取失败")
    if regs0 is None:
        return

    # 1) 心跳增长
    time.sleep(2.0)
    regs1 = read_regs(client, unit)
    check(f"{name}: 心跳增长(HR5)", regs1 is not None and regs1[HR_HB] > regs0[HR_HB],
          f"{regs0[HR_HB]} -> {regs1[HR_HB] if regs1 else '?'}")

    # 2) PV/OP 范围（×100 定标，PV 物理上 0~10mg/L，OP 0~10000）
    pv, op = regs1[HR_PV], regs1[HR_OP]
    check(f"{name}: PV/OP 在合理范围", 0 <= pv <= 1000 and 0 <= op <= 10000,
          f"PV={pv/100:.2f} OP={op/100:.2f}")

    # 3) OP 基准采样：SP 阶跃前与 regs1 共采 3 次（间隔 1s=控制周期），
    #    取均值压噪（单点 OP 抖动 σ≈1.5~2.5，见模块 docstring 判据说明）
    op_base = [regs1[HR_OP] / 100.0]
    for _ in range(OP_BASE_SAMPLES - 1):
        time.sleep(1.0)
        r = read_regs(client, unit)
        if r is not None:
            op_base.append(r[HR_OP] / 100.0)
    op_base_mean = sum(op_base) / len(op_base)

    # 4) 模式切换：手动 -> 回读 -> 自动（须先于 SP 阶跃，时序约定见 docstring）
    ok_w = write_reg(client, unit, HR_MODE, 0)
    m_regs = read_regs(client, unit)
    m0 = m_regs[HR_MODE] if m_regs else -1
    ok_w2 = write_reg(client, unit, HR_MODE, 1)
    m_regs = read_regs(client, unit)
    m1 = m_regs[HR_MODE] if m_regs else -1
    check(f"{name}: 手/自动模式写入回读", ok_w and ok_w2 and m0 == 0 and m1 == 1,
          f"手动回读={m0} 自动回读={m1}")

    # 5) SP 下发：下调 30% 阶跃并回读（自动模式下生效；恢复在判据之后）
    sp_old = regs1[HR_SP]
    sp_new = int(round(sp_old * (1 - SP_STEP_FRAC)))
    w_ok = write_reg(client, unit, HR_SP, sp_new)
    back = read_regs(client, unit)
    check(f"{name}: 写 HR0(SP 下调{SP_STEP_FRAC:.0%}) 回读一致",
          w_ok and back is not None and back[HR_SP] == sp_new,
          f"{sp_old/100:.2f} -> {sp_new/100:.2f}")

    # 6) 闭环验证：SP 下调阶跃后 OP 必须即时下降（判据说明见模块 docstring）
    print(f"  ... 等待 {OP_SETTLE_S:.0f}s 后采样 OP 响应"
          f"（{OP_RESP_SAMPLES} 次，间隔 1s）")
    time.sleep(OP_SETTLE_S)
    op_resp: list[float] = []
    for i in range(OP_RESP_SAMPLES):
        r = read_regs(client, unit)
        if r is not None:
            op_resp.append(r[HR_OP] / 100.0)
        if i < OP_RESP_SAMPLES - 1:
            time.sleep(1.0)
    op_resp_mean = (sum(op_resp) / len(op_resp)) if op_resp else float("nan")
    delta = op_resp_mean - op_base_mean
    check(f"{name}: SP 阶跃后 OP 即时响应(闭环在调)",
          len(op_base) == OP_BASE_SAMPLES
          and len(op_resp) == OP_RESP_SAMPLES
          and delta <= -OP_DELTA_MIN,
          f"OP均值 {op_base_mean:.2f} -> {op_resp_mean:.2f}"
          f"（Δ={delta:+.2f}，判据 Δ≤-{-OP_DELTA_MIN}，SP→{sp_new/100:.2f}）")

    # 7) 恢复原始 SP
    write_reg(client, unit, HR_SP, sp_old)
    print(f"  已恢复 SP={sp_old/100:.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Modbus/TCP 从站自测")
    parser.add_argument("--host", default="127.0.0.1", help="从站地址")
    parser.add_argument("--port", type=int, default=5020, help="从站端口")
    args = parser.parse_args()

    print("=" * 64)
    print(f"Modbus/TCP 主站自测：{args.host}:{args.port}")
    print("=" * 64)
    client = ModbusTcpClient(host=args.host, port=args.port)
    ok = client.connect()
    check("TCP 连接", ok, f"{args.host}:{args.port}")
    if not ok:
        print("无法连接从站，请先运行: python -m plc_link.modbus_server")
        sys.exit(1)

    try:
        for unit in sorted(UNIT_NAMES):
            test_unit(client, unit)
        guard_offline_check()
    finally:
        client.close()

    n_fail = sum(0 if r[1] else 1 for r in _results)
    print("=" * 64)
    print(f"测试完成：共 {len(_results)} 项，失败 {n_fail} 项")
    print("=" * 64)
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
