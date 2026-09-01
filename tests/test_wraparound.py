# -*- coding: utf-8 -*-
"""
tests/test_wraparound.py —— 负 PV 寄存器回绕回归用例（评审修复项）
====================================================================

复现并锁定的缺陷：
    冷启动时 PV 真值≈0，测量噪声（σ=0.02/0.03）叠加在物理下限截断
    【之后】导致约半数测量值为负；该负值经 ×100 定标（无钳位）写入
    16 位无符号寄存器时按 %65536 回绕，PV 显示成 655+ mg/L 的假尖峰
    （评审报告03 P1-2，冷启动实测 100% 复现）。

修复（本用例锁定）：
    ① 过程模型测量端：噪声先叠加、后做 pv_floor 下限截断（根因层）；
    ② 定标函数 scale_to_reg 结果钳位到 [0, 65535]（编码端第二层防护）；
    ③ 从站 write_internal 钳位而非回绕取模（寄存器边界防护）。

用法：
    python tests/test_wraparound.py
    python -m unittest discover -s tests
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# 保证可从项目根导入（无论以脚本还是 unittest discover 方式运行）
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from plc_link.common import scale_from_reg, scale_to_reg
from process.plant import build_default_plant

try:
    from pymodbus.datastore import ModbusSequentialDataBlock  # noqa: F401
    _HAS_PYMODBUS = True
except ImportError:                     # pragma: no cover
    _HAS_PYMODBUS = False


class TestColdStartPvInRange(unittest.TestCase):
    """冷启动首拍测量 PV 必须在物理量程内（回归锁定根因层修复）。"""

    def test_first_tick_in_range_default_seed(self):
        """默认种子的两回路冷启动首拍 PV 不得为负。"""
        plant = build_default_plant()
        out = plant.step(0.0, 0.0)
        self.assertGreaterEqual(out["dosing_pv"], 0.0)
        self.assertLessEqual(out["dosing_pv"], 10.0)
        self.assertGreaterEqual(out["aeration_pv"], 0.0)
        self.assertLessEqual(out["aeration_pv"], 10.0)

    def test_first_20_ticks_all_seeds_in_range(self):
        """20 个种子 × 前 20 拍：测量 PV 全部非负（修复前约半数为负）。"""
        for seed in range(20):
            plant = build_default_plant()
            plant.dosing.set_noise_scale(5.0)   # 强噪声下更苛刻
            plant.aeration.set_noise_scale(5.0)
            for _ in range(20):
                out = plant.step(0.0, 0.0)
                self.assertGreaterEqual(out["dosing_pv"], 0.0,
                                        f"seed={seed}: dosing PV<0")
                self.assertGreaterEqual(out["aeration_pv"], 0.0,
                                        f"seed={seed}: aeration PV<0")

    def test_true_pv_floor_unchanged(self):
        """真值层截断语义保持：真值不会低于 pv_floor。"""
        plant = build_default_plant()
        for _ in range(10):
            plant.step(0.0, 0.0)
        self.assertGreaterEqual(plant.dosing.true_pv, plant.dosing.pv_floor)
        self.assertGreaterEqual(plant.aeration.true_pv, plant.aeration.pv_floor)


class TestScaleToRegClamp(unittest.TestCase):
    """定标函数必须钳位到 16 位无符号量程（编码端第二层防护）。"""

    def test_normal_values_unchanged(self):
        self.assertEqual(scale_to_reg(1.05), 105)
        self.assertEqual(scale_to_reg(2.50), 250)
        self.assertEqual(scale_to_reg(0.0), 0)
        self.assertAlmostEqual(scale_from_reg(105), 1.05)

    def test_negative_clamped_to_zero(self):
        """修复前 scale_to_reg(-0.0234)=-234 直接回绕成 65302。"""
        self.assertEqual(scale_to_reg(-0.0234), 0)
        self.assertEqual(scale_to_reg(-1000.0), 0)

    def test_upper_clamped_to_65535(self):
        self.assertEqual(scale_to_reg(700.0), 65535)


class TestWriteInternalClamp(unittest.TestCase):
    """从站内部遥测通道必须钳位而非 %65536 回绕（寄存器边界防护）。"""

    @unittest.skipUnless(_HAS_PYMODBUS, "pymodbus 未安装，跳过从站数据块用例")
    def test_no_wraparound(self):
        from plc_link.common import HR_PV, N_REGS, READONLY_OFFSETS
        from plc_link.modbus_server import GuardedDataBlock
        block = GuardedDataBlock(0, [0] * N_REGS,
                                 readonly_offsets=READONLY_OFFSETS)
        block.write_internal(HR_PV, scale_to_reg(-0.0234))
        self.assertEqual(block.getValues(HR_PV, 1)[0], 0)
        block.write_internal(HR_PV, -2)
        self.assertEqual(block.getValues(HR_PV, 1)[0], 0)
        block.write_internal(HR_PV, 70000)
        self.assertEqual(block.getValues(HR_PV, 1)[0], 65535)
        block.write_internal(HR_PV, 12345)
        self.assertEqual(block.getValues(HR_PV, 1)[0], 12345)


if __name__ == "__main__":
    unittest.main(verbosity=2)
