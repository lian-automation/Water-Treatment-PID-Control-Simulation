# -*- coding: utf-8 -*-
"""
process/plant.py —— 水处理过程对象模型（FOPDT 仿真）
====================================================================

职责：
    在没有真实仪表和 PLC 的条件下，用「一阶惯性 + 纯滞后」(FOPDT,
    First Order Plus Dead Time) 数学模型模拟两个独立受控回路：

      ① 加药回路：变频加药泵开度 OP(%)  -> 出水余氯 PV(mg/L)
      ② 曝气回路：鼓风机频率   OP(Hz)  -> 溶解氧    PV(mg/L)

建模依据（参数均为典型经验值，仅用于仿真验证，不代表任何真实水厂）：
    连续形式：
        T * dPV(t)/dt + PV(t) = K * OP(t - tau) + D(t)
    其中：
        K     过程增益（OP 每变化 1 单位，PV 稳态变化多少单位）
        T     时间常数(s)，惯性大小
        tau   纯滞后(s)，执行器/管路传输与混合延迟
        D(t)  负荷扰动项（如进水流量/水质波动），经一阶滤波后叠加
    离散化（采样周期 dt=1s，采用指数保持离散，数值无条件稳定）：
        a = exp(-dt / T)
        PV[k+1] = a * PV[k] + (1 - a) * (K * OP[k - round(tau/dt)] + D[k])
    测量噪声：
        PV_meas = PV_true + N(0, noise_std)   （高斯白噪声，模拟仪表测量误差）

外部接口：
    loop.step(op)              单步推进 dt 秒，返回叠加噪声后的测量 PV
    loop.apply_load_step(d%)   扰动注入函数：进水负荷阶跃（如 +30 表示 +30%）
    loop.set_noise_scale(x)    测量噪声倍乘系数（用于"噪声干扰"测试场景）
    loop.reset()               复位到初始状态
"""

from __future__ import annotations

import math
import random
from collections import deque

# ---------------------------------------------------------------------------
# 全局默认仿真步长（秒）。题目要求 dt = 1s。
# ---------------------------------------------------------------------------
DT_DEFAULT = 1.0


class FOPDTLoop:
    """单回路 FOPDT 过程仿真对象。

    一个实例只描述「一个操作量 OP -> 一个被控量 PV」的通道，
    加药回路和曝气回路各用一个实例，互不耦合（双独立回路）。
    """

    def __init__(
        self,
        name: str,
        pv_unit: str,
        op_unit: str,
        k: float,
        t: float,
        tau: float,
        op_min: float = 0.0,
        op_max: float = 100.0,
        pv0: float = 0.0,
        pv_floor: float = 0.0,
        noise_std: float = 0.02,
        load_gain: float = 0.0,
        load_filter_t: float = 60.0,
        load0: float = 100.0,
        dt: float = DT_DEFAULT,
        seed: int = 42,
    ) -> None:
        """
        参数说明（均为典型经验值）：
            name           回路名称，如 "加药回路(余氯)"
            pv_unit        被控量工程单位，如 "mg/L"
            op_unit        操作量工程单位，如 "%" 或 "Hz"
            k              过程增益：PV稳态变化 / OP单位变化
            t              时间常数(s)
            tau            纯滞后(s)
            op_min/op_max  操作量物理限幅（加药泵 0~100%，风机 0~50 Hz）
            pv0            初始 PV 值
            pv_floor       PV 物理下限（浓度/溶解氧不可能为负），仿真中截断
            noise_std      测量高斯噪声标准差（与 pv 同单位）
            load_gain      负荷扰动增益：负荷每偏离基准 +1%，PV 稳态变化量
                           （进水流量增大 → 余氯被稀释、耗氧增加，故为负值）
            load_filter_t  负荷扰动对 PV 影响的一阶滤波时间常数(s)
            load0          初始进水负荷(%，100 为设计基准负荷)
            dt             仿真步长(s)，默认 1s
            seed           随机数种子（保证实验可复现）
        """
        # ---- 模型参数 -------------------------------------------------------
        self.name = name
        self.pv_unit = pv_unit
        self.op_unit = op_unit
        self.k = float(k)
        self.t = float(t)
        self.tau = float(tau)
        self.op_min = float(op_min)
        self.op_max = float(op_max)
        self.pv0 = float(pv0)
        self.pv_floor = float(pv_floor)
        self.noise_std = float(noise_std)
        self.load_gain = float(load_gain)
        self.load_filter_t = max(float(load_filter_t), 1e-6)
        self.dt = float(dt)

        # ---- 运行状态 -------------------------------------------------------
        self._load = float(load0)          # 当前进水负荷(%)
        self._dist_state = self.load_gain * (self._load - 100.0)  # 扰动滤波状态
        self._pv_true = float(pv0)         # 真实 PV（未加噪声）
        self._pv_meas = float(pv0)         # 测量 PV（叠加噪声，供控制器使用）
        self._sim_time = 0.0               # 本回路的仿真时钟(s)
        self.noise_scale = 1.0             # 噪声倍乘系数（场景3用）
        self._rng = random.Random(seed)    # 独立随机源，互不影响
        # 纯滞后用 OP 历史队列实现：队列长度 = 滞后步数 + 1，
        # 队尾是当前 OP，队首就是 round(tau/dt) 步前的 OP。
        delay_steps = int(round(self.tau / self.dt))
        if delay_steps < 1:
            delay_steps = 1  # 至少滞后 1 步，避免索引越界
        self.delay_steps = delay_steps
        self._op_buf: deque = deque([self.op_min] * (delay_steps + 1),
                                    maxlen=delay_steps + 1)

    # ------------------------------------------------------------------
    # 核心接口
    # ------------------------------------------------------------------
    def step(self, op: float) -> float:
        """推进一个采样周期 dt，输入操作量 op，返回带噪声的测量 PV。

        步骤：
            1) 操作量限幅到物理范围并压入滞后队列；
            2) 负荷扰动项经一阶滤波逼近其稳态影响；
            3) 一阶惯性环节按指数保持离散递推；
            4) 叠加高斯测量噪声后作为仪表读数返回。
        """
        # 1) OP 限幅 + 纯滞后
        op_clamped = min(max(op, self.op_min), self.op_max)
        self._op_buf.append(op_clamped)
        delayed_op = self._op_buf[0]  # tau 秒之前的 OP

        # 2) 负荷扰动一阶滤波：D 向 D_target = load_gain*(load-100) 靠近
        a_dist = math.exp(-self.dt / self.load_filter_t)
        dist_target = self.load_gain * (self._load - 100.0)
        self._dist_state += (1.0 - a_dist) * (dist_target - self._dist_state)

        # 3) 一阶惯性递推：PV 向稳态目标 (K*OP_delayed + D) 靠近
        a_pv = math.exp(-self.dt / self.t)
        steady_target = self.k * delayed_op + self._dist_state
        self._pv_true += (1.0 - a_pv) * (steady_target - self._pv_true)

        # 物理下限截断（余氯/溶解氧不可能为负）
        if self._pv_true < self.pv_floor:
            self._pv_true = self.pv_floor

        # 4) 测量噪声
        noise = self._rng.gauss(0.0, self.noise_std) * self.noise_scale
        self._pv_meas = self._pv_true + noise
        self._sim_time += self.dt
        return self._pv_meas

    def apply_load_step(self, delta_percent: float) -> None:
        """外部扰动注入：进水负荷阶跃。

        例如 apply_load_step(+30) 表示进水流量比设计基准增加 30%，
        对余氯回路表现为稀释（PV 稳态下降），对曝气回路表现为耗氧增加。
        """
        self._load += float(delta_percent)

    def set_noise_scale(self, scale: float) -> None:
        """设置测量噪声倍乘系数（1.0 为正常工况，场景3中放大到 5~8 倍）。"""
        self.noise_scale = float(scale)

    def reset(self) -> None:
        """复位到初始状态（保留模型参数与随机种子，重新排随机序列）。"""
        self._load = 100.0
        self._dist_state = self.load_gain * (self._load - 100.0)
        self._pv_true = self.pv0
        self._pv_meas = self.pv0
        self._sim_time = 0.0
        self.noise_scale = 1.0
        self._rng = random.Random(abs(hash(self.name)) % (2**32))
        self._op_buf = deque([self.op_min] * (self.delay_steps + 1),
                             maxlen=self.delay_steps + 1)

    # ------------------------------------------------------------------
    # 只读属性 / 辅助方法
    # ------------------------------------------------------------------
    @property
    def pv(self) -> float:
        """当前测量 PV（叠加噪声后的仪表读数）。"""
        return self._pv_meas

    @property
    def true_pv(self) -> float:
        """当前真实 PV（无噪声，仅用于评估指标，控制器不可见）。"""
        return self._pv_true

    @property
    def load(self) -> float:
        """当前进水负荷(%)。"""
        return self._load

    @property
    def sim_time(self) -> float:
        """本回路已推进的仿真时长(s)。"""
        return self._sim_time

    def snapshot_params(self, keep_noise: bool = False) -> dict:
        """导出可重建本回路的参数字典（供辨识/对比实验复制对象用）。

        默认关闭噪声（keep_noise=False），因为响应曲线辨识需要干净曲线。
        """
        return dict(
            name=self.name, pv_unit=self.pv_unit, op_unit=self.op_unit,
            k=self.k, t=self.t, tau=self.tau,
            op_min=self.op_min, op_max=self.op_max,
            pv0=self.pv0, pv_floor=self.pv_floor,
            noise_std=(self.noise_std if keep_noise else 0.0),
            load_gain=self.load_gain, load_filter_t=self.load_filter_t,
            load0=self._load, dt=self.dt,
        )

    def clone(self, keep_noise: bool = False, seed: int | None = None) -> "FOPDTLoop":
        """按当前参数复制一个新回路实例。"""
        params = self.snapshot_params(keep_noise=keep_noise)
        if seed is not None:
            params["seed"] = seed
        return FOPDTLoop(**params)


class ProcessPlant:
    """水处理过程厂：两个独立 FOPDT 回路的容器。

    加药回路与曝气回路在物理上相互独立（不同介质、不同执行器），
    但共享同一个"进水负荷"扰动源——下雨/高峰供水时进水量增大，
    会同时稀释余氯并增加曝气池耗氧，因此负荷阶跃默认同时注入两回路。
    """

    def __init__(self, dosing: FOPDTLoop, aeration: FOPDTLoop) -> None:
        self.dosing = dosing        # 加药回路（余氯）
        self.aeration = aeration    # 曝气回路（溶解氧）

    def step(self, op_dosing: float, op_aeration: float) -> dict:
        """双回路同步推进一步，返回测量值字典。"""
        return {
            "dosing_pv": self.dosing.step(op_dosing),      # 余氯测量值 mg/L
            "aeration_pv": self.aeration.step(op_aeration),  # 溶解氧测量值 mg/L
            "dosing_true": self.dosing.true_pv,
            "aeration_true": self.aeration.true_pv,
            "load": self.dosing.load,                      # 进水负荷 %
        }

    def apply_load_step(self, delta_percent: float, loop: str | None = None) -> None:
        """进水负荷阶跃扰动注入。

        loop=None 时同时注入两个回路（模拟同一进水量变化）；
        loop="dosing"/"aeration" 时只注入指定回路。
        """
        if loop is None or loop == "dosing":
            self.dosing.apply_load_step(delta_percent)
        if loop is None or loop == "aeration":
            self.aeration.apply_load_step(delta_percent)

    def reset(self) -> None:
        """双回路全部复位。"""
        self.dosing.reset()
        self.aeration.reset()

    @staticmethod
    def build_default() -> "ProcessPlant":
        """用典型经验值参数构建默认过程厂。

        参数选取依据（典型经验值，仅供仿真验证）：
          加药回路：接触池容积大、混合慢 → T=120s；投药管线+取样管路滞后 τ=30s；
                    泵开度每 +1%，余氯稳态约 +0.08 mg/L；进水每 +10% 稀释约 -0.06 mg/L。
          曝气回路：生化反应放热缓冲小 → T=90s；风管+液相传氧滞后 τ=15s；
                    风机每 +1Hz，DO 稳态约 +0.12 mg/L；进水每 +10% 耗氧约 -0.08 mg/L。
        """
        dosing = FOPDTLoop(
            name="加药回路-出水余氯", pv_unit="mg/L", op_unit="%",
            k=0.08, t=120.0, tau=30.0,
            op_min=0.0, op_max=100.0,
            pv0=0.0, pv_floor=0.0,
            noise_std=0.02,
            load_gain=-0.006, load_filter_t=60.0,
            seed=101,
        )
        aeration = FOPDTLoop(
            name="曝气回路-溶解氧", pv_unit="mg/L", op_unit="Hz",
            k=0.12, t=90.0, tau=15.0,
            op_min=0.0, op_max=50.0,
            pv0=0.0, pv_floor=0.0,
            noise_std=0.03,
            load_gain=-0.008, load_filter_t=45.0,
            seed=202,
        )
        return ProcessPlant(dosing=dosing, aeration=aeration)


def build_default_plant() -> "ProcessPlant":
    """模块级工厂函数：用典型经验值构建默认过程厂（详见 ProcessPlant.build_default）。"""
    return ProcessPlant.build_default()


# ---------------------------------------------------------------------------
# 模块自检：直接运行本文件可观察两回路的开环阶跃响应
# 用法：python -m process.plant
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    plant = ProcessPlant.build_default()
    print("=" * 64)
    print("FOPDT 开环自检：t=50s 时两回路分别施加 30%/25Hz 阶跃")
    print("=" * 64)
    for i in range(600):  # 仿真 600s
        op_d = 30.0 if i >= 50 else 0.0    # 加药泵 30% 开度
        op_a = 25.0 if i >= 50 else 0.0    # 风机 25 Hz
        out = plant.step(op_d, op_a)
        if i % 60 == 0:
            print(
                f"t={i:>4}s  余氯={out['dosing_pv']:.3f}mg/L"
                f"(真值{out['dosing_true']:.3f})  "
                f"DO={out['aeration_pv']:.3f}mg/L"
                f"(真值{out['aeration_true']:.3f})"
            )
    print("-" * 64)
    print(f"理论稳态: 余氯≈{30*plant.dosing.k:.3f}mg/L  DO≈{25*plant.aeration.k:.3f}mg/L")
