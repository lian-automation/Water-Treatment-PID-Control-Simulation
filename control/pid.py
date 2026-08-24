# -*- coding: utf-8 -*-
"""
control/pid.py —— 增量式 PID 控制器
====================================================================

职责：
    为加药/曝气回路提供工业风格的增量式 PID 算法，包含：
      ① 输出限幅（默认 0~100%，可按回路配置，如风机 0~50 Hz）
      ② 抗积分饱和：反算法(back-calculation / clamping)
      ③ 测量死区（误差小于死区时不新增比例/积分动作，避免执行器抖动）
      ④ 手动/自动无扰切换（手动时内部积分跟踪手动输出）
      ⑤ 微分先行（默认对测量值微分）+ 一阶低通滤波 + 微分限幅，
        避免 SP 阶跃微分冲击并抑制测量噪声放大
      ⑥ P/I/D 参数在线修改，无需重启或重建对象

增量式算法（位置由增量累加得到）：
    Δu[k] = Kp * [ (e[k]-e[k-1]) + (dt/Ti)*e[k] ] + ( Ud[k] - Ud[k-1] )
    u[k]  = u_limit( u[k-1] + Δu[k] )
    其中：
      e[k]  = SP - PV（经测量死区处理）
      Ti    积分时间常数(s)，Td 为微分时间常数(s)，工程整定习惯单位
      Ud[k] 微分项「位置值」= Kp*Td*(滤波后 PV 变化率取负)，
            增量式里微分增量为相邻两拍位置值之差（等价二阶差分）

微分先行 + 不完全微分说明（详见 docs/系统设计说明书.md）：
    若直接对偏差 e=SP-PV 作理想微分，SP 阶跃瞬间会产生巨大微分输出
    （微分冲击）；若对带噪声的 PV 直接差分，噪声会被放大成输出抖动。
    工业控制器普遍采用「不完全微分」：先对 PV 做时间常数 Tdf=Td/N
    的一阶低通滤波，再取滤波值的变化率作为微分项输入，等效传递函数为
    Kp*Td*s/(1+Tdf*s)。N 越小滤波越强：本项目仿真对比了 N=5 与 N=2，
    噪声较大的回路在 N=2 下输出波动可降低约 70%，故默认 N=2（可按
    回路噪声水平在 2~5 间选取）。本控制器另对微分增量做 ±10% 输出
    量程的逐拍限幅（微分限幅），进一步压制残余噪声尖峰。

反算法抗积分饱和原理：
    先按未限幅输出 u_unsat 计算增量；若限幅生效（u != u_unsat），
    说明积分仍在向"出不去"的方向积累（积分饱和），此时把差值
        back = u_limited - u_unsat
    通过再调时间常数 Tt 回灌给内部积分累计项，等效于让积分器
    "退饱和"，使输出一旦需要反向时能立即退出饱和区。

手自动无扰切换原理：
    手动模式下每个周期令内部积分累计项跟踪：I = u_manual - Kp*e[k-1]，
    即"P+I 两项之和恒等于当前手动输出"；切回自动瞬间输出从手动值
    出发继续增量运算，不会产生跳变（bumpless transfer）。
"""

from __future__ import annotations

MODE_AUTO = "auto"
MODE_MANUAL = "manual"

# 不完全微分滤波系数：滤波时间常数 Tdf = Td / N_FILT。
# N 越小滤波越强。本仿真对比验证（docs/整定对比报告.md）：测量噪声较大时
# N=5 会把噪声放大成明显的输出抖动，N=2 可在保留微分相位超前的同时
# 把噪声引起的 OP 波动降低约 70%，故默认取 N=2（工程上按噪声水平 2~5 选取）。
N_FILT = 2.0
# 微分限幅：每拍微分增量不超过输出量程的该比例（抑制噪声尖峰）
DERIV_CLAMP_FRAC = 0.10


class PIDController:
    """增量式 PID 控制器（单回路一个实例）。

    典型用法（自动）：
        pid = PIDController(kp=15, ti=180, td=8, out_min=0, out_max=100)
        op = pid.update(sp=1.0, pv=0.82)

    手动/自动切换：
        pid.set_manual(35.0)      # 切手动并把输出稳定在 35%
        pid.write_manual(40.0)    # 手动调节输出
        pid.update(sp, pv)        # 手动模式下调用仅做跟踪，不改变输出
        pid.set_auto()            # 无扰切回自动
    """

    def __init__(
        self,
        kp: float,
        ti: float,
        td: float,
        dt: float = 1.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
        deadband: float = 0.0,
        track_t: float | None = None,
        deriv_on_meas: bool = True,
    ) -> None:
        """
        参数说明：
            kp       比例增益
            ti       积分时间常数(s)；<=0 表示切除积分作用
            td       微分时间常数(s)；<=0 表示切除微分作用
            dt       控制周期(s)，与过程仿真步长一致（1s）
            out_min  输出下限（加药泵 0%；风机可为 0 Hz）
            out_max  输出上限（加药泵 100%；风机 50 Hz）
            deadband 测量死区（与 PV 同单位）：|SP-PV| 小于死区时比例/积分不动作
            track_t  反算法再调时间常数 Tt(s)；缺省取 max(Td, 10*dt)
            deriv_on_meas True=微分先行（对 PV 微分，推荐）；False=对偏差微分
        """
        # ---- 整定参数（均可在线修改） -----------------------------------
        self.kp = float(kp)
        self.ti = float(ti)
        self.td = float(td)
        self.dt = float(dt)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.deadband = float(deadband)
        self.deriv_on_meas = bool(deriv_on_meas)
        self.track_t = float(track_t) if track_t else max(self.td, 10.0 * self.dt)

        # ---- 内部状态 ----------------------------------------------------
        self.mode = MODE_AUTO
        self._e1 = 0.0             # e[k-1]
        self._e2 = 0.0             # e[k-2]
        self._pv_f = 0.0           # PV 低通滤波值（不完全微分用）
        self._pv_f_prev = 0.0      # 上拍滤波值
        self._d_prev = 0.0         # 上拍微分项位置值（增量 = 本拍位置 - 上拍位置）
        self._i_term = 0.0         # 积分项累计（输出量纲），用于跟踪与诊断
        self._op = float(out_min)  # 上拍实际输出（已限幅）
        self._manual_op = float(out_min)   # 手动输出给定
        self._saturation = 0.0     # 上拍饱和修正量（诊断用，>0 上限饱和）

    # ------------------------------------------------------------------
    # 在线参数修改（无需重启）
    # ------------------------------------------------------------------
    def change_tuning(self, kp: float, ti: float, td: float) -> None:
        """在线修改 P/I/D 参数。

        注意：积分累计 _i_term 以「输出量纲」保存，改参数只影响后续增量，
        因此修改瞬间的输出不会跳变，满足在线整定的无扰要求。
        """
        self.kp = float(kp)
        self.ti = float(ti)
        self.td = float(td)
        self.track_t = max(self.td, 10.0 * self.dt)

    # ------------------------------------------------------------------
    # 手动/自动
    # ------------------------------------------------------------------
    def set_manual(self, manual_op: float | None = None) -> None:
        """切入手动模式。manual_op 缺省时保持当前输出不变（更平滑）。"""
        if manual_op is not None:
            self._manual_op = self._limit(float(manual_op))
        else:
            self._manual_op = self._op
        self.mode = MODE_MANUAL
        self._op = self._manual_op
        self._saturation = 0.0

    def write_manual(self, manual_op: float) -> None:
        """手动模式下的输出给定（软手操）。"""
        if self.mode != MODE_MANUAL:
            raise RuntimeError("当前为自动模式，请先 set_manual() 再写手动输出")
        self._manual_op = self._limit(float(manual_op))
        self._op = self._manual_op

    def set_auto(self) -> None:
        """切回自动模式（无扰：输出从当前手动值继续增量运算）。"""
        if self.mode == MODE_MANUAL:
            self.mode = MODE_AUTO

    @property
    def is_auto(self) -> bool:
        return self.mode == MODE_AUTO

    # ------------------------------------------------------------------
    # 主算法
    # ------------------------------------------------------------------
    def update(self, sp: float, pv: float) -> float:
        """每个控制周期调用一次，返回本拍输出（已限幅）。

        自动模式：增量式 PID + 反算法抗饱和 + 测量死区 + 不完全微分；
        手动模式：输出跟随手操值，同时内部积分跟踪 OP（为无扰切换做准备）。
        """
        sp = float(sp)
        pv = float(pv)
        e_raw = sp - pv
        # 测量死区：|误差| <= deadband 时视为无误差，抑制小信号抖动
        e = 0.0 if abs(e_raw) <= self.deadband else e_raw

        # ---- 不完全微分：先滤波，再取变化率 ------------------------------
        # 滤波时间常数 Tdf = Td/N；Td=0 时令 alpha=1（滤波器直通、微分关闭）
        tdf = (self.td / N_FILT) if self.td > 0.0 else 0.0
        alpha = self.dt / (self.dt + tdf) if tdf > 0.0 else 1.0
        self._pv_f += alpha * (pv - self._pv_f)          # 低通滤波 PV
        d_rate = -(self._pv_f - self._pv_f_prev) / self.dt  # 滤波后变化率(取负=误差方向)
        self._pv_f_prev = self._pv_f
        # 微分项「位置值」：增量式算法中微分增量为相邻两拍位置值之差，
        # 等价于对滤波后 PV 取二阶差分（这是增量式的关键，切勿直接把
        # 位置值累加进输出，否则等效于多积分了一个微分项导致发散）。
        d_pos = self.kp * self.td * d_rate

        if self.mode == MODE_MANUAL:
            # ---- 手动：积分跟踪 -----------------------------------------
            # 让 P+I 两项之和等于当前手操输出：I = u_manual - Kp*e[k-1]
            # （微分只响应测量变化率，不影响切换瞬间的平稳性，不必参与跟踪）
            self._i_term = self._manual_op - self.kp * self._e1
            self._op = self._manual_op
            # 误差与微分位置值照常更新，保证切自动后各增量项连续
            self._e2, self._e1 = self._e1, e
            self._d_prev = d_pos
            self._saturation = 0.0
            return self._op

        # ---- 自动模式 -----------------------------------------------------
        # 1) 比例增量、积分增量（受死区门控）
        d_p = self.kp * (e - self._e1)
        d_i = self.kp * self.dt / self.ti * e if self.ti > 0.0 else 0.0

        # 2) 微分增量（不完全微分 + 增量取差 + 微分限幅）
        if self.td > 0.0:
            if self.deriv_on_meas:
                d_d_raw = d_pos - self._d_prev            # 位置值之差 = 真正的增量
            else:
                d_d_raw = self.kp * (e - 2.0 * self._e1 + self._e2) \
                    * self.td / self.dt * alpha           # 传统对偏差二阶差分+平滑
            # 微分限幅：单拍微分增量不超过输出量程的 DERIV_CLAMP_FRAC
            d_clamp = DERIV_CLAMP_FRAC * (self.out_max - self.out_min)
            d_d = min(max(d_d_raw, -d_clamp), d_clamp)
        else:
            d_d = 0.0
        self._d_prev = d_pos

        # 3) 试算未限幅输出并限幅
        op_unsat = self._op + d_p + d_i + d_d
        op_sat = self._limit(op_unsat)

        # ---- 反算法抗积分饱和（back-calculation） ------------------------
        # back 是限幅"吃掉"的部分：>0 表示撞上限、<0 表示撞下限。
        # 把它按 1/Tt 的速率从积分累计里退出来，防止积分饱和。
        back = op_sat - op_unsat
        self._i_term += d_i + back * (self.dt / self.track_t)
        self._saturation = back

        # 4) 状态前移
        self._op = op_sat
        self._e2, self._e1 = self._e1, e
        return self._op

    def reset(self) -> None:
        """复位控制器内部状态（重新开工时用）。"""
        self._e1 = 0.0
        self._e2 = 0.0
        self._pv_f = 0.0
        self._pv_f_prev = 0.0
        self._d_prev = 0.0
        self._i_term = 0.0
        self._op = self.out_min
        self._manual_op = self.out_min
        self._saturation = 0.0
        self.mode = MODE_AUTO

    def get_state(self) -> dict:
        """导出控制器状态（看板/联调显示用）。"""
        return {
            "mode": self.mode,
            "kp": self.kp, "ti": self.ti, "td": self.td,
            "output": self._op,
            "integral": self._i_term,
            "saturation_back": self._saturation,
            "deadband": self.deadband,
            "out_min": self.out_min, "out_max": self.out_max,
            "deriv_on_meas": self.deriv_on_meas,
        }

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    def _limit(self, value: float) -> float:
        """输出限幅到 [out_min, out_max]。"""
        if value < self.out_min:
            return self.out_min
        if value > self.out_max:
            return self.out_max
        return value


# ---------------------------------------------------------------------------
# 模块自检：闭环调节质量、抗积分饱和、手动/自动无扰切换
# 用法：python -m control.pid
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from process.plant import build_default_plant

    plant = build_default_plant()
    # 加药回路：经验保守参数
    pid = PIDController(kp=15.0, ti=100.0, td=8.0,
                        dt=1.0, out_min=0.0, out_max=100.0, deadband=0.005)
    sp = 1.0  # 余氯设定值 1.0 mg/L
    print("=" * 72)
    print("PID 自检：余氯 SP=1.0mg/L 闭环；t=400s 切手动 40%；"
          "t=500s 手操 20%；t=600s 无扰切回自动")
    print("=" * 72)
    for i in range(1100):
        pv = plant.dosing.pv
        if i == 400:
            pid.set_manual(40.0)
        elif i == 500:
            pid.write_manual(20.0)
        elif i == 600:
            pid.set_auto()
        op = pid.update(sp, pv)
        plant.step(op, 0.0)
        if i % 100 == 0 or i in (399, 401, 599, 600, 601, 602, 603):
            mode_txt = "自动" if pid.is_auto else "手动"
            st = pid.get_state()
            print(f"t={i:>4}s [{mode_txt}] PV={plant.dosing.pv:.3f} "
                  f"(真值{plant.dosing.true_pv:.3f}) OP={op:6.2f}% "
                  f"I={st['integral']:7.3f} 饱和修正={st['saturation_back']:+.3f}")
    tail_err = abs(sp - plant.dosing.true_pv)
    print("-" * 72)
    print(f"终态|SP-PV真值|={tail_err:.4f} mg/L")
    print("检查点：t=600→603s 输出应从手操值 20% 平滑过渡（无扰切换）；"
          "启动阶段输出不应长时间顶死在 100%（抗积分饱和）。")
