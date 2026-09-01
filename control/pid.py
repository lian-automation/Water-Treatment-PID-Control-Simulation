# -*- coding: utf-8 -*-
"""
control/pid.py —— 增量式 PID 控制器
====================================================================

职责：
    为加药/曝气回路提供工业风格的增量式 PID 算法，包含：
      ① 输出限幅（默认 0~100%，可按回路配置，如风机 0~50 Hz）
      ② 抗积分饱和：增量式固有钳位（clamping），另设退饱和诊断量
      ③ 测量死区（误差小于死区时不新增比例/积分动作，避免执行器抖动）
      ④ 手动/自动无扰切换（输出状态直接承接手操值，天然无扰）
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
    Kp*Td*s/(1+Tdf*s)。N 越小滤波越强：本项目对照实验（Z-N 工程修正
    参数、数据见 docs/整定对比报告.md 附录，由 `python -m tune.tuner`
    自动生成）：
      · 曝气回路伺服场景：N=5 时超调 127.0%、调节时间覆盖整个观察窗
        （1200s，呈慢频振荡）；N=2 同参数超调 22.4%、调节时间 19s；
      · 噪声×5 场景：N=2 相比 N=5 的 OP 动作强度降约 7%~35%，
        IAE(真值) 降约 32%~64%。
    故默认 N=2（工程上按噪声水平在 2~5 间选取）。N 可按实例经构造
    参数 deriv_filt_n 配置，不必改源码。本控制器另对微分增量做
    ±10% 输出量程的逐拍限幅（微分限幅），进一步压制残余噪声尖峰。

抗积分饱和机制（如实说明）：
    增量式算法的输出由增量直接累加在「上拍已限幅输出」上：
        u[k] = u_limit( u[k-1] + Δu[k] )
    累加基点永远是被限幅后的实际输出，输出不可能越限堆积——增量式
    + 限幅天然不会积分饱和（工业上称 clamping/遇限停积的连续化形式）。
    本控制器另维护一个积分诊断量 _i_sat_diag：把限幅截断差
        back = u_limited - u_unsat
    按 1/Tt 的速率回灌给它，使其在饱和期间呈退饱和趋势（增速低于名义
    积分增速）。注意两点：
      ① _i_sat_diag 只用于显示与诊断，【不参与控制律】；真正防止积分
         饱和、保证退出饱和无拖尾的，是上述增量式固有钳位；
      ② 长时间持续饱和下它仍会缓慢增长（每拍净增约 d_i*(1-dt/Tt)），
         这是诊断量的固有局限（可运行本模块自检的强制饱和段观察）。

手自动无扰切换原理：
    输出状态 _op 始终保存"上拍实际（已限幅）输出"：切手动时输出取
    手操值（缺省即当前输出，天然连续）；切回自动时增量直接累加在
    手操值上，不产生跳变（bumpless transfer）。手动期间按
    I = u_manual - Kp*e[k-1] 刷新的只是积分诊断量 _i_sat_diag，
    供显示核对，不参与控制。
"""

from __future__ import annotations

MODE_AUTO = "auto"
MODE_MANUAL = "manual"

# 不完全微分滤波系数（默认值）：滤波时间常数 Tdf = Td / N_FILT。
# N 越小滤波越强。对照实验数据（Z-N 工程修正参数，由 `python -m tune.tuner`
# 自动生成于 docs/整定对比报告.md 附录）：曝气回路伺服场景 N=5 超调 127.0%、
# 调节时间覆盖整个观察窗（慢频振荡），N=2 同参数超调 22.4%、19s；噪声×5
# 场景 N=2 相比 N=5 的 OP 动作强度降约 7%~35%、IAE(真值) 降约 32%~64%。
# 故默认取 N=2（工程上按噪声水平 2~5 选取，可经构造参数 deriv_filt_n 覆盖）。
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
        deriv_filt_n: float | None = None,
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
            track_t  积分诊断量退饱和时间常数 Tt(s)（仅作用于诊断量，
                     不影响控制律）；缺省取 max(Td, 10*dt)
            deriv_on_meas True=微分先行（对 PV 微分，推荐）；False=对偏差微分
            deriv_filt_n 不完全微分滤波系数 N（Tdf=Td/N），None=用模块默认
                     N_FILT。按回路噪声水平 2~5 选取（对照实验数据见
                     docs/整定对比报告.md 附录），可按实例配置不必改源码
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
        if deriv_filt_n is None:
            self.deriv_filt_n = N_FILT
        else:
            if float(deriv_filt_n) <= 0:
                raise ValueError("deriv_filt_n 必须为正")
            self.deriv_filt_n = float(deriv_filt_n)
        self.track_t = float(track_t) if track_t else max(self.td, 10.0 * self.dt)

        # ---- 内部状态 ----------------------------------------------------
        self.mode = MODE_AUTO
        self._e1 = 0.0             # e[k-1]
        self._e2 = 0.0             # e[k-2]
        self._pv_f = 0.0           # PV 低通滤波值（不完全微分用）
        self._pv_f_prev = 0.0      # 上拍滤波值
        self._d_prev = 0.0         # 上拍微分项位置值（增量 = 本拍位置 - 上拍位置）
        self._i_sat_diag = 0.0     # 积分【诊断量】（输出量纲）：仅供显示核对，
                                   # 不参与控制律（命名如实反映其作用域）
        self._op = float(out_min)  # 上拍实际输出（已限幅）——抗饱和与无扰切换的核心
        self._manual_op = float(out_min)   # 手动输出给定
        self._saturation = 0.0     # 上拍限幅截断差（诊断用，>0 下限饱和、<0 上限饱和）

    # ------------------------------------------------------------------
    # 在线参数修改（无需重启）
    # ------------------------------------------------------------------
    def change_tuning(self, kp: float, ti: float, td: float) -> None:
        """在线修改 P/I/D 参数。

        注意：真正的无扰来自输出状态 _op 的续算——增量只作用在「上拍
        已限幅输出」上，改参数瞬间输出不会跳变。积分诊断量 _i_sat_diag
        同样以「输出量纲」保存，改参数只影响其后续增量，显示亦无跳变。
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

        自动模式：增量式 PID（输出限幅天然抗积分饱和）+ 测量死区 +
                  不完全微分；积分诊断量 _i_sat_diag 仅随之刷新供显示。
        手动模式：输出跟随手操值，同时刷新积分诊断量（为无扰切换做显示核对）。
        """
        sp = float(sp)
        pv = float(pv)
        e_raw = sp - pv
        # 测量死区：|误差| <= deadband 时视为无误差，抑制小信号抖动
        e = 0.0 if abs(e_raw) <= self.deadband else e_raw

        # ---- 不完全微分：先滤波，再取变化率 ------------------------------
        # 滤波时间常数 Tdf = Td/N（N 可按实例经 deriv_filt_n 配置）；
        # Td=0 时令 alpha=1（滤波器直通、微分关闭）
        tdf = (self.td / self.deriv_filt_n) if self.td > 0.0 else 0.0
        alpha = self.dt / (self.dt + tdf) if tdf > 0.0 else 1.0
        self._pv_f += alpha * (pv - self._pv_f)          # 低通滤波 PV
        d_rate = -(self._pv_f - self._pv_f_prev) / self.dt  # 滤波后变化率(取负=误差方向)
        self._pv_f_prev = self._pv_f
        # 微分项「位置值」：增量式算法中微分增量为相邻两拍位置值之差，
        # 等价于对滤波后 PV 取二阶差分（这是增量式的关键，切勿直接把
        # 位置值累加进输出，否则等效于多积分了一个微分项导致发散）。
        d_pos = self.kp * self.td * d_rate

        if self.mode == MODE_MANUAL:
            # ---- 手动：刷新积分诊断量（仅显示核对，不参与控制） ---------
            # 按 I = u_manual - Kp*e[k-1] 刷新，使诊断量与当前手操输出自洽。
            # 真正的无扰切换由输出状态 _op 直接承接手操值保证（见下），
            # 不依赖该诊断量。
            self._i_sat_diag = self._manual_op - self.kp * self._e1
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

        # ---- 积分退饱和诊断量（不参与控制律！）---------------------------
        # back 是限幅"吃掉"的部分：>0 表示撞下限（op_sat>op_unsat）、
        # <0 表示撞上限。增量式算法把增量累加在「上拍已限幅输出」上
        # （本函数末尾 _op = op_sat），输出本身不可能越限堆积——真正防
        # 积分饱和的是这一固有钳位（clamping），与 back 无关。
        # 这里仅维护诊断量：按 1/Tt 的速率把截断差回灌给它，使其在饱和
        # 期间呈退饱和趋势（增速低于名义积分增速），供看板/调试观察。
        back = op_sat - op_unsat
        self._i_sat_diag += d_i + back * (self.dt / self.track_t)
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
        self._i_sat_diag = 0.0
        self._op = self.out_min
        self._manual_op = self.out_min
        self._saturation = 0.0
        self.mode = MODE_AUTO

    def get_state(self) -> dict:
        """导出控制器状态（看板/联调显示用）。

        字段说明：
            integral        积分【诊断量】_i_sat_diag——仅供显示与调试，
                            不参与控制律；持续饱和期间它会缓慢增长，
                            不代表有效积分内容（见模块 docstring）。
            saturation_back 上拍限幅截断差（>0 撞下限、<0 撞上限，0=未饱和）。
        """
        return {
            "mode": self.mode,
            "kp": self.kp, "ti": self.ti, "td": self.td,
            "output": self._op,
            "integral": self._i_sat_diag,
            "saturation_back": self._saturation,
            "deadband": self.deadband,
            "out_min": self.out_min, "out_max": self.out_max,
            "deriv_on_meas": self.deriv_on_meas,
            "deriv_filt_n": self.deriv_filt_n,
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
                  f"I_diag={st['integral']:7.3f} "
                  f"饱和修正={st['saturation_back']:+.3f}")
    tail_err = abs(sp - plant.dosing.true_pv)
    print("-" * 72)
    print(f"终态|SP-PV真值|={tail_err:.4f} mg/L")
    print("检查点：t=600→603s 输出应从手操值 20% 平滑过渡（无扰切换）；"
          "启动阶段输出不应长时间顶死在 100%（抗积分饱和）。")

    # ---- 强制饱和场景（评审修复项：保证抗饱和路径在自检中被真实执行）----
    # 正常自检工作点（SP=1.0、稳态 OP≈12.5%）永远不会触及 0/100% 限幅；
    # 这里用小量程 out_max=20% + 不可达 SP=2.0mg/L（稳态需 OP≈25%），
    # 让输出必然撞上限，随后放宽 SP 观察退出饱和是否无拖尾。
    print("=" * 72)
    print("强制饱和自检：out_max=20%、SP=2.0mg/L（稳态需 OP≈25%，必然撞限）；"
          "t=400s 放宽 SP→1.0 观察无拖尾退出")
    print("=" * 72)
    plant2 = build_default_plant()
    plant2.dosing.set_noise_scale(0.0)   # 关噪声：隔离抗饱和机理本身
    pid2 = PIDController(kp=15.0, ti=100.0, td=8.0, dt=1.0,
                         out_min=0.0, out_max=20.0, deadband=0.005)
    sp2 = 2.0
    for i in range(800):
        if i == 400:
            sp2 = 1.0                     # 退出饱和：目标降回可达范围
        pv = plant2.dosing.pv
        op = pid2.update(sp2, pv)
        plant2.step(op, 0.0)
        if i % 100 == 0 or i in (399, 400, 401, 402, 405):
            st = pid2.get_state()
            print(f"t={i:>4}s PV={plant2.dosing.pv:.3f} OP={op:6.2f}% "
                  f"I_diag={st['integral']:7.3f} "
                  f"饱和修正={st['saturation_back']:+.3f}")
    print("-" * 72)
    print("检查点：饱和段 OP 顶在 20%、饱和修正为非零负值（撞上限）；退出后 "
          "OP 立即离开 20%（增量钳位抗饱和，无积分拖尾）。积分诊断量 I_diag "
          "在饱和期间仍缓慢增长，属诊断量固有局限（见模块 docstring ②）。")
