# -*- coding: utf-8 -*-
"""
dashboard/app.py —— Web 看板后端（Flask + ECharts 单页）
====================================================================

职责：
    提供水处理双回路 PID 仿真的实时监控网页与操作 REST 接口：
      ① 双回路 SP/PV/OP 实时趋势（ECharts，最近 500 点可回看）；
      ② 手动/自动无扰切换按钮、SP 下发按钮；
      ③ PID 参数在线修改面板（无需重启仿真）；
      ④ 报警列表（PV 高限 / 低限 / SP-PV 偏差，5s 延时确认消抖）。

两种运行模式：
    默认（单机模式）：本进程内部跑「过程模型 + 双回路 PID」仿真线程；
    联调模式(--modbus host:port)：作为 Modbus/TCP 主站连接
        plc_link/modbus_server.py 启动的从站，读写 HR 寄存器，
        验证"看板 ↔ PLC"的真实通信链路（此时参数在线修改不可用，
        因为标准寄存器表中没有 P/I/D 参数区——这正是真实工程的常见边界）。

用法：
    python -m dashboard.app                        # 单机模式 http://127.0.0.1:5000
    python -m dashboard.app --port 5001            # 指定端口
    python -m dashboard.app --modbus 127.0.0.1:5020   # 联调模式（需先启动从站）
"""

from __future__ import annotations

import argparse
import threading
import time
from collections import deque
from pathlib import Path

from flask import Flask, jsonify, render_template, request

from control.pid import PIDController
from plc_link.common import (AlarmMonitor, HR_ALM, HR_HB, HR_MANUAL,
                             HR_MODE, HR_OP, HR_PV, HR_SP, LOOP_CONFIG,
                             LOOP_NAMES, UNIT_OF_LOOP, alarm_edges)
from process.plant import build_default_plant

HISTORY_LEN = 500          # 历史缓冲长度（最近 500 点回看）
DEFAULT_INTERVAL = 1.0     # 控制周期(秒)


class LocalSim:
    """单机模式后端：本地过程仿真 + 双回路增量式 PID + 报警判断。"""

    def __init__(self, interval: float = DEFAULT_INTERVAL) -> None:
        self.interval = interval
        self.plant = build_default_plant()
        self.loops: dict[str, dict] = {}
        for key in ("dosing", "aeration"):
            cfg = LOOP_CONFIG[key]
            loop_obj = getattr(self.plant, key)
            self.loops[key] = {
                "cfg": cfg,
                "loop": loop_obj,
                "pid": PIDController(
                    kp=cfg["kp"], ti=cfg["ti"], td=cfg["td"],
                    dt=interval,
                    out_min=loop_obj.op_min, out_max=loop_obj.op_max,
                    deadband=cfg["deadband"],
                ),
                "sp": float(cfg["sp0"]),
                # 越限延时确认报警器（与从站共用 common.AlarmMonitor）
                "alm": AlarmMonitor(cfg["high"], cfg["low"],
                                    cfg["dev"], confirm=5),
                "alarm_code": 0,
            }
        self.sim_seconds = 0.0
        self.t0_wall = time.time()
        # 历史缓冲：每回路各一条
        self.hist: dict[str, dict[str, deque]] = {
            key: {"t": deque(maxlen=HISTORY_LEN),
                  "sp": deque(maxlen=HISTORY_LEN),
                  "pv": deque(maxlen=HISTORY_LEN),
                  "op": deque(maxlen=HISTORY_LEN)}
            for key in ("dosing", "aeration")
        }
        self.alarm_log: deque = deque(maxlen=100)   # 报警事件流水
        # 后台线程诊断状态（评审修复项：线程逐拍捕获异常，不再静默死亡）
        self.step_errors = 0
        self.last_error: str | None = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def _eval_alarm(self, key: str) -> int:
        """报警判断（与从站共用的 common.AlarmMonitor 延时确认逻辑），
        返回报警码并在边沿时刻写入报警流水。"""
        st = self.loops[key]
        pv, sp = st["loop"].true_pv, st["sp"]
        code_before = st["alarm_code"]
        code = st["alm"].evaluate(pv, sp)      # 5 拍越限延时确认
        st["alarm_code"] = code
        # 边沿检测 → 写报警流水（激活/恢复各记一条，共用 alarm_edges）
        now = time.strftime("%H:%M:%S")
        for bit, name, activated in alarm_edges(code_before, code):
            self.alarm_log.appendleft({
                "time": now, "loop": LOOP_NAMES[key],
                "type": f"PV{name}报警",
                "status": "激活" if activated else "恢复",
            })
        return code

    def step_once(self) -> None:
        """推进一个控制周期（双回路同步）。"""
        with self._lock:
            ops = {}
            for key in ("dosing", "aeration"):
                st = self.loops[key]
                # update() 自动模式返回 PID 输出、手动模式原样返回手操值
                ops[key] = st["pid"].update(st["sp"], st["loop"].pv)
            out = self.plant.step(ops["dosing"], ops["aeration"])
            self.sim_seconds += self.interval
            for key in ("dosing", "aeration"):
                st = self.loops[key]
                h = self.hist[key]
                h["t"].append(round(self.sim_seconds))
                h["sp"].append(round(st["sp"], 3))
                h["pv"].append(round(out[f"{key}_pv"], 4))
                h["op"].append(round(ops[key], 2))
                self._eval_alarm(key)

    def run_forever(self) -> None:
        """后台仿真线程主循环（实时节拍）。

        评审修复项：逐拍 try/except——后台线程若裸跑，step_once 一次
        未预期异常就会静默死亡，看板数据永久冻结且无告警（原先与
        ModbusBridge.run_forever 的异常保护纪律不一致）。现在异常被
        记录到 last_error/step_errors 并打印后继续运行。
        """
        next_t = time.time()
        while True:
            try:
                self.step_once()
            except Exception as exc:            # 保持线程存活，仅跳过本拍
                self.step_errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                print(f"[LocalSim] 仿真线程异常（第 {self.step_errors} 次），"
                      f"本拍跳过：{self.last_error}", flush=True)
            next_t += self.interval
            delay = next_t - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.time()

    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """导出看板所需的完整状态快照（深拷贝历史数据）。"""
        with self._lock:
            loops_json = {}
            for key in ("dosing", "aeration"):
                st = self.loops[key]
                pidst = st["pid"].get_state()
                loops_json[key] = {
                    "name": LOOP_NAMES[key],
                    "sp": round(st["sp"], 3),
                    "pv": round(st["loop"].pv, 4),
                    "true_pv": round(st["loop"].true_pv, 4),
                    "op": round(pidst["output"], 2),
                    "mode": pidst["mode"],
                    "kp": pidst["kp"], "ti": pidst["ti"], "td": pidst["td"],
                    "out_min": pidst["out_min"], "out_max": pidst["out_max"],
                    "pv_unit": st["loop"].pv_unit,
                    "op_unit": st["loop"].op_unit,
                    "alarm_code": st["alarm_code"],
                    "load": round(self.plant.dosing.load, 1),
                    "history": {k: list(v) for k, v in self.hist[key].items()},
                }
            return {
                "ok": True,
                "sim_seconds": round(self.sim_seconds),
                "wall_clock": time.strftime("%Y-%m-%d %H:%M:%S"),
                "loops": loops_json,
                "alarm_log": list(self.alarm_log)[:30],
            }

    # ---------------- 操作接口（供 Flask 路由调用） -------------------
    def set_sp(self, key: str, value: float) -> None:
        with self._lock:
            self.loops[key]["sp"] = float(value)

    def set_mode(self, key: str, auto: bool, manual_op: float | None = None) -> None:
        with self._lock:
            st = self.loops[key]
            if auto and not st["pid"].is_auto:
                st["pid"].set_auto()          # 无扰切自动
            elif not auto and st["pid"].is_auto:
                cur = st["pid"].get_state()["output"]
                st["pid"].set_manual(cur)     # 无扰切手动：保持当前输出
            if manual_op is not None and not st["pid"].is_auto:
                st["pid"].write_manual(float(manual_op))

    def write_manual(self, key: str, op: float) -> None:
        with self._lock:
            st = self.loops[key]
            if st["pid"].is_auto:
                raise RuntimeError("自动模式下不能软手操")
            st["pid"].write_manual(float(op))

    def change_tuning(self, key: str, kp: float, ti: float, td: float) -> None:
        """PID 参数在线修改（无需重启）。"""
        if kp <= 0 or ti < 0 or td < 0:
            raise ValueError("P/I/D 参数取值不合法")
        with self._lock:
            self.loops[key]["pid"].change_tuning(kp, ti, td)


# ---------------------------------------------------------------------------
# 联调模式后端：Modbus/TCP 主站桥接
# ---------------------------------------------------------------------------
try:                                    # pymodbus 缺失时单机模式仍可用
    from pymodbus.client import ModbusTcpClient
    _HAS_PYMODBUS = True
except ImportError:                     # pragma: no cover
    _HAS_PYMODBUS = False


class ModbusBridge:
    """联调模式后端：轮询从站寄存器并维护同样的历史/报警结构。

    说明：SP、手/自动与手操值（HR8）通过寄存器下发；PID 参数不在
    标准寄存器表内，change_tuning 返回错误提示（真实工程需扩展
    寄存器表或专用功能码）。
    """

    def __init__(self, host: str, port: int,
                 interval: float = DEFAULT_INTERVAL) -> None:
        if not _HAS_PYMODBUS:
            raise RuntimeError("未安装 pymodbus，无法使用联调模式")
        self.host, self.port, self.interval = host, port, interval
        self.client = ModbusTcpClient(host=host, port=port)
        # 关键：pymodbus 客户端不是线程安全的。Flask 请求线程（下发 SP/
        # 模式）与后台轮询线程共用同一条 TCP 连接，必须加锁串行化，
        # 否则事务 ID 会错乱、请求互相干扰。
        self._lock = threading.Lock()
        self.sim_seconds = 0
        self.hist = {key: {"t": deque(maxlen=HISTORY_LEN),
                           "sp": deque(maxlen=HISTORY_LEN),
                           "pv": deque(maxlen=HISTORY_LEN),
                           "op": deque(maxlen=HISTORY_LEN)}
                     for key in ("dosing", "aeration")}
        self.last_regs: dict[str, list[int]] = {}
        self.alarm_log: deque = deque(maxlen=100)
        self._prev_alm: dict[str, int] = {"dosing": 0, "aeration": 0}
        self._connected = False

    def _read_regs(self, unit: int, retries: int = 3) -> list[int] | None:
        """带重试地读一个单元的 HR0~HR5（调用方需持有 _lock）。"""
        for _ in range(retries):
            try:
                rr = self.client.read_holding_registers(
                    HR_SP, count=6, slave=unit)
            except Exception:
                time.sleep(0.2)
                continue
            if not rr.isError():
                return list(rr.registers)
            time.sleep(0.2)
        return None

    def poll_once(self) -> None:
        """轮询一次两个从站单元，刷新历史与报警。"""
        if not self._connected:
            with self._lock:
                self._connected = self.client.connect()
            if not self._connected:
                return
        ok_all = True
        with self._lock:
            self.sim_seconds += 1          # 每轮询一次记 1s（两回路共用时钟）
            for key, unit in UNIT_OF_LOOP.items():
                regs = self._read_regs(unit)
                if regs is None:
                    ok_all = False          # 本轮失败，下轮重连
                    continue
                self.last_regs[key] = regs
                h = self.hist[key]
                h["t"].append(self.sim_seconds)
                h["sp"].append(regs[HR_SP] / 100.0)
                h["pv"].append(regs[HR_PV] / 100.0)
                h["op"].append(regs[HR_OP] / 100.0)
                # 报警码边沿 → 流水（边沿判定共用 common.alarm_edges）
                alm = regs[HR_ALM]
                prev = self._prev_alm[key]
                now = time.strftime("%H:%M:%S")
                for bit, name, activated in alarm_edges(prev, alm):
                    self.alarm_log.appendleft({
                        "time": now, "loop": LOOP_NAMES[key],
                        "type": f"PV{name}报警",
                        "status": "激活" if activated else "恢复"})
                self._prev_alm[key] = alm
        if not ok_all:
            self._connected = False

    def run_forever(self) -> None:
        while True:
            t0 = time.time()
            try:
                self.poll_once()
            except Exception as exc:            # 断线重连
                print(f"[ModbusBridge] 轮询异常：{exc}，2s 后重连")
                self._connected = False
                time.sleep(2)
            time.sleep(max(0.0, self.interval - (time.time() - t0)))

    def snapshot(self) -> dict:
        loops_json = {}
        for key in ("dosing", "aeration"):
            regs = self.last_regs.get(key)
            h = self.hist[key]
            unit_name = LOOP_CONFIG[key]["name"]
            loops_json[key] = {
                "name": LOOP_NAMES[key],
                "sp": (regs[HR_SP] / 100.0) if regs else 0,
                "pv": (regs[HR_PV] / 100.0) if regs else 0,
                "true_pv": (regs[HR_PV] / 100.0) if regs else 0,
                "op": (regs[HR_OP] / 100.0) if regs else 0,
                "mode": ("auto" if (regs and regs[HR_MODE] == 1) else "manual"),
                "kp": LOOP_CONFIG[key]["kp"],       # 从站不开放参数区，展示配置值
                "ti": LOOP_CONFIG[key]["ti"],
                "td": LOOP_CONFIG[key]["td"],
                "out_min": 0.0,
                "out_max": 100.0 if key == "dosing" else 50.0,
                "pv_unit": "mg/L",
                "op_unit": "%" if key == "dosing" else "Hz",
                "alarm_code": (regs[HR_ALM] if regs else 0),
                "load": 100.0,
                "heartbeat": (regs[HR_HB] if regs else 0),
                "history": {k: list(v) for k, v in h.items()},
            }
        return {
            "ok": True, "mode": "modbus",
            "sim_seconds": self.sim_seconds,
            "wall_clock": time.strftime("%Y-%m-%d %H:%M:%S"),
            "loops": loops_json,
            "alarm_log": list(self.alarm_log)[:30],
        }

    def _write(self, unit: int, address: int, value: int) -> bool:
        """写寄存器（与轮询线程互斥，带重试）。"""
        if not self._connected:
            with self._lock:
                self._connected = self.client.connect()
            if not self._connected:
                return False
        with self._lock:
            for _ in range(3):
                try:
                    w = self.client.write_register(address, value, slave=unit)
                except Exception:
                    time.sleep(0.2)
                    continue
                if not w.isError():
                    return True
                time.sleep(0.2)
        return False

    def set_sp(self, key: str, value: float) -> None:
        self._write(UNIT_OF_LOOP[key], HR_SP, int(round(value * 100)))

    def set_mode(self, key: str, auto: bool, manual_op: float | None = None) -> None:
        unit = UNIT_OF_LOOP[key]
        ok = self._write(unit, HR_MODE, 1 if auto else 0)
        # 评审修复项：切手动的初始手操值此前被静默丢弃（UI 询问形同虚设）。
        # 现在切手动后把手操初值写入 HR8（从站在手动模式下边沿生效）；
        # 未给初值时从站保持切换瞬间输出（无扰缺省）。
        if ok and not auto and manual_op is not None:
            self._write(unit, HR_MANUAL, int(round(float(manual_op) * 100)))

    def write_manual(self, key: str, op: float) -> None:
        """软手操：写 HR8 手操值寄存器（从站仅在手动模式下应用）。"""
        regs = self.last_regs.get(key)
        if regs is not None and regs[HR_MODE] == 1:
            raise RuntimeError("自动模式下不能软手操，请先切手动")
        self._write(UNIT_OF_LOOP[key], HR_MANUAL, int(round(float(op) * 100)))

    def change_tuning(self, key: str, kp: float, ti: float, td: float) -> None:
        raise RuntimeError("联调模式：标准寄存器表无 P/I/D 参数区，"
                           "请切回单机模式体验在线整定")


# ---------------------------------------------------------------------------
# Flask 应用与路由
# ---------------------------------------------------------------------------
app = Flask(__name__,
            template_folder=str(Path(__file__).resolve().parent / "templates"))
BACKEND: LocalSim | ModbusBridge | None = None


@app.route("/")
def index():
    """看板主页。"""
    return render_template("index.html")


@app.route("/api/data")
def api_data():
    """全量快照：当前值 + 最近 500 点历史 + 报警流水。"""
    return jsonify(BACKEND.snapshot())


@app.route("/api/state")
def api_state():
    """轻量状态快照（不含历史数组，供低带宽轮询/第三方探活）。"""
    snap = BACKEND.snapshot()
    for loop in snap["loops"].values():
        loop.pop("history", None)
    return jsonify(snap)


@app.route("/api/history")
def api_history():
    """历史数据回看：/api/history?points=500（默认 500，上限 500）。"""
    points = request.args.get("points", default=HISTORY_LEN, type=int)
    points = max(1, min(points, HISTORY_LEN))
    snap = BACKEND.snapshot()
    result = {"ok": True, "points": points, "loops": {}}
    for key, loop in snap["loops"].items():
        result["loops"][key] = {
            "name": loop["name"], "pv_unit": loop["pv_unit"],
            "op_unit": loop["op_unit"],
            **{k: v[-points:] for k, v in loop["history"].items()},
        }
    return jsonify(result)


@app.errorhandler(404)
def not_found(_error):
    """未匹配路由统一返回 JSON 提示（方便前端/脚本排错）。"""
    return jsonify({
        "ok": False,
        "message": "接口不存在。可用接口：GET /api/data | /api/state | "
                   "/api/history?points=N ；POST /api/sp | /api/mode | "
                   "/api/manual | /api/tuning",
    }), 404


def _get_loop_key(payload: dict) -> str:
    key = payload.get("loop", "")
    if key not in ("dosing", "aeration"):
        raise ValueError("loop 必须是 dosing 或 aeration")
    return key


@app.route("/api/sp", methods=["POST"])
def api_sp():
    """SP 下发。"""
    payload = request.get_json(force=True)
    try:
        key = _get_loop_key(payload)
        value = float(payload.get("value"))
        if value < 0 or value > 20:
            raise ValueError("设定值须在 0~20 mg/L 内")
        BACKEND.set_sp(key, value)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
    return jsonify({"ok": True})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    """手/自动切换（无扰），可在切手动时给定初始手操值。"""
    payload = request.get_json(force=True)
    try:
        key = _get_loop_key(payload)
        auto = bool(payload.get("auto"))
        manual_op = payload.get("manual_op")
        BACKEND.set_mode(key, auto,
                         float(manual_op) if manual_op is not None else None)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
    return jsonify({"ok": True})


@app.route("/api/manual", methods=["POST"])
def api_manual():
    """手动模式下的软手操。"""
    payload = request.get_json(force=True)
    try:
        key = _get_loop_key(payload)
        BACKEND.write_manual(key, float(payload.get("value")))
    except (TypeError, ValueError, RuntimeError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
    return jsonify({"ok": True})


@app.route("/api/tuning", methods=["POST"])
def api_tuning():
    """PID 参数在线修改。"""
    payload = request.get_json(force=True)
    try:
        key = _get_loop_key(payload)
        BACKEND.change_tuning(key,
                              float(payload.get("kp")),
                              float(payload.get("ti")),
                              float(payload.get("td")))
    except (TypeError, ValueError, RuntimeError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
    return jsonify({"ok": True})


def main() -> None:
    global BACKEND
    parser = argparse.ArgumentParser(description="水处理 PID 仿真 Web 看板")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL,
                        help="控制周期(秒)")
    parser.add_argument("--modbus", default=None,
                        help="联调模式：从站地址 host:port（默认为单机模式）")
    args = parser.parse_args()

    if args.modbus:
        host, _, port_str = args.modbus.partition(":")
        BACKEND = ModbusBridge(host, int(port_str or "5020"), args.interval)
        mode_txt = f"联调模式（Modbus/TCP → {args.modbus}）"
    else:
        BACKEND = LocalSim(args.interval)
        mode_txt = "单机模式（内置 FOPDT 过程仿真）"

    th = threading.Thread(target=BACKEND.run_forever,
                          name="dashboard-backend", daemon=True)
    th.start()

    print("=" * 64)
    print(f"Web 看板已启动：http://{args.host}:{args.port}")
    print(f"运行方式：{mode_txt}")
    print("Ctrl+C 退出")
    print("=" * 64)
    # debug=False 避免热重载把后台仿真线程启动两次
    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
