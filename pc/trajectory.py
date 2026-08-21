# -*- coding: utf-8 -*-
"""trajectory.py — 关节空间插补 + 单方向逼近消隙。

规划（PC 端，ADR-6）：梯形速度轮廓（加速-匀速-减速），
参数 MAX_VEL=30°/s, MAX_ACC=15°/s²（MG996 极限 60°/s 的 50%，待实测调参）。
插补周期 DT_MS=20ms 对齐舵机 50Hz 刷新。

消隙（架构书 §4）：每关节记录上次运动方向；本次反向运动时先同向过冲
OVERSHOOT=15°，再从过冲点逼近目标 —— 保证最终逼近方向与上次一致，
消除虚位（最大 ~8°）。

执行：单关节 M 命令逐点下发（固件回显 clamp 值 → 回显铁律逐点校验），
不用 MALL（MALL 无逐关节回显，回显铁律失效）。CLAMP/TIMEOUT 抛异常，
由状态机转 ERROR。
"""

import math
import time

import config


class TrajectoryError(RuntimeError):
    """轨迹执行失败（CLAMP / TIMEOUT / BAD_ECHO）。调用方必须转 ERROR。"""


def plan_joint_move(from_q, to_q, dt_ms=None, max_vel=None, max_acc=None):
    """梯形速度规划。返回插补点列表（每点 q[6]），间隔 dt_ms。"""
    dt_ms = dt_ms or config.DT_MS
    v = max_vel or config.MAX_VEL
    a = max_acc or config.MAX_ACC
    dt = dt_ms / 1000.0

    dq = [to_q[i] - from_q[i] for i in range(6)]
    dmax = max(abs(d) for d in dq)
    if dmax < 1e-6:
        return [list(from_q)]

    t_acc = v / a
    if dmax <= a * t_acc * t_acc:          # 三角轮廓（到不了最大速度）
        t_acc = math.sqrt(dmax / a)
        t_tot = 2.0 * t_acc
    else:                                   # 梯形轮廓
        t_cruise = dmax / v - t_acc
        t_tot = 2.0 * t_acc + t_cruise

    pts = []
    t = 0.0
    while t <= t_tot + 1e-9:
        if t <= t_acc:                       # 加速（抛物线位置）
            s = 0.5 * a * t * t
        elif t <= t_tot - t_acc:             # 匀速
            s = 0.5 * a * t_acc * t_acc + v * (t - t_acc)
        else:                                # 减速
            tr = t_tot - t
            s = dmax - 0.5 * a * tr * tr
        scale = s / dmax
        pts.append([from_q[i] + dq[i] * scale for i in range(6)])
        t += dt
    # 保证终点精确（浮点最后一步可能略欠）
    pts[-1] = [round(x, 6) for x in to_q]
    return pts


class Trajectory:
    """轨迹执行器：保存每关节上次运动方向（消隙），逐点校验回显。

    定时：目标时间戳 + 忙等补偿。Windows time.sleep(0.02) 实际粒度 ~31ms
    （系统时钟 15.6ms 向上取整），直接 sleep 会让 20ms 插补周期漂到 31ms，
    长轨迹累计超时。忙等补偿保证每点精确 DT_MS（上位机 PC 无 CPU 压力）。
    """

    def __init__(self):
        self.last_dir = [0] * 6

    def _exec_point(self, serial, pt, deadline):
        """逐关节下发插补点，忙等到 deadline。返回下一 deadline。

        跳过零增量关节（dq≈0 不重发）：夹爪/静止关节免去 5/6 串口流量，
        直接决定 GRIP/RELEASE 是否假超时（P1-2）。
        """
        from_q = serial.joint_state
        for n in range(1, 7):
            ang = pt[n - 1]
            if abs(ang - from_q[n - 1]) < 1e-9:
                continue
            ok, msg = serial.move_joint(n, ang)
            if not ok:
                raise TrajectoryError("joint %d: %s" % (n, msg))
        while time.perf_counter() < deadline:
            pass
        return deadline + config.DT_MS / 1000.0

    def _run_plan(self, serial, pts):
        deadline = time.perf_counter()
        for pt in pts:
            deadline = self._exec_point(serial, pt, deadline)

    def move_to(self, serial, to_q, overshoot=True):
        """从 serial.joint_state（实际回显状态）规划并执行到 to_q。

        overshoot=False 用于朝工作面/目标的最终逼近（DESCEND / FINAL_ALIGN /
        _servo_align）：反向消隙过冲会把末端压到目标点之外（15° 关节角在臂展
        上 ≈ 数十 mm），靠近木块/桌面时有撞击风险，这些阶段必须禁用。
        """
        from_q = list(serial.joint_state)

        # --- 单方向逼近：反向关节先过冲（过冲不超限位，P2-2）---
        overshoot_q = list(to_q)
        needs_overshoot = False
        if overshoot:
            for i in range(6):
                d = to_q[i] - from_q[i]
                if abs(d) < 1e-6:
                    continue
                dir_sign = 1 if d > 0 else -1
                if self.last_dir[i] != 0 and dir_sign != self.last_dir[i]:
                    o = to_q[i] + dir_sign * config.OVERSHOOT
                    # clamp 到限位：边界区余量不足时只过冲到限位（消隙部分补偿）
                    overshoot_q[i] = min(config.JOINT_MAX[i],
                                         max(config.JOINT_MIN[i], o))
                    needs_overshoot = True

        if needs_overshoot:
            self._run_plan(serial, plan_joint_move(from_q, overshoot_q))
            from_q = overshoot_q

        # --- 主运动（最终逼近方向 = 本次方向）---
        self._run_plan(serial, plan_joint_move(from_q, to_q))

        # --- 记录方向 ---
        for i in range(6):
            if abs(to_q[i] - overshoot_q[i]) > 1e-6:
                self.last_dir[i] = 1 if to_q[i] > overshoot_q[i] else -1

        time.sleep(config.SETTLE_MS / 1000.0)   # 到位稳定
