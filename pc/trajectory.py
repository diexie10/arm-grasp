# -*- coding: utf-8 -*-
"""trajectory.py — waypoint 模式执行 + 单方向逼近消隙（ADR-3）。

执行模型（ADR-3 P1/P2）：PC 只发关键拐点（G 命令，六轴批量目标），
MCU 自主梯形执行（30°/s, 15°/s²），PC 轮询 Q 等 DONE。PC 不再逐 20ms
刷点——90° 运动 = 1 条 G（原 ~250 条 M），Windows 卡顿不再影响运动中时序。

消隙（架构书 §4）：每关节记录上次运动方向；本次反向运动时先发过冲点
OVERSHOOT=15°，等 DONE 后再发目标点 —— 最终逼近方向与上次一致。
"""

import time

import config


class TrajectoryError(RuntimeError):
    """轨迹执行失败（CLAMP / TIMEOUT / BAD_ECHO）。调用方必须转 ERROR。"""


class Trajectory:
    """waypoint 执行器：保存每关节上次运动方向（消隙），发 G 等 DONE。

    时间轴归属：运动中的时间轴在 MCU（梯形自主执行），PC 只决定
    "下一个拐点去哪"。DONE 超时 = 预估时长 × FACTOR + EXTRA，超时抛
    TrajectoryError 由状态机转 ERROR（不重试，IWDG 是 MCU 侧底线）。
    """

    def __init__(self):
        self.last_dir = [0] * 6

    def move_to(self, serial, to_q, overshoot=True):
        """从 serial.joint_state 执行到 to_q（1~2 条 G 命令）。

        overshoot=False 用于朝工作面/目标的最终逼近（DESCEND / FINAL_ALIGN /
        _servo_align）：反向消隙过冲会把末端压到目标点之外（15° 关节角在臂展
        上 ≈ 数十 mm），靠近木块/桌面时有撞击风险，这些阶段必须禁用。
        """
        from_q = list(serial.joint_state)

        # --- 单方向逼近：反向关节先发过冲点（过冲不超限位，P2-2）---
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
            ok, msg = serial.move_waypoint(overshoot_q)
            if not ok:
                raise TrajectoryError("overshoot: %s" % msg)

        # --- 主运动（最终逼近方向 = 本次方向）---
        ok, msg = serial.move_waypoint(to_q)
        if not ok:
            raise TrajectoryError("waypoint: %s" % msg)

        # --- 记录方向 ---
        for i in range(6):
            if abs(to_q[i] - overshoot_q[i]) > 1e-6:
                self.last_dir[i] = 1 if to_q[i] > overshoot_q[i] else -1

        time.sleep(config.SETTLE_MS / 1000.0)   # 到位稳定
