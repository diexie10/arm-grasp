# -*- coding: utf-8 -*-
"""servo_controller.py — 视觉伺服策略层（纯数学，无 I/O）。

从 state_machine.py 抽取，为 Phase E 调参解耦。持有：
  - 偏移目标计算（CAMERA_OFFSET_X/Y + 实拍帧尺寸 → cx0/cy0）
  - EMA 状态（ema_cx/ema_cy）
  - _align_delta 的 dq0/dq12 数学（含限幅 clamp 引用 + J4 微分约束）
  - 六轴限位检查 helper

接口设计：输入纯数值（像素误差/帧尺寸/关节角列表），输出增量或判定。
不 import pyserial/cv2，不持有 serial/vision 引用。
"""

import config
from kinematics import clamp


class ServoController:
    """视觉伺服策略层：像素误差 → 关节增量的纯计算。"""

    def __init__(self):
        self.ema_cx = None
        self.ema_cy = None

    def reset_ema(self):
        """重置 EMA 状态（每次 _servo_align 调用前清零）。"""
        self.ema_cx = None
        self.ema_cy = None

    def compute_offset_target(self, frame_h, frame_w):
        """计算伺服目标点 = 画面中心 + 相机-夹爪偏移。

        用实拍帧尺寸而非 cap.get()（部分摄像头属性与实际帧不一致）。
        偏移默认 0 = 目标即画面中心（旧行为）。
        """
        cx0 = frame_w / 2.0 + config.CAMERA_OFFSET_X
        cy0 = frame_h / 2.0 + config.CAMERA_OFFSET_Y
        return cx0, cy0

    def smooth_ema(self, cx, cy):
        """EMA 平滑检测坐标。EMA_ALPHA=1.0 直通 = 旧行为。

        Returns:
            (cx_smooth, cy_smooth) 平滑后的坐标
        """
        if 0.0 < config.EMA_ALPHA < 1.0:
            if self.ema_cx is None:
                self.ema_cx, self.ema_cy = cx, cy
            else:
                self.ema_cx = config.EMA_ALPHA * cx + (1.0 - config.EMA_ALPHA) * self.ema_cx
                self.ema_cy = config.EMA_ALPHA * cy + (1.0 - config.EMA_ALPHA) * self.ema_cy
                cx, cy = self.ema_cx, self.ema_cy
        return cx, cy

    def compute_error(self, cx, cy, cx0, cy0):
        """像素误差 = 目标位置 - 伺服目标点。

        Returns:
            (ex, ey, err) 水平/垂直误差 + 欧氏距离
        """
        ex, ey = cx - cx0, cy - cy0
        err = (ex * ex + ey * ey) ** 0.5
        return ex, ey, err

    def align_delta(self, ex, ey, max_step):
        """像素误差 → 关节增量限幅值（含 J4 微分约束注释）。

        ΔJ1 = Kp·ex（水平），ΔJ2 = ΔJ3 = Kp·ey·0.5（垂直）。
        竖直约束微分形式：ΔJ4 = −ΔJ2 − ΔJ3 = −2·dq12（dq12=0 不碰 J4）。
        不用绝对式 90−J2−J3：其零位参考是真机标定项（S2 实测舵机 93°=水平，
        已推翻 offset=90 假设），标定前绝对式会把校准好的 HOME 姿态拉到非法值
        （实测复现：90−122−167=−199 → ALIGN 首步即 ERROR）。

        Args:
            ex: 水平像素误差
            ey: 垂直像素误差
            max_step: 单次最大关节增量 °

        Returns:
            (dq0, dq12) 两个限幅后的增量值
        """
        dq0 = clamp(config.SERVO_KP * ex, -max_step, max_step)
        dq12 = clamp(config.SERVO_KP * ey * 0.5, -max_step, max_step)
        return dq0, dq12

    def apply_joint_deltas(self, q_start, dq0, dq12):
        """应用关节增量到起始关节角，含 J4 微分约束。

        Args:
            q_start: 起始关节角列表 [6]
            dq0: J1 水平增量
            dq12: J2/J3 垂直增量（J4 = -2*dq12）

        Returns:
            q_new: 更新后的关节角列表
        """
        q = list(q_start)
        q[0] += dq0
        q[1] += dq12
        q[2] += dq12
        if dq12 != 0.0:
            q[3] -= 2.0 * dq12
        return q

    def check_limits(self, q, state_label):
        """六轴限位检查：超限返回 (False, error_msg)，通过返回 (True, None)。"""
        for i in range(6):
            if q[i] < config.JOINT_MIN[i] or q[i] > config.JOINT_MAX[i]:
                return False, ("%s: joint %d out of range %.1f"
                               % (state_label, i + 1, q[i]))
        return True, None
