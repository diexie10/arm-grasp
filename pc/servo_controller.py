# -*- coding: utf-8 -*-
"""servo_controller.py — 视觉伺服策略层（纯数学，无 I/O）。

从 state_machine.py 抽取，为 Phase E 调参解耦。持有：
  - 偏移目标计算（CAMERA_OFFSET_X/Y + 实拍帧尺寸 → cx0/cy0）
  - EMA 状态（ema_cx/ema_cy）
  - _align_delta 的 dq0/dq12 数学（含限幅 clamp 引用 + J4 微分约束）
  - 六轴限位检查 helper
  - 运动时间估算（镜像 servo.c bite 模型，跨端同步责任）
  - 腕层 J4 修正 helper（量子化 + slack 钳制）

接口设计：输入纯数值（像素误差/帧尺寸/关节角列表），输出增量或判定。
不 import pyserial/cv2，不持有 serial/vision 引用。
"""

import math

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
        """应用关节增量到起始关节角，含 J4 微分约束（#28）。

        竖直约束微分形式：ΔJ4 = −ΔJ2 − ΔJ3 = −2·dq12（dq12=0 不碰 J4）。
        不用绝对式 90−J2−J3：其零位参考是真机标定项（KNOWN_TRAPS #28），
        标定前绝对式会把校准好的 HOME 姿态拉到非法值
        （实测复现：90−122−167=−199 → ALIGN 首步即 ERROR）。
        J1（基座回转）不参与竖直俯仰约束——J4 公式与 dq0 无关。

        Args:
            q_start: 起始关节角列表 [6]
            dq0: J1 水平增量
            dq12: J2/J3 公共垂直增量（ΔJ2=ΔJ3=dq12）

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


# =====================================================================
#  纯函数（无状态，无 I/O）——供 servo_controller 自身 + state_machine 调用
# =====================================================================

def estimate_move_seconds(from_q, to_q):
    """估算六轴运动耗时（镜像 servo.c bite 模型，跨端同步责任）。

    bite 轴（AXIS_STEP_MODE[i]==1）：自适应 bite 序列，每 bite 从静止起速。
      Δ > BITE_FAR_THR → bite=BITE_FAR_DEG；> BITE_MID_THR → MID；否则 NEAR。
      每 bite 滑动时间：峰值 v=sqrt(a·b)≤vmax 时 t=2·sqrt(b/a)，
      否则 t=b/vmax+vmax/a（梯形）。n_bites=ceil(Δ/b)，间停 (n-1)·DWELL。
    连续轴（AXIS_STEP_MODE[i]==0）：t=Δ/vmax+vmax/a（梯形速度剖面）。
    六轴取 max，加 SETTLE 余量。

    Args:
        from_q: 起始关节角列表 [6]（°）
        to_q:   目标关节角列表 [6]（°）

    Returns:
        float 预估运动秒数（≥ 0）
    """
    a = config.BITE_ACC_DEG_S2
    _SETTLE = 0.2  # settle 余量 s（并入调用方 EXTRA 或独立使用）

    max_t = 0.0
    for i in range(6):
        delta = abs(to_q[i] - from_q[i])
        if delta < 1e-6:
            continue
        vmax = config.AXIS_MAX_VEL[i]
        if config.AXIS_STEP_MODE[i] == 0:
            # 连续轴：梯形速度剖面 t = Δ/vmax + vmax/a
            t = delta / vmax + vmax / a
        else:
            # bite 轴：自适应 bite 宽度
            if delta > config.BITE_FAR_THR:
                b = config.BITE_FAR_DEG
            elif delta > config.BITE_MID_THR:
                b = config.BITE_MID_DEG
            else:
                b = config.BITE_NEAR_DEG
            n_bites = max(1, int(delta / b + 0.999))  # ceil
            # 每 bite 滑动时间（从静止起速）
            v_peak = (a * b) ** 0.5
            if v_peak <= vmax:
                t_bite = 2.0 * (b / a) ** 0.5  # 三角形：2·sqrt(b/a)
            else:
                t_bite = b / vmax + vmax / a    # 梯形
            t = n_bites * t_bite + max(0, n_bites - 1) * config.BITE_DWELL_S
        if t > max_t:
            max_t = t
    return max_t + _SETTLE


def compute_wrist_correction(err_mm, q_current, center=None):
    """计算 J4 腕层修正量（量子化 + slack 钳制）。

    center = J4 回合锚点：最近一次粗层（IK）解设定的 J4 值，腕层相对它
    钳制 slack。**不用绝对式 90−J2−J3**——其零位参考是真机标定项
    （KNOWN_TRAPS #28，HOME 处给出 −199 非法中心）。腕修正期间
    q1/q2/q3 静止、center 不漂移；粗层介入后调用方置 center=None 重锚。

    方向分解（J4 管 z 向 / J5 管横向）待装机日实测，v1 按幅度路由。

    Args:
        err_mm:    像素误差换算后的毫米误差（|err|，正值）
        q_current: 当前六轴关节角列表 [6]
        center:    J4 锚点 °；None → 锚在当前 q4（回合起点）

    Returns:
        (dq4, q4_target)  dq4=修正增量 °（含符号），q4_target=修正后 J4 目标角。
        若无需修正返回 (0.0, q_current[3])。
    """
    # L4·sin(1°) = 每度对应的末端位移 mm（自动跟随 L4 变化）
    mm_per_deg = config.L4 * math.sin(math.radians(1.0))
    # 所需 J4 角度修正量
    dq4_raw = err_mm / mm_per_deg
    # 量子化到 WRIST_QUANTUM_DEG 倍数（保号）
    sign = 1.0 if dq4_raw >= 0 else -1.0
    dq4_q = sign * config.WRIST_QUANTUM_DEG * int(abs(dq4_raw) / config.WRIST_QUANTUM_DEG + 0.999)
    if abs(dq4_q) < 1e-6:
        return 0.0, q_current[3]
    if center is None:
        center = q_current[3]
    q4_new = q_current[3] + dq4_q
    # slack 钳制：|q4 − center| ≤ J4_SLACK_DEG
    q4_lo = center - config.J4_SLACK_DEG
    q4_hi = center + config.J4_SLACK_DEG
    q4_clamped = max(q4_lo, min(q4_hi, q4_new))
    dq4 = q4_clamped - q_current[3]
    if abs(dq4) < 1e-6:
        return 0.0, q_current[3]
    return dq4, q_current[3] + dq4
