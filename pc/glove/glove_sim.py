# -*- coding: utf-8 -*-
"""glove_sim.py — 手势手套仿真主程序（Sim1 + Sim2）。

管线：
  关键帧轨迹 → 合成 IMU（体轴重力+角速度+噪声）
  → Mahony @100Hz → 欧拉角流
  → 20Hz 抽取 → 映射（A/B 同时算）→ 死区 → EMA → 限步 → 舵机目标流
  → 指标统计。

指标：
  Sim1 判据：静态段欧拉误差 mean < 1°、max < 1°。
  Sim2 判据：抖动 std < 0.3°/拍；servo_clamp ≤ 轴拍数的 5%（饱和检查）。

用法：
  cd pc && python glove\glove_sim.py --seconds 12 --noise-scale 1.0 --scheme both
"""

import argparse
import json
import math
import os
import random
import sys
import time

# 同级 pc 模块 import（照抄 state_machine.py 风格）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
import kinematics

from glove.imu_filter import MahonyFilter
from glove.mapping import scheme_a, scheme_b, scheme_b_init


# =====================================================================
#  轨迹定义（关键帧插值）
# =====================================================================

# 默认轨迹 ~12s：≥3 静止保持段 + ≥3 连续运动段
# 格式：(t, pitch_deg, roll_deg, yaw_deg)
DEFAULT_KEYFRAMES = [
    # --- 保持段 1：中性位 ---
    (0.0,   0.0,   0.0,   0.0),
    (2.0,   0.0,   0.0,   0.0),
    # --- 运动段 1：pitch↑ roll↑ yaw↑ ---
    (4.0,  30.0,  20.0,  15.0),
    # --- 保持段 2 ---
    (6.0,  30.0,  20.0,  15.0),
    # --- 运动段 2：pitch↓ roll↓ ---
    (8.0, -20.0, -10.0,  15.0),
    # --- 保持段 3 ---
    (10.0, -20.0, -10.0,  15.0),
    # --- 运动段 3：pitch↑ yaw↓ ---
    (12.0,  10.0, -10.0, -10.0),
]


def _lerp(a, b, t):
    """线性插值。t ∈ [0, 1]。"""
    return a + (b - a) * t


def interpolate_keyframes(keyframes, t):
    """关键帧线性插值，返回 (pitch, roll, yaw) [deg]。

    t 超出范围时钳位到首/末帧值。
    """
    if t <= keyframes[0][0]:
        return keyframes[0][1], keyframes[0][2], keyframes[0][3]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1], keyframes[-1][2], keyframes[-1][3]

    for i in range(len(keyframes) - 1):
        t0, p0, r0, y0 = keyframes[i]
        t1, p1, r1, y1 = keyframes[i + 1]
        if t0 <= t <= t1:
            frac = (t - t0) / (t1 - t0)
            return (_lerp(p0, p1, frac),
                    _lerp(r0, r1, frac),
                    _lerp(y0, y1, frac))

    # 理论不可达
    return keyframes[-1][1], keyframes[-1][2], keyframes[-1][3]


def is_hold_segment(keyframes, t):
    """判断 t 是否处于保持段（前后帧值相同）。"""
    for i in range(len(keyframes) - 1):
        t0, p0, r0, y0 = keyframes[i]
        t1, p1, r1, y1 = keyframes[i + 1]
        if t0 <= t <= t1:
            # 保持段 = pitch/roll/yaw 全部不变
            return (abs(p1 - p0) < 1e-6 and
                    abs(r1 - r0) < 1e-6 and
                    abs(y1 - y0) < 1e-6)
    return False


# =====================================================================
#  IMU 合成
# =====================================================================

class IMUGenerator:
    """从真值轨迹合成体轴 IMU 数据。

    体轴约定（MPU6050 贴手背，X 沿手指方向）：
      - 加速度计：静止时体轴 Z 向上 = [0, 0, +g]
      - 陀螺仪：体轴角速度 = 世界角速度旋转到体轴

    注意：这里用欧拉角运动学方程 E(θ)·θ̇ 近似体轴角速度，
    比简单近似精确得多（误差 <1°，仿真可接受）。
    """

    G = 9.80665  # m/s²

    def __init__(self, noise_scale=1.0):
        self.noise_scale = noise_scale
        # 噪声常量（从 config 读取，乘以 noise_scale）
        self._gyro_bias_x = config.GLOVE_GYRO_BIAS_X_DEG_S
        self._gyro_bias_y = config.GLOVE_GYRO_BIAS_Y_DEG_S
        self._gyro_bias_z = config.GLOVE_GYRO_BIAS_Z_DEG_S
        self._gyro_noise_std = config.GLOVE_GYRO_NOISE_DEG_S
        self._accel_noise_std = config.GLOVE_ACCEL_NOISE_M_S2

    def generate(self, roll_deg, pitch_deg, yaw_deg,
                 roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s):
        """由真值姿态 + 角速度 → 合成体轴 IMU 数据。

        Args:
            roll_deg, pitch_deg, yaw_deg: 真值欧拉角 [deg]。
            roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s: 欧拉角导数 [deg/s]。

        Returns:
            (gyro_rad_s[3], accel_m_s2[3])
        """
        ns = self.noise_scale

        # --- 体轴重力（加速度计读数）---
        # 加速度计静止时读支撑力 [0, 0, +g]（世界系 Z 向上）。
        # 旋转到体轴：a_body = g * [−sin(pitch), sin(roll)·cos(pitch), cos(roll)·cos(pitch)]
        # 推导：R_body_to_world 第三行 = [−sin(p), sin(r)·cos(p), cos(r)·cos(p)]
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)

        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)

        ax = -self.G * sp
        ay = self.G * sr * cp
        az = self.G * cr * cp

        # 加速度白噪
        ax += random.gauss(0, self._accel_noise_std * ns)
        ay += random.gauss(0, self._accel_noise_std * ns)
        az += random.gauss(0, self._accel_noise_std * ns)

        # --- 体轴角速度（欧拉角运动学方程）---
        # ω_body = E(θ) · θ̇
        #   ωx = θ̇roll − sin(pitch) · θ̇yaw
        #   ωy = cos(roll) · θ̇pitch + sin(roll) · cos(pitch) · θ̇yaw
        #   ωz = −sin(roll) · θ̇pitch + cos(roll) · cos(pitch) · θ̇yaw
        gx_rad = math.radians(roll_rate_deg_s) - sp * math.radians(yaw_rate_deg_s)
        gy_rad = cr * math.radians(pitch_rate_deg_s) + sr * cp * math.radians(yaw_rate_deg_s)
        gz_rad = -sr * math.radians(pitch_rate_deg_s) + cr * cp * math.radians(yaw_rate_deg_s)

        # 陀螺零偏 + 白噪
        gx_rad += math.radians(self._gyro_bias_x * ns + random.gauss(0, self._gyro_noise_std * ns))
        gy_rad += math.radians(self._gyro_bias_y * ns + random.gauss(0, self._gyro_noise_std * ns))
        gz_rad += math.radians(self._gyro_bias_z * ns + random.gauss(0, self._gyro_noise_std * ns))

        return [gx_rad, gy_rad, gz_rad], [ax, ay, az]


# =====================================================================
#  管线
# =====================================================================

def run_pipeline(seconds, noise_scale, scheme, keyframes=None):
    """运行仿真管线，返回指标和 trace 数据。

    Args:
        seconds: 仿真时长 s。
        noise_scale: 噪声缩放因子。
        scheme: "a" / "b" / "both"。
        keyframes: 自定义关键帧列表（None 用默认）。

    Returns:
        (metrics_dict, trace_list)
    """
    if keyframes is None:
        keyframes = DEFAULT_KEYFRAMES

    dt_imu = 1.0 / config.GLOVE_IMU_HZ  # 0.01s
    dt_send = 1.0 / config.GLOVE_SEND_HZ  # 0.05s
    n_imu = int(seconds * config.GLOVE_IMU_HZ)
    send_interval = max(1, int(config.GLOVE_IMU_HZ / config.GLOVE_SEND_HZ))  # 5
    n_send = int(seconds * config.GLOVE_SEND_HZ)  # 总发送拍数（用于 5% 阈值）

    # --- 初始化 ---
    mahony = MahonyFilter()
    imu_gen = IMUGenerator(noise_scale)
    state_b = scheme_b_init() if scheme in ("b", "both") else None

    # EMA 状态（首拍快照初始化，见主循环 ema_snapshot 标记）
    ema_a = [0.0] * 6  # 方案 A EMA（首拍会被覆盖）
    ema_b = [0.0] * 6  # 方案 B EMA（首拍会被覆盖）
    prev_a_servo = [0.0] * 6  # 首拍会被覆盖
    prev_b_servo = [0.0] * 6
    prev_a = [0.0] * 6  # 首拍会被覆盖
    prev_b = [0.0] * 6

    # 指标收集
    static_errors_a = []
    static_errors_b = []
    dynamic_errors_a = []
    dynamic_errors_b = []
    servo_jitter_a = []    # 逐拍增量（用于 std）
    servo_jitter_b = []

    # 钳位拆分
    servo_clamp_a = 0  # 方案 A：servo_limits 饱和（最终输出在限位边界）
    step_clamp_a = 0   # 方案 A：限步器触发（正常平滑信号）
    joint_clamp_a = 0  # 方案 A：关节域钳位（j4 等越界被 clamp）
    servo_clamp_b = 0
    step_clamp_b = 0
    joint_clamp_b = 0

    # span 追踪（每轴在整个运行中的 max - min）
    span_a_min = [float('inf')] * 6
    span_a_max = [float('-inf')] * 6
    span_b_min = [float('inf')] * 6
    span_b_max = [float('-inf')] * 6

    # 响应性追踪
    max_delta_a = 0.0   # 方案 A 单拍最大关节增量 |Δ|
    max_delta_b = 0.0
    sum_abs_delta_a = 0.0  # 方案 A |Δ| 累计（用于平均速度）
    sum_abs_delta_b = 0.0

    ik_fail_count_b = 0
    ee_path_length = 0.0
    ee_max_vel = 0.0
    ee_prev = None  # None = 首拍不计增量（避免跳变污染）

    # trace 数据
    trace = []

    # 运动段/保持段计数
    n_hold = 0
    n_motion = 0
    for i in range(len(keyframes) - 1):
        t0 = keyframes[i][0]
        t1 = keyframes[i + 1][0]
        p0, r0, y0 = keyframes[i][1], keyframes[i][2], keyframes[i][3]
        p1, r1, y1 = keyframes[i + 1][1], keyframes[i + 1][2], keyframes[i + 1][3]
        if abs(p1 - p0) < 1e-6 and abs(r1 - r0) < 1e-6 and abs(y1 - y0) < 1e-6:
            n_hold += 1
        else:
            n_motion += 1

    print("轨迹: %d 帧, %.1fs, %d 保持段, %d 运动段" % (len(keyframes), seconds, n_hold, n_motion))

    # --- 主循环 ---
    send_counter = 0
    true_euler_prev = None
    warmup_steps = int(0.5 * config.GLOVE_IMU_HZ)  # 前 0.5s 滤波器收敛期，不计入指标
    ema_snapshot_a = False  # 方案 A：首拍 EMA 快照标记
    ema_snapshot_b = False  # 方案 B：首拍 EMA 快照标记

    for step in range(n_imu):
        t = step * dt_imu

        # 真值姿态
        pitch_t, roll_t, yaw_t = interpolate_keyframes(keyframes, t)

        # 欧拉角导数（有限差分）
        if true_euler_prev is not None:
            roll_rate = (roll_t - true_euler_prev[0]) / dt_imu
            pitch_rate = (pitch_t - true_euler_prev[1]) / dt_imu
            yaw_rate = (yaw_t - true_euler_prev[2]) / dt_imu
        else:
            roll_rate, pitch_rate, yaw_rate = 0.0, 0.0, 0.0
        true_euler_prev = (roll_t, pitch_t, yaw_t)

        # 合成 IMU
        gyro, accel = imu_gen.generate(roll_t, pitch_t, yaw_t,
                                       roll_rate, pitch_rate, yaw_rate)

        # Mahony 滤波
        roll_est, pitch_est, yaw_est = mahony.update(gyro, accel, dt_imu)

        # 20Hz 抽取
        send_counter += 1
        if send_counter < send_interval:
            continue
        send_counter = 0

        t_send = t  # 近似（误差 ≤ dt_imu，可接受）

        # 真值（20Hz 层）
        pitch_t_20, roll_t_20, yaw_t_20 = interpolate_keyframes(keyframes, t_send)
        is_hold = is_hold_segment(keyframes, t_send)

        # 误差 = 估计 - 真值（仅用于 Sim1 滤波精度指标）
        err_pitch = pitch_est - pitch_t_20
        err_roll = roll_est - roll_t_20
        err_yaw = yaw_est - yaw_t_20

        # 保持/运动段分类（跳过前 0.5s 滤波器收敛期）
        if step < warmup_steps:
            pass  # 滤波器未收敛，不计入指标
        elif is_hold:
            static_errors_a.append((abs(err_roll), abs(err_pitch), abs(err_yaw)))
            static_errors_b.append((abs(err_roll), abs(err_pitch), abs(err_yaw)))
        else:
            dynamic_errors_a.append((abs(err_roll), abs(err_pitch), abs(err_yaw)))
            dynamic_errors_b.append((abs(err_roll), abs(err_pitch), abs(err_yaw)))

        # 映射输入：滤波后欧拉角相对于中性姿态的误差（规格 §4 AD-4d）
        # 中性姿态 = (0,0,0)，故输入就是滤波后欧拉角本身。
        # 死区作用于映射输入，过滤小幅抖动，不是过滤滤波误差。
        map_pitch = _apply_deadband(pitch_est, config.GLOVE_DEADBAND_DEG)
        map_roll = _apply_deadband(roll_est, config.GLOVE_DEADBAND_DEG)
        map_yaw = _apply_deadband(yaw_est, config.GLOVE_DEADBAND_DEG)

        # --- 方案 A ---
        if scheme in ("a", "both"):
            targets_a_raw, q_a, jc_a = scheme_a(map_pitch, map_roll, map_yaw)
            joint_clamp_a += jc_a

            # EMA 平滑（在舵机域）
            if not ema_snapshot_a:
                # 首拍：快照 EMA 到原始目标（跳过爬坡瞬态）
                ema_a = list(targets_a_raw)
                prev_a_servo = list(targets_a_raw)
                prev_a = list(targets_a_raw)
                ema_snapshot_a = True
            else:
                for j in range(6):
                    ema_a[j] = (config.GLOVE_EMA_ALPHA * targets_a_raw[j]
                                + (1.0 - config.GLOVE_EMA_ALPHA) * ema_a[j])

            # EMA 输出钳位到舵机限位（防止 EMA 积累越界）
            for j in range(6):
                lo, hi = kinematics.servo_limits(j)
                if ema_a[j] < lo:
                    ema_a[j] = lo
                elif ema_a[j] > hi:
                    ema_a[j] = hi

            # 限步（舵机域逐拍增量 ≤ GLOVE_STEP_MAX_DEG）
            targets_a, sc_a = _step_limit_with_count(ema_a, prev_a_servo,
                                                      config.GLOVE_STEP_MAX_DEG)
            step_clamp_a += sc_a
            prev_a_servo = list(targets_a)

            # servo_limits 饱和检测：最终输出是否在限位边界
            for j in range(6):
                lo, hi = kinematics.servo_limits(j)
                if abs(targets_a[j] - lo) < 1e-6 or abs(targets_a[j] - hi) < 1e-6:
                    servo_clamp_a += 1

            # span 追踪
            for j in range(6):
                if targets_a[j] < span_a_min[j]:
                    span_a_min[j] = targets_a[j]
                if targets_a[j] > span_a_max[j]:
                    span_a_max[j] = targets_a[j]

            # 抖动 + 响应性
            delta_a = [targets_a[j] - prev_a[j] for j in range(6)]
            servo_jitter_a.append(delta_a)
            for j in range(6):
                ad = abs(delta_a[j])
                if ad > max_delta_a:
                    max_delta_a = ad
                sum_abs_delta_a += ad
            prev_a = list(targets_a)

        # --- 方案 B ---
        targets_b_raw = None
        q_b = None
        ee_pos = None
        if scheme in ("b", "both"):
            targets_b_raw, q_b, ee_pos_b, jc_b = scheme_b(map_pitch, map_roll,
                                                            map_yaw, dt_send, state_b)
            joint_clamp_b += jc_b
            if q_b is not None:
                # IK 成功 → 完整处理
                ee_pos = ee_pos_b

                # EMA 平滑
                if not ema_snapshot_b:
                    # 首拍：快照 EMA 到原始目标
                    ema_b = list(targets_b_raw)
                    prev_b_servo = list(targets_b_raw)
                    prev_b = list(targets_b_raw)
                    ema_snapshot_b = True
                else:
                    for j in range(6):
                        ema_b[j] = (config.GLOVE_EMA_ALPHA * targets_b_raw[j]
                                    + (1.0 - config.GLOVE_EMA_ALPHA) * ema_b[j])

                # EMA 输出钳位到舵机限位
                for j in range(6):
                    lo, hi = kinematics.servo_limits(j)
                    if ema_b[j] < lo:
                        ema_b[j] = lo
                    elif ema_b[j] > hi:
                        ema_b[j] = hi

                # 限步
                targets_b, sc_b = _step_limit_with_count(ema_b, prev_b_servo,
                                                          config.GLOVE_STEP_MAX_DEG)
                step_clamp_b += sc_b
                prev_b_servo = list(targets_b)

                # servo_limits 饱和检测
                for j in range(6):
                    lo, hi = kinematics.servo_limits(j)
                    if abs(targets_b[j] - lo) < 1e-6 or abs(targets_b[j] - hi) < 1e-6:
                        servo_clamp_b += 1

                # span 追踪
                for j in range(6):
                    if targets_b[j] < span_b_min[j]:
                        span_b_min[j] = targets_b[j]
                    if targets_b[j] > span_b_max[j]:
                        span_b_max[j] = targets_b[j]

                # 抖动 + 响应性
                delta_b = [targets_b[j] - prev_b[j] for j in range(6)]
                servo_jitter_b.append(delta_b)
                for j in range(6):
                    ad = abs(delta_b[j])
                    if ad > max_delta_b:
                        max_delta_b = ad
                    sum_abs_delta_b += ad
                prev_b = list(targets_b)

                # EE 路径长度（3D）— 首拍不计（ee_prev=None）
                if ee_prev is not None and ee_pos is not None:
                    dx = ee_pos[0] - ee_prev[0]
                    dy = ee_pos[1] - ee_prev[1]
                    dz = ee_pos[2] - ee_prev[2]
                    step_len = math.sqrt(dx * dx + dy * dy + dz * dz)
                    ee_path_length += step_len
                    ee_max_vel = max(ee_max_vel, step_len / dt_send)
                if ee_pos is not None:
                    ee_prev = list(ee_pos)
            else:
                # IK 失败 → 保持上一拍（targets_b_raw = prev servo）
                ik_fail_count_b += 1
                targets_b = prev_b_servo  # 保持上一拍

        # --- trace ---
        entry = {
            "t": round(t_send, 4),
            "euler_true": [round(roll_t_20, 3), round(pitch_t_20, 3), round(yaw_t_20, 3)],
            "euler_est": [round(roll_est, 3), round(pitch_est, 3), round(yaw_est, 3)],
        }
        if scheme in ("a", "both"):
            entry["servo_a"] = [round(v, 2) for v in targets_a]
        if scheme in ("b", "both"):
            entry["servo_b"] = [round(v, 2) for v in targets_b] if q_b is not None else None
            entry["ee"] = ([round(ee_pos[0], 2), round(ee_pos[1], 2), round(ee_pos[2], 2)]
                           if ee_pos is not None else None)
        trace.append(entry)

    # --- 汇总指标 ---
    metrics = {}
    if scheme in ("a", "both") and static_errors_a:
        metrics["a_static_mean"] = _mean_triple(static_errors_a)
        metrics["a_static_max"] = _max_triple(static_errors_a)
    if scheme in ("a", "both") and dynamic_errors_a:
        metrics["a_dynamic_mean"] = _mean_triple(dynamic_errors_a)
        metrics["a_dynamic_max"] = _max_triple(dynamic_errors_a)
    if scheme in ("b", "both") and static_errors_b:
        metrics["b_static_mean"] = _mean_triple(static_errors_b)
        metrics["b_static_max"] = _max_triple(static_errors_b)
    if scheme in ("b", "both") and dynamic_errors_b:
        metrics["b_dynamic_mean"] = _mean_triple(dynamic_errors_b)
        metrics["b_dynamic_max"] = _max_triple(dynamic_errors_b)
    if servo_jitter_a:
        metrics["a_jitter_std"] = _jitter_std(servo_jitter_a)
    if servo_jitter_b:
        metrics["b_jitter_std"] = _jitter_std(servo_jitter_b)

    # 钳位拆分
    metrics["a_servo_clamp"] = servo_clamp_a
    metrics["a_step_clamp"] = step_clamp_a
    metrics["a_joint_clamp"] = joint_clamp_a
    metrics["b_servo_clamp"] = servo_clamp_b
    metrics["b_step_clamp"] = step_clamp_b
    metrics["b_joint_clamp"] = joint_clamp_b

    # span（每轴 max - min）
    if scheme in ("a", "both"):
        metrics["a_span"] = [round(span_a_max[j] - span_a_min[j], 2) if span_a_min[j] != float('inf') else 0.0
                             for j in range(6)]
    if scheme in ("b", "both"):
        metrics["b_span"] = [round(span_b_max[j] - span_b_min[j], 2) if span_b_min[j] != float('inf') else 0.0
                             for j in range(6)]

    # 响应性（avg_vel = 6轴累计/6，单轴平均 °/s）
    metrics["a_max_delta"] = round(max_delta_a, 3)
    metrics["b_max_delta"] = round(max_delta_b, 3)
    if servo_jitter_a:
        metrics["a_avg_vel"] = round(sum_abs_delta_a / 6.0 / seconds, 3)  # 单轴平均 °/s
    else:
        metrics["a_avg_vel"] = 0.0
    if servo_jitter_b:
        metrics["b_avg_vel"] = round(sum_abs_delta_b / 6.0 / seconds, 3)
    else:
        metrics["b_avg_vel"] = 0.0

    metrics["b_ik_fail_count"] = ik_fail_count_b
    metrics["b_ee_path_length"] = round(ee_path_length, 2)
    metrics["b_ee_max_vel"] = round(ee_max_vel, 2)
    metrics["n_send"] = n_send  # 总发送拍数（用于 5% 阈值计算）

    return metrics, trace


# =====================================================================
#  辅助函数
# =====================================================================

def _apply_deadband(error_deg, deadband_deg):
    """死区：|error| ≤ deadband → 0；否则保持原值。"""
    if abs(error_deg) <= deadband_deg:
        return 0.0
    return error_deg


def _step_limit_with_count(targets, prev, max_step):
    """限步：逐拍增量不超过 max_step [deg]。

    Returns:
        (result[6], clamp_count) — 结果和触发限步的轴数。
    """
    result = []
    count = 0
    for j in range(6):
        delta = targets[j] - prev[j]
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
            count += 1
        result.append(prev[j] + delta)
    return result, count


def _mean_triple(error_list):
    """三轴误差列表 → (roll_mean, pitch_mean, yaw_mean)。"""
    n = len(error_list)
    if n == 0:
        return (0.0, 0.0, 0.0)
    sr = sum(e[0] for e in error_list) / n
    sp = sum(e[1] for e in error_list) / n
    sy = sum(e[2] for e in error_list) / n
    return (round(sr, 3), round(sp, 3), round(sy, 3))


def _max_triple(error_list):
    """三轴误差列表 → (roll_max, pitch_max, yaw_max)。"""
    if not error_list:
        return (0.0, 0.0, 0.0)
    mr = max(e[0] for e in error_list)
    mp = max(e[1] for e in error_list)
    my = max(e[2] for e in error_list)
    return (round(mr, 3), round(mp, 3), round(my, 3))


def _jitter_std(delta_list):
    """舵机增量列表 → 6 轴 std 的平均值。"""
    if not delta_list:
        return 0.0
    n = len(delta_list)
    stds = []
    for j in range(6):
        vals = [d[j] for d in delta_list]
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / n
        stds.append(math.sqrt(var))
    return round(sum(stds) / 6, 4)


def home_ee_pos():
    """HOME 姿态的 EE (x, y, z)。"""
    x, y, z = kinematics.fk(config.JOINT_HOME)
    return x, y, z


# =====================================================================
#  指标打印
# =====================================================================

def print_metrics(metrics, scheme):
    """控制台对齐指标表。"""
    print()
    print("=" * 72)
    print("  手套仿真指标报告")
    print("=" * 72)

    # Sim1 判据
    print("\n【Sim1 滤波精度】")
    print("  判据: 静态段误差 mean < 1.0°, max < 1.0°")
    if scheme in ("a", "both"):
        sm = metrics.get("a_static_mean", (0, 0, 0))
        sx = metrics.get("a_static_max", (0, 0, 0))
        s1_pass = all(v < 1.0 for v in sx)
        print("  方案 A 静态: mean=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)  %s" % (
            sm[0], sm[1], sm[2], sx[0], sx[1], sx[2], "PASS" if s1_pass else "FAIL"))
    if scheme in ("b", "both"):
        sm = metrics.get("b_static_mean", (0, 0, 0))
        sx = metrics.get("b_static_max", (0, 0, 0))
        s1_pass = all(v < 1.0 for v in sx)
        print("  方案 B 静态: mean=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)  %s" % (
            sm[0], sm[1], sm[2], sx[0], sx[1], sx[2], "PASS" if s1_pass else "FAIL"))

    print("\n  动态段误差:")
    if scheme in ("a", "both"):
        dm = metrics.get("a_dynamic_mean", (0, 0, 0))
        dx = metrics.get("a_dynamic_max", (0, 0, 0))
        print("  方案 A 动态: mean=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)" % (
            dm[0], dm[1], dm[2], dx[0], dx[1], dx[2]))
    if scheme in ("b", "both"):
        dm = metrics.get("b_dynamic_mean", (0, 0, 0))
        dx = metrics.get("b_dynamic_max", (0, 0, 0))
        print("  方案 B 动态: mean=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)" % (
            dm[0], dm[1], dm[2], dx[0], dx[1], dx[2]))

    # Sim2 判据
    print("\n【Sim2 映射质量】")
    n_send = metrics.get("n_send", 240)
    clamp_5pct = int(n_send * 0.05)  # 轴拍数的 5%
    print("  判据: 抖动 std < 0.3°/拍, servo_clamp ≤ %d (5%% of %d 轴拍)" % (clamp_5pct, n_send))

    if scheme in ("a", "both"):
        jit = metrics.get("a_jitter_std", 0.0)
        sc = metrics.get("a_servo_clamp", 0)
        s2_pass = jit < 0.3 and sc <= clamp_5pct
        print("  方案 A 抖动 std: %.4f°/拍  %s" % (jit, "PASS" if jit < 0.3 else "FAIL"))
        print("  方案 A servo_clamp: %d  %s" % (sc, "PASS" if sc <= clamp_5pct else "FAIL"))
        print("  方案 A step_clamp: %d" % metrics.get("a_step_clamp", 0))
        jc_a = metrics.get("a_joint_clamp", 0)
        jc_pct = jc_a * 100.0 / (6 * n_send) if n_send > 0 else 0.0
        print("  方案 A joint_clamp: %d (%.1f%%)%s" % (
            jc_a, jc_pct, "  WARN >20%" if jc_pct > 20.0 else ""))
        span = metrics.get("a_span", [0.0] * 6)
        print("  方案 A span: [%s]°" % ", ".join("%.2f" % v for v in span))
        print("  方案 A 响应性: max|Δ|=%.3f°, avg_vel=%.3f°/s（单轴平均）" % (
            metrics.get("a_max_delta", 0.0), metrics.get("a_avg_vel", 0.0)))

    if scheme in ("b", "both"):
        jit = metrics.get("b_jitter_std", 0.0)
        sc = metrics.get("b_servo_clamp", 0)
        s2_pass = jit < 0.3 and sc <= clamp_5pct
        print("  方案 B 抖动 std: %.4f°/拍  %s" % (jit, "PASS" if jit < 0.3 else "FAIL"))
        print("  方案 B servo_clamp: %d  %s" % (sc, "PASS" if sc <= clamp_5pct else "FAIL"))
        print("  方案 B step_clamp: %d" % metrics.get("b_step_clamp", 0))
        jc_b = metrics.get("b_joint_clamp", 0)
        jc_pct_b = jc_b * 100.0 / (6 * n_send) if n_send > 0 else 0.0
        print("  方案 B joint_clamp: %d (%.1f%%)%s" % (
            jc_b, jc_pct_b, "  WARN >20%" if jc_pct_b > 20.0 else ""))
        span = metrics.get("b_span", [0.0] * 6)
        print("  方案 B span: [%s]°" % ", ".join("%.2f" % v for v in span))
        print("  方案 B 响应性: max|Δ|=%.3f°, avg_vel=%.3f°/s（单轴平均）" % (
            metrics.get("b_max_delta", 0.0), metrics.get("b_avg_vel", 0.0)))
        print("  方案 B IK 失败: %d" % metrics.get("b_ik_fail_count", 0))
        print("  方案 B EE 路径长度: %.2f mm" % metrics.get("b_ee_path_length", 0.0))
        print("  方案 B EE 最大速度: %.2f mm/s" % metrics.get("b_ee_max_vel", 0.0))

    print("\n" + "=" * 72)


# =====================================================================
#  主入口
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description="手势手套仿真（Sim1+Sim2）")
    parser.add_argument("--seconds", type=float, default=12.0,
                        help="仿真时长 s（默认 12）")
    parser.add_argument("--noise-scale", type=float, default=1.0,
                        help="噪声缩放因子（默认 1.0）")
    parser.add_argument("--scheme", type=str, default="both",
                        choices=["a", "b", "both"],
                        help="映射方案（默认 both）")
    parser.add_argument("--ema-alpha", type=float, default=None,
                        help="覆盖 GLOVE_EMA_ALPHA（调参用）")
    parser.add_argument("--step-max", type=float, default=None,
                        help="覆盖 GLOVE_STEP_MAX_DEG（调参用）")
    args = parser.parse_args()

    # 调参覆盖
    if args.ema_alpha is not None:
        config.GLOVE_EMA_ALPHA = args.ema_alpha
    if args.step_max is not None:
        config.GLOVE_STEP_MAX_DEG = args.step_max

    print("手势手套仿真 — Sim1+Sim2")
    print("  时长: %.1fs, 噪声: ×%.1f, 方案: %s" % (
        args.seconds, args.noise_scale, args.scheme))
    print("  IMU: %.0fHz, 发送: %.0fHz, 死区: %.1f°, EMA: %.2f, 限步: %.1f°" % (
        config.GLOVE_IMU_HZ, config.GLOVE_SEND_HZ, config.GLOVE_DEADBAND_DEG,
        config.GLOVE_EMA_ALPHA, config.GLOVE_STEP_MAX_DEG))

    t0 = time.time()
    metrics, trace = run_pipeline(args.seconds, args.noise_scale, args.scheme)
    elapsed = time.time() - t0
    print("\n计算耗时: %.2fs" % elapsed)

    print_metrics(metrics, args.scheme)

    # 输出 trace.json
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(out_dir, exist_ok=True)
    trace_path = os.path.join(out_dir, "trace.json")
    with open(trace_path, "w", encoding="utf-8") as f:
        json.dump(trace, f, ensure_ascii=False, indent=2)
    print("\ntrace.json 已写入: %s (%d 条, %.1f KB)" % (
        trace_path, len(trace), os.path.getsize(trace_path) / 1024.0))


if __name__ == "__main__":
    main()
