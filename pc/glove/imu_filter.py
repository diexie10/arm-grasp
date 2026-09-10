# -*- coding: utf-8 -*-
"""imu_filter.py — Mahony AHRS 姿态滤波器（纯 Python，无 numpy）。

标准 Mahony 互补滤波器：四元数 + 比例积分修正，重力参考 [0,0,1]（Z 向上）。
输入：体轴角速度 [rad/s]、体轴加速度 [m/s²]、时间步长 dt [s]。
输出：ZYX 欧拉角 (roll, pitch, yaw) [deg]，pitch 截断 [-90, 90]。

参考：Mahony, R. et al. "Nonlinear Complementary Filters on the Special Orthogonal Group"
"""

import math

import config


def _quat_normalize(q):
    """单位化四元数 q = [w, x, y, z]。"""
    norm = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    if norm < 1e-10:
        return [1.0, 0.0, 0.0, 0.0]
    inv = 1.0 / norm
    return [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]


def _quat_multiply(a, b):
    """四元数乘法 a ⊗ b。"""
    return [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]


def _quat_to_euler(q):
    """四元数 → ZYX 欧拉角 (roll, pitch, yaw) [deg]。

    ZYX 约定：先 yaw(Z)，再 pitch(Y)，最后 roll(X)。
    roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
    pitch = asin(2(q0q2 - q3q1))   截断到 [-90°, 90°]
    yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
    """
    w, x, y, z = q
    # roll (X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # pitch (Y) — 截断到 [-90°, 90°]，防止 gimbal lock 处符号翻转
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # yaw (Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


class MahonyFilter:
    """Mahony AHRS 滤波器。

    使用方法：
        f = MahonyFilter()
        f.reset()
        for each sample:
            roll, pitch, yaw = f.update(gyro, accel, dt)
    """

    def __init__(self):
        """从 config 读取 kp/ki，初始化单位四元数。"""
        self.kp = config.GLOVE_MAHONY_KP
        self.ki = config.GLOVE_MAHONY_KI
        self._q = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        self._integral_fb = [0.0, 0.0, 0.0]  # 积分反馈项

    def reset(self):
        """重置为单位四元数（无姿态）。"""
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._integral_fb = [0.0, 0.0, 0.0]

    def update(self, gyro_rad_s, accel_m_s2, dt):
        """一步滤波更新。

        Args:
            gyro_rad_s: 体轴角速度 [gx, gy, gz]，单位 rad/s。
            accel_m_s2: 体轴加速度 [ax, ay, az]，单位 m/s²。
            dt: 时间步长 s（通常 1/IMU_HZ）。

        Returns:
            (roll_deg, pitch_deg, yaw_deg) — ZYX 欧拉角，pitch 截断 [-90, 90]。

        Raises:
            ValueError: dt 非正时。
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive, got %.6f" % dt)

        gx, gy, gz = gyro_rad_s
        ax, ay, az = accel_m_s2

        # 归一化加速度（重力参考）
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 1e-10:
            # 加速度计失效（自由落体），跳过修正，仅用陀螺积分
            ax_n, ay_n, az_n = 0.0, 0.0, 1.0
        else:
            inv_a = 1.0 / a_norm
            ax_n = ax * inv_a
            ay_n = ay * inv_a
            az_n = az * inv_a

        # 估计重力方向（从当前四元数提取）
        q0, q1, q2, q3 = self._q
        # 旋转矩阵第三列（重力在体轴的估计投影）
        v_x = 2.0 * (q1 * q3 - q0 * q2)
        v_y = 2.0 * (q0 * q1 + q2 * q3)
        v_z = 1.0 - 2.0 * (q1 * q1 + q2 * q2)

        # 误差 = 估计重力 × 加速度方向（叉积）
        ex = ay_n * v_z - az_n * v_y
        ey = az_n * v_x - ax_n * v_z
        ez = ax_n * v_y - ay_n * v_x

        # 积分反馈（仅在有加速度计时）
        if a_norm >= 1e-10:
            self._integral_fb[0] += config.GLOVE_MAHONY_KI * ex * dt
            self._integral_fb[1] += config.GLOVE_MAHONY_KI * ey * dt
            self._integral_fb[2] += config.GLOVE_MAHONY_KI * ez * dt

        # 修正角速度 = 原始 + P + I
        gx_corr = gx + self.kp * ex + self._integral_fb[0]
        gy_corr = gy + self.kp * ey + self._integral_fb[1]
        gz_corr = gz + self.kp * ez + self._integral_fb[2]

        # 四元数导数：dq/dt = 0.5 * q ⊗ [0, gx_corr, gy_corr, gz_corr]
        qDot = [
            0.5 * (-q1 * gx_corr - q2 * gy_corr - q3 * gz_corr),
            0.5 * (q0 * gx_corr + q2 * gz_corr - q3 * gy_corr),
            0.5 * (q0 * gy_corr - q1 * gz_corr + q3 * gx_corr),
            0.5 * (q0 * gz_corr + q1 * gy_corr - q2 * gx_corr),
        ]

        # 一阶积分更新
        self._q[0] += qDot[0] * dt
        self._q[1] += qDot[1] * dt
        self._q[2] += qDot[2] * dt
        self._q[3] += qDot[3] * dt

        # 单位化（防止漂移）
        self._q = _quat_normalize(self._q)

        return _quat_to_euler(self._q)
