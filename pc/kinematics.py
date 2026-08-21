# -*- coding: utf-8 -*-
"""kinematics.py — 5 轴解析 IK + FK 验证闭环 + 可达性检查。

几何模型（右手系，Z 向上，J1 轴心为桌面原点）：
    J1 底座旋转（yaw），J2 大臂，J3 小臂，J4 腕俯仰（保证末端竖直向下），
    J5 固定 90°，J6 夹爪。
    大臂角 θ2 相对水平，小臂角 θ3 相对大臂延长线，末端竖直 ⇒ θ4 = 90° − θ2 − θ3。

铁律：IK 无解/不可达 → 返回 (None, reason)，调用方转 ERROR，绝不盲抓。
所有角度单位：度。所有长度单位：mm。
"""

import math

import config


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def to_servo(q):
    """关节角 → 舵机角（固件 M 命令角度，直接转 PWM）。

    安装假设：J2/J3/J4/J5 舵机 90°=关节 0°，J1/J6 无 offset（config.JOINT_OFFSET）。
    """
    return [q[i] + config.JOINT_OFFSET[i] for i in range(6)]


def from_servo(servo_q):
    """舵机角（固件回显）→ 关节角。回显铁律：joint_state 存关节角。"""
    return [servo_q[i] - config.JOINT_OFFSET[i] for i in range(6)]


def ik_solve(x, y, z, grip_angle=None):
    """由末端位置 (x, y, z) 求关节角 q[6]。

    Args:
        x, y, z: 末端（夹爪中心）桌面坐标，mm，Z 向上。
        grip_angle: J6 夹爪角度；None 表示保持默认。

    Returns:
        (q_list, None) 成功；q_list = [J1..J6] 关节角度，保证在关节限位内。
        (None, reason_str) 不可达（几何或限位超界；调用方必须处理，转 ERROR）。
        绝不静默 clamp——clamp 是固件兜底，上位机必须返回真实可达性。
    """
    # --- 可达性预检 ---
    r = math.hypot(x, y)                 # J1 轴心到目标点的水平距离
    dz = z - config.L1                   # 目标相对 J2 轴心的高度
    d = math.hypot(r, dz)                # J2 轴心到末端的直线距离
    if d > config.L2 + config.L3 + 1e-6:
        return None, "unreachable: target distance %.1f > L2+L3=%.1f" % (
            d, config.L2 + config.L3)
    if d < abs(config.L2 - config.L3) - 1e-6:
        return None, "unreachable: target distance %.1f < |L2-L3|=%.1f" % (
            d, abs(config.L2 - config.L3))

    # --- 解析解（余弦定理，2R 臂）---
    j1 = math.degrees(math.atan2(y, x))          # 底座 yaw
    cos_j3 = (d * d - config.L2 * config.L2 - config.L3 * config.L3) \
        / (2.0 * config.L2 * config.L3)
    j3 = math.degrees(math.acos(_clamp(cos_j3, -1.0, 1.0)))
    # θ2 = atan2(Z', R) − atan2(L3·sinθ3, L2 + L3·cosθ3)
    a2 = math.atan2(dz, r)
    b2 = math.atan2(config.L3 * math.sin(math.radians(j3)),
                    config.L2 + config.L3 * math.cos(math.radians(j3)))
    j2 = math.degrees(a2 - b2)
    j4 = 90.0 - j2 - j3                    # 末端竖直向下的腕补偿

    q = [j1, j2, j3, j4, config.J5_FIXED,
         config.GRIP_OPEN if grip_angle is None else grip_angle]

    # --- 限位检查：超限 = 不可达，绝不静默 clamp ---
    # （clamp 是固件兜底；上位机若静默 clamp，虚拟角度与物理脱节 → IK 漂移）
    for i in range(6):
        if q[i] < config.JOINT_MIN[i] - 1e-6 or q[i] > config.JOINT_MAX[i] + 1e-6:
            return None, ("joint %d out of range: %.1f not in [%.0f, %.0f]"
                          % (i + 1, q[i], config.JOINT_MIN[i], config.JOINT_MAX[i]))
    return q, None


def fk(q):
    """正向运动学：由关节角 q[6] 求末端位置 (x, y, z)。用于验证 IK 闭环。"""
    j1 = math.radians(q[0])
    j2 = math.radians(q[1])
    j3 = math.radians(q[2])
    r = config.L2 * math.cos(j2) + config.L3 * math.cos(j2 + j3)
    z = config.L1 + config.L2 * math.sin(j2) + config.L3 * math.sin(j2 + j3)
    x = r * math.cos(j1)
    y = r * math.sin(j1)
    return (x, y, z)


def verify_ik(pose, grip_angle=None, tol_mm=1.0):
    """FK(IK(T)) ≈ T 闭环验证。

    Returns:
        (True, err_mm) 闭环误差 mm；或 (False, reason)。
    """
    q, reason = ik_solve(pose[0], pose[1], pose[2], grip_angle)
    if q is None:
        return False, reason
    x, y, z = fk(q)
    err = math.hypot(x - pose[0], y - pose[1]) + abs(z - pose[2])
    if err > tol_mm:
        return False, "FK(IK) error %.2f mm > tol %.1f (q=%s)" % (
            err, tol_mm, [round(a, 1) for a in q])
    return True, err


def is_reachable(x, y, z):
    """快速可达性判断（状态机在调用 IK 前先用）。"""
    r = math.hypot(x, y)
    dz = z - config.L1
    d = math.hypot(r, dz)
    return (abs(config.L2 - config.L3) <= d <= config.L2 + config.L3)


if __name__ == "__main__":
    # 单元自测：J1 限位内可达空间多点 FK(IK) 闭环验证
    # fail 只统计"IK 成功但 FK 闭环错"（真 bug）；
    # 限位拒绝（舵机行程）与几何不可达是预期行为
    import random
    random.seed(42)
    ok, fail, rejected = 0, 0, 0
    for _ in range(200):
        r = random.uniform(60.0, 200.0)
        a = random.uniform(20.0, 160.0)      # 方位角须在 J1 限位内
        z = random.uniform(40.0, 200.0)      # J2=-60,J3=90 时末端最低 ~36mm
        pose = (r * math.cos(math.radians(a)),
                r * math.sin(math.radians(a)), z)
        if not is_reachable(*pose):
            rejected += 1
            continue
        good, info = verify_ik(pose)
        if good:
            ok += 1
        elif "FK(IK) error" in str(info):    # 闭环错 = 真 bug
            fail += 1
            if fail <= 3:
                print("FAIL", pose, info)
        else:
            rejected += 1                    # 限位拒绝 = 预期
    print("verify_ik: %d pass, %d FAIL(bug), %d rejected(limit/unreachable)"
          % (ok, fail, rejected))

    # 可达边界测试
    for pose in [(0, 120, 50), (150, 0, 100), (100, 100, 30), (0, 0, 300)]:
        print("reachable %s -> %s" % (pose, is_reachable(*pose)))
        q, reason = ik_solve(*pose)
        if q is not None:
            print("  q=%s fk=%s" % (
                [round(a, 1) for a in q], [round(v, 1) for v in fk(q)]))
        else:
            print("  ik: %s" % reason)
