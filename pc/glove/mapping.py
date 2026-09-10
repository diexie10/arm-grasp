# -*- coding: utf-8 -*-
"""mapping.py — 手势手套姿态→舵机目标映射（方案 A 关节镜像 / 方案 B 笛卡尔速度杆）。

输入：滤波后欧拉角 (roll, pitch, yaw) [deg]，相对于 config.GLOVE_NEUTRAL_DEG 中性姿态的误差。
输出：舵机域 6 轴目标 [s1..s6]（已 clamp 到 JOINT_MIN/MAX + JOINT_OFFSET 范围）。

方案 A（关节镜像）：
  基准位由 IK 可达点导出（GLOVE_SCH_A_REF_R/Z/S1），与未标定的 JOINT_HOME 解耦（#28）。
  pitch_err → S3 关节角偏移（相对参考位），roll_err → S1，yaw_err → S5。
  S2 保持参考位；S4 由竖直腕约束 j4 = 90 - j2 - j3 推导。
  注意：JOINT_HOME 不满足竖直腕不变量（j4=90-122-167=-199°），
  IK 不可解 → 方案 A 不以 JOINT_HOME 为基准。

方案 B（笛卡尔速度杆）：
  pitch_err → EE 竖直速度 vz，roll_err → EE 水平速度 vx，yaw_err → S1 角速度。
  圆柱坐标 (r, z) 积分 → 转 3D (x, y, z) → IK 求关节目标。
  IK 失败 → 保持上一拍舵机目标（规格要求：IK 失败→保持，禁止绕过 IK 的近似运动）。
"""

import math

import config
import kinematics


# =====================================================================
#  工具函数
# =====================================================================

def _apply_deadband(error_deg, deadband_deg):
    """死区：|error| ≤ deadband → 0；否则保持原值。"""
    if abs(error_deg) <= deadband_deg:
        return 0.0
    return error_deg


def _clamp_servo_targets(q_joint):
    """关节角 → 舵机角并 clamp 到限位。

    Args:
        q_joint: 6 元素关节角列表 [deg]。

    Returns:
        6 元素舵机角列表 [deg]，已 clamp。
    """
    servo = kinematics.to_servo(q_joint)
    return [kinematics.servo_clamp(i, servo[i]) for i in range(6)]


def _clamp_joints(q_joint):
    """关节角 clamp 到 JOINT_MIN/MAX。"""
    for i in range(6):
        q_joint[i] = kinematics.clamp(q_joint[i],
                                       config.JOINT_MIN[i],
                                       config.JOINT_MAX[i])
    return q_joint


# =====================================================================
#  方案 A：关节镜像（IK 可达参考位）
# =====================================================================

# 惰性缓存：模块级，首次调用时 IK 求解参考位，之后复用。
_scheme_a_q_ref = None  # None = 未初始化；列表 = [j1..j6]


def _scheme_a_ref():
    """求解方案 A 的 IK 可达参考关节角（惰性缓存）。

    由 GLOVE_SCH_A_REF_R / REF_Z / REF_S1 定义笛卡尔参考点，
    IK 求解得到关节角 q_ref。若 IK 不可解 → raise RuntimeError（禁止静默）。

    返回值：6 元素关节角列表 [deg]。
    """
    global _scheme_a_q_ref
    if _scheme_a_q_ref is not None:
        return _scheme_a_q_ref

    s1_rad = math.radians(config.GLOVE_SCH_A_REF_S1)
    x = config.GLOVE_SCH_A_REF_R * math.cos(s1_rad)
    y = config.GLOVE_SCH_A_REF_R * math.sin(s1_rad)
    z = config.GLOVE_SCH_A_REF_Z

    q, reason = kinematics.ik_solve(x, y, z)
    if q is None:
        raise RuntimeError(
            "方案A参考位 IK 不可解，检查 GLOVE_SCH_A_REF_* "
            "(r=%.1f, z=%.1f, s1=%.1f): %s" % (config.GLOVE_SCH_A_REF_R,
                                                  config.GLOVE_SCH_A_REF_Z,
                                                  config.GLOVE_SCH_A_REF_S1,
                                                  reason))
    _scheme_a_q_ref = list(q)
    return _scheme_a_q_ref


def scheme_a(pitch_err_deg, roll_err_deg, yaw_err_deg):
    """关节镜像映射（基于 IK 可达参考位）。

    增益读 config.GLOVE_SCH_A_*_GAIN，输出舵机域 6 轴目标。

    逻辑：
      q_ref = IK(REF_R·cos(REF_S1), REF_R·sin(REF_S1), REF_Z)  （惰性缓存）
      S1关节 = q_ref[0] + roll_err * GAIN   （roll 控制底座旋转）
      S2关节 = q_ref[1]                      （大臂保持参考位）
      S3关节 = q_ref[2] + pitch_err * GAIN   （pitch 控制小臂）
      S4关节 = 90 - S2关节 - S3关节          （竖直腕约束，IK 参考位天然满足）
      S5关节 = q_ref[4] + yaw_err * GAIN     （yaw 控制腕旋转）
      S6关节 = q_ref[5]                      （夹爪不动）

    发现：JOINT_HOME 不满足竖直腕不变量（j4=-199°，#28 未标定）→ 基准位改由 IK 导出。
    """
    q_ref = _scheme_a_ref()

    j1 = q_ref[0] + roll_err_deg * config.GLOVE_SCH_A_ROLL_GAIN
    j2 = q_ref[1]  # 大臂保持参考位
    j3 = q_ref[2] + pitch_err_deg * config.GLOVE_SCH_A_PITCH_GAIN
    # 竖直腕约束：j4 = 90 - j2 - j3（与 kinematics._ik_solve_one 一致）
    j4 = 90.0 - j2 - j3
    j5 = q_ref[4] + yaw_err_deg * config.GLOVE_SCH_A_YAW_GAIN
    j6 = q_ref[5]

    q_joint = [j1, j2, j3, j4, j5, j6]

    # clamp 到关节限位（计数饱和拍数）
    joint_clamp = 0
    for i in range(6):
        old = q_joint[i]
        q_joint[i] = kinematics.clamp(q_joint[i],
                                       config.JOINT_MIN[i],
                                       config.JOINT_MAX[i])
        if abs(q_joint[i] - old) > 1e-6:
            joint_clamp += 1

    return _clamp_servo_targets(q_joint), q_joint, joint_clamp


# =====================================================================
#  方案 B：笛卡尔速度杆（圆柱坐标）
# =====================================================================

def _find_ik_start_position():
    """寻找一个 IK 可解的起始位置（r, z, s1）。

    HOME (r≈14, z≈40) 在 IK 的竖直腕约束下不可解（j2+j3 过大导致 j4 越界）。
    扫描可达域，返回第一个 IK 可解的位置。
    """
    s1_deg = config.JOINT_HOME[0]
    s1_rad = math.radians(s1_deg)
    # 从远到近扫描，优先选较近的起始位
    for r in range(80, 201, 10):
        for z in range(60, 251, 20):
            x = r * math.cos(s1_rad)
            y = r * math.sin(s1_rad)
            q, _reason = kinematics.ik_solve(x, y, float(z))
            if q is not None:
                return r, float(z), s1_deg
    # 兜底：用最远可达位
    return 150, 200.0, s1_deg


def _validate_workspace_box(s1_deg):
    """验证 config 工作域四角 IK 可达性，若不可达则收缩到全角点有效的最大盒子。

    网格搜索法：在名义盒子内以 5mm 步长采样，对每对 (r_min, r_max) 求公共
    有效 z 范围，取面积最大的矩形作为生效盒子。
    额外鲁棒性：四角在 s1 ±30° 范围内均 IK 可解（防止 s1 漂移导致边界失效）。
    盒子内缩 10% 安全余量（防积分器在边界附近反复触发 IK 失败）。

    Args:
        s1_deg: 用于 IK 检查的 J1 方位角 deg。

    Returns:
        (r_min, r_max, z_min, z_max) — 实际生效的工作域盒子。
    """
    STEP = 5.0
    MIN_RANGE = 20.0
    MARGIN = 0.10  # 10% 安全余量
    S1_SWEEP = [s1_deg - 30.0, s1_deg, s1_deg + 30.0]  # 多角度验证

    def _ik_ok_at(r_val, z_val, s1):
        """检查 (r, z, s1) 下 IK 是否可解。"""
        s1_rad = math.radians(s1)
        x = r_val * math.cos(s1_rad)
        y = r_val * math.sin(s1_rad)
        q, _ = kinematics.ik_solve(x, y, z_val)
        return q is not None

    def _ik_ok_robust(r_val, z_val):
        """在 s1 ±30° 范围内均 IK 可解。"""
        return all(_ik_ok_at(r_val, z_val, s) for s in S1_SWEEP)

    # 采样网格
    rn0 = config.GLOVE_SCH_B_WORK_R_MIN
    rx0 = config.GLOVE_SCH_B_WORK_R_MAX
    zn0 = config.GLOVE_SCH_B_WORK_Z_MIN
    zx0 = config.GLOVE_SCH_B_WORK_Z_MAX
    r_samples = []
    r = rn0
    while r <= rx0 + 1e-6:
        r_samples.append(round(r, 1))
        r += STEP
    z_samples = []
    z = zn0
    while z <= zx0 + 1e-6:
        z_samples.append(round(z, 1))
        z += STEP

    # 预计算每行（每个 r）的有效 z 范围（鲁棒版）
    r_z_ranges = {}
    for r_val in r_samples:
        valid_z = [z for z in z_samples if _ik_ok_robust(r_val, z)]
        if valid_z:
            r_z_ranges[r_val] = (min(valid_z), max(valid_z))

    # 搜索最大面积矩形：对每对 (r_min, r_max)，求公共 z 范围
    best_area = 0.0
    best_rect = None
    for i, rmin in enumerate(r_samples):
        if rmin not in r_z_ranges:
            continue
        for j in range(i, len(r_samples)):
            rmax = r_samples[j]
            if rmax not in r_z_ranges:
                continue
            # 公共 z 范围
            z_lo = max(r_z_ranges[rmin][0], r_z_ranges[rmax][0])
            z_hi = min(r_z_ranges[rmin][1], r_z_ranges[rmax][1])
            if z_hi - z_lo >= MIN_RANGE and rmax - rmin >= MIN_RANGE:
                area = (rmax - rmin) * (z_hi - z_lo)
                if area > best_area:
                    best_area = area
                    best_rect = (rmin, rmax, z_lo, z_hi)

    if best_rect is None:
        raise RuntimeError(
            "方案B工作域内无有效盒子 (r∈[%.0f,%.0f] z∈[%.0f,%.0f], s1=%.1f)"
            % (rn0, rx0, zn0, zx0, s1_deg))

    rn, rx, zn, zx = best_rect

    # 内缩安全余量（防积分器在边界反复触发 IK 失败）
    r_margin = (rx - rn) * MARGIN
    z_margin = (zx - zn) * MARGIN
    rn += r_margin
    rx -= r_margin
    zn += z_margin
    zx -= z_margin

    # 四角 IK 验证（防御性，主方位）
    s1_rad = math.radians(s1_deg)
    def _corner_ik(rv, zv):
        x = rv * math.cos(s1_rad)
        y = rv * math.sin(s1_rad)
        q, _ = kinematics.ik_solve(x, y, zv)
        return q is not None

    assert _corner_ik(rn, zn) and _corner_ik(rn, zx) and _corner_ik(rx, zn) and _corner_ik(rx, zx), \
        "四角验证失败（内部 bug）"

    return rn, rx, zn, zx


def scheme_b_init():
    """初始化方案 B 状态：IK 可解的起始位置 + 有效工作域盒子。

    工作域验证（2026-09 第四轮新增）：
      config 盒子是名义值，init 时对四角做 IK 验证（用 REF_S1 方位），
      不可达角点 → 自动收缩并打印生效值。收缩后仍无解 → RuntimeError。

    注意：HOME 位 (r≈14, z≈40) 在当前 IK 约束下不可解
    （j4=90-j2-j3 在 HOME 的 j2=122, j3=167 时 = -199°，超出 [7°, 187°]）。
    因此从可达域内最近的 IK 可解位置起步。
    """
    r, z, s1 = _find_ik_start_position()

    # 工作域四角 IK 验证 + 自动收缩
    eff_rmin, eff_rmax, eff_zmin, eff_zmax = _validate_workspace_box(s1)
    print("方案B 工作域: r∈[%.0f, %.0f] z∈[%.0f, %.0f] (名义 r∈[%.0f,%.0f] z∈[%.0f,%.0f], s1=%.1f)"
          % (eff_rmin, eff_rmax, eff_zmin, eff_zmax,
             config.GLOVE_SCH_B_WORK_R_MIN, config.GLOVE_SCH_B_WORK_R_MAX,
             config.GLOVE_SCH_B_WORK_Z_MIN, config.GLOVE_SCH_B_WORK_Z_MAX, s1))
    # 四角 IK 验证打印
    s1_rad = math.radians(s1)
    for label, rv, zv in [("R_MIN,Z_MIN", eff_rmin, eff_zmin),
                           ("R_MIN,Z_MAX", eff_rmin, eff_zmax),
                           ("R_MAX,Z_MIN", eff_rmax, eff_zmin),
                           ("R_MAX,Z_MAX", eff_rmax, eff_zmax)]:
        x = rv * math.cos(s1_rad)
        y = rv * math.sin(s1_rad)
        q, reason = kinematics.ik_solve(x, y, zv)
        status = "OK" if q is not None else "FAIL: %s" % reason
        print("  角点 %s (r=%.0f,z=%.0f): %s" % (label, rv, zv, status))

    # 初始化到 IK 可解的关节角（用于 prev servo）
    s1_rad = math.radians(s1)
    x0 = r * math.cos(s1_rad)
    y0 = r * math.sin(s1_rad)
    q0, _reason = kinematics.ik_solve(x0, y0, z)
    if q0 is None:
        # 理论不可达（扫描时已验证），但防御性检查
        raise RuntimeError("方案B起始位 IK 不可解 (r=%.1f, z=%.1f, s1=%.1f)" % (r, z, s1))
    return {
        "r": r,                          # 径向距离 mm
        "z": z,                          # 高度 mm
        "s1_joint": s1,                  # J1 关节角 deg
        "prev_servo": _clamp_servo_targets(q0),  # 上一拍舵机目标（IK 失败时保持）
        "ik_failed_count": 0,
        "ik_ok_count": 0,
        # 实际生效工作域（init 时验证并可能收缩）
        "eff_r_min": eff_rmin,
        "eff_r_max": eff_rmax,
        "eff_z_min": eff_zmin,
        "eff_z_max": eff_zmax,
    }


def scheme_b(pitch_err_deg, roll_err_deg, yaw_err_deg, dt, state):
    """笛卡尔速度杆映射（圆柱坐标）。

    逻辑：
      vz = pitch_err * GAIN  →  target_z += vz * dt
      vx = roll_err  * GAIN  →  target_r += vx * dt   （径向速度）
      s1_vel = yaw_err * GAIN →  s1_joint += s1_vel * dt
      转 3D：x = r·cos(s1), y = r·sin(s1) → IK(x, y, z) 求关节目标。
      IK 失败 → 保持上一拍舵机目标（规格要求，禁止绕过 IK 的近似运动）。

    Args:
        pitch_err_deg, roll_err_deg, yaw_err_deg: 死区后的误差 [deg]。
        dt: 时间步长 s（= 1 / GLOVE_SEND_HZ）。
        state: dict，可变状态。

    Returns:
        (servo_targets[6], q_joint[6], ee_pos[3]) — 舵机目标、关节角、EE (x,y,z)。
    """
    # 速度指令
    vz = pitch_err_deg * config.GLOVE_SCH_B_VZ_GAIN  # mm/s
    vx = roll_err_deg * config.GLOVE_SCH_B_VX_GAIN    # mm/s（径向）
    s1_vel = yaw_err_deg * config.GLOVE_SCH_B_S1_GAIN  # deg/s

    # 积分（圆柱坐标）
    new_r = state["r"] + vx * dt
    new_z = state["z"] + vz * dt
    new_s1 = state["s1_joint"] + s1_vel * dt

    # 钳位到生效工作域（init 时验证并可能收缩后的盒子）
    new_r = kinematics.clamp(new_r,
                             state["eff_r_min"],
                             state["eff_r_max"])
    new_z = kinematics.clamp(new_z,
                             state["eff_z_min"],
                             state["eff_z_max"])

    # S1 关节限位 + atan2 安全范围（>180° 时 atan2 返回负值，J1 限位拒绝）
    new_s1 = kinematics.clamp(new_s1,
                               config.JOINT_MIN[0],
                               min(config.JOINT_MAX[0], 180.0))

    # 转 3D 笛卡尔坐标（用 J1 角度确定方位）
    s1_rad = math.radians(new_s1)
    target_x = new_r * math.cos(s1_rad)
    target_y = new_r * math.sin(s1_rad)

    # IK 求解
    q, reason = kinematics.ik_solve(target_x, target_y, new_z)
    if q is not None:
        state["ik_ok_count"] += 1
        # IK 成功 → 更新圆柱坐标状态
        state["r"] = new_r
        state["z"] = new_z
        state["s1_joint"] = new_s1
        servo = _clamp_servo_targets(q)
        state["prev_servo"] = servo
        return servo, q, [target_x, target_y, new_z], 0  # joint_clamp=0（IK 已保证限位）

    # --- IK 失败 → 保持上一拍舵机目标（规格要求，禁止近似运动） ---
    state["ik_failed_count"] += 1
    prev = state["prev_servo"]
    return prev, None, None, 0
