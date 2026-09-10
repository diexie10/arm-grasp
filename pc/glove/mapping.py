# -*- coding: utf-8 -*-
"""mapping.py — 手势手套姿态→舵机目标映射（方案 A 关节镜像 / 方案 B 笛卡尔速度杆）。

输入：滤波后欧拉角 (roll, pitch, yaw) [deg]，相对于 config.GLOVE_NEUTRAL_DEG 中性姿态的误差。
输出：舵机域 6 轴目标 [s1..s6]（已 clamp 到 JOINT_MIN/MAX + JOINT_OFFSET 范围）。

两阶段限位（MoveIt Servo jointLimitVelocityScalingFactor 语义）：
  阶段 1：关节距限位 < GLOVE_LIMIT_MARGIN_DEG → 全局速度缩放（保持运动方向，j2/j3/j4 耦合一致减速）
  阶段 2：硬停（钳位进限内，兜底防越界）

方案 A（关节镜像）：
  有状态：跟踪 q_achieved（初始 = IK 参考位）。
  每拍：q_target 由误差×增益算出 → delta = q_target − q_achieved → _limit_scale → q_achieved 更新。

方案 B（笛卡尔速度杆）：
  有状态：跟踪 q_achieved（初始 = 起始 IK 解）。
  每拍：FK(q_achieved) → (r,z,s1) → 误差积分 → desired (r,z,s1) → IK → q_desired → delta → _limit_scale。
"""

import math

import config
import kinematics


# =====================================================================
#  两阶段限位（MoveIt Servo 语义）
# =====================================================================

def _limit_scale(current_q, target_q):
    """两阶段限位：1) 限位带内全局速度缩放 2) 越界硬停。

    全局缩放保持运动方向（MoveIt Servo 语义），j2/j3/j4 耦合一致减速。

    Args:
        current_q: 当前已达成关节角 [deg]（6 元素）。
        target_q: 目标关节角 [deg]（6 元素）。

    Returns:
        (scaled_q[6], scale_factor, halt_count) — 缩放后关节角、缩放因子、硬停关节·拍数。
    """
    deltas = [t - c for c, t in zip(current_q, target_q)]
    scale = 1.0
    for i, d in enumerate(deltas):
        if abs(d) < 1e-9:
            continue
        lo, hi = config.JOINT_MIN[i], config.JOINT_MAX[i]
        m = config.GLOVE_LIMIT_MARGIN_DEG
        if d < 0 and current_q[i] < lo + m:
            scale = min(scale, max(0.0, (current_q[i] - lo) / m))
        elif d > 0 and current_q[i] > hi - m:
            scale = min(scale, max(0.0, (hi - current_q[i]) / m))
    scaled = [c + d * scale for c, d in zip(current_q, deltas)]
    # 阶段 2：硬停（钳位进限内）
    halt_count = 0
    for i in range(6):
        old = scaled[i]
        scaled[i] = kinematics.clamp(scaled[i], config.JOINT_MIN[i], config.JOINT_MAX[i])
        if abs(scaled[i] - old) > 1e-6:
            halt_count += 1
    return scaled, scale, halt_count


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


# =====================================================================
#  方案 A：关节镜像（IK 可达参考位，有状态）
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


def scheme_a_init():
    """初始化方案 A 状态：q_achieved = IK 参考位。

    Returns:
        dict — 初始状态（q_achieved 6 元素列表）。
    """
    q_ref = _scheme_a_ref()
    return {"q_achieved": list(q_ref)}


def scheme_a(pitch_err_deg, roll_err_deg, yaw_err_deg, state):
    """关节镜像映射（基于 IK 可达参考位，有状态 + 两阶段限位）。

    每拍：
      q_ref = IK 参考位（惰性缓存）
      q_target = [j1+roll·gain, j2_ref, j3_ref+pitch·gain, 90-j2-j3, j5+yaw·gain, j6_ref]
      delta = q_target − q_achieved
      _limit_scale(delta) → q_achieved += 缩放后 delta
      输出 to_servo(q_achieved)

    Args:
        pitch_err_deg, roll_err_deg, yaw_err_deg: 死区后的误差 [deg]。
        state: dict，含 q_achieved（可变）。

    Returns:
        (servo_targets[6], q_joint[6], scale, halt_count)
    """
    q_ref = _scheme_a_ref()

    # 目标关节角（从参考位偏移）
    j1 = q_ref[0] + roll_err_deg * config.GLOVE_SCH_A_ROLL_GAIN
    j2 = q_ref[1]  # 大臂保持参考位
    j3 = q_ref[2] + pitch_err_deg * config.GLOVE_SCH_A_PITCH_GAIN
    # 竖直腕约束：j4 = 90 - j2 - j3（不钳位，交给 _limit_scale 处理）
    j4 = 90.0 - j2 - j3
    j5 = q_ref[4] + yaw_err_deg * config.GLOVE_SCH_A_YAW_GAIN
    j6 = q_ref[5]

    q_target = [j1, j2, j3, j4, j5, j6]

    # 两阶段限位
    q_achieved, scale, halt_count = _limit_scale(state["q_achieved"], q_target)
    state["q_achieved"] = q_achieved

    return _clamp_servo_targets(q_achieved), q_achieved, scale, halt_count


# =====================================================================
#  方案 B：笛卡尔速度杆（圆柱坐标，有状态）
# =====================================================================

def _validate_workspace(s1_deg):
    """网格搜索最大 IK 全通过矩形，返回 (r_min, r_max, z_min, z_max)。

    用固定 s1（与 _find_ik_start_position 共享），在 (r, z) 网格上逐点 IK，
    找最大轴对齐矩形（所有网格点 IK 成功）。
    s1 对 (r,z) 可行性无影响（IK 的 J1 解与 s1 独立），所以只需扫一次。

    Returns:
        (r_min, r_max, z_min, z_max) — 可行矩形边界 mm。
    """
    s1_rad = math.radians(s1_deg)

    # 构建可行性网格
    r_vals = []
    r = config.GLOVE_B_WS_R_MIN
    while r <= config.GLOVE_B_WS_R_MAX + 1e-9:
        r_vals.append(r)
        r += config.GLOVE_B_WS_R_STEP

    z_vals = []
    z = config.GLOVE_B_WS_Z_MIN
    while z <= config.GLOVE_B_WS_Z_MAX + 1e-9:
        z_vals.append(z)
        z += config.GLOVE_B_WS_Z_STEP

    feasible = {}  # (ri, zi) → bool
    for ri, rv in enumerate(r_vals):
        for zi, zv in enumerate(z_vals):
            x = rv * math.cos(s1_rad)
            y = rv * math.sin(s1_rad)
            q, _ = kinematics.ik_solve(x, y, zv)
            feasible[(ri, zi)] = q is not None

    n_r = len(r_vals)
    n_z = len(z_vals)

    # 找最大矩形：枚举 r 列对 [c1, c2]，找最长连续 z 行全通过段
    best_area = 0
    best_r1 = best_r2 = 0
    best_z1 = best_z2 = 0

    for c1 in range(n_r):
        # col_ok[zi] = 该 z 行在 c1..c2 列对上是否全部通过
        col_ok = [True] * n_z
        for c2 in range(c1, n_r):
            # 更新 col_ok：AND 上 (c2, zi) 的可行性
            for zi in range(n_z):
                col_ok[zi] = col_ok[zi] and feasible.get((c2, zi), False)

            # 在 col_ok 里找最长连续 True 段
            z_start = 0
            for zi in range(n_z + 1):
                if zi < n_z and col_ok[zi]:
                    continue
                # 段结束 [z_start, zi)
                length = zi - z_start
                if length > 0:
                    area = (c2 - c1 + 1) * length
                    if area > best_area:
                        best_area = area
                        best_r1, best_r2 = c1, c2
                        best_z1, best_z2 = z_start, zi - 1
                z_start = zi + 1

    if best_area > 0:
        r_min = r_vals[best_r1]
        r_max = r_vals[best_r2]
        z_min = z_vals[best_z1]
        z_max = z_vals[best_z2]
        print("workspace 可行盒 (s1=%.1f): r∈[%.0f,%.0f] z∈[%.0f,%.0f] (%d 网格点)" % (
            s1_deg, r_min, r_max, z_min, z_max, best_area))
        return r_min, r_max, z_min, z_max

    # 兜底：用粗盒
    print("workspace 盒: 网格搜索无可行矩形，使用粗盒")
    return (config.GLOVE_SCH_B_WORK_R_MIN, config.GLOVE_SCH_B_WORK_R_MAX,
            config.GLOVE_SCH_B_WORK_Z_MIN, config.GLOVE_SCH_B_WORK_Z_MAX)


def _find_ik_start_position(ws_box):
    """在可行盒内寻找 IK 可解的起始位置，优先选 J4 余量最大的点。

    J4 余量 = |j4 − (JOINT_MIN[3]+JOINT_MAX[3])/2|（越小越好，避免落在 J4 边缘）。

    Args:
        ws_box: (r_min, r_max, z_min, z_max) 可行矩形边界。

    Returns:
        (r, z, s1_deg) 起始位置。
    """
    r_min, r_max, z_min, z_max = ws_box
    s1_deg = config.GLOVE_SCH_A_REF_S1
    s1_rad = math.radians(s1_deg)
    j4_center = (config.JOINT_MIN[3] + config.JOINT_MAX[3]) / 2.0

    best_r, best_z = None, None
    best_margin = float('inf')

    # 扫描可行盒内的网格点
    r = r_min
    while r <= r_max + 1e-9:
        z = z_min
        while z <= z_max + 1e-9:
            x = r * math.cos(s1_rad)
            y = r * math.sin(s1_rad)
            q, _ = kinematics.ik_solve(x, y, z)
            if q is not None:
                margin = abs(q[3] - j4_center)
                if margin < best_margin:
                    best_margin = margin
                    best_r, best_z = r, z
            z += config.GLOVE_B_WS_Z_STEP
        r += config.GLOVE_B_WS_R_STEP

    if best_r is not None:
        return best_r, best_z, s1_deg

    # 兜底：粗盒中心
    return (r_min + r_max) / 2, (z_min + z_max) / 2, s1_deg


def scheme_b_init():
    """初始化方案 B 状态：IK 可解的起始位置 + 可行盒。

    1. _validate_workspace → 可行矩形盒（网格搜索最大 IK 全通过区域）
    2. 应用 GLOVE_B_WS_MARGIN 缩进 → 积分器有效盒（远离 IK 脆弱边缘）
    3. _find_ik_start_position → 盒内 J4 余量最大起始点
    4. 积分器目标初始化到起始点；后续每拍钳位到有效盒内。

    Returns:
        dict — 初始状态。
    """
    s1_deg = config.GLOVE_SCH_A_REF_S1

    # 1. 网格搜索可行矩形
    raw_box = _validate_workspace(s1_deg)
    r_min_raw, r_max_raw, z_min_raw, z_max_raw = raw_box

    # 2. 应用安全余量（积分器远离盒边缘，减少 IK 脆败）
    margin = config.GLOVE_B_WS_MARGIN
    r_min = r_min_raw + margin
    r_max = r_max_raw - margin
    z_min = z_min_raw + margin
    z_max = z_max_raw - margin
    # 保证盒非空
    if r_min >= r_max:
        r_min, r_max = (r_min_raw + r_max_raw) / 2, (r_min_raw + r_max_raw) / 2 + 1.0
    if z_min >= z_max:
        z_min, z_max = (z_min_raw + z_max_raw) / 2, (z_min_raw + z_max_raw) / 2 + 1.0
    ws_box = (r_min, r_max, z_min, z_max)

    # 3. 盒内 J4 余量最大起始点
    r, z, s1 = _find_ik_start_position(ws_box)

    print("方案B 起始位: r=%.0f z=%.0f s1=%.1f" % (r, z, s1))
    print("方案B 积分盒: r∈[%.1f, %.1f] z∈[%.1f, %.1f] (margin=%.0fmm)" % (
        r_min, r_max, z_min, z_max, margin))

    # 初始化到 IK 可解的关节角
    s1_rad = math.radians(s1)
    x0 = r * math.cos(s1_rad)
    y0 = r * math.sin(s1_rad)
    q0, _reason = kinematics.ik_solve(x0, y0, z)
    if q0 is None:
        raise RuntimeError("方案B起始位 IK 不可解 (r=%.1f, z=%.1f, s1=%.1f)" % (r, z, s1))

    return {
        "q_achieved": list(q0),             # 已达成关节角（两阶段限位基准）
        "r": r,                              # 径向距离 mm（积分目标）
        "z": z,                              # 高度 mm（积分目标）
        "s1_joint": s1,                      # J1 关节角 deg（积分目标）
        "prev_servo": _clamp_servo_targets(q0),  # 上一拍舵机目标（IK 失败时保持）
        "ik_failed_count": 0,
        "ik_ok_count": 0,
        "ws_box": ws_box,                    # 积分盒 (r_min, r_max, z_min, z_max)
    }


def scheme_b(pitch_err_deg, roll_err_deg, yaw_err_deg, dt, state):
    """笛卡尔速度杆映射（圆柱坐标，有状态 + 可行盒钳位 + 两阶段限位 + EE 级步长回退梯子）。

    每拍：
      从 state 读当前 (r, z, s1)（积分目标，非 FK）
      误差积分 → desired (r, z, s1)
      钳位到可行盒（_validate_workspace 网格搜索的最大 IK 全通过矩形）
      IK(desired) → q_desired；若失败，按 GLOVE_B_STEP_SCALES 缩小 delta 重试
      delta = q_desired − q_achieved → _limit_scale → q_achieved 更新
      积分目标回写 state（保持积分连续性）

    IK 全部失败 → 保持上一拍舵机目标（规格要求，禁止近似运动）。

    Args:
        pitch_err_deg, roll_err_deg, yaw_err_deg: 死区后的误差 [deg]。
        dt: 时间步长 s（= 1 / GLOVE_SEND_HZ）。
        state: dict，可变状态。

    Returns:
        (servo_targets[6], q_joint[6], ee_pos[3], scale, halt_count)
    """
    # 速度指令
    vz = pitch_err_deg * config.GLOVE_SCH_B_VZ_GAIN  # mm/s
    vx = roll_err_deg * config.GLOVE_SCH_B_VX_GAIN    # mm/s（径向）
    s1_vel = yaw_err_deg * config.GLOVE_SCH_B_S1_GAIN  # deg/s

    # 积分（圆柱坐标，从 state 读当前值）
    new_r = state["r"] + vx * dt
    new_z = state["z"] + vz * dt
    new_s1 = state["s1_joint"] + s1_vel * dt

    # 钳位到可行盒（_validate_workspace 网格搜索的最大 IK 全通过矩形）
    r_min, r_max, z_min, z_max = state["ws_box"]
    new_r = kinematics.clamp(new_r, r_min, r_max)
    new_z = kinematics.clamp(new_z, z_min, z_max)
    # s1 钳位：防止积分漂出 atan2 有效范围（j1 > 180° → atan2 返回负值 → IK 拒绝）
    new_s1 = kinematics.clamp(new_s1,
                              config.JOINT_MIN[0],
                              min(config.JOINT_MAX[0], config.GLOVE_S1_WRAP_DEG))

    # EE 级步长回退梯子：IK 失败时按比例缩小 delta 重试
    # delta = desired - current(r, z, s1)；每级用 scale × delta 重试
    delta_r = new_r - state["r"]
    delta_z = new_z - state["z"]
    delta_s1 = new_s1 - state["s1_joint"]

    q_desired = None
    q_reason = None
    used_scale = 1.0
    for scale in config.GLOVE_B_STEP_SCALES:
        trial_r = state["r"] + delta_r * scale
        trial_z = state["z"] + delta_z * scale
        trial_s1 = state["s1_joint"] + delta_s1 * scale
        # 钳位到可行盒
        trial_r = kinematics.clamp(trial_r, r_min, r_max)
        trial_z = kinematics.clamp(trial_z, z_min, z_max)
        trial_s1 = kinematics.clamp(trial_s1,
                                     config.JOINT_MIN[0],
                                     min(config.JOINT_MAX[0], config.GLOVE_S1_WRAP_DEG))
        # 转 3D 笛卡尔坐标
        s1_rad = math.radians(trial_s1)
        target_x = trial_r * math.cos(s1_rad)
        target_y = trial_r * math.sin(s1_rad)

        q, reason = kinematics.ik_solve(target_x, target_y, trial_z)
        if q is not None:
            q_desired = q
            used_scale = scale
            # 更新积分目标为实际采纳值
            new_r = trial_r
            new_z = trial_z
            new_s1 = trial_s1
            break
        q_reason = reason

    if q_desired is None:
        # 全部失败 → 保持上一拍
        state["ik_failed_count"] += 1
        prev = state["prev_servo"]
        return prev, None, None, 1.0, 0

    # IK 成功 → 两阶段限位（关节域）
    q_achieved, scale, halt_count = _limit_scale(state["q_achieved"], list(q_desired))
    state["q_achieved"] = q_achieved
    state["ik_ok_count"] += 1

    # 积分目标回写 state（保持积分连续性；两阶段限位仅影响伺服输出）
    state["r"] = new_r
    state["z"] = new_z
    state["s1_joint"] = new_s1

    # EE 位置从 FK(q_achieved) 算（反映实际运动，非积分目标）
    x_ee, y_ee, z_ee = kinematics.fk(q_achieved)
    servo = _clamp_servo_targets(q_achieved)
    state["prev_servo"] = servo
    return servo, q_achieved, [x_ee, y_ee, z_ee], scale, halt_count
