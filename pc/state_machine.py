# -*- coding: utf-8 -*-
"""state_machine.py — 抓取状态机（架构书 §4，视觉伺服版）。

流程：IDLE → SEARCH（J1 转圈扫描，检测到锁存方位）
     → ALIGN（视觉伺服：像素误差 → 关节增量，收敛到画面中心）
     → DESCEND（逐级下降 + 每级对齐，木块像素占比达阈值）
     → GRIP（夹爪闭合）→ TRANSPORT（抬升→移到放置位→降）→ RELEASE（张开）
     → HOME → IDLE

视觉伺服方案（用户确认，2026-08）：
- 摄像头装在机械臂上（eye-in-hand），升最高点俯瞰
- 全程像素域闭环，不依赖毫米标定（calibration.py 降级备用）
- 正方形木块 90° 对称 → 角度容错 ±45°，J5 = θ − J1 对齐夹爪
- 粗定位（搜索）→ 精对准（伺服）→ 红外确认（抓取瞬间）

安全（架构书 §8 L3）：
- 每状态超时 → ERROR
- IK 无解 / 不可达 → ERROR（绝不盲抓）
- 轨迹执行 CLAMP/TIMEOUT → ERROR
- ERROR 恢复：先抬 Z 到安全高度，再回 HOME（臂不撞物）
- DRY_RUN：ArmSerial 假回显，状态机照跑（联调必开）
"""

import logging
import math
import time

import config
import kinematics
from servo_controller import ServoController, estimate_move_seconds, compute_wrist_correction
from trajectory import TrajectoryError

log = logging.getLogger(__name__)


class StateMachine:
    def __init__(self, serial, traj, ir, vision=None, cap=None):
        self.serial = serial
        self.traj = traj
        self.ir = ir
        self.vision = vision          # 视觉伺服用（SEARCH/ALIGN 需要）
        self.cap = cap                # 摄像头（SEARCH/ALIGN 需要）
        self.ctrl = ServoController()  # 视觉伺服策略层（纯数学，无 I/O）
        self.state = "IDLE"
        self._target = None            # 当前抓取目标 (x, y)，ERROR 抬升用（P1-1）
        self._theta = None             # 当前目标角度（J5 对齐用）

    # ---------- 内部 ----------
    def _set_state(self, s):
        self.state = s
        print("[SM] -> %s" % s)

    def _timeout(self, start, state):
        return (time.monotonic() - start) * 1000.0 > config.STATE_TIMEOUTS[state]

    def _to_error(self, reason):
        self._set_state("ERROR")
        print("[SM] ERROR: %s" % reason)
        log.warning("SM ERROR: %s", reason)
        # 安全恢复（P1-1）：先抬 Z 到"当前目标上方 SAFE_Z"（与 APPROACH 同构必可达），
        # 再回 HOME。绝不能用 (0,0,SAFE_Z)——底座正上方是奇异折叠区，J3 超限不可达，
        # 直接回 HOME 会低空横扫扫过桌面物件。
        try:
            if self._target is not None:
                q, _ = kinematics.ik_solve(self._target[0], self._target[1],
                                           config.SAFE_Z)
            else:
                x, y, _ = kinematics.fk(self.serial.joint_state)
                q, _ = kinematics.ik_solve(x, y, config.SAFE_Z)
            if q is not None:
                self.traj.move_to(self.serial, q)
        except TrajectoryError as e:
            log.warning("ERROR 抬升失败: %s", e)
        try:
            self.traj.move_to(self.serial, config.JOINT_HOME)
        except TrajectoryError as e:
            log.warning("ERROR 回 HOME 失败: %s", e)
        self._set_state("IDLE")
        return False

    def _move(self, q, state, t0, overshoot=True):
        """轨迹执行 + 超时检查。失败转 ERROR。

        overshoot=False 透传给 move_to（朝工作面的逼近禁消隙过冲）。
        """
        try:
            self.traj.move_to(self.serial, q, overshoot=overshoot)
        except TrajectoryError as e:
            return self._to_error(str(e))
        if self._timeout(t0, state):
            return self._to_error("%s timeout" % state)
        return True

    def _at_home(self, tol=1.0):
        """当前关节角是否在 HOME（甜区）附近（容差 tol°）。"""
        return all(abs(self.serial.joint_state[i] - config.JOINT_HOME[i]) <= tol
                   for i in range(6))

    def _descend_level(self, t0):
        """DESCEND 单级：对齐 → 面积比检查 → 下降一档 → 红外判定。

        面积比计算公式和阈值引用（DESCEND_AREA_TARGET/SERVO_SWITCH_AREA_RATIO）一字不变。

        Returns:
            "OK"     — 成功下降一档，调用方继续下一级
            "TARGET" — 面积比 ≥ DESCEND_AREA_TARGET，够近，停止下降
            "SWITCH" — 面积比 ≥ SERVO_SWITCH_AREA_RATIO，切 FINAL_ALIGN
            "IR"     — 红外触发（当前位置即抓取位）
            "LOST"   — 目标丢失（调用方回 SEARCH 重扫）
            "ERR"    — 失败（已转 ERROR）
        """
        if self._timeout(t0, "DESCEND"):
            return "ERR"  # caller prints timeout
        # 每级先对齐再降
        res = self._servo_align(t0, "DESCEND")
        if res[0] == "LOST":
            return "LOST"
        if res[0] != "OK":
            return "ERR"
        # 复用 _servo_align 缓存的 detect_full 结果（单次推理，无二次 detect）
        full = self._last_detect
        if full is not None:
            ratio = full['area_ratio']
            if ratio >= config.DESCEND_AREA_TARGET:
                print("[SM] DESCEND: 面积占比 %.3f 够近，"
                      "停止下降" % ratio)
                return "TARGET"
            if ratio >= config.SERVO_SWITCH_AREA_RATIO:
                print("[SM] DESCEND: 面积占比 %.3f 达切换阈值"
                      "（>%.3f），切 FINAL_ALIGN" % (
                          ratio, config.SERVO_SWITCH_AREA_RATIO))
                return "SWITCH"
        # 下降一档：FK 当前 x,y → IK 解降 z（水平不漂移，抬升必可达）
        x_now, y_now, z_now = kinematics.fk(self.serial.joint_state)
        q, reason = kinematics.ik_solve(
            x_now, y_now, z_now - config.DESCEND_STEP,
            config.GRIP_OPEN)
        if q is None:
            self._to_error("DESCEND: IK failed: %s" % reason)
            return "ERR"
        try:
            # 朝桌面/木块下降：禁消隙过冲（过冲点在目标下方，会撞物）
            self.traj.move_to(self.serial, q, overshoot=False)
        except TrajectoryError as e:
            self._to_error("DESCEND move: %s" % e)
            return "ERR"
        # 红外触发 = 已接触目标，提前结束下降
        if self.ir.wait_blocked(300):
            return "IR"
        return "OK"

    def _spiral_search(self, t0):
        """DESCEND 红外未触发时的微搜索：当前高度做阿基米德螺线扫描。

        UTwente 2026：接触阶段螺旋搜索补偿残余对准误差——覆盖均匀、
        无方向偏好，优于十字扫。每点经 FK+IK 保持 z 不变（水平不漂移），
        越限/不可达点跳过；移动禁消隙过冲（近距过冲会撞物）。

        Returns:
            True = 红外触发（当前位置即抓取位，调用方继续 GRIP 流程）
            False = 扫完全部点未触发（调用方转 ERROR）
        """
        if not config.SPIRAL_ENABLED:
            return False
        self._set_state("SPIRAL")
        t0_local = time.monotonic()             # 螺旋独立计时（不占 DESCEND 预算）
        x0, y0, z0 = kinematics.fk(self.serial.joint_state)
        print("[SM] SPIRAL: 起点 (%.0f, %.0f, %.0f) mm，%d 点 / %d 圈 / R=%.0fmm"
              % (x0, y0, z0, config.SPIRAL_POINTS, config.SPIRAL_TURNS,
                 config.SPIRAL_R_MAX))
        for k in range(1, config.SPIRAL_POINTS + 1):
            if self._timeout(t0_local, "SPIRAL"):
                return False
            frac = float(k) / config.SPIRAL_POINTS
            r = config.SPIRAL_R_MAX * frac
            th = 2.0 * math.pi * config.SPIRAL_TURNS * frac
            xs = x0 + r * math.cos(th)
            ys = y0 + r * math.sin(th)
            q, reason = kinematics.ik_solve(xs, ys, z0, config.GRIP_OPEN)
            if q is None:
                continue                     # 越限/不可达点跳过
            try:
                self.traj.move_to(self.serial, q, overshoot=False)
            except TrajectoryError:
                continue                     # 单点失败不放弃整轮螺旋
            time.sleep(config.SPIRAL_SETTLE_MS / 1000.0)
            # 去抖窗口最少样本数 × 轮询间隔（短于它必超时）
            ir_timeout_ms = (config.IR_TRIGGER_COUNT + 2) * config.IR_POLL_MS
            blocked = self.ir.wait_blocked(ir_timeout_ms)
            print("[SM] SPIRAL %d/%d: (%.0f, %.0f) IR=%s"
                  % (k, config.SPIRAL_POINTS, xs, ys, blocked))
            if blocked:
                print("[SM] SPIRAL: 红外触发于第 %d 点" % k)
                return True
        return False

    # ---------- 视觉伺服 ----------
    def _servo_align(self, t0, state):
        """视觉伺服：让目标收敛到画面中心（wrist-first 分层）。

        误差路由（按幅度）：
          < ALIGN_DEADBAND_MM → 不动（收敛判定不变）
          ≤ 腕层权限 → J4 单轴修正（量子化 + slack 钳制）
          超腕层或停滞 → 粗层 J1/J2/J3（最小步距 COARSE_MIN_STEP_DEG）
        方向级分解（J4 管 z 向 / J5 管横向）待装机日实测，v1 按幅度路由。

        丢失保护：连续 LOST_FRAME_THRESHOLD 帧未检出 → 判定目标丢失。
        跳变检测：误差突然放大（>上帧×RATIO+JUMP_PX）→ 异常帧丢弃。

        Returns:
            ("OK", (cx, cy, theta_deg)) 对齐完成
            ("LOST", None) 目标丢失（调用方回 SEARCH 重扫）
            ("ERR", None) 失败（已转 ERROR）

        Side effect:
            self._last_detect — 最后一次 detect_full 结果 dict（供 DESCEND 复用，
            消除同帧二次推理）。LOST/ERR 时为 None。
        """
        self._last_detect = None
        if self.vision is None or self.cap is None:
            self._to_error("%s: vision/cap not provided" % state)
            return ("ERR", None)
        cx0 = cy0 = None               # 伺服目标点（首帧实拍坐标计算，见下）
        lost = 0
        prev_err = None
        wrist_stall_count = 0          # 腕层停滞计数（连续腕修正误差未下降 → 升粗层）
        wrist_center = None            # J4 回合锚点（最近粗层解的 J4；None=待锚定）
        prev_wrist_err = None          # 上次腕层误差（用于停滞判定）
        self.ctrl.reset_ema()          # EMA 状态每次对齐独立，不跨调用
        for _ in range(config.ALIGN_MAX_ITER):
            if self._timeout(t0, state):
                self._to_error("%s timeout" % state)
                return ("ERR", None)
            ret, frame = self.cap.read()
            if not ret:
                continue
            if cx0 is None:
                # 伺服目标 = 画面中心 + 相机-夹爪偏移（config.CAMERA_OFFSET_X/Y）。
                # 用实拍帧尺寸而非 cap.get()（部分摄像头属性与实际帧不一致）；
                # 偏移默认 0 = 目标即画面中心（旧行为）。
                cx0, cy0 = self.ctrl.compute_offset_target(
                    frame.shape[0], frame.shape[1])
            hit = self.vision.detect(frame)
            if hit is None:
                lost += 1
                if lost >= config.LOST_FRAME_THRESHOLD:
                    print("[SM] %s: 连续 %d 帧丢失，判定目标丢失"
                          % (state, lost))
                    return "LOST", None
                time.sleep(config.ALIGN_POLL_MS / 1000.0)
                continue
            lost = 0
            cx, cy, theta_deg, conf = hit
            # 缓存 detect_full 结果（DESCEND 复用，消除同帧二次推理）
            full = self.vision.detect_full(frame)
            if full is not None:
                self._last_detect = full
            # EMA 平滑检测坐标（EMA_ALPHA=1.0 直通 = 旧行为；0.3-0.7 联调再降）
            cx, cy = self.ctrl.smooth_ema(cx, cy)
            ex, ey, err = self.ctrl.compute_error(cx, cy, cx0, cy0)
            # 跳变检测：误差突然放大 = 误检/检测跳变，丢弃本帧不动作
            if prev_err is not None and \
                    err > prev_err * config.ALIGN_JUMP_RATIO \
                    + config.ALIGN_JUMP_PX:
                print("[SM] %s: 误差跳变 %.0f→%.0f px，丢弃异常帧"
                      % (state, prev_err, err))
                continue
            prev_err = err
            # --- 收敛判定（不变）---
            if abs(ex) < config.ALIGN_PX_TOL and abs(ey) < config.ALIGN_PX_TOL:
                return "OK", (cx, cy, theta_deg)
            # --- 像素误差 → 毫米误差（粗略换算，用于 wrist-first 路由）---
            # PX_TO_MM 未标定预置值（config；装机日标定板实测回填）
            err_mm = err * config.PX_TO_MM
            # --- 路由：deadband → 腕层 → 粗层 ---
            if err_mm < config.ALIGN_DEADBAND_MM:
                # 误差极小，本轮不动（收敛判定已在上面处理，这里防抖）
                time.sleep(config.ALIGN_POLL_MS / 1000.0)
                continue
            mm_per_deg = config.L4 * math.sin(math.radians(1.0))
            wrist_budget_mm = config.J4_SLACK_DEG * mm_per_deg
            if err_mm <= wrist_budget_mm and wrist_stall_count < config.WRIST_STALL_LIMIT:
                # --- 腕层路径：J4 单轴修正 ---
                if wrist_center is None:
                    wrist_center = self.serial.joint_state[3]  # 回合锚：最近粗层解的 J4
                dq4, q4_target = compute_wrist_correction(err_mm,
                                                          self.serial.joint_state,
                                                          wrist_center)
                if abs(dq4) < 1e-6:
                    # J4 已在 slack 边界饱和（或量化为 0）→ 记停滞，
                    # 下轮路由升粗层，避免边界无限循环
                    wrist_stall_count = config.WRIST_STALL_LIMIT
                    time.sleep(config.ALIGN_POLL_MS / 1000.0)
                    continue
                # 构造目标：只变 J4，其余轴用当前 joint_state
                q = list(self.serial.joint_state)
                q[3] = q4_target
                ok, msg = self.ctrl.check_limits(q, state)
                if not ok:
                    self._to_error(msg)
                    return ("ERR", None)
                try:
                    self.traj.move_to(self.serial, q, overshoot=False)
                except TrajectoryError as e:
                    self._to_error("%s wrist move: %s" % (state, e))
                    return ("ERR", None)
                # 停滞检测：连续腕修正误差未下降 → 升粗层
                if prev_wrist_err is not None and err_mm >= prev_wrist_err:
                    wrist_stall_count += 1
                else:
                    wrist_stall_count = 0
                prev_wrist_err = err_mm
            else:
                # --- 粗层路径：J1/J2/J3（最小步距纪律）---
                dq0, dq12 = self.ctrl.align_delta(ex, ey, config.ALIGN_MAX_STEP)
                # 最小步距：非零 delta < COARSE_MIN_STEP_DEG → 提升到该值（保号）
                if abs(dq0) > 1e-6 and abs(dq0) < config.COARSE_MIN_STEP_DEG:
                    dq0 = config.COARSE_MIN_STEP_DEG if dq0 > 0 else -config.COARSE_MIN_STEP_DEG
                if abs(dq12) > 1e-6 and abs(dq12) < config.COARSE_MIN_STEP_DEG:
                    dq12 = config.COARSE_MIN_STEP_DEG if dq12 > 0 else -config.COARSE_MIN_STEP_DEG
                q = self.ctrl.apply_joint_deltas(self.serial.joint_state,
                                                 dq0, dq12)
                # J4 由 IK 解重设，腕层预算复位并重锚回合
                wrist_stall_count = 0
                prev_wrist_err = None
                wrist_center = None
                ok, msg = self.ctrl.check_limits(q, state)
                if not ok:
                    self._to_error(msg)
                    return ("ERR", None)
                try:
                    # 对齐微调朝木块逼近：禁消隙过冲（过冲会把末端压向目标外）
                    self.traj.move_to(self.serial, q, overshoot=False)
                except TrajectoryError as e:
                    self._to_error("%s servo move: %s" % (state, e))
                    return ("ERR", None)
            time.sleep(config.ALIGN_POLL_MS / 1000.0)
        self._to_error("%s: align not converged" % state)
        return ("ERR", None)

    # ---------- 主流程 ----------
    def run_grasp(self, x=None, y=None):
        """视觉伺服抓取：搜索 → 对齐 → 逐级下降 → 红外确认 → 抓取。

        x, y 为可选毫米坐标（旧路线兼容）；None 时走视觉伺服全流程。
        成功 True，失败 False（已回 HOME）。
        """
        self._set_state("IDLE")
        self._target = (x, y) if x is not None else None

        # 1. SEARCH：J1 转圈扫描，检测到锁存方位（视觉伺服路线）
        #    起点 = JOINT_HOME（最大甜区）；目标丢失 → 回 SEARCH 重扫（最多 N 次）
        if self.vision is not None and self.cap is not None:
            for attempt in range(config.SEARCH_RETRY_MAX):
                # 确保在甜区（HOME）再开始搜索
                if attempt > 0 or not self._at_home():
                    try:
                        self.traj.move_to(self.serial, config.JOINT_HOME)
                    except TrajectoryError as e:
                        return self._to_error("SEARCH: 回 HOME 失败: %s" % e)
                self._set_state("SEARCH")
                t0 = time.monotonic()
                found = False
                search_dir = 1.0  # J1 行程 0-180°，往复扫描（到限位反向）
                for step in range(config.SEARCH_MAX_STEPS):
                    if self._timeout(t0, "SEARCH"):
                        return self._to_error("SEARCH timeout")
                    ret, frame = self.cap.read()
                    if ret:
                        hit = self.vision.detect(frame)
                        if hit is not None:
                            cx, cy, theta_deg, conf = hit
                            self._theta = theta_deg
                            print("[SM] SEARCH 命中: px=(%.0f,%.0f) θ=%.1f° conf=%.2f"
                                  % (cx, cy, theta_deg, conf))
                            found = True
                            break
                    # 转一步（J1 增量），到限位反向（往复扫描，不触发 CLAMP）
                    q = list(self.serial.joint_state)
                    nq = q[0] + search_dir * config.SEARCH_ANGLE_STEP
                    if nq > config.JOINT_MAX[0] or nq < config.JOINT_MIN[0]:
                        search_dir = -search_dir
                        nq = q[0] + search_dir * config.SEARCH_ANGLE_STEP
                    q[0] = nq
                    try:
                        # 扫描运动禁消隙过冲：反向时 15° 过冲会让画面跳变、扫描非单调
                        self.traj.move_to(self.serial, q, overshoot=False)
                    except TrajectoryError as e:
                        return self._to_error("SEARCH move: %s" % e)
                    time.sleep(config.ALIGN_POLL_MS / 1000.0)
                if not found:
                    return self._to_error("SEARCH: 未检测到目标（%d 步）"
                                          % config.SEARCH_MAX_STEPS)

                # 2. ALIGN：伺服收敛到画面中心
                self._set_state("ALIGN")
                t0 = time.monotonic()
                res = self._servo_align(t0, "ALIGN")
                if res[0] == "LOST":
                    print("[SM] ALIGN 目标丢失，回 SEARCH 重扫（%d/%d）"
                          % (attempt + 1, config.SEARCH_RETRY_MAX))
                    continue
                if res[0] != "OK":
                    return False
                cx, cy, theta_deg = res[1]
                self._theta = theta_deg
                print("[SM] ALIGN 完成: px=(%.0f,%.0f) θ=%.1f°"
                      % (cx, cy, theta_deg))

                # 3. DESCEND：逐级下降 + 每级对齐（防目标跑出画面）
                #    远距连续伺服 → 面积比达 SERVO_SWITCH_AREA_RATIO → 切 FINAL_ALIGN
                self._set_state("DESCEND")
                t0 = time.monotonic()
                descend_ok = True
                for level in range(config.DESCEND_LEVELS):
                    result = self._descend_level(t0)
                    if result == "ERR":
                        return False
                    if result == "LOST":
                        print("[SM] DESCEND 目标丢失，回 SEARCH 重扫（%d/%d）"
                              % (attempt + 1, config.SEARCH_RETRY_MAX))
                        descend_ok = False
                        break
                    if result in ("IR", "TARGET", "SWITCH"):
                        break
                    # "OK" → continue to next level
                else:
                    # 全部下降档未触发红外 → 螺旋微搜索（当前高度，补偿
                    # 残余对准误差）；扫到则继续 FINAL_ALIGN，否则报错
                    if not self._spiral_search(t0):
                        return self._to_error("DESCEND: 未触发红外（%d 级）"
                                              % config.DESCEND_LEVELS)
                if descend_ok is False:
                    continue  # 目标丢失 → 回 SEARCH 重扫

                # 3b. FINAL_ALIGN：停-看-动精对准（近距，像素级收敛）
                #    停稳 → 拍 → 误差大则小幅限幅修正 → 停 → 再拍确认
                self._set_state("FINAL_ALIGN")
                t0 = time.monotonic()
                time.sleep(config.FINAL_ALIGN_SETTLE_MS / 1000.0)  # 停稳防画面抖
                final_ok = False
                for _ in range(config.FINAL_ALIGN_MAX_ITER):
                    if self._timeout(t0, "FINAL_ALIGN"):
                        return self._to_error("FINAL_ALIGN timeout")
                    ret, frame = self.cap.read()
                    if not ret:
                        continue
                    hit = self.vision.detect(frame)
                    if hit is None:
                        print("[SM] FINAL_ALIGN: 未检出，重试")
                        time.sleep(config.ALIGN_POLL_MS / 1000.0)
                        continue
                    cx, cy, theta_deg, conf = hit
                    # 与 _servo_align 同一目标点（画面中心+相机-夹爪偏移）：
                    # 否则 ALIGN 用偏移目标、FINAL_ALIGN 用几何中心，两阶段拉扯
                    cx0, cy0 = self.ctrl.compute_offset_target(
                        frame.shape[0], frame.shape[1])
                    ex, ey, _err = self.ctrl.compute_error(cx, cy, cx0, cy0)
                    if abs(ex) < config.ALIGN_PX_TOL \
                            and abs(ey) < config.ALIGN_PX_TOL:
                        print("[SM] FINAL_ALIGN: 确认在中心 (%.1f, %.1f) θ=%.1f°"
                              % (cx, cy, theta_deg))
                        final_ok = True
                        break
                    # 死区：误差在 deadband 内但未达收敛阈值 → 不修正（防微抖）
                    if _err * config.PX_TO_MM < config.ALIGN_DEADBAND_MM:
                        time.sleep(config.ALIGN_POLL_MS / 1000.0)
                        continue
                    # 小幅限幅修正（近距一步不能过头）
                    dq0, dq12 = self.ctrl.align_delta(ex, ey, config.FINAL_ALIGN_MAX_STEP)
                    q = self.ctrl.apply_joint_deltas(self.serial.joint_state,
                                                     dq0, dq12)
                    if config.J5_ALIGN_ENABLED:     # θ 符号/象限实测后才启用（config）
                        q[4] = theta_deg - q[0]    # J5 对齐夹爪朝向（θ − J1）
                    ok, msg = self.ctrl.check_limits(q, "FINAL_ALIGN")
                    if not ok:
                        return self._to_error(msg)
                    try:
                        # 近距精对准朝木块逼近：禁消隙过冲（防撞物）
                        self.traj.move_to(self.serial, q, overshoot=False)
                    except TrajectoryError as e:
                        return self._to_error("FINAL_ALIGN move: %s" % e)
                    time.sleep(config.FINAL_ALIGN_SETTLE_MS / 1000.0)  # 修后停稳再拍
                if not final_ok:
                    return self._to_error(
                        "FINAL_ALIGN: 未收敛（%d 轮）"
                        % config.FINAL_ALIGN_MAX_ITER)
                break  # 全流程成功，退出重试循环

        else:
            # 旧路线兼容：毫米坐标抓取（无视觉伺服时）
            if x is None or y is None:
                return self._to_error("run_grasp: 需要 (x,y) 或 vision+cap")
            # 可达性预检（APPROACH 高度）
            if not kinematics.is_reachable(x, y, config.SAFE_Z):
                return self._to_error("target unreachable at (%.0f, %.0f)" % (x, y))
            self._set_state("APPROACH")
            t0 = time.monotonic()
            q, reason = kinematics.ik_solve(x, y, config.SAFE_Z, config.GRIP_OPEN)
            if q is None:
                return self._to_error(reason)
            if not self._move(q, "APPROACH", t0):
                return False
            # DESCEND：逐步下降（单调递减），红外触发停
            self._set_state("DESCEND")
            t0 = time.monotonic()
            z = config.APPROACH_Z
            z_min = config.APPROACH_Z - config.DESCEND_MAX
            while z > z_min:
                z = max(z_min, z - config.DESCEND_STEP)
                q, reason = kinematics.ik_solve(x, y, z, config.GRIP_OPEN)
                if q is None:
                    return self._to_error(reason)
                # 朝桌面下降：禁消隙过冲（过冲点在目标下方，会撞物）
                if not self._move(q, "DESCEND", t0, overshoot=False):
                    return False
                if self.ir.wait_blocked(300):      # 红外触发 = 末端已接触目标
                    break
            else:
                # 下降全程未触发红外 → 螺旋微搜索（旧路线同样受益）
                if not self._spiral_search(t0):
                    return self._to_error(
                        "DESCEND: no IR trigger within %.0f mm" % config.DESCEND_MAX)

        # 4. GRIP：夹爪闭合，等待
        self._set_state("GRIP")
        t0 = time.monotonic()
        q = list(self.serial.joint_state)
        q[5] = config.GRIP_CLOSE
        if not self._move(q, "GRIP", t0):
            return False
        time.sleep(config.GRIP_WAIT_MS / 1000.0)

        # 5. TRANSPORT：抬升 → 移到放置位上方 → 降到放置高度
        self._set_state("TRANSPORT")
        t0 = time.monotonic()
        # 抬升：FK 算当前位置 → IK 解同 x,y 的 SAFE_Z（不依赖毫米标定）
        x_now, y_now, _ = kinematics.fk(self.serial.joint_state)
        q_up, reason = kinematics.ik_solve(x_now, y_now, config.SAFE_Z,
                                           config.GRIP_CLOSE)
        if q_up is None:
            return self._to_error("TRANSPORT: lift IK failed: %s" % reason)
        if not self._move(q_up, "TRANSPORT", t0):
            return False
        # 动态放置：沿当前 J1 方向偏移 PLACE_RADIUS_MM（臂不扭转，恒在可达域）
        j1_rad = math.radians(self.serial.joint_state[0])
        px = config.PLACE_RADIUS_MM * math.cos(j1_rad)
        py = config.PLACE_RADIUS_MM * math.sin(j1_rad)
        # 可达性自检（放置点可能因臂构型不同而不可达）
        if not kinematics.is_reachable(px, py, config.PLACE_Z):
            return self._to_error(
                "TRANSPORT: place unreachable at r=%.0f z=%.0f — "
                "adjust PLACE_RADIUS_MM/PLACE_Z" % (
                    config.PLACE_RADIUS_MM, config.PLACE_Z))
        q_place, _ = kinematics.ik_solve(px, py, config.SAFE_Z, config.GRIP_CLOSE)
        if q_place is None:
            return self._to_error("TRANSPORT: place IK failed")
        if not self._move(q_place, "TRANSPORT", t0):
            return False
        q_low, _ = kinematics.ik_solve(px, py, config.PLACE_Z, config.GRIP_CLOSE)
        if q_low is None:
            return self._to_error("TRANSPORT: lower IK failed")
        if not self._move(q_low, "TRANSPORT", t0):
            return False

        # 6. RELEASE：张开夹爪
        self._set_state("RELEASE")
        t0 = time.monotonic()
        q_low[5] = config.GRIP_OPEN
        if not self._move(q_low, "RELEASE", t0):
            return False
        time.sleep(config.RELEASE_WAIT_MS / 1000.0)

        # 7. HOME：抬升 → 回中位
        self._set_state("HOME")
        t0 = time.monotonic()
        q_up2, _ = kinematics.ik_solve(x_now, y_now, config.SAFE_Z, config.GRIP_OPEN)
        if q_up2 is not None:
            if not self._move(q_up2, "HOME", t0):
                return False
        if not self._move(config.JOINT_HOME, "HOME", t0):
            return False

        self._set_state("IDLE")
        return True