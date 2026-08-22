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

import cv2

import config
import kinematics
from trajectory import TrajectoryError

log = logging.getLogger(__name__)


class StateMachine:
    def __init__(self, serial, traj, ir, vision=None, cap=None):
        self.serial = serial
        self.traj = traj
        self.ir = ir
        self.vision = vision          # 视觉伺服用（SEARCH/ALIGN 需要）
        self.cap = cap                # 摄像头（SEARCH/ALIGN 需要）
        self.state = "IDLE"
        self.log = []
        self._target = None            # 当前抓取目标 (x, y)，ERROR 抬升用（P1-1）
        self._theta = None             # 当前目标角度（J5 对齐用）

    # ---------- 内部 ----------
    def _set_state(self, s):
        self.state = s
        self.log.append((time.monotonic(), s))
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
        x0, y0, z0 = kinematics.fk(self.serial.joint_state)
        print("[SM] SPIRAL: 起点 (%.0f, %.0f, %.0f) mm，%d 点 / %d 圈 / R=%.0fmm"
              % (x0, y0, z0, config.SPIRAL_POINTS, config.SPIRAL_TURNS,
                 config.SPIRAL_R_MAX))
        for k in range(1, config.SPIRAL_POINTS + 1):
            if self._timeout(t0, "SPIRAL"):
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
            blocked = self.ir.wait_blocked(config.IR_POLL_MS)
            print("[SM] SPIRAL %d/%d: (%.0f, %.0f) IR=%s"
                  % (k, config.SPIRAL_POINTS, xs, ys, blocked))
            if blocked:
                print("[SM] SPIRAL: 红外触发于第 %d 点" % k)
                return True
        return False

    # ---------- 视觉伺服 ----------
    def _servo_align(self, t0, state):
        """视觉伺服：让目标收敛到画面中心（像素域 PID）。

        误差 = 目标像素位置 - 画面中心；关节增量 = Kp·e（先只调 P）。
        丢失保护：连续 LOST_FRAME_THRESHOLD 帧未检出 → 判定目标丢失。
        跳变检测：误差突然放大（>上帧×RATIO+JUMP_PX）→ 异常帧丢弃（防误检污染）。

        Returns:
            ("OK", (cx, cy, theta_deg)) 对齐完成
            ("LOST", None) 目标丢失（调用方回 SEARCH 重扫）
            ("ERR", None) 失败（已转 ERROR）
        """
        if self.vision is None or self.cap is None:
            return self._to_error("%s: vision/cap not provided" % state), None
        frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
        frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
        cx0, cy0 = frame_w / 2.0, frame_h / 2.0
        lost = 0
        prev_err = None
        for _ in range(config.ALIGN_MAX_ITER):
            if self._timeout(t0, state):
                return self._to_error("%s timeout" % state), None
            ret, frame = self.cap.read()
            if not ret:
                continue
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
            ex, ey = cx - cx0, cy - cy0
            err = (ex * ex + ey * ey) ** 0.5
            # 跳变检测：误差突然放大 = 误检/检测跳变，丢弃本帧不动作
            if prev_err is not None and \
                    err > prev_err * config.ALIGN_JUMP_RATIO \
                    + config.ALIGN_JUMP_PX:
                print("[SM] %s: 误差跳变 %.0f→%.0f px，丢弃异常帧"
                      % (state, prev_err, err))
                continue
            prev_err = err
            if abs(ex) < config.ALIGN_PX_TOL and abs(ey) < config.ALIGN_PX_TOL:
                return "OK", (cx, cy, theta_deg)
            # 像素误差 → 关节增量（J1 水平纠偏，J2/J3 垂直纠偏）
            # 增量方向/符号待实测标定（摄像头安装方向决定），先按正方向
            # 限幅：大误差时一步不能超 ALIGN_MAX_STEP（防超限/过冲）
            q = list(self.serial.joint_state)
            dq0 = max(-config.ALIGN_MAX_STEP,
                      min(config.ALIGN_MAX_STEP, config.SERVO_KP * ex))
            dq12 = max(-config.ALIGN_MAX_STEP,
                       min(config.ALIGN_MAX_STEP, config.SERVO_KP * ey * 0.5))
            q[0] += dq0
            q[1] += dq12
            q[2] += dq12
            # 限位保护：伺服增量不得推出关节限位（超限 = 目标不可达，转 ERROR）
            for i in range(6):
                if q[i] < config.JOINT_MIN[i] or q[i] > config.JOINT_MAX[i]:
                    return self._to_error(
                        "%s: servo target joint %d out of range %.1f" % (
                            state, i + 1, q[i])), None
            try:
                # 对齐微调朝木块逼近：禁消隙过冲（过冲会把末端压向目标外）
                self.traj.move_to(self.serial, q, overshoot=False)
            except TrajectoryError as e:
                return self._to_error("%s servo move: %s" % (state, e))
            time.sleep(config.ALIGN_POLL_MS / 1000.0)
        return self._to_error("%s: align not converged" % state), None

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
                #    远距连续伺服 → 木块占比达 SERVO_SWITCH_RATIO → 切 FINAL_ALIGN
                self._set_state("DESCEND")
                t0 = time.monotonic()
                descend_ok = True
                for level in range(config.DESCEND_LEVELS):
                    if self._timeout(t0, "DESCEND"):
                        return self._to_error("DESCEND timeout")
                    # 每级先对齐再降
                    res = self._servo_align(t0, "DESCEND")
                    if res[0] == "LOST":
                        print("[SM] DESCEND 目标丢失，回 SEARCH 重扫（%d/%d）"
                              % (attempt + 1, config.SEARCH_RETRY_MAX))
                        descend_ok = False
                        break
                    if res[0] != "OK":
                        return False
                    cx, cy, theta_deg = res[1]
                    # 算木块像素占比，达到切换阈值 → 退出下降，进 FINAL_ALIGN
                    ret, frame = self.cap.read()
                    if ret:
                        hit = self.vision.detect(frame)
                        if hit is not None:
                            results = self.vision.model.predict(
                                frame, conf=self.vision.conf, verbose=False)
                            if results and results[0].obb is not None \
                                    and len(results[0].obb) > 0:
                                _, _, w, h, _ = results[0].obb.xywhr[0].tolist()
                                ratio = max(w, h) / frame.shape[1]
                                if ratio >= config.DESCEND_PX_TARGET:
                                    print("[SM] DESCEND: 木块占比 %.2f 够近，"
                                          "停止下降" % ratio)
                                    break
                                if ratio >= config.SERVO_SWITCH_RATIO:
                                    print("[SM] DESCEND: 占比 %.2f 达切换阈值"
                                          "（>%.2f），切 FINAL_ALIGN" % (
                                              ratio, config.SERVO_SWITCH_RATIO))
                                    break
                    # 下降一档：FK 当前 x,y → IK 解降 z（水平不漂移，抬升必可达）
                    x_now, y_now, z_now = kinematics.fk(self.serial.joint_state)
                    q, reason = kinematics.ik_solve(
                        x_now, y_now, z_now - config.DESCEND_STEP,
                        config.GRIP_OPEN)
                    if q is None:
                        return self._to_error("DESCEND: IK failed: %s" % reason)
                    try:
                        # 朝桌面/木块下降：禁消隙过冲（过冲点在目标下方，会撞物）
                        self.traj.move_to(self.serial, q, overshoot=False)
                    except TrajectoryError as e:
                        return self._to_error("DESCEND move: %s" % e)
                    # 红外触发 = 已接触目标，提前结束下降
                    if self.ir.wait_blocked(300):
                        break
                    if descend_ok is False:
                        break
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
                    frame_w = frame.shape[1]
                    ex, ey = cx - frame_w / 2.0, cy - frame.shape[0] / 2.0
                    if abs(ex) < config.ALIGN_PX_TOL \
                            and abs(ey) < config.ALIGN_PX_TOL:
                        print("[SM] FINAL_ALIGN: 确认在中心 (%.1f, %.1f) θ=%.1f°"
                              % (cx, cy, theta_deg))
                        final_ok = True
                        break
                    # 小幅限幅修正（近距一步不能过头）
                    q = list(self.serial.joint_state)
                    dq0 = config.SERVO_KP * ex
                    dq0 = max(-config.FINAL_ALIGN_MAX_STEP,
                              min(config.FINAL_ALIGN_MAX_STEP, dq0))
                    dq12 = config.SERVO_KP * ey * 0.5
                    dq12 = max(-config.FINAL_ALIGN_MAX_STEP,
                               min(config.FINAL_ALIGN_MAX_STEP, dq12))
                    q[0] += dq0
                    q[1] += dq12
                    q[2] += dq12
                    for i in range(6):
                        if q[i] < config.JOINT_MIN[i] or q[i] > config.JOINT_MAX[i]:
                            return self._to_error(
                                "FINAL_ALIGN: joint %d out of range %.1f"
                                % (i + 1, q[i]))
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
        px, py = config.PLACE_POS
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
        q_up2, _ = kinematics.ik_solve(px, py, config.SAFE_Z, config.GRIP_OPEN)
        if q_up2 is not None:
            if not self._move(q_up2, "HOME", t0):
                return False
        if not self._move(config.JOINT_HOME, "HOME", t0):
            return False

        self._set_state("IDLE")
        return True