# -*- coding: utf-8 -*-
"""arm_serial.py — 与 STM32 固件的 USART3 串口通信层。

协议（固件 usbd_cdc_interface.c，CRLF 结尾）：
    M<n> <angle>  → OK M<n> <clamped_angle>  （回显 clamp 后的实际角度！）
    MALL <a1..a6> → OK MALL ...
    S             → OK S1=<a1> ... S6=<a6>    （当前关节状态）
    I             → OK IR1=<0|1> IR2=<0|1>    （遮挡=0）
    H             → OK H（全回中位 + 清急停）
    E             → OK E（急停：停全部 PWM，拒绝 M 直到 H）

校验和（借鉴 Panthera-HT 双 CRC，文本协议简化版）：
    本模块发送的每条命令自动附加 "*XX" 后缀（XX = 命令字节的 XOR，
    两位大写十六进制）。固件校验失败回 "ERR CKS" 并拒绝执行——
    防止舵机 EMI 把 "M2 90" 干扰成 "M2 98"（错角度）或 "M5 90"（错关节）。
    无 "*XX" 的裸命令固件仍接受（串口助手手动测试用）。
    应答方向不加校验：回显铁律的角度交叉核对已覆盖该方向。

回显铁律（架构书 §6 / CLAUDE.md）：M<n> 回显 clamp 后实际角度，
本模块必须解析回显更新 joint_state；请求 vs 回显偏差 > ANGLE_ECHO_TOL
→ 返回警告（调用方暂停）。严禁忽略回显，否则虚拟角度与物理脱节。
"""

import re
import time

import serial  # pip install pyserial

import config
from kinematics import from_servo, to_servo


def _checksum(cmd):
    """命令字节 XOR 校验和，两位大写十六进制。"""
    x = 0
    for ch in cmd:
        x ^= ord(ch)
    return "%02X" % x


class ArmSerial:
    """同步命令-应答模型：发一条等一条，无接收线程，简单可靠。

    DRY_RUN 模式（联调必开）：不发串口，假回显 OK，回显=请求值。
    """

    def __init__(self, port=None, dry_run=False):
        self.dry_run = dry_run
        self.joint_state = list(config.JOINT_HOME)  # 当前实际关节角（由回显维护）
        self.estop_active = False
        self.timeout_active = False
        self._ser = None
        self.last_warning = None
        if not dry_run:
            try:
                self._ser = serial.Serial(port or config.COM_PORT,
                                          config.BAUDRATE,
                                          timeout=config.SERIAL_TIMEOUT)
            except serial.SerialException as e:
                raise SystemExit(
                    "无法打开串口 %s: %s\n"
                    "检查：设备管理器 COM 号 / 是否被占用 / 板子是否枚举" % (
                        port or config.COM_PORT, e))

    # ---------- 底层 ----------
    def _send(self, cmd, timeout=None):
        """发命令并读一行应答。返回应答字符串（去 CRLF）或 None（超时）。

        真实串口路径自动附加 "*XX" XOR 校验后缀（固件校验失败回 ERR CKS）；
        DRY_RUN 路径传裸命令（_dry_echo 的正则按裸命令匹配）。
        """
        if self.dry_run:
            return self._dry_echo(cmd)
        timeout = timeout or config.RESP_TIMEOUT
        assert self._ser is not None
        payload = "%s*%s" % (cmd, _checksum(cmd))
        self._ser.write((payload + "\r\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._ser.readline()
            if line:
                return line.decode(errors="replace").strip()
        # 超时：排空迟到的旧回显，防错位回显污染后续解析（P2-3）
        self._ser.reset_input_buffer()
        return None

    def _dry_echo(self, cmd):
        """DRY_RUN 假回显：模拟固件视角——收到舵机角，按关节限位 clamp 后回显。

        注意：move_joint 已做关节角→舵机角转换，这里绝不能再 +OFFSET（双重转换 bug）。
        clamp 用固件的 joint_min/max（舵机角域），与真实固件行为一致。
        限位表从 config 单一来源导出（消除与固件的重复维护）。
        """
        m = re.match(r"^M(\d)\s+([-\d.]+)$", cmd)
        if m:
            n, a = int(m.group(1)), float(m.group(2))
            # 从 config 关节限位 + 偏移量导出舵机域限位（单一真源）
            fw_min = [config.JOINT_MIN[i] + config.JOINT_OFFSET[i] for i in range(6)]
            fw_max = [config.JOINT_MAX[i] + config.JOINT_OFFSET[i] for i in range(6)]
            servo = max(float(fw_min[n - 1]), min(float(fw_max[n - 1]), a))
            return "OK M%d %.1f" % (n, servo)
        if cmd.startswith("MALL"):
            return "OK MALL"
        if cmd == "I":
            return "OK IR1=1 IR2=1"
        if cmd == "S":
            servos = to_servo(self.joint_state)
            return "OK " + " ".join("S%d=%.1f" % (i + 1, servos[i])
                                    for i in range(6))
        if cmd == "H":
            self.joint_state = list(config.JOINT_HOME)
            self.estop_active = False
            return "OK H"
        if cmd == "E":
            self.estop_active = True
            return "OK E"
        return "ERR unknown"

    # ---------- 命令 ----------
    def _parse_ok_echo(self, resp):
        """从 'OK M1 90.0' 解析 (joint_idx, echo_angle)；非 M 回显返回 None。"""
        m = re.match(r"^OK M(\d)\s+([-\d.]+)$", resp or "")
        if m:
            return int(m.group(1)) - 1, float(m.group(2))
        return None

    def move_joint(self, n, angle):
        """移动单关节（1-6）。angle 为关节角。

        发送：关节角 → 舵机角（+JOINT_OFFSET）发给固件。
        回显铁律：解析固件回显的 clamp 后舵机角 → 转回关节角更新 joint_state；
        请求关节角 vs 实际关节角偏差 > ANGLE_ECHO_TOL → 返回警告（调用方暂停）。

        Returns:
            (True, "ok") / (False, reason)。reason 含 "CLAMP" 时调用方必须暂停。
        """
        n = int(n)
        if not 1 <= n <= 6:
            return False, "ERR: joint out of range"
        # 限位预检拒绝（Panthera-HT 哲学）：规划层直接拒绝越限目标，
        # 早暴露 IK/伺服问题；固件钳位只是最后防线，不该被当常规路径。
        lo, hi = config.JOINT_MIN[n - 1], config.JOINT_MAX[n - 1]
        if not (lo - config.LIMIT_EPS <= angle <= hi + config.LIMIT_EPS):
            msg = ("REJECT: joint %d target %+.1f outside limits [%+.1f, %+.1f]"
                   % (n, angle, lo, hi))
            self.last_warning = msg
            return False, msg
        servo = angle + config.JOINT_OFFSET[n - 1]      # 关节角 → 舵机角
        servo_int = int(round(servo))                    # 量化为整数（固件 %d 解析）
        resp = self._send("M%d %d" % (n, servo_int))
        if resp is None:
            return False, "TIMEOUT: no response from MCU"
        if resp.startswith("ERR"):
            return False, resp
        echo = self._parse_ok_echo(resp)
        if echo is None:
            return False, "BAD_ECHO: %r (firmware protocol mismatch?)" % resp
        idx, servo_echo = echo
        if idx != n - 1:                         # 回显索引错位 = 协议/缓冲污染（P2-3）
            return False, "BAD_ECHO: %r (echo idx %d != requested %d)" % (resp, idx, n)
        actual = servo_echo - config.JOINT_OFFSET[idx]  # 舵机角 → 关节角
        self.joint_state[idx] = actual                   # ← 回显铁律：更新实际关节
        if abs(actual - angle) > config.ANGLE_ECHO_TOL:
            msg = ("CLAMP: joint %d requested %.1f, echoed %.1f "
                   "(limit exceeded, IK drift risk)" % (n, angle, actual))
            self.last_warning = msg
            return False, msg
        return True, "ok"

    def query_state(self):
        """发 S，刷新 joint_state（回显舵机角 → 关节角）。成功 True。"""
        resp = self._send("S")
        if resp is None:
            return False
        m = re.findall(r"S(\d)=([-\d.]+)", resp or "")
        if not m:
            return False
        servos = [0.0] * 6
        for idx_str, val in m:
            servos[int(idx_str) - 1] = float(val)
        self.joint_state = from_servo(servos)
        return True

    def query_ir(self):
        """发 I。返回 (ir1_blocked, ir2_blocked) 或 (None, None) 失败。"""
        resp = self._send("I")
        if resp is None:
            return (None, None)
        m = re.search(r"IR1=(\d)\s+IR2=(\d)", resp or "")
        if not m:
            return (None, None)
        return (m.group(1) == "0", m.group(2) == "0")  # 遮挡=0

    def home(self):
        """H：全回中位 + 清急停。成功 True。"""
        resp = self._send("H")
        if resp is None or resp.startswith("ERR"):
            return False
        self.joint_state = list(config.JOINT_HOME)
        self.estop_active = False
        return True

    def estop(self):
        """E：急停。成功 True。"""
        resp = self._send("E")
        if resp is None:
            return False
        self.estop_active = True
        return True

    def soft_start(self):
        """软启动：逐个关节回中位，间隔 SOFTSTART_INTERVAL，防启动瞬间 15A。"""
        if self.estop_active:
            self.home()
        for n in range(1, 7):
            ok, msg = self.move_joint(n, config.JOINT_HOME[n - 1])
            if not ok:
                return False, msg
            time.sleep(config.SOFTSTART_INTERVAL)
        return True, "ok"

    def close(self):
        if self._ser is not None:
            self._ser.close()
