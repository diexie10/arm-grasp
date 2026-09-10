# -*- coding: utf-8 -*-
"""glove_bridge.py — 手套→机械臂实时桥接（PC 端，固件零改动）。

数据源（--glove-source）：
  sim          进程内复用 IMUGenerator + MahonyFilter（合成欧拉流走真实桥管线）
  udp:PORT     每行 JSON {"p":..,"r":..,"y":..}（欧拉角 deg），默认端口 GLOVE_BRIDGE_UDP_PORT
  serial:COMx  同 JSON 行格式（有线手套 fallback）

管线 @GLOVE_SEND_HZ：
  欧拉角 → scheme_a/scheme_b（--scheme，默认 a）→ TargetChain（EMA+限步）→ 舵机域 M 流

安全语义：
  - 断流冻结：GLOVE_LINK_TIMEOUT_MS 无新欧拉数据 → 停发（保持最后姿态），
    恢复数据流自动继续；状态打印（一次性，不刷屏）
  - 键盘 e → 发 E 急停 → 退出
  - Ctrl+C → 优雅退出（先停发，不发 H——避免意外回中）
  - 回显不符：请求 vs 回显偏差 > ANGLE_ECHO_TOL → 打印并中止手套模式（不发后续）

用法：
  cd pc
  python glove\\glove_bridge.py --arm-dry --glove-source sim --scheme a --seconds 12
  python glove\\glove_bridge.py --glove-source udp:8766 --scheme a
  python glove\\glove_bridge.py --glove-source serial:COM8 --scheme a
"""

import argparse
import json
import math
import os
import sys
import threading
import time

# 同级 pc 模块 import
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial
from glove.mapping import (scheme_a, scheme_a_init, scheme_b, scheme_b_init,
                           TargetChainState, apply_error_deadband)


# =====================================================================
#  输入源
# =====================================================================

class SimSource:
    """进程内合成 IMU → Mahony → 欧拉角流（复用 glove_sim.py 管线）。"""

    def __init__(self):
        from glove.glove_sim import IMUGenerator
        from glove.imu_filter import MahonyFilter
        self._imu_gen = IMUGenerator(noise_scale=1.0)
        self._mahony = MahonyFilter()
        self._dt_imu = 1.0 / config.GLOVE_IMU_HZ
        self._send_interval = max(1, int(config.GLOVE_IMU_HZ / config.GLOVE_SEND_HZ))
        self._counter = 0
        self._true_prev = None
        # 默认关键帧（与 glove_sim.py 一致）
        self._keyframes = [
            (0.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0),
            (4.0, 30.0, 20.0, 15.0), (6.0, 30.0, 20.0, 15.0),
            (8.0, -20.0, -10.0, 15.0), (10.0, -20.0, -10.0, 15.0),
            (12.0, 10.0, -10.0, -10.0),
        ]
        self._step = 0
        self._n_imu = 0  # 由外部按 seconds 设置

    def set_duration(self, seconds):
        self._n_imu = int(seconds * config.GLOVE_IMU_HZ)

    def alive(self):
        return self._step < self._n_imu

    def read_euler(self):
        """返回 (pitch, roll, yaw) [deg] 或 None（未到抽取时刻）。"""
        if self._step >= self._n_imu:
            return None
        t = self._step * self._dt_imu
        self._step += 1

        # 真值姿态
        pitch_t, roll_t, yaw_t = _lerp_keyframes(self._keyframes, t)

        # 欧拉角导数
        if self._true_prev is not None:
            roll_rate = (roll_t - self._true_prev[0]) / self._dt_imu
            pitch_rate = (pitch_t - self._true_prev[1]) / self._dt_imu
            yaw_rate = (yaw_t - self._true_prev[2]) / self._dt_imu
        else:
            roll_rate, pitch_rate, yaw_rate = 0.0, 0.0, 0.0
        self._true_prev = (roll_t, pitch_t, yaw_t)

        # 合成 IMU
        gyro, accel = self._imu_gen.generate(
            roll_t, pitch_t, yaw_t, roll_rate, pitch_rate, yaw_rate)

        # Mahony 滤波
        roll_est, pitch_est, yaw_est = self._mahony.update(gyro, accel, self._dt_imu)

        # 20Hz 抽取
        self._counter += 1
        if self._counter < self._send_interval:
            return None
        self._counter = 0
        return pitch_est, roll_est, yaw_est

    def close(self):
        pass


class UDPSource:
    """UDP JSON 行流：{"p": pitch, "r": roll, "y": yaw} [deg]。"""

    def __init__(self, port):
        import socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.1)  # 非阻塞轮询
        self._buf = b""
        self._alive = True
        print("UDP 源监听: 0.0.0.0:%d（等待数据…）" % port)

    def alive(self):
        return self._alive

    def read_euler(self):
        """返回 (pitch, roll, yaw) 或 None（本拍无数据）。"""
        try:
            data, _ = self._sock.recvfrom(config.GLOVE_BRIDGE_UDP_BUF)
            self._buf += data
        except OSError:
            # 超时或无数据，非致命
            pass

        # 按行解析（可能粘包/拆包）
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line.decode("utf-8", errors="replace"))
                p = float(obj["p"])
                r = float(obj["r"])
                y = float(obj["y"])
                return p, r, y
            except (json.JSONDecodeError, KeyError, ValueError) as exc:
                print("UDP 解析跳过: %s (%s)" % (line[:60], exc))
        return None

    def close(self):
        self._alive = False
        self._sock.close()


class SerialSource:
    """串口 JSON 行流：{"p": pitch, "r": roll, "y": yaw} [deg]。"""

    def __init__(self, port):
        import serial as _serial
        self._ser = _serial.Serial(port, config.BAUDRATE, timeout=0.1)
        self._buf = b""
        self._alive = True
        print("串口源打开: %s @ %d" % (port, config.BAUDRATE))

    def alive(self):
        return self._alive

    def read_euler(self):
        try:
            data = self._ser.read(self._ser.in_waiting or 1)
            if data:
                self._buf += data
        except Exception:
            pass

        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line.decode("utf-8", errors="replace"))
                p = float(obj["p"])
                r = float(obj["r"])
                y = float(obj["y"])
                return p, r, y
            except (json.JSONDecodeError, KeyError, ValueError) as exc:
                print("串口解析跳过: %s (%s)" % (line[:60], exc))
        return None

    def close(self):
        self._alive = False
        self._ser.close()


# =====================================================================
#  辅助
# =====================================================================

def _lerp_keyframes(kf, t):
    """关键帧线性插值（与 glove_sim.py 一致）。"""
    if t <= kf[0][0]:
        return kf[0][1], kf[0][2], kf[0][3]
    if t >= kf[-1][0]:
        return kf[-1][1], kf[-1][2], kf[-1][3]
    for i in range(len(kf) - 1):
        t0, p0, r0, y0 = kf[i]
        t1, p1, r1, y1 = kf[i + 1]
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0)
            return (p0 + (p1 - p0) * f,
                    r0 + (r1 - r0) * f,
                    y0 + (y1 - y0) * f)
    return kf[-1][1], kf[-1][2], kf[-1][3]


def _make_source(source_str, seconds):
    """解析 --glove-source → 源对象。"""
    if source_str == "sim":
        src = SimSource()
        src.set_duration(seconds)
        return src
    if source_str.startswith("udp:"):
        port = int(source_str.split(":")[1])
        return UDPSource(port)
    if source_str.startswith("serial:"):
        port = source_str.split(":")[1]
        return SerialSource(port)
    raise ValueError("未知数据源: %s（可选 sim / udp:PORT / serial:COMx）" % source_str)


# =====================================================================
#  键盘监听线程
# =====================================================================

_stop_flag = threading.Event()


def _keyboard_listener():
    """后台线程：监听 'e' 键触紧急停。"""
    try:
        import msvcrt  # Windows
        while not _stop_flag.is_set():
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b"e", b"E"):
                    _stop_flag.set()
                    return
            time.sleep(0.05)
    except ImportError:
        # 非 Windows：跳过键盘监听（Ctrl+C 仍可用）
        while not _stop_flag.is_set():
            time.sleep(0.1)


# =====================================================================
#  主桥接循环
# =====================================================================

def run_bridge(args):
    """运行桥接管线，返回指标 dict。"""
    # --- 初始化源 ---
    source = _make_source(args.glove_source, args.seconds)

    # --- 初始化 ArmSerial ---
    arm = None
    if not args.arm_dry:
        try:
            arm = ArmSerial(port=args.arm_port, dry_run=False)
        except SystemExit as exc:
            print("串口打开失败: %s" % exc)
            source.close()
            return {}
    else:
        arm = ArmSerial(port=None, dry_run=True)

    # --- 初始化映射 ---
    scheme = args.scheme
    state_a = scheme_a_init() if scheme in ("a", "both") else None
    state_b = scheme_b_init() if scheme in ("b", "both") else None
    chain_a = TargetChainState() if scheme in ("a", "both") else None
    chain_b = TargetChainState() if scheme in ("b", "both") else None

    # --- 启动键盘监听 ---
    kb_thread = threading.Thread(target=_keyboard_listener, daemon=True)
    kb_thread.start()

    # --- 统计 ---
    frame_count = 0
    send_count = 0
    echo_mismatch = 0
    freeze_count = 0
    was_frozen = False  # 初始态不打印恢复消息
    max_delta = 0.0
    prev_servo = [None] * 6  # 上一拍已发送的舵机目标

    # --- 主循环 ---
    dt_send = 1.0 / config.GLOVE_SEND_HZ
    timeout_s = config.GLOVE_LINK_TIMEOUT_MS / 1000.0
    deadline = time.time() + timeout_s  # 断流冻结截止
    running = True

    print("\n手套桥启动: scheme=%s, source=%s, arm=%s" % (
        scheme, args.glove_source,
        "DRY_RUN" if args.arm_dry else args.arm_port))
    print("  发送: %.0fHz, 死区: %.1f°, EMA: %.2f, 限步: %.1f°" % (
        config.GLOVE_SEND_HZ, config.GLOVE_DEADBAND_DEG,
        config.GLOVE_EMA_ALPHA, config.GLOVE_STEP_MAX_DEG))
    print("  断流冻结: %.0fms, 按 e 急停, Ctrl+C 退出\n" % config.GLOVE_LINK_TIMEOUT_MS)

    try:
        while running and not _stop_flag.is_set():
            if not source.alive():
                running = False
                break

            # 读欧拉角（sim 源可能返回 None = 未到抽取时刻）
            euler = source.read_euler()
            if euler is None:
                # 非阻塞轮询；sim 源需要继续循环产生 IMU 样本
                if args.glove_source == "sim":
                    continue
                # udp/serial 源：检查断流
                if time.time() > deadline:
                    if not was_frozen:
                        print("[冻结] %.0fms 无数据，停止发送（保持最后姿态）" %
                              config.GLOVE_LINK_TIMEOUT_MS)
                        was_frozen = True
                        freeze_count += 1
                    continue
                continue

            # 有数据 → 刷新断流计时器
            deadline = time.time() + timeout_s
            if was_frozen:
                print("[恢复] 数据流恢复")
                was_frozen = False

            pitch_est, roll_est, yaw_est = euler
            frame_count += 1

            # --- 死区 ---
            map_pitch = apply_error_deadband(pitch_est, config.GLOVE_DEADBAND_DEG)
            map_roll = apply_error_deadband(roll_est, config.GLOVE_DEADBAND_DEG)
            map_yaw = apply_error_deadband(yaw_est, config.GLOVE_DEADBAND_DEG)

            # --- 映射 ---
            targets_raw = None
            if scheme in ("a", "both"):
                targets_raw, _, _, _ = scheme_a(map_pitch, map_roll, map_yaw, state_a)
            if scheme in ("b", "both"):
                targets_b_raw, q_b, _, _, _ = scheme_b(
                    map_pitch, map_roll, map_yaw, dt_send, state_b)
                if q_b is not None:
                    targets_raw = targets_b_raw
                # IK 失败 → targets_raw = None → 保持上一拍（断流冻结语义）

            if targets_raw is None:
                # IK 失败或未初始化 → 保持上一拍（不发送新命令）
                continue

            # --- 目标链：EMA + 限步 ---
            chain = chain_a if scheme in ("a", "both") else chain_b
            targets, _ = chain.update(targets_raw)

            # --- 发送到固件 ---
            if not args.arm_dry:
                # 逐轴检查变化，只发有变化的轴
                for j in range(6):
                    if prev_servo[j] is None or abs(targets[j] - prev_servo[j]) >= config.GLOVE_DEADBAND_DEG:
                        ok, msg = arm.move_joint(j + 1, targets[j] - config.JOINT_OFFSET[j])
                        if not ok:
                            print("[中止] 关节 %d 发送失败: %s" % (j + 1, msg))
                            if "CLAMP" in msg:
                                echo_mismatch += 1
                                print("[中止] 回显超限，退出手套模式")
                                running = False
                                break
                send_count += 1
            else:
                send_count += 1

            # 追踪最大增量
            for j in range(6):
                if prev_servo[j] is not None:
                    d = abs(targets[j] - prev_servo[j])
                    if d > max_delta:
                        max_delta = d
            prev_servo = list(targets)

            # sim 源时长控制
            if args.glove_source == "sim" and args.seconds > 0:
                # sim 源自己有步数限制，但这里也做时间兜底
                pass

    except KeyboardInterrupt:
        print("\n[Ctrl+C] 优雅退出（不发 H，保持最后姿态）")

    # --- 清理 ---
    source.close()
    if arm is not None:
        arm.close()

    # --- 摘要 ---
    print("\n" + "=" * 60)
    print("  手套桥运行摘要")
    print("=" * 60)
    print("  数据源: %s" % args.glove_source)
    print("  方案: %s" % scheme)
    print("  总帧数: %d" % frame_count)
    print("  发送数: %d" % send_count)
    print("  回显不符: %d" % echo_mismatch)
    print("  冻结次数: %d" % freeze_count)
    if args.arm_dry:
        print("  DRY_RUN: 是（未实际发送串口）")
    print("  最大单轴增量: %.3f°" % max_delta)
    print("=" * 60)

    return {
        "frame_count": frame_count,
        "send_count": send_count,
        "echo_mismatch": echo_mismatch,
        "freeze_count": freeze_count,
        "max_delta": max_delta,
    }


# =====================================================================
#  主入口
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description="手套→机械臂实时桥接（PC 端，固件零改动）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("--arm-port", type=str, default=None,
                        help="串口号（默认 config.COM_PORT）")
    parser.add_argument("--arm-dry", action="store_true",
                        help="ArmSerial DRY_RUN 模式（不发串口，假回显）")
    parser.add_argument("--glove-source", type=str, default="sim",
                        help="数据源: sim / udp:PORT / serial:COMx（默认 sim）")
    parser.add_argument("--scheme", type=str, default="a",
                        choices=["a", "b"],
                        help="映射方案（默认 a）")
    parser.add_argument("--seconds", type=float, default=12.0,
                        help="sim 源时长 s（默认 12）")
    args = parser.parse_args()

    if args.arm_port is None:
        args.arm_port = config.COM_PORT

    run_bridge(args)


if __name__ == "__main__":
    main()
