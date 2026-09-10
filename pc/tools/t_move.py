# -*- coding: utf-8 -*-
"""t_move.py — 舵机运动操作测试（真实轨迹路径）。

通过 G 命令发六轴目标，轮询 Q 等 DONE，对比运动时间估算与实测。
退出码 0=全通过，非0=有失败/超时。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial
from servo_controller import estimate_move_seconds


def main():
    ap = argparse.ArgumentParser(description="舵机运动操作测试")
    ap.add_argument("--dry", action="store_true", help="不连接硬件（dry_run）")
    ap.add_argument("--port", default=None, help="串口（默认 config.COM_PORT）")
    ap.add_argument("--joint", type=int, default=1, help="关节号 1-6（默认 1）")
    ap.add_argument("--angles", default="90,120,90,60,90",
                    help="逗号分隔的目标角度列表（关节域°）")
    args = ap.parse_args()

    fail = False
    targets = [float(x.strip()) for x in args.angles.split(",")]
    n = args.joint
    if not 1 <= n <= 6:
        print("[FAIL] 关节号 %d 超出范围 1-6" % n)
        return 1

    arm = ArmSerial(port=args.port, dry_run=args.dry)
    # 先归中确保已知起点
    arm.soft_start()

    print("--- J%d 运动测试 ---" % n)
    print("%-6s  %-10s  %-8s  %-8s  %-10s  %-12s" % (
        "Seq", "Target", "Predict", "Actual(s)", "Status", "Msg"))

    for i, target in enumerate(targets):
        lo, hi = config.JOINT_MIN[n - 1], config.JOINT_MAX[n - 1]
        if target < lo or target > hi:
            print("%-6d  %-10.1f  (skip)   (skip)    WARN      超限 [%.1f, %.1f]" % (
                i + 1, target, lo, hi))
            continue

        prev_q = list(arm.joint_state)
        est = estimate_move_seconds(prev_q, [target if j == n - 1
                                              else prev_q[j] for j in range(6)])

        q_new = list(arm.joint_state)
        q_new[n - 1] = target
        t0 = time.monotonic()
        ok, msg = arm.move_waypoint(q_new)
        elapsed = time.monotonic() - t0

        if ok:
            status = "OK"
        elif "CLAMP" in msg:
            status = "CLAMP"
        elif "TIMEOUT" in msg:
            status = "TIMEOUT"
        else:
            status = "REJECT"

        if status != "OK":
            fail = True

        print("%-6d  %-10.1f  %-8.2f  %-8.2f  %-10s  %-12s" % (
            i + 1, target, est, elapsed, status, msg))

    arm.close()
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
