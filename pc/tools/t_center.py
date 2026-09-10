# -*- coding: utf-8 -*-
"""t_center.py — 舵机归中测试。

逐关节（或全部）移动到 JOINT_HOME，然后 query_state 核对每个关节
是否在 HOME ±1° 内。退出码 0=全通过，非0=有失败。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial


def main():
    ap = argparse.ArgumentParser(description="舵机归中测试")
    ap.add_argument("--dry", action="store_true", help="不连接硬件（dry_run）")
    ap.add_argument("--port", default=None, help="串口（默认 config.COM_PORT）")
    ap.add_argument("--joint", type=int, default=0,
                    help="关节号 1-6（默认 0=全部）")
    args = ap.parse_args()

    fail = False
    joints = [args.joint] if 1 <= args.joint <= 6 else list(range(1, 7))

    arm = ArmSerial(port=args.port, dry_run=args.dry)

    # 逐关节归中
    for n in joints:
        target = config.JOINT_HOME[n - 1]
        ok, msg = arm.move_joint(n, target)
        status = "PASS" if ok else "FAIL"
        print("move_joint(%d, %.1f) -> [%s] %s" % (n, target, status, msg))
        if not ok:
            fail = True
        time.sleep(config.SOFTSTART_INTERVAL)

    # query_state 核对
    arm.query_state()
    print("\n--- 关节状态核对 ---")
    for n in joints:
        actual = arm.joint_state[n - 1]
        expected = config.JOINT_HOME[n - 1]
        diff = abs(actual - expected)
        ok = diff <= 1.0
        status = "PASS" if ok else "FAIL"
        print("J%d: actual=%.1f expected=%.1f diff=%.1f [%s]" % (
            n, actual, expected, diff, status))
        if not ok:
            fail = True

    arm.close()
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
