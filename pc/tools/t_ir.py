# -*- coding: utf-8 -*-
"""t_ir.py — 红外信号测试。

持续 N 秒轮询 IR 传感器（3/5 去抖），实时打印读数，统计遮挡↔通过跳变次数。
退出码 0（正常结束）。遮挡=0。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial
from ir_sensor import IRSensor


def main():
    ap = argparse.ArgumentParser(description="红外信号测试")
    ap.add_argument("--dry", action="store_true", help="不连接硬件（dry_run）")
    ap.add_argument("--port", default=None, help="串口（默认 config.COM_PORT）")
    ap.add_argument("--seconds", type=float, default=10, help="测试时长秒（默认 10）")
    args = ap.parse_args()

    arm = ArmSerial(port=args.port, dry_run=args.dry)
    ir = IRSensor(serial=arm, dry_run=arm.dry_run)

    print("--- 红外测试 %g 秒（遮挡=0）---" % args.seconds)
    transitions = 0
    prev_ir1 = None
    prev_ir2 = None
    t0 = time.monotonic()

    while time.monotonic() - t0 < args.seconds:
        b1 = ir.ir1_blocked()
        b2 = ir.ir2_blocked()
        any_b = ir.any_blocked()
        elapsed = time.monotonic() - t0

        # 统计跳变
        if prev_ir1 is not None:
            if b1 != prev_ir1 or b2 != prev_ir2:
                transitions += 1
        prev_ir1, prev_ir2 = b1, b2

        print("  t=%5.1fs  IR1=%s  IR2=%s  any=%s" % (
            elapsed, "遮挡" if b1 else "通过",
            "遮挡" if b2 else "通过",
            "遮挡" if any_b else "通过"))

        time.sleep(config.IR_POLL_MS / 1000.0)

    print("--- 测试结束，跳变次数: %d ---" % transitions)
    arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
