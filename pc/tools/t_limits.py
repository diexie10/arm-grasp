# -*- coding: utf-8 -*-
"""t_limits.py — 限位核对。

从固件读取舵机域限位（L 命令），与 PC config 逐关节对比。
退出码: 0=全匹配, 1=有不匹配, 2=固件无 L 命令。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial


def main():
    ap = argparse.ArgumentParser(description="限位核对")
    ap.add_argument("--dry", action="store_true", help="不连接硬件（dry_run）")
    ap.add_argument("--port", default=None, help="串口（默认 config.COM_PORT）")
    args = ap.parse_args()

    arm = ArmSerial(port=args.port, dry_run=args.dry)
    mcu = arm.query_limits()

    if mcu is None:
        print("WARNING: 固件无 L 命令或解析失败")
        arm.close()
        return 2

    # 打印对比表
    hdr = "%-6s | %-16s | %-16s | %s" % ("Joint", "MCU", "PC", "Match")
    print(hdr)
    print("-" * len(hdr))

    mismatch = False
    for i in range(6):
        mcu_min = mcu["min"][i]
        mcu_max = mcu["max"][i]
        mcu_home = mcu["home"][i]
        pc_min = int(config.SERVO_MIN[i])
        pc_max = int(config.SERVO_MAX[i])
        pc_home = int(config.HOME_SERVO[i])

        match = (mcu_min == pc_min and mcu_max == pc_max and mcu_home == pc_home)
        if not match:
            mismatch = True

        mark = "OK" if match else "MISMATCH"
        print("S%d     | %3d %3d %3d      | %3d %3d %3d      | %s" % (
            i + 1, mcu_min, mcu_max, mcu_home,
            pc_min, pc_max, pc_home, mark))

    arm.close()
    return 1 if mismatch else 0


if __name__ == "__main__":
    sys.exit(main())
