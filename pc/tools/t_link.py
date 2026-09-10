# -*- coding: utf-8 -*-
"""t_link.py — 信号通路测试。

验证 PC ↔ STM32 全链路：握手、状态查询、红外查询、运动查询、校验和拒绝。
无运动。退出码 0=全通过，非0=有失败。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config
from arm_serial import ArmSerial


def main():
    ap = argparse.ArgumentParser(description="信号通路测试")
    ap.add_argument("--dry", action="store_true", help="不连接硬件（dry_run）")
    ap.add_argument("--port", default=None, help="串口（默认 config.COM_PORT）")
    args = ap.parse_args()

    fail = False

    # --- 1. 构造 ArmSerial（内含 L 握手）---
    print("[1/4] 握手（ArmSerial 构造含 L 命令）...")
    try:
        arm = ArmSerial(port=args.port, dry_run=args.dry)
    except SystemExit as e:
        print("[FAIL] 无法构造 ArmSerial: %s" % e)
        return 1
    print("[PASS] ArmSerial 构造成功 (dry=%s)" % args.dry)
    print("  L 握手已在 __init__ 中执行，verify_limits() 通过")

    # --- 2. query_state ---
    print("[2/4] query_state (S 命令)...")
    ok = arm.query_state()
    if ok:
        print("[PASS] query_state=True, joint_state=%s" % [round(a, 1) for a in arm.joint_state])
    else:
        print("[FAIL] query_state=False")
        fail = True

    # --- 3. query_ir ---
    print("[3/4] query_ir (I 命令)...")
    ir = arm.query_ir()
    if ir != (None, None):
        print("[PASS] query_ir=%s (遮挡=0)" % (ir,))
    else:
        print("[FAIL] query_ir=(None, None)")
        fail = True

    # --- 4. Q 命令 ---
    print("[4/4] Q 命令 (运动查询)...")
    resp = arm._send("Q")
    if resp in ("DONE", "BUSY"):
        print("[PASS] Q -> %s" % resp)
    else:
        print("[FAIL] Q -> %r (期望 DONE/BUSY)" % resp)
        fail = True

    # --- 5. 校验和拒绝（仅真机）---
    if not args.dry:
        print("[5] 坏校验和拒绝测试...")
        try:
            bad = b"M1 90*00\r\n"
            arm._ser.write(bad)
            import time
            time.sleep(config.RESP_TIMEOUT)
            line = arm._ser.readline().decode(errors="replace").strip()
            if "ERR CKS" in line:
                print("[PASS] 坏校验和被拒绝: %s" % line)
            else:
                print("[FAIL] 坏校验和未被拒绝: %s" % line)
                fail = True
        except Exception as e:
            print("[FAIL] 校验和测试异常: %s" % e)
            fail = True
    else:
        print("[5] 校验和拒绝测试: --dry 跳过（无法写裸字节到虚拟串口）")

    arm.close()
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
