# -*- coding: utf-8 -*-
"""main.py — arm-grasp-pc 主流程（视觉伺服版）。

用法：
    python main.py --dry-run --mode test     # 无硬件全流程自测（联调必开）
    python main.py --mode calib              # 四点标定（旧路线，保留备用）
    python main.py --com COM7 --mode single  # 手动输入 mm 坐标抓取一次（旧路线）
    python main.py --com COM7 --mode auto    # 自动循环：搜索→伺服→抓取

视觉伺服方案（2026-08 用户确认）：
- 摄像头装机械臂上（eye-in-hand），SEARCH 转圈扫描 → ALIGN 伺服收敛
- 全程像素域闭环，不依赖毫米标定（calibration.py 降级备用）
- 正方形木块 90° 对称 → 角度容错 ±45°，J5 = θ − J1

安全：任何失败 → 状态机回 HOME；Ctrl+C 触发软件急停 E。
"""

import argparse
import logging
import sys
import time

import cv2

import config
from arm_serial import ArmSerial
from calibration import Calibration
from ir_sensor import IRSensor
from state_machine import StateMachine
from trajectory import Trajectory
from vision import Vision


def setup_logging():
    logging.basicConfig(
        filename=config.LOG_FILE, level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    logging.getLogger().addHandler(console)


def open_camera():
    cap = cv2.VideoCapture(config.CAMERA_IDX)
    if not cap.isOpened():
        print("!! 摄像头打开失败（idx=%d）。DroidCam 过渡：改 config.CAMERA_IDX 为 URL"
              % config.CAMERA_IDX)
        sys.exit(1)
    return cap


def ensure_calibration(cap):
    """加载已存标定；没有则交互标定。返回 Calibration。

    旧路线（毫米坐标）用；视觉伺服方案不调用本函数。
    """
    cal = Calibration()
    if cal.load():
        print("已加载标定 calib_H.npy")
        return cal
    print("== 首次运行：四点标定 ==")
    if cal.interactive(cap) is None:
        print("!! 标定失败/取消，无法继续")
        sys.exit(1)
    cal.save()
    print("标定已保存 calib_H.npy")
    return cal


def main():
    ap = argparse.ArgumentParser(description="arm-grasp-pc 上位机")
    ap.add_argument("--dry-run", action="store_true",
                    help="DRY_RUN：不发串口，假回显（联调必开）")
    ap.add_argument("--com", default=config.COM_PORT, help="串口号，如 COM7")
    ap.add_argument("--mode", default="auto",
                    choices=["auto", "calib", "single", "test"],
                    help="auto=自动循环 / calib=标定 / single=单次 / test=DRY_RUN自测")
    args = ap.parse_args()

    setup_logging()
    log = logging.getLogger("main")
    log.info("=== arm-grasp-pc start: mode=%s dry_run=%s com=%s ===",
             args.mode, args.dry_run, args.com)

    # --- 模式安全护栏 ---
    if args.mode == "test" and not args.dry_run:
        log.error("--mode test 必须配 --dry-run（测试模式禁止真实动臂，P1-3）")
        return 2

    # --- 硬件层 ---
    serial = ArmSerial(port=args.com, dry_run=args.dry_run)
    traj = Trajectory()
    ir = IRSensor(serial, dry_run=args.dry_run)

    # --- 软启动（auto/single/test 需要；calib 无需动臂）---
    if args.mode != "calib":
        ok, msg = serial.soft_start()
        if not ok:
            log.error("软启动失败: %s", msg)
            return 1
        log.info("软启动完成，关节状态: %s",
                 [round(a, 1) for a in serial.joint_state])

    # --- 模式分发 ---
    if args.mode == "test":
        log.info("== DRY_RUN 全流程自测 ==")
        # 视觉伺服路线：需要 vision + cap
        cap = open_camera()
        vision = Vision()
        sm = StateMachine(serial, traj, ir, vision=vision, cap=cap)
        ok = sm.run_grasp()
        log.info("grasp result: %s", ok)
        cap.release()
        serial.close()
        return 0 if ok else 1

    if args.mode == "calib":
        cap = open_camera()
        cal = ensure_calibration(cap)
        cap.release()
        serial.close()
        return 0

    # --- auto / single：需要视觉 ---
    cap = open_camera()
    vision = Vision()
    sm = StateMachine(serial, traj, ir, vision=vision, cap=cap)

    try:
        if args.mode == "single":
            while True:
                raw = input("目标 mm 坐标 (x y)，q 退出: ").strip()
                if raw.lower() == "q":
                    break
                try:
                    mx, my = map(float, raw.split())
                except ValueError:
                    print("格式错，如: 100 50")
                    continue
                ok = sm.run_grasp(mx, my)
                log.info("single grasp (%.0f, %.0f) -> %s", mx, my, ok)

        else:  # auto —— 视觉伺服全流程
            log.info("== 自动抓取循环（搜索→伺服→抓取）==")
            while True:
                ok = sm.run_grasp()
                if not ok:
                    log.warning("抓取失败（已回 HOME），继续下一轮")
                time.sleep(1.0)
    except KeyboardInterrupt:
        log.warning("Ctrl+C：软件急停 E")
        serial.estop()
    finally:
        cap.release()
        serial.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())