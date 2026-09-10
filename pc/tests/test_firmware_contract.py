# -*- coding: utf-8 -*-
"""test_firmware_contract.py — 跨边界契约测试（守住 P0 类 bug）。

背景：2026-09-10 修了一个 P0 —— PC `config.py` 把固件的【舵机域】限位当【关节域】用，
J2/J4 因此多算一次 offset（差 90°），真机会误报 CLAMP、HOME 错位。根因是**跨端手工同步常量**。

本测试**解析固件真实源码**（Core/Src/servo.c、cmd.c），断言与 PC `config` 一致：
  - 主机实际使用的舵机域限位 `kinematics.servo_limits(i)` 必须落在固件 clamp 内（可更紧，不可更松）
  - `config.HOME_SERVO` 与 `to_servo(config.JOINT_HOME)` 必须等于固件 `home_servo`
若再次出现域混乱（如 J2 限位算成 93..273），本测试立即 FAIL。

运行：`python pc/tests/test_firmware_contract.py`（也兼容 pytest）。
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "pc"))

import config          # noqa: E402
import kinematics      # noqa: E402

EPS = 1e-6


def _read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def _parse_u16_array(src, name):
    """从 C 源码解析 `name[6] = {a, b, ...}` 为 int 列表。"""
    m = re.search(r"%s\s*\[\s*6\s*\]\s*=\s*\{([^}]*)\}" % re.escape(name), src)
    if not m:
        raise AssertionError("固件数组未找到: %s" % name)
    return [int(x.strip().rstrip("Uu")) for x in m.group(1).split(",") if x.strip()]


def _firmware_limits():
    src = _read(os.path.join(ROOT, "Core", "Src", "servo.c"))
    return _parse_u16_array(src, "joint_min"), _parse_u16_array(src, "joint_max")


def _firmware_home():
    src = _read(os.path.join(ROOT, "Core", "Src", "cmd.c"))
    return _parse_u16_array(src, "home_servo")


def test_host_servo_limits_within_firmware():
    """主机舵机域限位必须 ⊆ 固件 clamp（更紧可以，更松=域混乱/会误报 CLAMP）。"""
    fw_min, fw_max = _firmware_limits()
    for i in range(6):
        lo, hi = kinematics.servo_limits(i)
        assert lo >= fw_min[i] - EPS, (
            "J%d 主机 servo_min=%.1f < 固件 %d（域混乱？offset 重复计入？）"
            % (i + 1, lo, fw_min[i]))
        assert hi <= fw_max[i] + EPS, (
            "J%d 主机 servo_max=%.1f > 固件 %d（域混乱？offset 重复计入？）"
            % (i + 1, hi, fw_max[i]))


def test_home_matches_firmware():
    """HOME：config.HOME_SERVO 与 to_servo(JOINT_HOME) 都必须等于固件 home_servo。"""
    fw = _firmware_home()
    host = kinematics.to_servo(config.JOINT_HOME)
    for i in range(6):
        assert abs(config.HOME_SERVO[i] - fw[i]) <= EPS, (
            "J%d HOME_SERVO=%.1f != 固件 home_servo %d" % (i + 1, config.HOME_SERVO[i], fw[i]))
        assert abs(host[i] - fw[i]) <= EPS, (
            "J%d to_servo(JOINT_HOME)=%.1f != 固件 home_servo %d（关节/舵机域不一致）"
            % (i + 1, host[i], fw[i]))


def test_internal_domain_derivation():
    """内部一致性：JOINT_MIN/MAX 必须 = SERVO_MIN/MAX − JOINT_OFFSET（派生正确）。"""
    for i in range(6):
        assert abs((config.JOINT_MIN[i] + config.JOINT_OFFSET[i]) - config.SERVO_MIN[i]) <= EPS
        assert abs((config.JOINT_MAX[i] + config.JOINT_OFFSET[i]) - config.SERVO_MAX[i]) <= EPS


def _run():
    fns = [test_host_servo_limits_within_firmware,
           test_home_matches_firmware,
           test_internal_domain_derivation]
    fails = 0
    for fn in fns:
        try:
            fn()
            print("PASS  %s" % fn.__name__)
        except AssertionError as e:
            fails += 1
            print("FAIL  %s\n      %s" % (fn.__name__, e))
    print("\n%s（%d/%d 通过）" % ("ALL PASS" if not fails else "FAILED",
                                 len(fns) - fails, len(fns)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(_run())
