# -*- coding: utf-8 -*-
"""sweep_test.py — J1 慢速往复扫描：0 → 90 → 180 → 90 → 0，循环 5 次。

用途：验证 trim 后零端是否真正到位（2026-08-25 零端偏差排查）。
用法：python sweep_test.py [COM口]   默认 COM8
注意：运行前断开浏览器 Web Serial 控制台（独占串口）。
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM8'
BAUD = 115200
STEP = 2          # 每步 °
INTERVAL = 0.05   # 每步间隔 s → 40°/s 慢速
CYCLES = 5


def m(ser, angle):
    cmd = f'M1 {angle}'
    ser.write((cmd + '\n').encode())
    time.sleep(0.03)
    resp = ser.read(64).decode(errors='replace').strip()
    print(f'{cmd:>8} -> {resp}')
    return resp


def ramp(ser, frm, to):
    """frm 到 to，步进 STEP，慢速。"""
    if frm < to:
        angles = range(frm, to + 1, STEP)
    else:
        angles = range(frm, to - 1, -STEP)
    for a in angles:
        r = m(ser, a)
        if not r.startswith('OK'):
            print(f'!! 异常应答: {r}')
            return False
        time.sleep(INTERVAL)
    return True


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f'打开 {PORT} 失败: {e}')
        print('如果提示拒绝访问 → 先断开浏览器控制台')
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()
    print(f'=== J1 往复扫描：0→90→180→90→0 ×{CYCLES}，'
          f'{STEP}°/{INTERVAL}s ≈ {STEP / INTERVAL:.0f}°/s ===')

    try:
        for cyc in range(1, CYCLES + 1):
            print(f'--- 第 {cyc}/{CYCLES} 圈 ---')
            ramp(ser, 0, 90)
            ramp(ser, 90, 180)
            time.sleep(0.5)          # 180 端停 0.5s 确认
            ramp(ser, 180, 90)
            ramp(ser, 90, 0)
            time.sleep(1.0)          # 0 端停 1s 确认
        print('=== 扫描完成 ===')
    except KeyboardInterrupt:
        print('\n手动中断')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
