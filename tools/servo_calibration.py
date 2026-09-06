# -*- coding: utf-8 -*-
"""servo_calibration.py — 逐轴校准：扫描每个舵机的脉宽响应范围。

已知信息：MG996R 克隆舵机的实际响应窗口比 500~2500µs 宽（J1 实测
500~2611µs），且不同个体不同。本脚本逐轴扫描找出每个舵机的真实边界。

用法：python servo_calibration.py [COM口]   默认 COM8
流程：对每个舵机先从 90° 向下扫到 0°，再从 90° 向上扫到 270°，
     用户逐点确认"动了/没动"，最终输出校准表。
"""
import sys
import time
import json

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM8'
BAUD = 115200
SERVO_NAMES = ['J1 底座', 'J2 大臂', 'J3 小臂', 'J4 腕俯仰', 'J5 腕旋转', 'J6 夹爪']
STEP = 3           # 扫描步进 °
SERVO_MAX_ANGLE = 180  # 固件公式中的基准（脉冲换算用）
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500


def pulse_width(angle, trim=0):
    """逻辑角→脉宽 µs（用于显示）。"""
    phys = angle + trim
    return SERVO_MIN_PULSE + phys * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) // SERVO_MAX_ANGLE


def m(ser, ch, angle):
    cmd = f'M{ch+1} {angle}'
    ser.write((cmd + '\n').encode())
    time.sleep(0.05)
    resp = ser.read(64).decode(errors='replace').strip()
    return resp


def ask_user(angle, resp):
    """让用户确认舵机是否动了。"""
    while True:
        ans = input(f'  M1 {angle:>3}° ({pulse_width(angle):>4}µs) → {resp}  '
                     f'动了? [y/n/skip] ').strip().lower()
        if ans in ('y', 'n', 's', ''):
            return ans if ans != '' else 'y'
        print('  请输入 y/n/skip')


def scan_servo(ser, ch):
    """扫描单个舵机的响应范围。"""
    print(f'\n{"="*50}')
    print(f'  舵机 {ch+1}：{SERVO_NAMES[ch]}')
    print(f'{"="*50}')

    # 先归中
    m(ser, ch, 90)
    time.sleep(1.5)
    input('  请确认舵机已到 90° 位，按 Enter 继续...')

    results = {'channel': ch+1, 'name': SERVO_NAMES[ch],
               'lower_limit': None, 'upper_limit': None, 'center': 90}

    # 向下扫描 90 → 0
    print('\n  --- 向下扫描 90° → 0° ---')
    last_moved = 90
    for angle in range(90, -1, -STEP):
        resp = m(ser, ch, angle)
        ans = ask_user(angle, resp)
        if ans == 'n':
            results['lower_limit'] = last_moved
            print(f'  ✓ 下限确认：逻辑 {last_moved}°（脉冲 {pulse_width(last_moved)}µs）')
            break
        elif ans == 's':
            break
        last_moved = angle
    else:
        results['lower_limit'] = 0
        print('  ✓ 一直扫到 0° 都在动')

    # 回到 90
    m(ser, ch, 90)
    time.sleep(1.5)

    # 向上扫描 90 → 270
    print('\n  --- 向上扫描 90° → 270° ---')
    last_moved = 90
    for angle in range(90, 271, STEP):
        resp = m(ser, ch, angle)
        ans = ask_user(angle, resp)
        if ans == 'n':
            results['upper_limit'] = last_moved
            print(f'  ✓ 上限确认：逻辑 {last_moved}°（脉冲 {pulse_width(last_moved)}µs）')
            break
        elif ans == 's':
            break
        last_moved = angle
    else:
        results['upper_limit'] = 270
        print('  ✓ 一直扫到 270° 都在动')

    # 回中
    m(ser, ch, 90)
    time.sleep(1)

    # 让用户标记真中位
    print(f'\n  当前舵机范围：逻辑 [{results["lower_limit"]}°, {results["upper_limit"]}°]')
    print(f'  请手动操作，找到这轴的"真正中位"（横平竖直），然后告诉我角度。')
    center = input('  真中位角度 = ').strip()
    if center.isdigit():
        results['center'] = int(center)

    return results


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f'打开 {PORT} 失败: {e}')
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    print('='*50)
    print('  舵机脉宽校准工具')
    print('  扫描范围：0° ~ 270°（步进 3°）')
    print('  每点确认：y=动了 / n=没动 / skip=跳过')
    print('='*50)

    all_results = []

    # 从用户指定的舵机开始（默认 S2，S1 已校准）
    start = input('\n从哪个舵机开始? (1-6，默认2) ').strip()
    start = int(start) if start.isdigit() and 1 <= int(start) <= 6 else 2

    for ch in range(start-1, 6):
        # 跳过已校准的
        if ch == 0:
            ans = input('\nS1 (J1) 已校准 [7,101,191]，跳过? [Y/n] ').strip().lower()
            if ans != 'n':
                all_results.append({
                    'channel': 1, 'name': SERVO_NAMES[0],
                    'lower_limit': 7, 'upper_limit': 191, 'center': 101
                })
                continue

        result = scan_servo(ser, ch)
        all_results.append(result)

        # 每轴结束显示摘要
        lo = result['lower_limit'] or '?'
        hi = result['upper_limit'] or '?'
        ct = result['center']
        print(f'\n  摘要：{result["name"]} → '
              f'下限={lo}° 中位={ct}° 上限={hi}°')

    ser.close()

    # 输出汇总
    print('\n' + '='*60)
    print('  校准结果汇总')
    print('='*60)
    print(f'  {"舵机":<12} {"下限°":<8} {"中位°":<8} {"上限°":<8} {"脉冲下限µs":<12} {"脉冲上限µs":<12}')
    for r in all_results:
        lo = r['lower_limit'] or 0
        hi = r['upper_limit'] or 270
        ct = r['center']
        print(f'  S{r["channel"]} {r["name"]:<10} {lo:<8} {ct:<8} {hi:<8} '
              f'{pulse_width(lo):<12} {pulse_width(hi):<12}')

    # 保存 JSON
    out_path = 'calibration_results.json'
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2)
    print(f'\n  结果已保存到 {out_path}')
    print('  下一步：根据结果更新 config.py 的 JOINT_MIN/MAX/HOME 和固件 joint_min/max')
    print('  ⚠️ 同步提醒：改 HOME 必须两端同步（固件 home_servo + PC JOINT_HOME/JOINT_OFFSET）')


if __name__ == '__main__':
    main()
