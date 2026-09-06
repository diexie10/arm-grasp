# -*- coding: utf-8 -*-
"""ir_sensor.py — 红外遮挡检测（QT30CM 计数模块，遮挡=0，PB12/PB13）。

去抖（架构书 §4）：3/5 多数判定——连续 5 次查询中 ≥3 次遮挡才算触发，
滤除舵机抖动/机械振动误触发。查询间隔 IR_POLL_MS。
"""

import time
from collections import deque

import config


class IRSensor:
    def __init__(self, serial, dry_run=False, window=config.IR_TRIGGER_COUNT,
                 poll_ms=config.IR_POLL_MS):
        self._serial = serial
        self._dry_run = dry_run
        self._window = window
        self._poll_ms = poll_ms
        self._ir1 = deque(maxlen=5)      # 最近 5 次原始读数
        self._ir2 = deque(maxlen=5)
        self._last_query = 0.0
        self._dry_blocks = 0             # DRY_RUN 模拟：第 3 次查询后"接触"

    def _poll(self):
        """按 IR_POLL_MS 节流查询；返回 (ir1_raw, ir2_raw) 或 (None, None)。"""
        now = time.monotonic()
        if now - self._last_query < self._poll_ms / 1000.0:
            return (None, None)
        self._last_query = now
        b1, b2 = self._serial.query_ir()
        if b1 is not None:
            self._ir1.append(b1)
            self._ir2.append(b2)
        return (b1, b2)

    def _debounced(self, dq):
        """3/5 去抖：窗口填满后才判定，遮挡计数 ≥ 窗口值才算遮挡（P3-6）。"""
        if len(dq) < self._window:
            return False
        return sum(1 for v in dq if v) >= self._window

    def ir1_blocked(self):
        self._poll()
        return self._debounced(self._ir1)

    def ir2_blocked(self):
        self._poll()
        return self._debounced(self._ir2)

    def any_blocked(self):
        self._poll()
        return self._debounced(self._ir1) or self._debounced(self._ir2)

    def wait_blocked(self, timeout_ms=5000):
        """轮询等待任一红外被遮挡。成功 True；超时 False。

        超时下限：必须 ≥ (IR_TRIGGER_COUNT+2) * IR_POLL_MS（=250ms），
        否则去抖窗口无法填满，真机恒返回 False。

        DRY_RUN：模拟接触——第 3 次查询返回 True（否则 DESCEND 永远走不完）。
        """
        if self._dry_run:
            # 每次等待 = 独立去抖窗口（比真实共享历史更保守，误差方向安全）
            self._dry_blocks = 0
            self._dry_blocks += 1
            # 语义等价：dry_run 模拟计数与真实红外去抖确认计数同义（IR_TRIGGER_COUNT=3）
            return self._dry_blocks >= config.IR_TRIGGER_COUNT
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            if self.any_blocked():
                return True
            time.sleep(config.IR_POLL_MS / 1000.0)
        return False
