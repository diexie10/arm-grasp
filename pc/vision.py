# -*- coding: utf-8 -*-
"""vision.py — YOLO-OBB 木块检测（类名 block）。

关键点（知识库）：
- 预热 1 帧：首次推理冷启动慢（模型加载 + 编译），抓取前先预热
- conf=0.6 / iou=0.45 / max_det=1（单目标，多目标取最高 conf）
- OBB 模型输出在 results[0].obb（xywhr），不是 boxes！boxes 对 OBB 模型为 None
- 角度 theta 为弧度，正方形 90° 对称 → 归一化到 [0, 90)
- 中文路径：YOLO 从路径加载模型由 ultralytics 内部处理；
  若未来换 cv2.dnn，需 np.fromfile 规避 cv2.imread 中文路径问题
"""

import math

import numpy as np

import config


class Vision:
    def __init__(self, model_path=None, conf=None, iou=None):
        from ultralytics import YOLO
        self.conf = conf or config.CONF_THRESH
        self.iou = iou or config.IOU_THRESH
        self.model = YOLO(model_path or config.YOLO_MODEL)
        self._warmup()

    def _warmup(self):
        """预热：空帧跑一次推理，消除首次调用延迟。"""
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, conf=self.conf, verbose=False)

    def detect(self, frame):
        """检测一帧（BGR numpy array）。

        Returns:
            (cx, cy, theta_deg, conf) 目标中心像素坐标 + 角度(度, [0,90)) + 置信度；
            未检出返回 None。
            只接受类别名为 "block" 的目标（P3-5），多类模型不误抓。
        """
        results = self.model.predict(
            frame, conf=self.conf, iou=self.iou,
            max_det=config.MAX_DET, verbose=False)
        if not results or results[0].obb is None or len(results[0].obb) == 0:
            return None
        names = results[0].names
        for obb in results[0].obb:        # max_det=1，但仍按类别过滤
            if names.get(int(obb.cls[0]), "") != "block":
                continue
            x, y, w, h, theta = obb.xywhr[0].tolist()
            conf = float(obb.conf[0])
            theta_deg = math.degrees(theta) % 90.0   # 正方形 90° 对称
            return (x, y, theta_deg, conf)
        return None

    def detect_loop(self, cap, max_tries=30, poll_ms=200):
        """连续取帧直到检出目标（自动抓取主循环用）。

        Returns:
            (cx, cy, theta_deg, conf)；超时返回 None。
        """
        import time
        for _ in range(max_tries):
            ret, frame = cap.read()
            if ret:
                hit = self.detect(frame)
                if hit is not None:
                    return hit
            time.sleep(poll_ms / 1000.0)
        return None


if __name__ == "__main__":
    # 单元自测：加载模型 + 预热 + 空帧（确认无异常即可）
    v = Vision()
    print("Vision ready: conf=%.2f iou=%.2f model=%s" % (
        v.conf, v.iou, config.YOLO_MODEL))
    print("empty frame detect ->", v.detect(np.zeros((480, 640, 3), dtype=np.uint8)))
