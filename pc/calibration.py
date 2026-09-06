# -*- coding: utf-8 -*-
"""calibration.py — 四点透视标定（像素坐标 ↔ 桌面毫米坐标）。

原理（架构书 §4）：桌面放置 4 个已知毫米坐标的标定点，
cv2.getPerspectiveTransform 求单应矩阵 H，px2mm / mm2px 互转。

使用：cv2.imshow 显示画面 → 点击 4 个标定点 → 输入对应毫米坐标
（以 J1 轴心为原点、Z 向上右手系；标定点取桌面可及区域四角）。
标定误差验证：4 点重投影误差 < 5mm（UNIMPLEMENTED P1 验收标准）。
"""

import cv2
import numpy as np

import config

_WINDOW = "calibration"


class Calibration:
    def __init__(self):
        self._H = None          # 3x3 单应：像素 → 毫米
        self._H_inv = None

    @property
    def ready(self):
        return self._H is not None

    def calibrate(self, px_pts, mm_pts):
        """由 4 点像素/毫米对应求单应。返回 (True, 重投影误差px)。"""
        assert len(px_pts) == len(mm_pts) == 4
        px = np.array(px_pts, dtype=np.float32)
        mm = np.array(mm_pts, dtype=np.float32)
        self._H, _ = cv2.findHomography(px, mm)
        if self._H is None:                       # 退化（三点共线）
            return False, float("inf")
        self._H_inv = np.linalg.inv(self._H)

        # 重投影误差（像素域：H_inv 把 mm 点映回像素域与原始像素比较），
        # 不是毫米域！阈值 5.0 即 5px（毫米域待真机标定后实现）
        px_back = cv2.perspectiveTransform(
            mm.reshape(-1, 1, 2), self._H_inv).reshape(-1, 2)
        err = float(np.mean(np.linalg.norm(px_back - px, axis=1)))
        ok = err < 5.0   # px
        return ok, err

    def px2mm(self, x, y):
        """像素 → 桌面毫米 (mm_x, mm_y)。未标定则抛 ValueError。"""
        if not self.ready:
            raise ValueError("calibration not ready")
        pt = self._H @ np.array([x, y, 1.0])
        return float(pt[0] / pt[2]), float(pt[1] / pt[2])

    def mm2px(self, mx, my):
        """桌面毫米 → 像素。"""
        if not self.ready:
            raise ValueError("calibration not ready")
        pt = self._H_inv @ np.array([mx, my, 1.0])
        return float(pt[0] / pt[2]), float(pt[1] / pt[2])

    def save(self, path="calib_H.npy"):
        """保存单应矩阵（标定一次，之后直接加载）。"""
        if self.ready:
            np.save(path, self._H)
            return True
        return False

    def load(self, path="calib_H.npy"):
        """加载单应矩阵。成功 True。"""
        try:
            self._H = np.load(path)
            self._H_inv = np.linalg.inv(self._H)
            return True
        except (IOError, ValueError, np.linalg.LinAlgError):
            self._H = None
            self._H_inv = None
            return False

    def interactive(self, cap):
        """交互标定：显示画面，点击 4 点，输入毫米坐标。

        Args:
            cap: 打开的 cv2.VideoCapture。

        Returns:
            Calibration 自身（ready=True）；Esc 取消返回 None。
        """
        clicks = []
        mm_pairs = []

        def on_mouse(event, x, y, *_):
            if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
                clicks.append((x, y))
                print("标定点 %d 像素=%s —— 请输入毫米坐标 (mm_x mm_y)："
                      % (len(clicks), (x, y)))

        cv2.namedWindow(_WINDOW)
        cv2.setMouseCallback(_WINDOW, on_mouse)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            for (x, y) in clicks:
                cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)
                cv2.putText(frame, str(clicks.index((x, y)) + 1),
                            (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)
            cv2.imshow(_WINDOW, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:                      # Esc 取消
                cv2.destroyWindow(_WINDOW)
                return None
            if len(clicks) == 4:
                break

        # 收集毫米坐标
        for i, (x, y) in enumerate(clicks):
            while True:
                raw = input("标定点 %d (%d,%d) 毫米坐标 (mm_x mm_y, "
                            "以 J1 轴心为原点): " % (i + 1, x, y)).strip()
                try:
                    mx, my = map(float, raw.split())
                    mm_pairs.append((mx, my))
                    break
                except ValueError:
                    print("格式错，重新输入，如: 100 50")

        cv2.destroyWindow(_WINDOW)
        ok, err = self.calibrate(clicks, mm_pairs)
        if not ok:
            print("!! 标定重投影误差 %.1f px > 5px，请重标（检查点序/共线）" % err)
            return None
        print("标定完成，重投影误差 %.2f px（像素域，毫米域待真机标定后实现）" % err)
        return self


if __name__ == "__main__":
    # 单元自测：已知单应逆推 4 点 → 标定 → 验证互转闭环
    H_true = np.array([[1.0, 0.02, 320.0],
                       [0.01, 1.2, 240.0],
                       [0.0, 0.0, 1.0]])
    mm4 = [(0, 0), (200, 0), (200, 150), (0, 150)]
    px4 = []
    for m in mm4:
        p = H_true @ np.array([m[0], m[1], 1.0])
        px4.append((p[0] / p[2], p[1] / p[2]))
    cal = Calibration()
    ok, err = cal.calibrate(px4, mm4)
    print("calibrate ok=%s err=%.3f mm" % (ok, err))
    for m in mm4:
        back = cal.mm2px(*m)
        print("mm2px(%s) -> %s, px2mm -> %s" % (
            m, (round(back[0], 1), round(back[1], 1)),
            (round(cal.px2mm(*back)[0], 2), round(cal.px2mm(*back)[1], 2))))
