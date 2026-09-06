# -*- coding: utf-8 -*-
"""vision.py — YOLO-OBB 木块检测（类名 block）。

关键点（知识库）：
- 预热 1 帧：首次推理冷启动慢（模型加载 + 编译），抓取前先预热
- conf=0.6 / iou=0.45 / max_det=1（单目标，多目标取最高 conf）
- OBB 模型输出在 results[0].obb（xywhr），不是 boxes！boxes 对 OBB 模型为 None
- 角度 theta 为弧度，正方形 90° 对称 → 归一化到 [0, 90)
- 中文路径：YOLO 从路径加载模型由 ultralytics 内部处理；
  若未来换 cv2.dnn，需 np.fromfile 规避 cv2.imread 中文路径问题

3D 位姿（solvePnP）：
- OBB 的 4 个角点（xyxyxyxy）就是透视投影后的四边形
- 用 solvePnP + 相机内参 + 物体尺寸 → 求出物体在相机坐标系的 3D 位姿
- 需要先做棋盘格标定得到相机内参（运行 python vision.py --calibrate）
"""

import math
import os

import cv2
import numpy as np

import config


class Vision:
    def __init__(self, model_path=None, conf=None, iou=None,
                 camera_matrix=None, dist_coeffs=None):
        from ultralytics import YOLO
        self.conf = conf or config.CONF_THRESH
        self.iou = iou or config.IOU_THRESH
        self.model = YOLO(model_path or config.YOLO_MODEL)
        # 相机内参：优先用参数传入，否则用 config 里的值
        self.K = camera_matrix if camera_matrix is not None else config.CAMERA_MATRIX
        self.D = dist_coeffs if dist_coeffs is not None else config.DIST_COEFFS
        # 物体3D模型点（物体坐标系，Z=0 平面，单位：米）
        hw = config.OBJECT_WIDTH_M / 2.0
        hh = config.OBJECT_HEIGHT_M / 2.0
        self.obj_points = np.array([
            [-hw, -hh, 0.0],
            [ hw, -hh, 0.0],
            [ hw,  hh, 0.0],
            [-hw,  hh, 0.0],
        ], dtype=np.float64)
        self._warmup()

    def _warmup(self):
        """预热：空帧跑一次推理，消除首次调用延迟。"""
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, conf=self.conf, verbose=False)

    def _run_predict(self, frame):
        """单次推理，返回第一个 'block' OBB 的 raw 数据。"""
        results = self.model.predict(
            frame, conf=self.conf, iou=self.iou,
            max_det=config.MAX_DET, verbose=False)
        if not results or results[0].obb is None or len(results[0].obb) == 0:
            return None
        names = results[0].names
        for obb in results[0].obb:
            if names.get(int(obb.cls[0]), "") != "block":
                continue
            return obb
        return None

    def detect(self, frame):
        """检测一帧（BGR numpy array）。

        Returns:
            (cx, cy, theta_deg, conf) 目标中心像素坐标 + 角度(度, [0,90)) + 置信度；
            未检出返回 None。
            只接受类别名为 "block" 的目标（P3-5），多类模型不误抓。
        """
        obb = self._run_predict(frame)
        if obb is None:
            return None
        x, y, w, h, theta = obb.xywhr[0].tolist()
        conf = float(obb.conf[0])
        theta_deg = math.degrees(theta) % 90.0   # 正方形 90° 对称
        return (x, y, theta_deg, conf)

    def detect_area_ratio(self, frame):
        """检测目标面积占比（OBB w*h / 画面面积）。

        Returns:
            float 面积比；未检出返回 0.0。
        """
        obb = self._run_predict(frame)
        if obb is None:
            return 0.0
        _, _, w, h, _ = obb.xywhr[0].tolist()
        return (w * h) / (frame.shape[0] * frame.shape[1])

    def detect_full(self, frame):
        """单次推理，返回完整检测结果 dict。

        Returns:
            dict {
                'cx': float, 'cy': float,
                'theta_deg': float,
                'conf': float,
                'w': float, 'h': float,
                'area_ratio': float,
                'corners': np.ndarray (4,2),
            }；未检出返回 None。
        """
        obb = self._run_predict(frame)
        if obb is None:
            return None
        x, y, w, h, theta = obb.xywhr[0].tolist()
        conf = float(obb.conf[0])
        theta_deg = math.degrees(theta) % 90.0
        area_ratio = (w * h) / (frame.shape[0] * frame.shape[1])
        corners = obb.xyxyxyxy[0].cpu().numpy().reshape(4, 2)
        return {
            'cx': x, 'cy': y,
            'theta_deg': theta_deg,
            'conf': conf,
            'w': w, 'h': h,
            'area_ratio': area_ratio,
            'corners': corners,
        }

    def detect_obb(self, frame):
        """检测一帧，返回 OBB 四角点（用于画框）。

        Returns:
            dict {
                'cx': float, 'cy': float,        # 中心像素
                'theta_deg': float,               # 角度
                'conf': float,                    # 置信度
                'corners': np.ndarray,            # 4 个角点 (4,2) 像素坐标
            }；未检出返回 None。
        """
        obb = self._run_predict(frame)
        if obb is None:
            return None
        x, y, w, h, theta = obb.xywhr[0].tolist()
        conf = float(obb.conf[0])
        theta_deg = math.degrees(theta) % 90.0
        corners = obb.xyxyxyxy[0].cpu().numpy().reshape(4, 2)
        return {
            'cx': x, 'cy': y,
            'theta_deg': theta_deg,
            'conf': conf,
            'corners': corners,
        }

    def detect_3d(self, frame):
        """检测 + solvePnP 求 3D 位姿（eye-in-hand）。

        OBB 的 4 个角点就是透视投影后的四边形，直接喂给 solvePnP。
        solvePnP 用相机内参反投影，输出物体在相机坐标系的 6DoF 位姿。

        Returns:
            dict {
                'x': float,  # 物体在相机坐标系 X（米），右正
                'y': float,  # 物体在相机坐标系 Y（米），下正
                'z': float,  # 物体在相机坐标系 Z（米），前正（深度）
                'rvec': np.ndarray,  # 旋转向量 (3,1)
                'tvec': np.ndarray,  # 平移向量 (3,1)
                'corners': np.ndarray,  # 2D 角点 (4,2) 像素坐标
                'theta_deg': float,  # OBB 角度（度）
                'conf': float,  # 置信度
            }
            未检出或 solvePnP 失败返回 None。
        """
        obb = self._run_predict(frame)
        if obb is None:
            return None
        conf = float(obb.conf[0])
        theta_deg = math.degrees(float(obb.xywhr[0, 4])) % 90.0
        # 拿 OBB 的 4 个角点（透视四边形）
        corners = obb.xyxyxyxy[0].cpu().numpy().reshape(4, 2)
        # solvePnP：3D 模型点 ↔ 2D 像素点
        img_pts = corners.astype(np.float64).reshape(4, 1, 2)
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            return None
        return {
            'x': float(tvec[0][0]),
            'y': float(tvec[1][0]),
            'z': float(tvec[2][0]),
            'rvec': rvec,
            'tvec': tvec,
            'corners': corners,
            'theta_deg': theta_deg,
            'conf': conf,
        }



# ===================== 棋盘格标定辅助 =====================

def calibrate_camera(image_dir, pattern_size=(7, 6), square_size_m=0.025):
    """从一组棋盘格照片标定相机内参。

    Args:
        image_dir: 存放棋盘格照片的文件夹路径
        pattern_size: 棋盘格内部角点数 (列, 行)
        square_size_m: 每格实际边长（米）

    Returns:
        camera_matrix, dist_coeffs, reproj_error
    """
    # 准备世界坐标（Z=0 平面）
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_m

    obj_points = []  # 3D 世界点
    img_points = []  # 2D 像素点

    # 找所有图片
    exts = ('.jpg', '.jpeg', '.png', '.bmp')
    files = sorted([f for f in os.listdir(image_dir)
                    if f.lower().endswith(exts)])
    if not files:
        raise FileNotFoundError("image_dir 中没有图片: %s" % image_dir)

    print("找到 %d 张图片，开始检测棋盘格..." % len(files))
    img_size = None

    for fname in files:
        path = os.path.join(image_dir, fname)
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]  # (w, h)

        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            print("  %s: 未检测到棋盘格，跳过" % fname)
            continue

        # 亚像素精化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners2)
        print("  %s: 检测到 %d 个角点" % (fname, len(corners2)))

    if len(obj_points) < 3:
        raise ValueError("有效图片不足（需要至少3张），只有 %d 张" % len(obj_points))

    print("\n开始标定（%d 张有效图片）..." % len(obj_points))
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)

    # 计算重投影误差
    total_err = 0
    for i in range(len(obj_points)):
        img_pts2, _ = cv2.projectPoints(
            obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        err = cv2.norm(img_points[i], img_pts2, cv2.NORM_L2) / len(img_pts2)
        total_err += err
    mean_err = total_err / len(obj_points)

    print("\n标定完成:")
    print("  重投影误差: %.4f px (%s)" % (
        mean_err, "好" if mean_err < 0.5 else "一般" if mean_err < 1.0 else "差"))
    print("  相机内参矩阵:")
    print("    %.2f  0  %.2f" % (mtx[0, 0], mtx[0, 2]))
    print("    0  %.2f  %.2f" % (mtx[1, 1], mtx[1, 2]))
    print("    0    0    1")
    print("  畸变系数:", dist.flatten())

    # 保存到文件
    out_path = os.path.join(image_dir, "camera_calib.npz")
    np.savez(out_path, camera_matrix=mtx, dist_coeffs=dist,
             reproj_error=mean_err)
    print("\n已保存到: %s" % out_path)
    print("请将以下值填入 config.py:")
    print("  CAMERA_MATRIX = np.array([")
    print("      [%.2f, 0.0, %.2f]," % (mtx[0, 0], mtx[0, 2]))
    print("      [0.0, %.2f, %.2f]," % (mtx[1, 1], mtx[1, 2]))
    print("      [0.0, 0.0, 1.0]], dtype=np.float64)")
    print("  DIST_COEFFS = np.array([%.6f, %.6f, %.6f, %.6f, %.6f])" % tuple(dist.flatten()))

    return mtx, dist, mean_err


def load_calibration(npz_path):
    """加载标定结果。

    Returns:
        camera_matrix, dist_coeffs
    """
    data = np.load(npz_path)
    return data['camera_matrix'], data['dist_coeffs']


if __name__ == "__main__":
    import sys

    # 棋盘格标定模式：python vision.py --calibrate <图片文件夹>
    if len(sys.argv) >= 3 and sys.argv[1] == "--calibrate":
        image_dir = sys.argv[2]
        pattern = (7, 6)
        if len(sys.argv) >= 5:
            pattern = (int(sys.argv[3]), int(sys.argv[4]))
        print("=== 棋盘格标定 ===")
        print("图片目录: %s" % image_dir)
        print("棋盘格角点: %dx%d" % pattern)
        calibrate_camera(image_dir, pattern_size=pattern)
        sys.exit(0)

    # 加载标定文件（如果存在）
    calib_path = os.path.join(os.path.dirname(config.YOLO_MODEL),
                              "camera_calib.npz")
    if os.path.exists(calib_path):
        mtx, dist = load_calibration(calib_path)
        print("已加载标定: %s" % calib_path)
    else:
        mtx, dist = config.CAMERA_MATRIX, config.DIST_COEFFS
        print("使用默认内参（未标定，精度差）")

    # 单元自测：加载模型 + 预热 + 空帧
    v = Vision(camera_matrix=mtx, dist_coeffs=dist)
    print("Vision ready: conf=%.2f iou=%.2f model=%s" % (
        v.conf, v.iou, config.YOLO_MODEL))
    print("empty frame detect ->", v.detect(np.zeros((480, 640, 3), dtype=np.uint8)))

    # 3D 检测测试（如果有摄像头）
    if "--test3d" in sys.argv:
        print("\n=== 3D 检测测试 ===")
        cap = cv2.VideoCapture(config.CAMERA_IDX)
        if not cap.isOpened():
            print("无法打开摄像头")
            sys.exit(1)
        print("按 q 退出")
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            det = v.detect_3d(frame)
            vis = frame.copy()
            if det is not None:
                # 画四边形
                pts = det['corners'].astype(np.int32)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
                # 画坐标轴
                cv2.drawFrameAxes(vis, v.K, v.D, det['rvec'], det['tvec'], 0.05)
                # 标注3D坐标
                cv2.putText(vis, "X=%.3fm Y=%.3fm Z=%.3fm" % (
                    det['x'], det['y'], det['z']),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(vis, "conf=%.2f theta=%.1f" % (
                    det['conf'], det['theta_deg']),
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis, "No detection", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("3D Detection Test", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
