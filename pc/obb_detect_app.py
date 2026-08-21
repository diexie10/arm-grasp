# -*- coding: utf-8 -*-
"""
OBB 检测小工具 - Flask 后端
============================
拖图检测: 浏览器上传图片 -> OBB 推理 -> 返回旋转框 + 角度

启动:
    python app.py
    # 浏览器打开 http://localhost:5000

依赖:
    flask + ultralytics 8.4.120+ (OBB 模型)
模型:
    默认加载 arm-grasp\pc\models\best.pt, 可通过 MODEL 修改
"""
import os
import math
import base64
import time
import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template, Response
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE_DIR, "models", "best.pt")
CONF = 0.25
MAX_SIDE = 1280  # 大图缩放上限(推理前)
CAM_INDEX = 1    # USB 摄像头索引(0=内置, 1=外接 USB 1080p)
CAM_W, CAM_H = 1280, 720  # 摄像头采集分辨率

app = Flask(__name__)
model = None


def imread_unicode(data: bytes) -> np.ndarray:
    """从字节流解码图片(绕开 cv2 中文路径问题)"""
    buf = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return img


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/detect", methods=["POST"])
def detect():
    if "image" not in request.files:
        return jsonify({"error": "未收到图片文件"}), 400
    file = request.files["image"]
    data = file.read()
    img = imread_unicode(data)
    if img is None:
        return jsonify({"error": "图片解码失败"}), 400

    orig_h, orig_w = img.shape[:2]
    scale = 1.0
    # 大图缩放(仅推理用), 返回坐标按比例还原到原图
    if max(orig_h, orig_w) > MAX_SIDE:
        scale = MAX_SIDE / max(orig_h, orig_w)
        img = cv2.resize(img, (int(orig_w * scale), int(orig_h * scale)))

    results = model.predict(img, conf=CONF, verbose=False)
    r = results[0]

    detections = []
    obb = getattr(r, "obb", None)
    if obb is not None and len(obb) > 0:
        for b in obb:
            x, y, w, h, theta = b.xywhr[0].tolist()
            conf = float(b.conf[0].item())
            cls = int(b.cls[0].item())
            name = r.names[cls]
            # 还原到原图坐标
            x /= scale; y /= scale; w /= scale; h /= scale
            detections.append({
                "class": name,
                "conf": round(conf, 3),
                "cx": round(x, 1),
                "cy": round(y, 1),
                "w": round(w, 1),
                "h": round(h, 1),
                "theta_rad": round(theta, 3),
                "theta_deg": round(math.degrees(theta), 1),
            })

    # 标注图(base64) - ultralytics 自带 OBB 旋转框绘制
    annotated = r.plot()
    ok, buf = cv2.imencode(".jpg", annotated,
                           [cv2.IMWRITE_JPEG_QUALITY, 92])
    img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8") if ok else ""

    return jsonify({
        "orig_size": [orig_w, orig_h],
        "detections": detections,
        "annotated_b64": img_b64,
        "hint": "J5 = theta - J1 (机械臂爪子对齐)" if detections else None,
    })


def gen_frames():
    """MJPEG 摄像头流: 逐帧检测 + 画旋转框 + 角度标注"""
    cap = None
    try:
        cap = cv2.VideoCapture(CAM_INDEX)
        if not cap.isOpened():
            print(f"[stream] 错误: 无法打开摄像头 {CAM_INDEX}", flush=True)
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        print(f"[stream] 摄像头 {CAM_INDEX} 已打开", flush=True)
        fps_interval = 1.0 / 10.0  # 目标 ~10fps(推理+编码开销)
        frame_count = 0
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[stream] 摄像头读帧失败", flush=True)
                break
            results = model.predict(frame, conf=CONF, verbose=False)
            annotated = results[0].plot()
            ok, buf = cv2.imencode(".jpg", annotated,
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   buf.tobytes() + b"\r\n")
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"[stream] 已输出 {frame_count} 帧", flush=True)
            dt = time.time() - t0
            if dt < fps_interval:
                time.sleep(fps_interval - dt)
    finally:
        if cap is not None:
            cap.release()
            print("[stream] 摄像头已释放", flush=True)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    if not os.path.isfile(MODEL):
        print(f"错误: 模型不存在: {MODEL}")
        print("请确认 arm-grasp\\pc\\models\\best.pt 存在")
        raise SystemExit(1)
    print(f"加载模型: {MODEL}")
    model = YOLO(MODEL)
    print(f"模型 task = {model.task}")
    print("启动服务: http://localhost:5000  (Ctrl+C 退出)")
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=False)
