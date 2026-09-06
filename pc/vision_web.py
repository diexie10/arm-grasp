# -*- coding: utf-8 -*-
"""vision_web.py — 网页版 YOLO-OBB 实时检测。

启动后浏览器打开 http://localhost:8080 即可看到摄像头画面 + 检测框。
无需 opencv GUI（解决了 Windows 中文路径 / 高 DPI 等问题）。

用法：
    python vision_web.py                  # 默认 Camera 1, 端口 8080
    python vision_web.py --camera 0       # 指定摄像头
    python vision_web.py --port 9090      # 指定端口
"""

import argparse
import os
import sys
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vision import Vision
import config

# ===================== 全局状态 =====================
vision = None
cap = None
lock = threading.Lock()
latest_frame = None
latest_det = None
frame_count = 0
fps = 0.0
fps_timer = time.time()


def grab_frame():
    """取一帧 + 检测，返回 (jpg_bytes, detection_info)。"""
    global latest_frame, latest_det, frame_count, fps, fps_timer
    ret, frame = cap.read()
    if not ret:
        return None, None

    det = vision.detect_obb(frame)
    vis = frame.copy()

    info_text = "No detection"
    color = (0, 0, 255)
    if det is not None:
        cx, cy = det['cx'], det['cy']
        theta, conf = det['theta_deg'], det['conf']
        corners = det['corners']

        # 画 OBB 四边形（绿色填充半透明 + 白色边框）
        pts = corners.astype(np.int32).reshape((-1, 1, 2))
        overlay = vis.copy()
        cv2.fillPoly(overlay, [pts], (0, 255, 0))
        cv2.addWeighted(overlay, 0.15, vis, 0.85, 0, vis)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)

        # 画角点（黄色小圆）
        for i, pt in enumerate(corners.astype(np.int32)):
            cv2.circle(vis, tuple(pt), 5, (0, 255, 255), -1)
            cv2.putText(vis, str(i), tuple(pt + [5, -5]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 画中心 + 十字线
        icx, icy = int(cx), int(cy)
        cv2.circle(vis, (icx, icy), 6, (0, 0, 255), 2)
        cv2.line(vis, (icx - 20, icy), (icx + 20, icy), (255, 255, 255), 1)
        cv2.line(vis, (icx, icy - 20), (icx, icy + 20), (255, 255, 255), 1)

        info_text = f"conf={conf:.2f} theta={theta:.1f} center=({cx:.0f},{cy:.0f})"
        color = (0, 255, 0)

    # FPS 计算
    frame_count += 1
    now = time.time()
    if now - fps_timer >= 1.0:
        fps = frame_count / (now - fps_timer)
        frame_count = 0
        fps_timer = now

    # 信息栏
    h, w = vis.shape[:2]
    cv2.rectangle(vis, (0, 0), (w, 50), (0, 0, 0), -1)
    cv2.putText(vis, f"FPS: {fps:.1f}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(vis, info_text, (160, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    _, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpg.tobytes(), det


# ===================== MJPEG 流 =====================
class StreamHandler(BaseHTTPRequestHandler):
    """MJPEG 推流 + 简单控制页。"""

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self._serve_html()
        elif parsed.path == "/stream":
            self._serve_stream()
        elif parsed.path == "/snap":
            self._serve_snapshot()
        else:
            self.send_error(404)

    def _serve_html(self):
        """主页：MJPEG 实时流 + 检测信息。"""
        html = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>arm-grasp Vision Test</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #1a1a2e; color: #eee; font-family: 'Consolas', monospace; }
  .header { padding: 12px 20px; background: #16213e; border-bottom: 2px solid #0f3460;
             display: flex; align-items: center; gap: 20px; }
  .header h1 { font-size: 18px; color: #e94560; }
  .header .status { font-size: 13px; color: #aaa; }
  .main { display: flex; justify-content: center; padding: 20px; }
  .stream-box { position: relative; border: 2px solid #0f3460; border-radius: 8px; overflow: hidden; }
  .stream-box img { display: block; max-width: 80vw; max-height: 75vh; }
  .overlay { position: absolute; top: 10px; right: 10px; background: rgba(0,0,0,0.7);
             padding: 8px 14px; border-radius: 6px; font-size: 13px; line-height: 1.6; }
  .overlay .label { color: #aaa; }
  .overlay .val { color: #0f0; font-weight: bold; }
  .overlay .no-det { color: #e94560; }
  .controls { text-align: center; padding: 15px; }
  .controls button { background: #0f3460; color: #eee; border: 1px solid #e94560;
    padding: 8px 20px; margin: 0 5px; border-radius: 4px; cursor: pointer; font-size: 13px; }
  .controls button:hover { background: #e94560; }
  #info-panel { max-width: 600px; margin: 10px auto; background: #16213e; padding: 12px;
                border-radius: 6px; font-size: 12px; line-height: 1.8; display: none; }
</style>
</head>
<body>
<div class="header">
  <h1>arm-grasp Vision Test</h1>
  <span class="status" id="status">Connecting...</span>
</div>
<div class="main">
  <div class="stream-box">
    <img id="stream" src="/stream" />
    <div class="overlay" id="overlay">
      <div><span class="label">Status: </span><span class="val" id="det-status">Waiting...</span></div>
    </div>
  </div>
</div>
<div class="controls">
  <button onclick="snap()">Snapshot</button>
  <button onclick="toggleInfo()">Detection Info</button>
</div>
<div id="info-panel"></div>
<script>
  const img = document.getElementById('stream');
  const status = document.getElementById('status');
  let infoPanel = document.getElementById('info-panel');
  let showInfo = false;

  img.onload = function() {
    status.textContent = 'Live';
    status.style.color = '#0f0';
  };
  img.onerror = function() {
    status.textContent = 'Disconnected';
    status.style.color = '#e94560';
  };

  function snap() {
    fetch('/snap').then(r => r.blob()).then(b => {
      const url = URL.createObjectURL(b);
      const a = document.createElement('a');
      a.href = url; a.download = 'snapshot_' + Date.now() + '.jpg';
      a.click(); URL.revokeObjectURL(url);
    });
  }

  function toggleInfo() {
    showInfo = !showInfo;
    infoPanel.style.display = showInfo ? 'block' : 'none';
  }
</script>
</body>
</html>"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def _serve_stream(self):
        """MJPEG 推流。"""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        try:
            while True:
                jpg_data, det = grab_frame()
                if jpg_data is None:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg_data)))
                self.end_headers()
                self.wfile.write(jpg_data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _serve_snapshot(self):
        """单帧快照。"""
        jpg_data, det = grab_frame()
        if jpg_data is None:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg_data)))
        self.end_headers()
        self.wfile.write(jpg_data)

    def log_message(self, format, *args):
        pass  # 静默日志


def main():
    parser = argparse.ArgumentParser(description="arm-grasp Vision Web Test")
    parser.add_argument("--camera", type=int, default=1, help="Camera index (default: 1)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    args = parser.parse_args()

    global vision, cap

    print(f"Loading model from {config.YOLO_MODEL} ...")
    vision = Vision()
    print(f"Vision ready: conf={vision.conf} iou={vision.iou}")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Camera {args.camera} not found")
        sys.exit(1)
    print(f"Camera {args.camera} opened")

    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    server.daemon_threads = True
    print(f"\n=== Open in browser ===")
    print(f"  http://localhost:{args.port}")
    print(f"  http://127.0.0.1:{args.port}")
    print(f"\nPress Ctrl+C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        cap.release()
        server.server_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
