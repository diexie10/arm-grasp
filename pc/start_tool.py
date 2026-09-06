# -*- coding: utf-8 -*-
"""
视觉检测工具 - 一键启动脚本
===========================
启动 vision_web.py（MJPEG 网页检测）并自动打开浏览器。双击运行本脚本即可。

用法:
    python start_tool.py
"""
import os
import sys
import time
import threading
import webbrowser
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(BASE_DIR, "vision_web.py")
PYTHON = sys.executable
URL = "http://localhost:8080"
ARGS = ["--camera", "1", "--port", "8080"]   # eye-in-hand USB 摄像头


def open_browser_later():
    """延迟 12 秒打开浏览器（等模型加载完）"""
    time.sleep(12)
    print(f"[startup] 打开浏览器: {URL}")
    webbrowser.open(URL)


def main():
    if not os.path.isfile(APP):
        print(f"错误: 找不到 {APP}")
        return 1

    print("=" * 48)
    print("  视觉检测工具 一键启动 (vision_web)")
    print(f"  页面: {URL}")
    print("  前端: 自动打开浏览器")
    print("  Ctrl+C 退出")
    print("=" * 48)

    # 后台线程: 延迟打开浏览器
    threading.Thread(target=open_browser_later, daemon=True).start()

    # 前台启动检测服务 (日志直接显示)
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        proc = subprocess.run([PYTHON, APP] + ARGS, cwd=BASE_DIR, env=env)
        return proc.returncode
    except KeyboardInterrupt:
        print("\n[startup] 已退出")
        return 0


if __name__ == "__main__":
    sys.exit(main())
