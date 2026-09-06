@echo off
chcp 65001 >nul
title 视觉检测工具 - vision_web
cd /d C:\Users\diexie\Desktop\arm-grasp\pc

echo ========================================
echo   视觉检测工具 一键启动 (vision_web)
echo   页面: http://localhost:8080
echo   前端: 自动打开浏览器
echo   Ctrl+C 退出
echo ========================================
echo.

REM 后台延迟打开浏览器（等服务起来）
start "" cmd /c "timeout /t 15 /nobreak >nul && start http://localhost:8080"

REM 前台启动后端（日志实时显示，方便看摄像头状态）
C:\Users\diexie\.venvs\yolo\Scripts\python.exe vision_web.py --camera 1 --port 8080

echo.
echo 服务已退出。
pause
