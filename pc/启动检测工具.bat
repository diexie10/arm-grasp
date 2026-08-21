@echo off
chcp 65001 >nul
title OBB 检测工具 - 后端 + 前端
cd /d C:\Users\diexie\Desktop\arm-grasp\pc

echo ========================================
echo   OBB 检测工具 一键启动
echo   后端: Flask (localhost:5000)
echo   前端: 自动打开浏览器
echo   Ctrl+C 退出
echo ========================================
echo.

REM 后台延迟打开浏览器（等服务起来）
start "" cmd /c "timeout /t 15 /nobreak >nul && start http://localhost:5000"

REM 前台启动后端（日志实时显示，方便看摄像头状态）
C:\Users\diexie\.venvs\yolo\Scripts\python.exe obb_detect_app.py

echo.
echo 服务已退出。
pause
