@echo off
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0camera_720_mjpg_timing.ps1"
echo.
pause
