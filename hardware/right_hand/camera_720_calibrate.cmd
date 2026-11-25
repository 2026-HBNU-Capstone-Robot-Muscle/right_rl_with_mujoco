@echo off
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0camera_720_calibrate.ps1"
echo.
pause
