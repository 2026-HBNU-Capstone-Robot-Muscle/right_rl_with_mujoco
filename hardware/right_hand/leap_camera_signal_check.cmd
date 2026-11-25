@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_camera_signal_check.py"
set "LEAP_CAMERA_CHECK_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_CAMERA_CHECK_EXIT_CODE%
