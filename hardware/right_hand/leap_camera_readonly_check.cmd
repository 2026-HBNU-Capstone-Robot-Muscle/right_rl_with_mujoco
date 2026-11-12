@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_camera_readonly_check.py" --execute
set "LEAP_READONLY_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_READONLY_EXIT_CODE%
