@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0cube_pose_readonly_capture.py" --execute
set "LEAP_CUBE_POSE_READONLY_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_CUBE_POSE_READONLY_EXIT_CODE%
