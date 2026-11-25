@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_supported_yaw_empty.py" --execute
set "LEAP_SUPPORTED_EMPTY_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_SUPPORTED_EMPTY_EXIT_CODE%
