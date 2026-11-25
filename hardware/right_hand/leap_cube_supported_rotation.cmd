@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_cube_supported_rotation.py" --execute
set "LEAP_SUPPORTED_ROTATION_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_SUPPORTED_ROTATION_EXIT_CODE%
