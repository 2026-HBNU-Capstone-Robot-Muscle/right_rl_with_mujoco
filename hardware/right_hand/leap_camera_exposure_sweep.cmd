@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0camera_exposure_sweep.py" --execute
set "LEAP_CAMERA_EXPOSURE_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_CAMERA_EXPOSURE_EXIT_CODE%
