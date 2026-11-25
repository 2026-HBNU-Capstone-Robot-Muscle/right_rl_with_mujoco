@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0cube_visibility_capture_20261026.py" --capture
set "LEAP_CUBE_VISIBILITY_EXIT=%errorlevel%"
pause
exit /b %LEAP_CUBE_VISIBILITY_EXIT%
