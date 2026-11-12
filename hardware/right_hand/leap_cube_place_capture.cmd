@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_cube_place_capture.py" --execute
set "leap_place_exit=%errorlevel%"
pause
exit /b %leap_place_exit%
