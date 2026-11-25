@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_grasp_approach_empty.py" --execute
set "leap_grasp_exit=%errorlevel%"
pause
exit /b %leap_grasp_exit%
