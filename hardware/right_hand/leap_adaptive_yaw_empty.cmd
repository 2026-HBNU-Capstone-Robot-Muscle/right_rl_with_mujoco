@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_adaptive_yaw_empty_20261029.py" --execute
pause
