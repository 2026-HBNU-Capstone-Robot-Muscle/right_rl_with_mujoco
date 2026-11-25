@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B "leap_retained_slow_yaw_20261029.py" --execute
pause
