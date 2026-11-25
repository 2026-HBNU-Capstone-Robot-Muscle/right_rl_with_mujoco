@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_middle_clearance_probe_20261106.py" --execute
pause
