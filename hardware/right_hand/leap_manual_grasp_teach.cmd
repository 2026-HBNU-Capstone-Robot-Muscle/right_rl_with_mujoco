@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_manual_grasp_teach_20261106.py" --execute
pause
