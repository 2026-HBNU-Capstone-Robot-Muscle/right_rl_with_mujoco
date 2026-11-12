@echo off
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_single_forward_trial_20261030.py" --execute
pause
