@echo off
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_forward_hold_observe_20261104.py" --execute
pause
