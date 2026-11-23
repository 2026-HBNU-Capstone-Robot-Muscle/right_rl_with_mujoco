@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B leap_grip_to_vertical_20261118.py --execute --from-grip
pause
