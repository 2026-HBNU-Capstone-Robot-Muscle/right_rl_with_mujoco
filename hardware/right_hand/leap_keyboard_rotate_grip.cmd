@echo off
chcp 65001 >nul
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B leap_keyboard_rotate_grip_20261125.py --execute
pause
