@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B leap_rotation_speed_resume_20261125.py --axis vertical --execute
pause
