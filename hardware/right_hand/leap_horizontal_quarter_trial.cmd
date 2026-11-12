@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_horizontal_quarter_20261027.py" --execute
pause
