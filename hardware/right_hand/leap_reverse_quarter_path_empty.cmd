@echo off
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0leap_reverse_quarter_path_empty_20261025.py" --execute
pause
