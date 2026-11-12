@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B "cycle_test_readonly_20261111.py" --read --confirm-space
pause
