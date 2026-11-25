@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B stationary_comm_test_20261124.py --execute
pause
