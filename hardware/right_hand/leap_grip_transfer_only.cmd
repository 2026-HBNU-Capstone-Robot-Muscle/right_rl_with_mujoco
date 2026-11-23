@echo off
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B leap_retaught_transfer_rotation_20261118.py --execute --transfer-only
pause
