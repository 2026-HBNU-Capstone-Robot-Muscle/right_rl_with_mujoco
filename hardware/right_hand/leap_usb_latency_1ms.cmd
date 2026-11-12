@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0usb_latency_admin_workflow_20261027.py" --request-admin
pause
