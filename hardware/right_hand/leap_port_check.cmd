@echo off
chcp 65001 >nul
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_list_serial_ports.py"
pause
