@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -B "%~dp0bus_readonly_diagnose.py" --execute
set "LEAP_BUS_DIAG_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_BUS_DIAG_EXIT_CODE%
