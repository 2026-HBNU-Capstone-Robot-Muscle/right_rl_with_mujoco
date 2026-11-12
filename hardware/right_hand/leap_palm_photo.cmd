@echo off
chcp 65001 >nul
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_palm_photo.py"
pause
