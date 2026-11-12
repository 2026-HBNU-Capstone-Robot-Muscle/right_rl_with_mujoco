@echo off
chcp 65001 >nul
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_two_face_preview.py"
pause
