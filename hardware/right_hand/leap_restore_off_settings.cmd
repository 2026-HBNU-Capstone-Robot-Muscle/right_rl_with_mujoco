@echo off
chcp 65001 >nul
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0restore_known_off_settings_20261105.py" --restore
pause
