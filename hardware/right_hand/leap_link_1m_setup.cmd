@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0hand_link_commission_1m_20261028.py" --apply
pause
