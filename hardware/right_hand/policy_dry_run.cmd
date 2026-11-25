@echo off
chcp 65001 >nul
"%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" "%~dp0policy_dry_run.py"
pause
