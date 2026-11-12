@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_book_prepare_empty.py" --execute
set "book_prepare_exit=%errorlevel%"
pause
exit /b %book_prepare_exit%
