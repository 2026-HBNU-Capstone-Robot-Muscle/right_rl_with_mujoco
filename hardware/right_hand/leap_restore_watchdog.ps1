$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe -B (Join-Path $PSScriptRoot 'leap_restore_watchdog.py')
exit $LASTEXITCODE
