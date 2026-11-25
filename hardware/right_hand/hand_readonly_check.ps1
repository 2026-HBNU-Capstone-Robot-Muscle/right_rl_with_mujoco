$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe -B (Join-Path $PSScriptRoot 'hand_readonly_check.py') --port COM7
exit $LASTEXITCODE
