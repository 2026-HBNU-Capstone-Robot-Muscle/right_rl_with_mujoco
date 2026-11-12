$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe -B (Join-Path $PSScriptRoot 'sequence_flip_checked.py') --empty-check
exit $LASTEXITCODE
