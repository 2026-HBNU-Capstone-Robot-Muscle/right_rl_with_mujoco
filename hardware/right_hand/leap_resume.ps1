$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) {
    throw 'Run setup_leap_runtime.ps1 first.'
}
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe -B (Join-Path $PSScriptRoot 'leap_resume.py')
exit $LASTEXITCODE
