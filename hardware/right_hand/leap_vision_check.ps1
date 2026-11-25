param([switch]$Target14)
$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
$env:PYTHONIOENCODING = 'utf-8'
if ($Target14) {
    & $pythonExe -B (Join-Path $PSScriptRoot 'leap_vision_check.py') --cube-ids 1 4
} else {
    & $pythonExe -B (Join-Path $PSScriptRoot 'leap_vision_check.py')
}
exit $LASTEXITCODE
