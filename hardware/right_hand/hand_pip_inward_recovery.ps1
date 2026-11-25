param([Parameter(Mandatory=$true)][ValidateSet(7,11)][int]$MotorId)
$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.hw_runtime_10');$(Join-Path $project '.deps')"
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe (Join-Path $PSScriptRoot 'hand_pip_inward_recovery.py') --motor $MotorId
exit $LASTEXITCODE
