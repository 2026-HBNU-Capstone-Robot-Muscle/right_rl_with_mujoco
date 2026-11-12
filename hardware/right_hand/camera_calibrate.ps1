$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.cv4');$(Join-Path $project '.deps')"
$script = Join-Path $PSScriptRoot 'camera_calibrate.py'
& $pythonExe $script
exit $LASTEXITCODE
