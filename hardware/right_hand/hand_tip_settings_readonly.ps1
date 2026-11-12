$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.hw_runtime_10');$(Join-Path $project '.deps')"
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe (Join-Path $PSScriptRoot 'hand_tip_settings_readonly.py')
exit $LASTEXITCODE
