$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.cv4');$(Join-Path $project '.deps')"
$env:PYTHONIOENCODING = 'utf-8'
& $pythonExe (Join-Path $PSScriptRoot 'pose_stability.py') --hd --profile
exit $LASTEXITCODE
