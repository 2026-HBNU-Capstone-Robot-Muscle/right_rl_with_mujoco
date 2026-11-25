$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.cv4');$(Join-Path $project '.deps')"
& $pythonExe (Join-Path $PSScriptRoot 'cube_face_rot.py')
exit $LASTEXITCODE
