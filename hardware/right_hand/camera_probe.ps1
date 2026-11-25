$pythonExe = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\work\Leap_Test')).Path
$env:PYTHONPATH = "$(Join-Path $project '.cv4');$(Join-Path $project '.deps')"
$probe = Join-Path $PSScriptRoot 'camera_probe.py'
$result = Join-Path $PSScriptRoot 'camera_probe_result.txt'
& $pythonExe $probe 2>&1 | Tee-Object -FilePath $result
Write-Host "Result saved to $result"
exit $LASTEXITCODE
