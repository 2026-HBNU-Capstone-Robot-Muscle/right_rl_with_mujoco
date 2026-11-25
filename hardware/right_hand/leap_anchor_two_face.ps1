$ErrorActionPreference = 'Stop'
$pythonExe = Join-Path $PSScriptRoot '.runtime\python\python.exe'
$env:PYTHONIOENCODING = 'utf-8'
$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($ports -notcontains 'COM7') {
    Write-Host "STOP before camera/motion: motor USB adapter COM7 is absent. Available ports: $($ports -join ', ')"
    Write-Host 'Reconnect the motor USB adapter/cable, then rerun once COM7 appears. No motor command sent.'
    exit 2
}
& $pythonExe -B (Join-Path $PSScriptRoot 'sequence_flip_checked.py') --record-vision --capture-anchor-placement --anchor-two-face
exit $LASTEXITCODE
