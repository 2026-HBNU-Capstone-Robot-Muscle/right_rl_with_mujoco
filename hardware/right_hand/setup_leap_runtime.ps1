$ErrorActionPreference = 'Stop'
$runtime = Join-Path $PSScriptRoot '.runtime\python'
$pythonExe = Join-Path $runtime 'python.exe'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
if (-not (Test-Path -LiteralPath $pythonExe)) {
    $archive = Join-Path $runtime 'python-3.12.10-embed-amd64.zip'
    Invoke-WebRequest -UseBasicParsing 'https://www.python.org/ftp/python/3.12.10/python-3.12.10-embed-amd64.zip' -OutFile $archive
    Expand-Archive -LiteralPath $archive -DestinationPath $runtime -Force
}
# The embedded interpreter ignores PYTHONPATH. Include this project's scripts explicitly.
@('python312.zip', '.', 'Lib\site-packages', '..\..', 'import site') |
    Set-Content -LiteralPath (Join-Path $runtime 'python312._pth') -Encoding ASCII
& $pythonExe -B -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('pip') else 1)"
if ($LASTEXITCODE -ne 0) {
    $bootstrap = Join-Path $runtime 'get-pip.py'
    Invoke-WebRequest -UseBasicParsing 'https://bootstrap.pypa.io/get-pip.py' -OutFile $bootstrap
    & $pythonExe -B $bootstrap --disable-pip-version-check --no-warn-script-location
    if ($LASTEXITCODE -ne 0) { throw 'pip bootstrap failed' }
}
# Dependencies are explicitly listed; avoid pygrabber also installing a second
# opencv-python distribution over the existing opencv-contrib-python namespace.
& $pythonExe -B -m pip install --no-deps --disable-pip-version-check --no-warn-script-location -r (Join-Path $PSScriptRoot 'requirements-leap.txt')
if ($LASTEXITCODE -ne 0) { throw 'LEAP dependency installation failed' }
Write-Host "LEAP runtime ready: $pythonExe"
