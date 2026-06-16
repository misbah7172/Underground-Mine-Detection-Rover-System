Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

$Python = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
	$Python = "python"
}

& $Python -c "import platformio" 2>$null
if ($LASTEXITCODE -ne 0) {
	Write-Host "PlatformIO Core is not installed in this Python environment. Installing now..."
	& $Python -m pip install --upgrade platformio
	if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

& $Python -m platformio run -t upload --upload-port COM7
exit $LASTEXITCODE
