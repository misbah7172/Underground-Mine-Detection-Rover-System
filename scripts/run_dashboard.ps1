Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

$Python = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $Python)) {
	$Python = "python"
}

& $Python -m pip install -r dashboard/requirements.txt
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python -m streamlit run dashboard/app.py
exit $LASTEXITCODE
