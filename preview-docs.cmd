@echo off
setlocal

set "UV_BIN=uv"
where uv >nul 2>nul
if not errorlevel 1 goto preview

set "UV_BIN=%~dp0.cache\docs-preview\uv\uv.exe"
if exist "%UV_BIN%" goto preview

echo uv was not found. Installing it for this repository...
set "UV_UNMANAGED_INSTALL=%~dp0.cache\docs-preview\uv"
rem Let Windows PowerShell load its own modules when called from PowerShell 7.
set "PSModulePath="
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
    "$ErrorActionPreference = 'Stop';" ^
    "$ProgressPreference = 'SilentlyContinue';" ^
    "Invoke-RestMethod 'https://astral.sh/uv/install.ps1' |" ^
    "Invoke-Expression"
if errorlevel 1 (
    echo uv installation failed. See the error above, then retry. >&2
    exit /b 1
)
if not exist "%UV_BIN%" (
    echo uv installation did not create the expected executable. >&2
    exit /b 1
)

:preview
"%UV_BIN%" run --isolated --no-project --python 3.13 ^
    --with-requirements "%~dp0docs\requirements.txt" ^
    python -m mkdocs serve --config-file "%~dp0mkdocs.yml" ^
    --dev-addr 127.0.0.1:8000 %*
exit /b %errorlevel%
