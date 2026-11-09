@echo off
setlocal

set "BUNDLED_PYTHON=C:\Users\luyaning\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%BUNDLED_PYTHON%" (
    "%BUNDLED_PYTHON%" "%~dp0build_fsr_cycle_chart.py" %*
    exit /b %errorlevel%
)

if defined IDF_PYTHON_ENV_PATH (
    if exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
        "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%~dp0build_fsr_cycle_chart.py" %*
        exit /b %errorlevel%
    )
)

where python >nul 2>nul
if %errorlevel%==0 (
    python "%~dp0build_fsr_cycle_chart.py" %*
    exit /b %errorlevel%
)

echo ERROR: Python was not found. Run this from an ESP-IDF terminal.
exit /b 1
