@echo off
setlocal

if defined IDF_PYTHON_ENV_PATH (
    if exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
        "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%~dp0fsr_angle_sweep.py" %*
        exit /b %errorlevel%
    )
)

where python >nul 2>nul
if %errorlevel%==0 (
    python "%~dp0fsr_angle_sweep.py" %*
    exit /b %errorlevel%
)

echo ERROR: Python was not found. Run this from an ESP-IDF terminal.
exit /b 1
