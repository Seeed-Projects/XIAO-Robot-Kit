@echo off
rem =====================================================================
rem  xiao_robot_kit_ble_skill -- Build & Flash (ESP-IDF v6.0.2 or newer)
rem
rem  USAGE:
rem    run.bat          -> build only
rem    run.bat COMx     -> build + flash + monitor on COMx
rem
rem  DEPENDENCIES (checked at startup):
rem    - ESP-IDF SDK  : %IDF_PATH%\export.bat
rem    - Python env   : %IDF_PYTHON_ENV_PATH% (idf6.0 virtualenv)
rem =====================================================================
setlocal
set MSYSTEM=
rem  Set IDF_PATH / IDF_PYTHON_ENV_PATH yourself if ESP-IDF lives elsewhere.
if not defined IDF_PATH set IDF_PATH=%USERPROFILE%\esp\v6.0\esp-idf
if not defined IDF_PYTHON_ENV_PATH set IDF_PYTHON_ENV_PATH=%USERPROFILE%\.espressif\python_env\idf6.0_py3.11_env

if not exist "%IDF_PATH%\export.bat" (
    echo [ERR] ESP-IDF SDK not found: %IDF_PATH%\export.bat
    echo       Install ESP-IDF v6.0.2 or newer, or fix IDF_PATH at the top of this script.
    pause
    exit /b 1
)
if not exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
    echo [ERR] IDF Python env not found: %IDF_PYTHON_ENV_PATH%
    echo       Create it via ESP-IDF installer, or fix IDF_PYTHON_ENV_PATH.
    pause
    exit /b 1
)

call "%IDF_PATH%\export.bat" >nul
if errorlevel 1 (
    echo [ERR] Failed to activate ESP-IDF environment.
    pause
    exit /b 1
)

if "%~1"=="" (
    echo [INFO] Build only (no port given). To flash+monitor: run.bat COMx
    idf.py build
) else (
    echo [INFO] Build + flash + monitor on port %~1
    idf.py -p %~1 flash monitor
)
endlocal
if errorlevel 1 pause