@echo off
chcp 65001 >nul 2>&1
setlocal

call "%~dp0env.bat"
set UE_BUILD=%UE_ROOT%\Engine\Build\BatchFiles\Build.bat
set PROJECT_PATH=%PROJECT_ROOT%\uav_simulator.uproject
set LOG_FILE=Logs\build_output.log

if not exist Logs mkdir Logs

echo ========================================
echo UAV Simulator Build
echo ========================================

call "%UE_BUILD%" uav_simulatorEditor Win64 Development -Project="%PROJECT_PATH%" -WaitMutex > %LOG_FILE% 2>&1
type %LOG_FILE%

findstr /C:"Result: Succeeded" %LOG_FILE% >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo BUILD FAILED
    echo Error summary:
    findstr /I /C:"error" /C:"fatal" %LOG_FILE%
    exit /b 1
)

echo.
echo BUILD SUCCEEDED
endlocal
