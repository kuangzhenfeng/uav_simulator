@echo off
chcp 65001 >nul 2>&1
setlocal

call "%~dp0env.bat"
set UE_EDITOR=%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe
set PROJECT_PATH=%PROJECT_ROOT%\uav_simulator.uproject
set UAV_LOG=%PROJECT_ROOT%\Logs\uav.log
set DEFAULT_LOG=%PROJECT_ROOT%\Saved\Logs\uav_simulator.log
if not exist "%PROJECT_ROOT%\Logs" mkdir "%PROJECT_ROOT%\Logs"
set SIM_DURATION=60

if "%~1" NEQ "" set SIM_DURATION=%~1

echo ========================================
echo UAV Simulator - Headless Simulation (%SIM_DURATION%s)
echo ========================================

REM Launch background killer: waits, then force-stops UE5
set /a PING_COUNT=%SIM_DURATION%+1
start "" cmd /c "ping -n %PING_COUNT% 127.0.0.1 >nul & taskkill /F /IM UnrealEditor-Cmd.exe >nul 2>&1"

REM Run UE5 directly (blocking) - logs write normally
echo Starting UE5 (will be stopped after %SIM_DURATION%s)...
"%UE_EDITOR%" "%PROJECT_PATH%" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG >nul 2>&1

REM Kill lingering killer process so MSYS bash does not hang
taskkill /F /IM ping.exe >nul 2>&1

REM Copy log to project root
if exist "%DEFAULT_LOG%" (
    copy /Y "%DEFAULT_LOG%" "%UAV_LOG%" >nul
    echo [SIM] %TIME% Simulation complete. Log saved to Logs\uav.log
) else (
    echo [SIM] WARNING: log not found!
    exit /b 1
)

endlocal
