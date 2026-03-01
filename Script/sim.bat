@echo off
chcp 65001 >nul 2>&1
setlocal

call "%~dp0env.bat"
set UE_EDITOR=%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe
set PROJECT_PATH=%PROJECT_ROOT%\uav_simulator.uproject
set UAV_LOG=%PROJECT_ROOT%\Logs\uav.log
set UAV_FULL_LOG=%PROJECT_ROOT%\Logs\uav_full.log
set DEFAULT_LOG=%PROJECT_ROOT%\Saved\Logs\uav_simulator.log
if not exist "%PROJECT_ROOT%\Logs" mkdir "%PROJECT_ROOT%\Logs"
set SIM_DURATION=60
set SLOMO=8

if "%~1" NEQ "" set SIM_DURATION=%~1
if "%~2" NEQ "" set SLOMO=%~2

echo ========================================
echo UAV Simulator - Headless Simulation (%SIM_DURATION%s real, slomo=%SLOMO%)
echo ========================================

REM Launch background killer: waits, then force-stops UE5 (minimized window)
set /a PING_COUNT=%SIM_DURATION%+1
start /MIN "" cmd /c "ping -n %PING_COUNT% 127.0.0.1 >nul & taskkill /F /IM UnrealEditor-Cmd.exe >nul 2>&1"

REM Run UE5 directly (blocking) - suppress verbose logs
echo Starting UE5 (will be stopped after %SIM_DURATION%s real time)...
"%UE_EDITOR%" "%PROJECT_PATH%" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG -ExecCmds="slomo %SLOMO%" -silent -LogCmds="Global Warning, LogUAVActor Log, LogUAVPlanning Log, LogUAVMission Log, LogUAVAI Log, LogUAVAttitude Log" >nul 2>&1

REM Kill lingering killer process so MSYS bash does not hang
taskkill /F /IM ping.exe >nul 2>&1

REM Copy log to project root and create filtered version
if exist "%DEFAULT_LOG%" (
    REM Copy full log
    copy /Y "%DEFAULT_LOG%" "%UAV_FULL_LOG%" >nul

    REM Create filtered log: keep only UAV-related logs, exclude STARTUP logs and UE5 errors
    powershell -Command "Get-Content '%DEFAULT_LOG%' | Where-Object { ($_ -match 'LogUAV') -and ($_ -notmatch '\[STARTUP\]') -and ($_ -notmatch 'Failed to load') } | Set-Content '%UAV_LOG%'"

    echo [SIM] %TIME% Simulation complete. Logs saved:
    echo [SIM]   Filtered log: Logs\uav.log
    echo [SIM]   Full log: Logs\uav_full.log
) else (
    echo [SIM] WARNING: log not found!
    exit /b 1
)

endlocal
