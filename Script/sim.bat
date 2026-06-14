@echo off
chcp 65001 >nul 2>&1
setlocal

call "%~dp0env.bat"
set UE_EDITOR=%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe
set PROJECT_PATH=%PROJECT_ROOT%\uav_simulator.uproject
set UAV_LOG=%PROJECT_ROOT%\Logs\uav.log
set UAV_FULL_LOG=%PROJECT_ROOT%\Logs\uav_full.log
set SCENARIO_RESULT=%PROJECT_ROOT%\Logs\scenario_result.json
set DEFAULT_LOG=%PROJECT_ROOT%\Saved\Logs\uav_simulator.log
if not exist "%PROJECT_ROOT%\Logs" mkdir "%PROJECT_ROOT%\Logs"
set SIM_DURATION=60
set SLOMO=8
set SCENARIO_ASSET=

if "%~1" NEQ "" set SIM_DURATION=%~1
if "%~2" NEQ "" set SLOMO=%~2
if "%~3" NEQ "" set SCENARIO_ASSET=%~3

echo ========================================
echo UAV Simulator - Headless Simulation (%SIM_DURATION%s real, slomo=%SLOMO%)
echo ========================================

REM Launch background killer: waits, then force-stops UE5 (minimized window)
set /a PING_COUNT=%SIM_DURATION%+1
start /MIN "" cmd /c "ping -n %PING_COUNT% 127.0.0.1 >nul & taskkill /F /IM UnrealEditor-Cmd.exe >nul 2>&1"

REM 构造命令行：可选 -Scenario=<资产路径>
set EXTRA_ARGS=
if defined SCENARIO_ASSET (
    if "%SCENARIO_ASSET%" NEQ "" (
        set EXTRA_ARGS=-Scenario=%SCENARIO_ASSET%
        echo [SIM] Running scenario: %SCENARIO_ASSET%
    )
)

REM Run UE5 directly (blocking) - suppress verbose logs
echo Starting UE5 (will be stopped after %SIM_DURATION%s real time)...
"%UE_EDITOR%" "%PROJECT_PATH%" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG %EXTRA_ARGS% -ExecCmds="slomo %SLOMO%" -silent -LogCmds="Global Warning, LogUAVActor Log, LogUAVPlanning Log, LogUAVMission Log, LogUAVAI Log, LogUAVAttitude Log, LogUAVMultiAgent Log, LogUAVSensor Log, LogUAVMetrics Log, LogUAVProfiling Log, LogScenarioEval Log" >nul 2>&1

REM Kill lingering killer process so MSYS bash does not hang
taskkill /F /IM ping.exe >nul 2>&1

REM Copy log to project root and create filtered version
if exist "%DEFAULT_LOG%" (
    REM Copy full log
    copy /Y "%DEFAULT_LOG%" "%UAV_FULL_LOG%" >nul

    REM Create filtered log: keep only UAV-related logs, exclude STARTUP logs and UE5 errors
    powershell -Command "Get-Content '%DEFAULT_LOG%' | Where-Object { (($_ -match 'LogUAV') -or ($_ -match 'LogScenarioEval')) -and ($_ -notmatch '\[STARTUP\]') -and ($_ -notmatch 'Failed to load') } | Set-Content '%UAV_LOG%'"

    echo [SIM] %TIME% Simulation complete. Logs saved:
    echo [SIM]   Filtered log: Logs\uav.log
    echo [SIM]   Full log: Logs\uav_full.log
) else (
    echo [SIM] WARNING: log not found!
    exit /b 1
)

REM 退出码协议（仅当指定了场景资产时生效）：
REM   scenario_result.json verdict=PASS -> 退出 0
REM   verdict=FAIL                       -> 退出 1
REM   JSON 缺失                          -> 退出 2
if not defined SCENARIO_ASSET goto :done
if "%SCENARIO_ASSET%" EQU "" goto :done
if not exist "%SCENARIO_RESULT%" (
    echo [SIM] WARNING: scenario_result.json not found
    exit /b 2
)
powershell -Command "try { $j=Get-Content '%SCENARIO_RESULT%' -Raw | ConvertFrom-Json; if ($j.verdict -eq 'PASS') { exit 0 } elseif ($j.verdict -eq 'FAIL') { exit 1 } else { exit 2 } } catch { exit 2 }"
if %ERRORLEVEL% EQU 0 (
    echo [SIM] Scenario verdict: PASS
    exit /b 0
)
if %ERRORLEVEL% EQU 1 (
    echo [SIM] Scenario verdict: FAIL
    exit /b 1
)
echo [SIM] Scenario verdict: UNKNOWN (no verdict field)
exit /b 2

:done
endlocal
