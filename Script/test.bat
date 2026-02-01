@echo off
chcp 65001 >nul 2>&1
REM UAV Simulator Automation Unit Test Script
REM Using UE5 Automation Test Framework

setlocal

set UE_EDITOR="D:\mySoftware\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe"
set PROJECT_PATH="D:\myProject\Unreal Projects\uav_simulator\uav_simulator.uproject"
set TEST_FILTER=UAVSimulator
set TEMP_LOG=%TEMP%\uav_test_output.log

echo ========================================
echo UAV Simulator Unit Tests
echo ========================================
echo.

REM Run automation tests and capture output
%UE_EDITOR% %PROJECT_PATH% ^
    -ExecCmds="Automation RunTests %TEST_FILTER%; Quit" ^
    -NullRHI ^
    -NoSound ^
    -NoSplash ^
    -unattended ^
    -nopause ^
    -nosplash ^
    -NOSAVECONFIG ^
    -NoLogTimes ^
    -stdout > "%TEMP_LOG%" 2>&1

REM Filter and display test results
echo Test Results:
echo ----------------------------------------
findstr /C:"Found" /C:"Test Completed" /C:"TEST COMPLETE" "%TEMP_LOG%"
echo ----------------------------------------

REM Count results using PowerShell for better UTF-8 support
echo.
echo Summary:
for /f %%a in ('powershell -Command "(Get-Content '%TEMP_LOG%' | Select-String 'Test Completed').Count"') do set TOTAL=%%a
for /f %%a in ('powershell -Command "(Get-Content '%TEMP_LOG%' -Encoding UTF8 | Select-String 'Result=\{成功\}').Count"') do set PASSED=%%a
set /a FAILED=%TOTAL%-%PASSED%

echo   Total:  %TOTAL%
echo   Passed: %PASSED%
echo   Failed: %FAILED%

REM Show failed tests if any
if %FAILED% GTR 0 (
    echo.
    echo Failed Tests:
    powershell -Command "Get-Content '%TEMP_LOG%' -Encoding UTF8 | Select-String 'Test Completed' | Select-String -NotMatch '成功'"
)

echo.
echo ========================================
echo Test execution completed.
echo ========================================

REM Cleanup temp file
del "%TEMP_LOG%" >nul 2>&1

endlocal
