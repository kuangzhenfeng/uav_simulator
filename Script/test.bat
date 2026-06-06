@echo off
chcp 65001 >nul 2>&1
REM UAV Simulator 自动化单元测试脚本
REM 使用 UE5 Automation Test Framework

setlocal

call "%~dp0env.bat"
set UE_EDITOR="%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe"
set PROJECT_PATH="%PROJECT_ROOT%\uav_simulator.uproject"
set TEST_FILTER=UAVSimulator
set TEMP_LOG=Logs\test_output.log

if not exist Logs mkdir Logs

echo ========================================
echo UAV Simulator Unit Tests
echo ========================================
echo.

REM 运行自动化测试，不隐藏编辑器崩溃
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
set EDITOR_EXIT_CODE=%ERRORLEVEL%

REM 过滤并显示测试结果
echo Test Results:
echo ----------------------------------------
findstr /C:"Found" /C:"Test Completed" /C:"TEST COMPLETE" "%TEMP_LOG%"
echo ----------------------------------------

REM 分别统计发现数和完成数。崩溃可能导致所有已完成的测试都通过，但大部分测试未运行。
echo.
echo Summary:
for /f %%a in ('powershell -Command "(Get-Content '%TEMP_LOG%' | Select-String 'Found (\d+) automation tests' | ForEach-Object { if ($_.Line -match 'Found (\d+)') { $Matches[1] } } | Select-Object -First 1)"') do set DISCOVERED=%%a
for /f %%a in ('powershell -Command "(Get-Content '%TEMP_LOG%' | Select-String 'Test Completed').Count"') do set COMPLETED=%%a
for /f %%a in ('powershell -Command "(Get-Content '%TEMP_LOG%' -Encoding UTF8 | Select-String 'Result=\{成功\}').Count"') do set PASSED=%%a
set /a FAILED=%COMPLETED%-%PASSED%
set /a INCOMPLETE=%DISCOVERED%-%COMPLETED%

echo   Discovered: %DISCOVERED%
echo   Completed:  %COMPLETED%
echo   Passed:     %PASSED%
echo   Failed:     %FAILED%
echo   Incomplete: %INCOMPLETE%
echo   Editor exit code: %EDITOR_EXIT_CODE%

REM 显示失败的测试
if %FAILED% GTR 0 (
    echo.
    echo Failed Tests:
    powershell -Command "Get-Content '%TEMP_LOG%' -Encoding UTF8 | Select-String 'Test Completed' | Select-String -NotMatch '成功'"
)

REM 检测崩溃
powershell -Command "if (Get-Content '%TEMP_LOG%' -ErrorAction SilentlyContinue | Select-String -Pattern 'Assertion failed:|appError called:|Fatal error:|Unhandled exception:') { Write-Host ''; Write-Host 'Crash detected:'; Get-Content '%TEMP_LOG%' | Select-String -Pattern 'Assertion failed:|appError called:|Fatal error:|Unhandled exception:' | Select-Object -Last 10 }"

echo.
echo ========================================
echo Test execution completed.
echo ========================================

REM 任意条件失败则返回非零退出码
if %FAILED% GTR 0 exit /b 1
if %INCOMPLETE% GTR 0 exit /b 1
if %EDITOR_EXIT_CODE% NEQ 0 exit /b 1
powershell -Command "if (Get-Content '%TEMP_LOG%' -ErrorAction SilentlyContinue | Select-String -Pattern 'Assertion failed:|appError called:|Fatal error:|Unhandled exception:') { exit 1 }" && (echo ok >nul) || exit /b 1
findstr /C:"TEST COMPLETE. EXIT CODE: 0" "%TEMP_LOG%" >nul 2>&1 || exit /b 1

endlocal
