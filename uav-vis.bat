@echo off
REM UAV 仿真管理终端一键启动（Windows）
REM 用法: uav-vis.bat [port]    默认端口 8765
REM 自动 kill 旧实例 -> 启动后端 -> 打开浏览器

setlocal
set PROJECT_ROOT=%~dp0
set VIS_DIR=%PROJECT_ROOT%Tools\vis
set LOGS_DIR=%PROJECT_ROOT%Logs
set PORT=%1
if "%PORT%"=="" set PORT=8765
set PID_FILE=%LOGS_DIR%\.vis-server.pid

if not exist "%LOGS_DIR%" mkdir "%LOGS_DIR%"

REM ---- kill 旧实例 ----
if exist "%PID_FILE%" (
    set /p OLD_PID=<"%PID_FILE%"
    taskkill /PID %OLD_PID% /F >nul 2>&1
    del "%PID_FILE%" 2>nul
    timeout /t 1 /nobreak >nul
)

REM 端口兜底
for /f "tokens=5" %%a in ('netstat -aon ^| findstr ":%PORT% " ^| findstr "LISTENING"') do (
    taskkill /PID %%a /F >nul 2>&1
    timeout /t 1 /nobreak >nul
)

REM ---- 启动后端 ----
cd /d "%VIS_DIR%"
echo [UAV-VIS] 启动管理终端 -^> http://127.0.0.1:%PORT%
start /b python server.py --port %PORT% --project "%PROJECT_ROOT%" > "%LOGS_DIR%\vis_stdout.log" 2>&1

REM 获取 PID
for /f "tokens=2" %%a in ('tasklist /fi "imagename eq python.exe" /fo list ^| findstr "PID:"') do (
    echo %%a> "%PID_FILE%"
    goto :found_pid
)
:found_pid

REM ---- 等待就绪 + 打开浏览器 ----
set READY=0
for /l %%i in (1,1,10) do (
    curl -s "http://127.0.0.1:%PORT%/api/schema" >nul 2>&1
    if not errorlevel 1 (
        set READY=1
        goto :ready
    )
    timeout /t 1 /nobreak >nul
)
:ready

echo [UAV-VIS] 管理终端就绪: http://127.0.0.1:%PORT%
start "" "http://127.0.0.1:%PORT%"

REM 保持窗口
pause
endlocal
