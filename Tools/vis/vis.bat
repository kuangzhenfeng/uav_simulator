@echo off
chcp 65001 >nul 2>&1
REM UAV 仿真可视化 —— 启动脚本(Windows)
REM 后台启动 Python 标准库 HTTP 服务,实时监听 UE 追加写的遥测数据流,返回访问地址。
REM 用法:
REM   Tools\vis\vis.bat [--watch <ndjson文件>] [--port <端口>] [--foreground]
REM 数据源默认 Logs\telemetry.ndjson(UAV TelemetryRecorder 追加写的专用数据流)。

setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%I in ("%SCRIPT_DIR%") do set "TOOLS_DIR=%%~dpI"
set "TOOLS_DIR=%TOOLS_DIR:~0,-1%"
for %%I in ("%TOOLS_DIR%") do set "PROJECT_ROOT=%%~dpI"
set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"
if defined PYTHON ( set "PY=%PYTHON%" ) else ( set "PY=python" )
set "PORT=8765"
set "WATCH_ARG="
set "FOREGROUND=0"

:parse
if "%~1"=="" goto :parsed
if /I "%~1"=="--watch" ( set "WATCH_ARG=--watch %~2" & shift & shift & goto :parse )
if /I "%~1"=="--port" ( set "PORT=%~2" & shift & shift & goto :parse )
if /I "%~1"=="--foreground" ( set "FOREGROUND=1" & shift & goto :parse )
shift
goto :parse
:parsed

where %PY% >nul 2>&1
if errorlevel 1 (
    echo [VIS] 错误: 未找到 %PY%,请安装 Python 3。 1>&2
    exit /b 1
)

set "URL=http://127.0.0.1:%PORT%"

REM 检测端口占用:netstat 有 LISTENING 即复用
netstat -ano | findstr ":%PORT% " | findstr "LISTENING" >nul 2>&1
if not errorlevel 1 (
    echo [VIS] 端口 %PORT% 已有服务运行,复用: %URL%
    echo [VIS] 访问地址: %URL%
    exit /b 0
)

if "%FOREGROUND%"=="1" (
    echo [VIS] 前台启动: %URL%
    %PY% "%SCRIPT_DIR%\server.py" --port %PORT% %WATCH_ARG% --project "%PROJECT_ROOT%"
    exit /b %errorlevel%
)

REM 后台启动:输出重定向到日志文件,不阻塞调用方
set "VIS_LOG=%PROJECT_ROOT%\Logs\vis_server.log"
if not exist "%PROJECT_ROOT%\Logs" mkdir "%PROJECT_ROOT%\Logs"
start "UAV-Vis-Server" /MIN %PY% "%SCRIPT_DIR%\server.py" --port %PORT% %WATCH_ARG% --project "%PROJECT_ROOT%"

REM 等待服务就绪(最多 ~5 秒)
set /a TRIES=25
:waitready
powershell -Command "try { (Invoke-WebRequest -Uri '%URL%/api/data' -UseBasicParsing -TimeoutSec 1).StatusCode } catch { exit 1 }" >nul 2>&1
if not errorlevel 1 goto :ready
set /a TRIES-=1
if %TRIES% GTR 0 (
    powershell -Command "Start-Sleep -Milliseconds 200" >nul
    goto :waitready
)
echo [VIS] 警告: 服务可能未就绪,详见 Logs\vis_server.log
exit /b 0

:ready
echo [VIS] 可视化已启动: %URL%
echo [VIS] 服务日志: Logs\vis_server.log
exit /b 0
