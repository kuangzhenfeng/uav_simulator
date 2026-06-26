#!/bin/bash
# UAV 仿真日志可视化 —— 启动脚本(macOS/Linux)
# 后台启动 Python 标准库 HTTP 服务,实时监听 UE 运行时日志,返回访问地址。
# 用法:
#   Tools/vis/vis.sh [--watch <日志文件>] [--port <端口>] [--foreground]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PYTHON="${PYTHON:-python3}"
PORT=8765
WATCH_ARG=""
FOREGROUND=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --watch) WATCH_ARG="--watch $2"; shift 2;;
        --port) PORT="$2"; shift 2;;
        --foreground) FOREGROUND=1; shift;;
        *) shift;;
    esac
done

if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "[VIS] 错误: 未找到 $PYTHON,请安装 Python 3。" >&2
    exit 1
fi

URL="http://127.0.0.1:${PORT}"

# 检测端口占用,若已被占用则直接复用(避免重复拉起)
if command -v lsof >/dev/null 2>&1 && lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "[VIS] 端口 ${PORT} 已有服务运行,复用: ${URL}"
    echo "[VIS] 访问地址: ${URL}"
    exit 0
fi

if [[ "$FOREGROUND" -eq 1 ]]; then
    echo "[VIS] 前台启动: ${URL}"
    exec "$PYTHON" "$SCRIPT_DIR/server.py" --port "$PORT" $WATCH_ARG --project "$PROJECT_ROOT"
fi

# 后台启动:nohup 持久化,输出重定向到日志文件,不阻塞调用方
VIS_LOG="$PROJECT_ROOT/Logs/vis_server.log"
mkdir -p "$PROJECT_ROOT/Logs"
nohup "$PYTHON" "$SCRIPT_DIR/server.py" --port "$PORT" $WATCH_ARG --project "$PROJECT_ROOT" \
    > "$VIS_LOG" 2>&1 &
echo $! > "$PROJECT_ROOT/Logs/vis_server.pid"

# 等待服务就绪(最多 ~5 秒)
for _ in $(seq 1 25); do
    if curl -s -m 1 -o /dev/null "$URL/api/data" 2>/dev/null; then
        break
    fi
    sleep 0.2
done

if curl -s -m 1 -o /dev/null "$URL/api/data" 2>/dev/null; then
    echo "[VIS] 可视化已启动: ${URL}"
    echo "[VIS] 服务日志: Logs/vis_server.log  (停止: kill \$(cat Logs/vis_server.pid))"
else
    echo "[VIS] 警告: 服务可能未就绪,详见 Logs/vis_server.log"
fi
