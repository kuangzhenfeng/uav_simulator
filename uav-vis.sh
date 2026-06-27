#!/usr/bin/env bash
# UAV 仿真管理终端一键启动（macOS/Linux）
# 用法: ./uav-vis.sh [port]    默认端口 8765
# 自动 kill 旧实例 → 启动后端 → 打开浏览器

set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
VIS_DIR="$PROJECT_ROOT/Tools/vis"
LOGS_DIR="$PROJECT_ROOT/Logs"
PORT="${1:-8765}"
PID_FILE="$LOGS_DIR/.vis-server.pid"

mkdir -p "$LOGS_DIR"

# ---- kill 旧实例 ----
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE" 2>/dev/null || echo "")
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "[UAV-VIS] 停止旧实例 (pid=$OLD_PID)"
        kill "$OLD_PID" 2>/dev/null || true
        sleep 1
    fi
    rm -f "$PID_FILE"
fi

# 端口兜底
if command -v lsof &>/dev/null; then
    EXISTING_PIDS=$(lsof -ti :"$PORT" 2>/dev/null || true)
    if [ -n "$EXISTING_PIDS" ]; then
        echo "[UAV-VIS] 端口 $PORT 被占用，清理 (pids=$EXISTING_PIDS)"
        echo "$EXISTING_PIDS" | xargs kill 2>/dev/null || true
        sleep 1
    fi
fi

# ---- 启动后端 ----
cd "$VIS_DIR"
echo "[UAV-VIS] 启动管理终端 → http://127.0.0.1:$PORT"
python3 server.py --port "$PORT" --project "$PROJECT_ROOT" &
SERVER_PID=$!
echo "$SERVER_PID" > "$PID_FILE"

# ---- 等待就绪 + 打开浏览器 ----
for i in $(seq 1 10); do
    if curl -s "http://127.0.0.1:$PORT/api/schema" >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

URL="http://127.0.0.1:$PORT"
echo "[UAV-VIS] ✅ 管理终端就绪: $URL"
echo "[UAV-VIS] PID: $SERVER_PID  (Ctrl+C 停止)"

if command -v open &>/dev/null; then
    open "$URL" 2>/dev/null || true
elif command -v xdg-open &>/dev/null; then
    xdg-open "$URL" 2>/dev/null || true
fi

wait "$SERVER_PID"
