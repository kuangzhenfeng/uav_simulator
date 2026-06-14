#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

UE_EDITOR="$UE_ROOT/Engine/Binaries/Mac/UnrealEditor-Cmd"
PROJECT_PATH="$PROJECT_ROOT/uav_simulator.uproject"
UAV_LOG="$PROJECT_ROOT/Logs/uav.log"
UAV_FULL_LOG="$PROJECT_ROOT/Logs/uav_full.log"
SCENARIO_RESULT="$PROJECT_ROOT/Logs/scenario_result.json"

if [[ "$OSTYPE" == "darwin"* ]]; then
    DEFAULT_LOG="$HOME/Library/Logs/uav_simulator/uav_simulator.log"
else
    DEFAULT_LOG="$PROJECT_ROOT/Saved/Logs/uav_simulator.log"
fi

mkdir -p "$PROJECT_ROOT/Logs"

SIM_DURATION=${1:-60}
SLOMO=${2:-8}
SCENARIO_ASSET=${3:-}

echo "========================================"
echo "UAV Simulator - Headless Simulation (${SIM_DURATION}s real, slomo=$SLOMO)"
echo "========================================"

# 启动后台杀手：等待后强制停止 UE5
(sleep "$((SIM_DURATION + 1))" && pkill -f "UnrealEditor" 2>/dev/null || true) &
KILLER_PID=$!

# 构造命令行：可选 -Scenario=<资产路径>
EXTRA_ARGS=""
if [[ -n "$SCENARIO_ASSET" ]]; then
    EXTRA_ARGS="-Scenario=$SCENARIO_ASSET"
    echo "[SIM] Running scenario: $SCENARIO_ASSET"
fi

echo "Starting UE5 (will be stopped after ${SIM_DURATION}s real time)..."
"$UE_EDITOR" "$PROJECT_PATH" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG $EXTRA_ARGS -ExecCmds="slomo $SLOMO" -silent -LogCmds="Global Warning, LogUAVActor Log, LogUAVPlanning Log, LogUAVMission Log, LogUAVAI Log, LogUAVAttitude Log, LogUAVMultiAgent Log, LogUAVSensor Log, LogUAVMetrics Log, LogUAVProfiling Log" >/dev/null 2>&1 || true

# 停掉后台杀手（若仍存活）
kill $KILLER_PID 2>/dev/null || true

# Copy log to project root and create filtered version
if [ -f "$DEFAULT_LOG" ]; then
    # Copy full log
    cp "$DEFAULT_LOG" "$UAV_FULL_LOG"

    # Create filtered log: keep only UAV-related logs, exclude STARTUP logs and UE5 errors
    grep 'LogUAV' "$DEFAULT_LOG" | grep -v '\[STARTUP\]' | grep -v 'Failed to load' > "$UAV_LOG"

    echo "[SIM] $(date '+%H:%M:%S') Simulation complete. Logs saved:"
    echo "[SIM]   Filtered log: Logs/uav.log"
    echo "[SIM]   Full log: Logs/uav_full.log"
else
    echo "[SIM] WARNING: log not found at $DEFAULT_LOG"
    exit 1
fi

# 退出码协议（仅当指定了场景资产时生效）：
#   scenario_result.json verdict=PASS -> 退出 0
#   verdict=FAIL                       -> 退出 1
#   JSON 缺失                          -> 退出 2
if [[ -n "$SCENARIO_ASSET" ]]; then
    if [ -f "$SCENARIO_RESULT" ]; then
        if grep -q '"verdict": "PASS"' "$SCENARIO_RESULT"; then
            echo "[SIM] Scenario verdict: PASS"
            exit 0
        elif grep -q '"verdict": "FAIL"' "$SCENARIO_RESULT"; then
            echo "[SIM] Scenario verdict: FAIL"
            exit 1
        else
            echo "[SIM] Scenario verdict: UNKNOWN (no verdict field)"
            exit 2
        fi
    else
        echo "[SIM] WARNING: scenario_result.json not found"
        exit 2
    fi
fi
