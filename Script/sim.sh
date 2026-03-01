#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

UE_EDITOR="$UE_ROOT/Engine/Binaries/Mac/UnrealEditor-Cmd"
PROJECT_PATH="$PROJECT_ROOT/uav_simulator.uproject"
UAV_LOG="$PROJECT_ROOT/Logs/uav.log"
UAV_FULL_LOG="$PROJECT_ROOT/Logs/uav_full.log"

if [[ "$OSTYPE" == "darwin"* ]]; then
    DEFAULT_LOG="$HOME/Library/Logs/uav_simulator/uav_simulator.log"
else
    DEFAULT_LOG="$PROJECT_ROOT/Saved/Logs/uav_simulator.log"
fi

mkdir -p "$PROJECT_ROOT/Logs"

SIM_DURATION=${1:-60}
SLOMO=${2:-8}

echo "========================================"
echo "UAV Simulator - Headless Simulation (${SIM_DURATION}s real, slomo=$SLOMO)"
echo "========================================"

# Launch background killer: waits, then force-stops UE5
(sleep "$((SIM_DURATION + 1))" && pkill -f "UnrealEditor" 2>/dev/null || true) &
KILLER_PID=$!

echo "Starting UE5 (will be stopped after ${SIM_DURATION}s real time)..."
"$UE_EDITOR" "$PROJECT_PATH" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG -ExecCmds="slomo $SLOMO" -silent -LogCmds="Global Warning, LogUAVActor Log, LogUAVPlanning Log, LogUAVMission Log, LogUAVAI Log, LogUAVAttitude Log" >/dev/null 2>&1 || true

# Kill the background killer if still running
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


