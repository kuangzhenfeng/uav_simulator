#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

UE_EDITOR="$UE_ROOT/Engine/Binaries/Mac/UnrealEditor-Cmd"
PROJECT_PATH="$PROJECT_ROOT/uav_simulator.uproject"
UAV_LOG="$PROJECT_ROOT/Logs/uav.log"

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
"$UE_EDITOR" "$PROJECT_PATH" -game -NullRHI -NoSound -NoSplash -unattended -nopause -NOSAVECONFIG -ExecCmds="slomo $SLOMO" >/dev/null 2>&1 || true

# Kill the background killer if still running
kill $KILLER_PID 2>/dev/null || true

# Copy log to project root
if [ -f "$DEFAULT_LOG" ]; then
    cp "$DEFAULT_LOG" "$UAV_LOG"
    echo "[SIM] $(date '+%H:%M:%S') Simulation complete. Log saved to Logs/uav.log"
else
    echo "[SIM] WARNING: log not found at $DEFAULT_LOG"
    exit 1
fi


