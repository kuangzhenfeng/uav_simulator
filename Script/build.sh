#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

PROJECT_PATH="$PROJECT_ROOT/uav_simulator.uproject"
LOG_FILE="$PROJECT_ROOT/Logs/build_output.log"

mkdir -p "$PROJECT_ROOT/Logs"

echo "========================================"
echo "UAV Simulator Build"
echo "========================================"

"$UE_ROOT/Engine/Build/BatchFiles/Mac/Build.sh" uav_simulatorEditor Mac Development -Project="$PROJECT_PATH" -WaitMutex > "$LOG_FILE" 2>&1

cat "$LOG_FILE"

if grep -q "Result: Succeeded" "$LOG_FILE"; then
    echo ""
    echo "BUILD SUCCEEDED"
else
    echo ""
    echo "BUILD FAILED"
    echo "Error summary:"
    grep -i "error\|fatal" "$LOG_FILE" || true
    exit 1
fi


