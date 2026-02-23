#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

UE_EDITOR="$UE_ROOT/Engine/Binaries/Mac/UnrealEditor-Cmd"
PROJECT_PATH="$PROJECT_ROOT/uav_simulator.uproject"
TEST_FILTER="UAVSimulator"
TEMP_LOG="$PROJECT_ROOT/Logs/test_output.log"

mkdir -p "$PROJECT_ROOT/Logs"

echo "========================================"
echo "UAV Simulator Unit Tests"
echo "========================================"
echo ""

# Run automation tests and capture output
"$UE_EDITOR" "$PROJECT_PATH" \
    -ExecCmds="Automation RunTests $TEST_FILTER; Quit" \
    -NullRHI \
    -NoSound \
    -NoSplash \
    -unattended \
    -nopause \
    -nosplash \
    -NOSAVECONFIG \
    -NoLogTimes \
    -stdout > "$TEMP_LOG" 2>&1 || true

# Filter and display test results
echo "Test Results:"
echo "----------------------------------------"
grep -E "Found|Test Completed|TEST COMPLETE" "$TEMP_LOG" || true
echo "----------------------------------------"

# Count results
echo ""
echo "Summary:"
TOTAL=$(grep -c "Test Completed" "$TEMP_LOG" || echo 0)
PASSED=$(grep -c "Result={成功}" "$TEMP_LOG" || echo 0)
FAILED=$((TOTAL - PASSED))

echo "  Total:  $TOTAL"
echo "  Passed: $PASSED"
echo "  Failed: $FAILED"

# Show failed tests if any
if [ "$FAILED" -gt 0 ]; then
    echo ""
    echo "Failed Tests:"
    grep "Test Completed" "$TEMP_LOG" | grep -v "成功" || true
fi

echo ""
echo "========================================"
echo "Test execution completed."
echo "========================================"
