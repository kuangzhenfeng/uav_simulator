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

# 运行自动化测试，不隐藏编辑器崩溃
set +e
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
    -stdout > "$TEMP_LOG" 2>&1
EDITOR_EXIT_CODE=$?
set -e

# 过滤并显示测试结果
echo "Test Results:"
echo "----------------------------------------"
grep -E "Found|Test Completed|TEST COMPLETE" "$TEMP_LOG" || true
echo "----------------------------------------"

# 分别统计发现数和完成数。崩溃可能导致所有已完成的测试都通过，但大部分测试未运行。
echo ""
echo "Summary:"
DISCOVERED=$(awk '/Found [0-9]+ automation tests/ {
    for (i = 1; i <= NF; ++i) {
        if ($i == "Found") {
            found = $(i + 1)
        }
    }
} END { print found + 0 }' "$TEMP_LOG")
COMPLETED=$(awk '/Test Completed/ { count++ } END { print count + 0 }' "$TEMP_LOG")
PASSED=$(awk '/Test Completed/ && /Result={成功}/ { count++ } END { print count + 0 }' "$TEMP_LOG")
FAILED=$((COMPLETED - PASSED))
INCOMPLETE=$((DISCOVERED - COMPLETED))

echo "  Discovered: $DISCOVERED"
echo "  Completed:  $COMPLETED"
echo "  Passed:     $PASSED"
echo "  Failed:     $FAILED"
echo "  Incomplete: $INCOMPLETE"
echo "  Editor exit code: $EDITOR_EXIT_CODE"

# 显示失败的测试
if [ "$FAILED" -gt 0 ]; then
    echo ""
    echo "Failed Tests:"
    grep "Test Completed" "$TEMP_LOG" | grep -v "成功" || true
fi

# 检测崩溃
if grep -Eq "Assertion failed:|appError called:|Fatal error:|Unhandled exception:" "$TEMP_LOG"; then
    echo ""
    echo "Crash detected:"
    grep -E "Assertion failed:|appError called:|Fatal error:|Unhandled exception:" "$TEMP_LOG" | tail -n 10
fi

echo ""
echo "========================================"
echo "Test execution completed."
echo "========================================"

if [ "$EDITOR_EXIT_CODE" -ne 0 ] \
    || [ "$FAILED" -gt 0 ] \
    || [ "$INCOMPLETE" -ne 0 ] \
    || ! grep -q "TEST COMPLETE. EXIT CODE: 0" "$TEMP_LOG"; then
    exit 1
fi
