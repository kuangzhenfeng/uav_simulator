#!/bin/bash

# Unreal Engine 5.7 项目编译脚本
# 使用 UnrealBuildTool 编译 uav_simulator 项目

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/uav_simulator.uproject"
UE_PATH="/Users/Shared/Epic Games/UE_5.7"

# 默认配置
CONFIG="${1:-Development}"

# 帮助信息
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "用法: ./build.sh [配置]"
    echo ""
    echo "配置选项:"
    echo "  Development  - 开发版本（默认）"
    echo "  Shipping     - 发布版本"
    echo "  Debug        - 调试版本"
    echo ""
    echo "示例:"
    echo "  ./build.sh              # 编译 Development 版本"
    echo "  ./build.sh Shipping     # 编译 Shipping 版本"
    exit 0
fi

echo "开始编译 uav_simulator 项目 (${CONFIG})..."

"${UE_PATH}/Engine/Build/BatchFiles/Mac/Build.sh" \
    uav_simulatorEditor \
    Mac \
    "${CONFIG}" \
    "${PROJECT_PATH}" \
    -WaitMutex

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
else
    echo "❌ 编译失败，请检查错误信息"
    exit 1
fi
