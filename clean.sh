#!/bin/bash

# Unreal Engine 5.7 项目清理脚本
# 清理编译缓存和中间文件

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 帮助信息
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "用法: ./clean.sh"
    echo ""
    echo "清理以下目录:"
    echo "  - Binaries          (编译输出)"
    echo "  - Intermediate      (中间文件)"
    echo "  - DerivedDataCache  (派生数据缓存)"
    exit 0
fi

echo "开始清理项目缓存..."

# 删除 Binaries 目录
if [ -d "${SCRIPT_DIR}/Binaries" ]; then
    echo "删除 Binaries 目录..."
    rm -rf "${SCRIPT_DIR}/Binaries"
fi

# 删除 Intermediate 目录
if [ -d "${SCRIPT_DIR}/Intermediate" ]; then
    echo "删除 Intermediate 目录..."
    rm -rf "${SCRIPT_DIR}/Intermediate"
fi

# 删除 DerivedDataCache 目录
if [ -d "${SCRIPT_DIR}/DerivedDataCache" ]; then
    echo "删除 DerivedDataCache 目录..."
    rm -rf "${SCRIPT_DIR}/DerivedDataCache"
fi

echo "✅ 清理完成！"
echo "提示：运行 ./build.sh 重新编译项目"
