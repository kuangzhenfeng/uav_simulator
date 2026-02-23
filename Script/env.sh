#!/bin/bash
# === Shared environment config (macOS/Linux) ===

# Detect OS and set default UE_ROOT
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS: modify this path to match your UE installation
    UE_ROOT="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.7}"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux: modify this path to match your UE installation
    UE_ROOT="${UE_ROOT:-$HOME/Epic Games/UE_5.7}"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

# Get project root (parent of Script directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

export UE_ROOT
export PROJECT_ROOT
