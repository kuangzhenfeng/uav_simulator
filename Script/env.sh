#!/bin/bash
# === Shared environment config (macOS/Linux) ===

# Detect OS and set default UE_ROOT
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS: modify this path to match your UE installation
    UE_ROOT="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.7}"

    # Guard: UE 5.7 requires macOS SDK ≤ 26.9. Xcode beta SDK versions
    # exceed this ceiling and cause UBT to reject the Mac platform entirely.
    # If the current xcode-select points at a too-new SDK, override DEVELOPER_DIR
    # to the stable Xcode.app (if it exists) instead.
    if [[ -z "$DEVELOPER_DIR" ]]; then
        _sdk_major=$(xcrun --sdk macosx --show-sdk-version 2>/dev/null | cut -d. -f1) || true
        if [[ -n "$_sdk_major" && "$_sdk_major" -gt 26 ]]; then
            _stable="/Applications/Xcode.app/Contents/Developer"
            if [[ -d "$_stable" ]]; then
                export DEVELOPER_DIR="$_stable"
                echo "[ENV] SDK ${_sdk_major}.x too new for UE 5.7 (max 26.9); using $DEVELOPER_DIR"
            else
                echo "[ENV] WARNING: macOS SDK ${_sdk_major}.x exceeds UE 5.7 limit (26.9) and no stable Xcode.app found." >&2
                echo "[ENV] Install a compatible Xcode or run: sudo xcode-select -s /Applications/Xcode.app/Contents/Developer" >&2
            fi
        fi
        unset _sdk_major _stable
    fi
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
