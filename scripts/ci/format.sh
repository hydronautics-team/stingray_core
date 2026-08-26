#!/usr/bin/env bash
set -euo pipefail

cd /stingray_core

echo "[CI] Checking clang-format..."

find src \
    \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \) \
    -print0 |
    xargs -0 clang-format --dry-run --Werror

echo "[CI] clang-format check passed."
