#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

source /opt/ros/humble/setup.bash

echo "[INFO] Building ROS 2 workspace..."

colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "[INFO] Updating compile_commands.json..."

if [ -f .devcontainer/update-compile-commands.sh ]; then
    if ! bash .devcontainer/update-compile-commands.sh --skip-build; then
        echo "[WARN] Build succeeded, but compile_commands.json was not refreshed." >&2
    fi
fi

echo "[INFO] Build complete."
