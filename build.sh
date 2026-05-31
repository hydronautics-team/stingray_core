#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

colcon build --packages-select \
    dvl_msgs \
    stingray_core_control \
    stingray_core_communication \
    serial_driver \
    io_context \
    asio_cmake_module \
    vectornav_msgs \
    vectornav \
    pressure_sensor \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ -f .devcontainer/update-compile-commands.sh ]; then
    if ! bash .devcontainer/update-compile-commands.sh --skip-build; then
        echo "[WARN] Build succeeded, but compile_commands.json was not refreshed." >&2
    fi
fi
