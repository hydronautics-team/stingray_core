#!/usr/bin/env bash
set -euo pipefail

cd /stingray_core

if ! grep -Fq "source /opt/ros/humble/setup.bash" /root/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
fi

if ! grep -Fq "/stingray_core/install/setup.bash" /root/.bashrc; then
    echo "[ -f /stingray_core/install/setup.bash ] && source /stingray_core/install/setup.bash" >> /root/.bashrc
fi

source /opt/ros/humble/setup.bash

if [ "${STINGRAY_SKIP_INITIAL_BUILD:-0}" = "1" ]; then
    echo "[INFO] Initial build skipped by STINGRAY_SKIP_INITIAL_BUILD=1."
    bash .devcontainer/update-compile-commands.sh --skip-build || true
    exit 0
fi

if ! bash ./build.sh; then
    echo "[WARN] Initial workspace build failed. The container is ready; run the VS Code build task after fixing dependencies or code errors."
    bash .devcontainer/update-compile-commands.sh --skip-build || true
fi
