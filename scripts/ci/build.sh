#!/usr/bin/env bash
set -euo pipefail

cd /stingray_core

echo "[CI] Building ROS workspace..."

colcon build \
    --event-handlers console_direct+

echo "[CI] ROS workspace build completed."
