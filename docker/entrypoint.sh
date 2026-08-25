#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if [ -f /stingray_core/install/setup.bash ]; then
    echo "[INFO] Using existing workspace build."
    source /stingray_core/install/setup.bash
else
    echo "[INFO] Workspace build not found."
fi

exec "$@"