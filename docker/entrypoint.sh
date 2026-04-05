#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# Если проект уже собран
if [ -f  /stingray_core/install/setup.bash ]; then
  echo "[INFO] Используем существующую сборку."
  source /stingray_core/install/setup.bash
else
  echo "[INFO] Локальная сборка не найдена (install/setup.bash отсутствует)"
fi

exec "$@"
