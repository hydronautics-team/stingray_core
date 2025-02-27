#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /stingray_core/install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source /stingray_core/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим
  source "/opt/ros/humble/setup.bash"
  source "/additional_packages/install/setup.bash"
  if ! colcon build --packages-select stingray_core_launch stingray_core_interfaces stingray_core_communication; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /stingray_core/install/setup.bash
fi

exec "$@"
