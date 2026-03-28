#!/bin/bash

set -euo pipefail

echo "----------------------------------------"
echo "🔧 Building all Stingray packages (including launch)..."
echo "----------------------------------------"

TARGET_PACKAGES=(
    dvl_msgs
    stingray_core_control
    stingray_core_communication
    serial_driver
    io_context
    asio_cmake_module
    vectornav_msgs
    vectornav
    stingray_core_launch
    pressure_sensor
    ms5837_pressure_sensor
)

mapfile -t AVAILABLE_PACKAGES < <(colcon list --names-only)
SELECTED_PACKAGES=()

for pkg in "${TARGET_PACKAGES[@]}"; do
    if printf '%s\n' "${AVAILABLE_PACKAGES[@]}" | grep -qx "${pkg}"; then
        SELECTED_PACKAGES+=("${pkg}")
    else
        echo "⚠️ Skipping missing package: ${pkg}"
    fi
done

if [ "${#SELECTED_PACKAGES[@]}" -eq 0 ]; then
    echo "❌ No matching packages found to build."
    exit 1
fi

colcon build --cmake-clean-cache --packages-select "${SELECTED_PACKAGES[@]}"

echo "----------------------------------------"
echo "🔄 Sourcing install/setup.bash ..."
echo "----------------------------------------"

source install/setup.bash

echo "----------------------------------------"
echo "🚀 Launching Stingray ROV..."
echo "----------------------------------------"

ros2 launch stingray_core_launch run_rov.launch.py
