#!/bin/bash

set -e

echo "----------------------------------------"
echo "🔧 Building all Stingray packages (including launch)..."
echo "----------------------------------------"

colcon build --packages-select \
    stingray_core_control \
    stingray_core_communication \
    serial_driver \
    io_context \
    asio_cmake_module \
    power_control \
    stingray_core_launch

echo "----------------------------------------"
echo "🔄 Sourcing install/setup.bash ..."
echo "----------------------------------------"

source install/setup.bash

echo "----------------------------------------"
echo "🚀 Launching Stingray ROV..."
echo "----------------------------------------"

ros2 launch stingray_core_launch run_rov.launch.py
