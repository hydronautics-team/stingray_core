#!/bin/bash

set -e

echo "----------------------------------------"
echo "🔄 Sourcing install/setup.bash ..."
echo "----------------------------------------"

source install/setup.bash

echo "----------------------------------------"
echo "🚀 Launching Stingray ROV..."
echo "----------------------------------------"

ros2 launch stingray_core_launch run_rov.launch.py