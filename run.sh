#!/bin/bash

# Stop on error
set -e

echo "sourcing ros2"
source /opt/ros/humble/setup.sh

echo "🔧 Building the workspace..."
colcon build --packages-select my_robot_package --symlink-install

echo "✅ Build complete. Sourcing the environment..."
source install/setup.bash

echo "🚀 Running real_button_node..."
ros2 run my_robot_package real_button_node
