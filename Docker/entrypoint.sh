#!/bin/bash
set -e

# Source ROS2 setup
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: ROS2 setup file not found at /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

# Source workspace setup if it exists (check both locations)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
elif [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Execute command passed to container
exec "$@"
