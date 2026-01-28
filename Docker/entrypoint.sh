#!/bin/bash
set -e

if [ -d /ros2_ws/src ] && [ "$(ls -A /ros2_ws/src 2>/dev/null)" ]; then
    echo "Building ROS2 workspace..."
    cd /ros2_ws

    source /opt/ros/jazzy/setup.bash

    if [ -f /ros2_ws/src/rwifi_nvblox_slam_ros2/requirements.txt ]; then
        echo "Installing Python dependencies..."
        python3 -m pip install --break-system-packages -q -r /ros2_ws/src/rwifi_nvblox_slam_ros2/requirements.txt || true
    fi

    echo "Installing ROS2 dependencies via rosdep..."
    rosdep update >/dev/null 2>&1 || true
    rosdep install --from-paths src --ignore-src -r -y || true

    echo "Building ROS2 packages with colcon..."
    colcon build --symlink-install || true

    chown -R 1000:1000 /ros2_ws/build /ros2_ws/install /ros2_ws/log 2>/dev/null || true

    echo "Workspace build complete."
fi

source /opt/ros/jazzy/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

if [ -n "$HOME" ] && [ -f "$HOME/.bashrc" ]; then
    if ! grep -Fxq "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"; then
        echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"
    fi
fi

if [ -n "$HOME" ] && [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc" 2>/dev/null || true
fi

exec "${@:-bash}"
