#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -d /ros2_ws/src ] && [ "$(ls -A /ros2_ws/src 2>/dev/null)" ]; then
    echo "Building ROS2 workspace..."
    cd /ros2_ws

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

    if [ -f /ros2_ws/install/setup.bash ]; then
        source /ros2_ws/install/setup.bash
    fi
else
    echo "No ROS2 workspace sources found in /ros2_ws/src; skipping workspace build."
fi

if [ -n "${HOME:-}" ] && [ -f "$HOME/.bashrc" ]; then
    grep -Fxq "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc" || echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"

    if [ -f /ros2_ws/install/setup.bash ]; then
        grep -Fxq "source /ros2_ws/install/setup.bash" "$HOME/.bashrc" || echo "source /ros2_ws/install/setup.bash" >> "$HOME/.bashrc"
    fi
fi


exec "${@}"
