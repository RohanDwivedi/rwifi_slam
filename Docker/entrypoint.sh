#!/bin/bash
# Container entrypoint: source ROS 2, build the workspace if it has not been built yet,
# then run the container command.
#
# The release image ships a pre-built workspace, so the build step is skipped there.
# The dev image mounts src/ at runtime and builds it on first start.
set -e

source /opt/ros/jazzy/setup.bash

WS=/ros2_ws

if [ -n "$(ls -A "$WS/src" 2>/dev/null)" ] && [ ! -f "$WS/install/setup.bash" ]; then
    echo "Building ROS 2 workspace in $WS..."
    cd "$WS"

    if [ -f src/rwifi_slam_ros2/requirements.txt ]; then
        python3 -m pip install --break-system-packages -q -r src/rwifi_slam_ros2/requirements.txt \
            || echo "WARNING: could not install src/rwifi_slam_ros2/requirements.txt" >&2
    fi

    if ! rosdep check --from-paths src --ignore-src -q >/dev/null 2>&1; then
        echo "Installing missing ROS dependencies..."
        { apt-get update -qq && rosdep install --from-paths src --ignore-src -y -q; } \
            || echo "WARNING: rosdep could not install every dependency" >&2
    fi

    if colcon build --symlink-install; then
        echo "Workspace build complete."
    else
        echo "WARNING: colcon build failed. Fix the errors above, then run: colcon build --symlink-install" >&2
    fi

    chown -R 1000:1000 "$WS/build" "$WS/install" "$WS/log" 2>/dev/null || true
fi

if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

if [ -n "${HOME:-}" ] && [ -w "$HOME" ]; then
    for line in "source /opt/ros/jazzy/setup.bash" "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"; do
        grep -Fxq "$line" "$HOME/.bashrc" 2>/dev/null || echo "$line" >> "$HOME/.bashrc"
    done
fi

exec "$@"
