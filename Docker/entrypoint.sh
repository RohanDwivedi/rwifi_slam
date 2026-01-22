#!/bin/bash
set -e

# Set up SSH for GitHub access
if [ -d ~/.ssh ]; then
    # Ensure proper permissions for SSH directory and keys
    chmod 700 ~/.ssh 2>/dev/null || true
    find ~/.ssh -type f -name "id_*" ! -name "*.pub" -exec chmod 600 {} \; 2>/dev/null || true
    find ~/.ssh -type f -name "*.pub" -exec chmod 644 {} \; 2>/dev/null || true
    
    # Add GitHub to known_hosts if not already present
    if [ ! -f ~/.ssh/known_hosts ] || ! grep -q "github.com" ~/.ssh/known_hosts 2>/dev/null; then
        mkdir -p ~/.ssh
        ssh-keyscan -t rsa,ecdsa,ed25519 github.com >> ~/.ssh/known_hosts 2>/dev/null || true
        chmod 644 ~/.ssh/known_hosts 2>/dev/null || true
    fi
fi

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
