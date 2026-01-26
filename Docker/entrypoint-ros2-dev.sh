#!/bin/bash

set -e

# ============================================================================
# Configuration
# ============================================================================

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP_PATHS=(
    "/ros2_ws/install/setup.bash"
    "/workspace/ros2_ws/install/setup.bash"
)

# ============================================================================
# Setup Functions
# ============================================================================

setup_ros() {
    if [ ! -f "$ROS_SETUP" ]; then
        echo "Warning: ROS2 setup file not found at $ROS_SETUP" >&2
        exit 1
    fi
    source "$ROS_SETUP"
}

setup_workspace() {
    for setup_file in "${WORKSPACE_SETUP_PATHS[@]}"; do
        if [ -f "$setup_file" ]; then
            source "$setup_file"
            return 0
        fi
    done
}

# ============================================================================
# Zenoh Router Management
# ============================================================================

start_zenoh_router() {
    echo "[INFO] Starting Zenoh router in background..."
    zenoh-router &
    ZENOH_ROUTER_PID=$!

    # Wait a moment for the router to start
    sleep 1

    # Check if the router is still running
    if ! kill -0 "$ZENOH_ROUTER_PID" 2>/dev/null; then
        echo "[WARNING] Zenoh router failed to start" >&2
        unset ZENOH_ROUTER_PID
    else
        echo "[INFO] Zenoh router started (PID: $ZENOH_ROUTER_PID)"

        # Set up cleanup trap to kill router on exit
        trap 'cleanup_zenoh_router' EXIT
    fi
}

cleanup_zenoh_router() {
    if [ -n "${ZENOH_ROUTER_PID:-}" ]; then
        echo "[INFO] Stopping Zenoh router..."
        kill "$ZENOH_ROUTER_PID" 2>/dev/null || true
        wait "$ZENOH_ROUTER_PID" 2>/dev/null || true
    fi
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    setup_ros
    setup_workspace

    # Start Zenoh router if RMW_IMPLEMENTATION is set to rmw_zenoh_cpp
    if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ]; then
        start_zenoh_router
    fi

    # Execute command passed to container
    # Use exec only if no Zenoh router is running, otherwise run normally so trap works
    if [ -n "${ZENOH_ROUTER_PID:-}" ]; then
        "$@"
    else
        exec "$@"
    fi
}

main "$@"
