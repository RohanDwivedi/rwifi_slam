#!/bin/bash

set -e

# ============================================================================
# Configuration
# ============================================================================

ROS_SETUP="/opt/ros/jazzy/setup.bash"
BUILD_DIR="/isaac-sim/IsaacSim/_build/linux-x86_64/release"
SOURCE_DIR="/isaac-sim/IsaacSim/source"
KIT_CACHE_DIR="/isaac-sim/.cache/packman/chk/kit-kernel/109.0.2+production.256998.bb24931b.gl.manylinux_2_35_x86_64.release"

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    source /opt/ros/jazzy/setup.bash
}

main "$@"
