# rwifi_slam_ros2

ROS2 package for WiFi Localization and Mapping - A Wifi RTT approach to Simultaneous Localization and Mapping system for indoor robots.

## Overview

This ROS2 package provides the core SLAM functionality, integrating WiFi RTT measurements.

## Package Information

- **Package Name**: `rwifi_slam_ros2`
- **Version**: 0.11.0
- **Build Type**: `ament_python`
- **License**: Apache 2.0
- **Maintainer**: Rohan Dwivedi (myemail.rohan@gmail.com)

## Package Structure

```
rwifi_slam_ros2/
├── package.xml                     # ROS2 package manifest
├── setup.py                        # Python package setup
├── setup.cfg                       # Setuptools configuration
├── resource/
│   └── rwifi_slam_ros2             # Package marker file
└── rwifi_slam_ros2/                # Python package
    └── __init__.py                 # Package initialization
```

## Building

### Prerequisites

- ROS2 Jazzy
- Python 3.8+
- ROS2 workspace setup

### Build Instructions

From the ROS2 workspace root (`/ros2_ws`):

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build the package
colcon build --packages-select rwifi_slam_ros2 --symlink-install

# Source the workspace
source install/setup.bash
```

### Build with Dependencies

```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

### Dependencies

- **Core** (required): `tf2_ros`, `wifi_rtt_msgs`, `wifi_rtt_interface`, `python3-numpy`, `python3-scipy` (and `g2opy` via `setup.py`).
- **Launch/companion** (used by the default launch or optional workflows): `nav2_lifecycle_manager` (lifecycle transitions), `rviz2` (visualization), `teleop_twist_keyboard`, `rviz_imu_plugin`, `imu_tools`. These are not imported by this package but are listed in `package.xml` for the recommended setup.
