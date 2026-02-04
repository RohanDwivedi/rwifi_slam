# rwifi_slam_ros2

ROS2 package for WiFi Localization and Mapping - A Wifi RTT approach to Simultaneous Localization and Mapping system for indoor robots.

## Overview

This ROS2 package provides the core SLAM functionality, integrating WiFi RTT measurements.

## Package Information

- **Package Name**: `rwifi_slam_ros2`
- **Version**: 0.2.0
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
