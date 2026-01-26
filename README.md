# WiLD-SLAM

WiFi localisation with Lidar fusion SLAM - A hybrid SLAM system that combines WiFi-based localization with Lidar-based mapping and localization for robust indoor navigation.
The system fuses WiFi signal strength measurements with Lidar point cloud data to provide accurate pose estimation and mapping in GPS-denied environments.

## Overview

WiLD-SLAM is a comprehensive robotics project that combines:
- **ROS2 Jazzy** with Zenoh middleware for distributed robotics communication
- **Isaac Sim 6.0** for high-fidelity physics simulation
- **Docker-based development environment** for consistent builds across platforms

The project provides a complete development and simulation environment for SLAM algorithms, with support for both native Linux and WSL2 platforms.

## Project Structure

```
.
├── Docker/                    # Docker configuration and build scripts
│   ├── dockerfile             # Development container (ROS2 Jazzy)
│   ├── dockerfile.isaac-sim-6.0  # Isaac Sim 6.0 container
│   ├── compose.yaml           # Docker Compose configuration
│   └── entrypoint*.sh         # Container entrypoint scripts
├── src/
│   ├── simulation/
│   │   └── isaac_sim/         # Isaac Sim simulation assets and scripts
│   │       ├── assets/       # USD scene and robot files
│   │       ├── configs/       # Robot configuration files
│   │       ├── docs/         # Isaac Sim documentation
│   │       └── main.py        # Isaac Sim entrypoint script
│   └── wild_slam_ros2/       # ROS2 package for WiLD-SLAM
│       └── wild_slam_ros2/   # Package source code
├── .github/workflows/         # CI/CD workflows
├── .devcontainer/            # VS Code dev container configuration
└── pyproject.toml            # Python tooling configuration

```

## Documentation

### Main Documentation
- **[Docker Documentation](Docker/README.md)** - Docker setup, build instructions, and container usage
- **[Isaac Sim Documentation](src/simulation/isaac_sim/docs/README.md)** - Simulation setup, asset management, and platform-specific notes
- **[ROS2 Package Documentation](src/wild_slam_ros2/README.md)** - ROS2 package structure and usage

## Quick Start

### Prerequisites

- Docker and Docker Compose
- NVIDIA GPU with drivers (for Isaac Sim)
- NVIDIA Container Toolkit (for GPU passthrough)

### Development Environment

1. **Start the development container:**
   ```bash
   docker compose -f Docker/compose.yaml up dev
   ```

2. **Build the ROS2 workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd /ros2_ws
   colcon build
   source install/setup.bash
   ```

### Isaac Sim Simulation

1. **Start Isaac Sim container:**
   ```bash
   docker compose -f Docker/compose.yaml up isaac-sim
   ```

2. **Run a simulation:**
   ```bash
   python main.py --scene test_scene --robot test_robot
   ```

3. **Access WebRTC viewer (WSL2/headless mode):**
   ```
   http://localhost:8211/streaming/webrtc-client
   ```

## Features

- **Cross-platform support**: Automatic WSL2 detection with headless mode and WebRTC streaming
- **ROS2 integration**: Full ROS2 Jazzy support with Zenoh middleware
- **Isaac Sim 6.0**: Built from source with GPU acceleration
- **Docker-based**: Consistent development environment across platforms
- **CI/CD**: Automated builds, linting, and testing

## Platform Support

### Native Linux
- Full GUI support for Isaac Sim
- Direct GPU passthrough
- Standard ROS2 tools

### WSL2 (Windows Subsystem for Linux)
- Automatic headless mode detection
- WebRTC streaming for visualization
- GPU passthrough support (requires NVIDIA drivers on Windows)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

1. Follow the code style guidelines (Black, isort, flake8)
2. Run linters before committing: `black src/ && isort src/ && flake8 src/`
3. Ensure all tests pass
4. Update documentation as needed

## CI/CD

The project includes automated workflows for:
- **Linting**: Code quality checks (Black, isort, flake8, mypy, pydocstyle, yamllint)
- **Docker builds**: Automated container builds and releases
- **Isaac Sim builds**: Isaac Sim container builds (requires significant resources)

See `.github/workflows/` for details.

## Support

For issues and questions, please open an issue on the project repository.
