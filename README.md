# rwifi_slam

WiFi based Simultaneous Localization and Mapping (SLAM) system for indoor environments

## Overview

rwifi_slam is a WiFi based SLAM system for localization and mapping in indoor environments.
The project provides a complete development environment including Docker containers, Isaac Sim integration for simulation, 
and ROS2 packages for real-world deployment.

## Features

- **ROS2 Integration**: Built on ROS2 Jazzy for modern robotics development
- **Isaac Sim Support**: GPU-accelerated simulation environment using NVIDIA Isaac Sim 6.0
- **Docker Development**: Containerized development environment with GPU support
- **WiFi Localization**: WiFi-based positioning capabilities
- **LiDAR Mapping**: Depth mapping and SLAM functionality

## Prerequisites

### System Requirements

- **OS**: Linux (Ubuntu 22.04 recommended)
- **GPU**: NVIDIA GPU with CUDA support (required for Isaac Sim) (optional)
- **Docker**: Docker Engine with NVIDIA Container Toolkit
- **RAM**: Minimum 16GB (32GB recommended for Isaac Sim)
- **Disk Space**: At least 50GB free space

### Required Software

- Docker Engine 20.10+
- Docker Compose 2.0+
- NVIDIA Driver 535+ (for Isaac Sim 6.0)
- NVIDIA Container Toolkit

## Quick Start

### 1. Install Prerequisites

#### Install Docker
```bash
./scripts/install-docker.sh
```

#### Install NVIDIA Driver and Container Toolkit
```bash
./scripts/install_nvidia_isaac_sim.sh
sudo reboot  # Required after driver installation
```

### 2. Build Development Environment

#### Build Docker Image
```bash
# Build development image (default)
./scripts/build-docker.sh dev

# Build release image
./scripts/build-docker.sh rel
```

#### Build Isaac Sim Locally (Optional)
If you need to build Isaac Sim from source:
```bash
./scripts/build-isaac-sim-local.sh
```

### 3. Run Development Container

```bash
cd Docker
docker compose run dev
```

Or using VS Code Dev Containers:
- Open the project in VS Code
- Press `F1` → "Dev Containers: Reopen in Container"

## Project Structure

```
rwifi_slam/
├── Docker/                 # Docker configuration files
│   ├── dockerfile.dev      # Development container Dockerfile
│   ├── dockerfile.rel      # Release container Dockerfile
│   ├── compose.yaml        # Docker Compose configuration
│   └── entrypoint.sh       # Container entrypoint script
├── scripts/                # Utility scripts
│   ├── build-docker.sh     # Build Docker images (dev/rel)
│   ├── build-isaac-sim-local.sh
│   ├── install-docker.sh
│   └── install_nvidia_isaac_sim.sh
├── src/
│   ├── rwifi_slam_ros2/    # Main ROS2 package
│   └── simulation/        # Isaac Sim simulation code
│       └── isaac_sim_dev/ # Isaac Sim development environment
└── .devcontainer/         # VS Code Dev Container configuration
```

## Development

### Pre-commit (formatting & linting)

Install hooks to run **Black** (formatting), **isort** (import sorting), **flake8** (linting), **pydocstyle** (docstrings), and **yamllint** (YAML) before each commit:

```bash
pip install -r Docker/requirements-dev.txt
pre-commit install
```

Run on all files: `pre-commit run --all-files`. Configuration: `Docker/pyproject.toml`, `Docker/.flake8`.

### Building ROS2 Packages

Inside the development container:
```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Running Simulation

#### Using Isaac Sim
```bash
# From within the container
isaac-wildslam --robot nova_carter --scene warehouse

# Or headless mode
isaac-wildslam --robot nova_carter --scene warehouse --headless
```

#### Available Robots
- `nova_carter` (default)

#### Available Scenes
- `warehouse` (default)

### ROS2 Domain Configuration

The project uses ROS Domain ID 42 by default. To change it:
```bash
export ROS_DOMAIN_ID=<your_domain_id>
```

## Docker Configuration

The development container includes:
- ROS2 Jazzy Desktop
- Python 3 with development tools
- Pre-built ROS2 workspace
- Isaac Sim integration
- GPU support via NVIDIA Container Toolkit

See [Docker/README.md](Docker/README.md) for detailed Docker documentation.

## Scripts

Utility scripts are available in the `scripts/` directory:
- `build-docker.sh`: Build Docker images (dev or rel)
- `build-isaac-sim-local.sh`: Build Isaac Sim from source
- `install-docker.sh`: Install Docker Engine
- `install_nvidia_isaac_sim.sh`: Install NVIDIA drivers and Container Toolkit

See [scripts/README.md](scripts/README.md) for detailed script documentation.

## Simulation

The simulation environment uses NVIDIA Isaac Sim 6.0 for high-fidelity robot simulation. See [src/simulation/README.md](src/simulation/README.md) for simulation-specific documentation.

## ROS2 Package

The main ROS2 package `rwifi_slam_ros2` provides SLAM functionality. See [src/rwifi_slam_ros2/README.md](src/rwifi_slam_ros2/README.md) for package-specific documentation.

## Troubleshooting

### GPU Not Detected in Container
```bash
# Verify NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi

# Check Docker Compose GPU configuration
docker compose config
```

### Isaac Sim Build Issues
- Ensure you have sufficient disk space (50GB+)
- Check NVIDIA driver version: `nvidia-smi`
- Verify CUDA compatibility

### ROS2 Communication Issues
- Check ROS_DOMAIN_ID matches across all nodes
- Verify network configuration for multi-machine setups

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Maintainer

- **Rohan Dwivedi** - myemail.rohan@gmail.com

## Acknowledgments

- NVIDIA Isaac Sim team for the simulation platform
- ROS2 community for the robotics framework
- rWiFiSLAM: Effective WiFi Ranging based SLAM System in Ambient Environments
  Bo Wei1∗, Mingcen Gao2, Chengwen Luo3, Sen Wang4, Jin Zhang3
