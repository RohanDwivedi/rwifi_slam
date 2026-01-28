# Docker Configuration

This directory contains Docker configuration files for the rwifi_nvblox_slam  development environment.

## Files

- **`dockerfile.dev`**: Dockerfile for building the development container
- **`dockerfile.rel`**: Dockerfile for building the release container (depends on dev image)
- **`compose.yaml`**: Docker Compose configuration for running containers
- **`entrypoint.sh`**: Entrypoint script that sources ROS2 environment
- **`requirements-dev.txt`**: Python development dependencies

## Building the Image

### Using the Build Script
```bash
cd ..
# Build development image
./scripts/build-docker.sh dev

# Build release image
./scripts/build-docker.sh rel
```

### Manual Build
```bash
cd ..
# Build development image
docker build -f Docker/dockerfile.dev -t rwifi_nvblox_slam-dev .

# Build release image (requires dev image first)
docker build -f Docker/dockerfile.rel -t ghcr.io/rohandwivedi/rwifi_nvblox_slam:latest .
```

## Running the Container

### Using Docker Compose (Recommended)
```bash
cd Docker
docker compose run dev
```

### Using Docker Run
```bash
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY=$DISPLAY \
  -v $(pwd)/../src/simulation/isaac_sim_dev/isaac-sim-6.0:/ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0:rw \
  -v $(pwd)/../.isaac-sim-cache:/workspace/.isaac-sim-cache:rw \
  ghcr.io/rohandwivedi/rwifi_nvblox_slam:latest
```

## Container Features

### Pre-installed Software
- ROS2 Jazzy Desktop
- Python 3 with pip
- Development tools (build-essential, cmake, git)
- ROS2 workspace pre-built at `/ros2_ws`
- Python development dependencies

### Environment Variables
- `ROS_DOMAIN_ID=42`: ROS2 domain ID
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`: ROS2 middleware implementation
- `DISPLAY`: X11 display for GUI applications

### Volume Mounts
- **Isaac Sim**: Mounted at `/ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0`
- **Cache**: Isaac Sim cache at `/workspace/.isaac-sim-cache`

### GPU Support
The container is configured with:
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=all`
- GPU access via NVIDIA Container Toolkit

## Development Workflow

### Inside the Container

1. **Source ROS2 Environment**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /ros2_ws/install/setup.bash
   ```

2. **Build ROS2 Packages**
   ```bash
   cd /ros2_ws
   colcon build --symlink-install
   ```

3. **Run Isaac Sim**
   ```bash
   isaac-wildslam --robot nova_carter --scene warehouse
   ```

### VS Code Dev Containers

The project includes VS Code Dev Container configuration (`.devcontainer/devcontainer.json`):
- Automatically opens in the Docker container
- Pre-configured extensions (Python, ROS2, YAML, etc.)
- Pre-configured settings (formatting, linting)

To use:
1. Open project in VS Code
2. Press `F1` → "Dev Containers: Reopen in Container"

## Image Registry

The image is published to GitHub Container Registry:
- **Image**: `ghcr.io/rohandwivedi/rwifi_nvblox_slam:latest`
- **Build**: Automated via GitHub Actions (`.github/workflows/docker-build-release.yml`)

### Pulling Pre-built Image
```bash
docker pull ghcr.io/rohandwivedi/rwifi_nvblox_slam:latest
```

## Troubleshooting

### Container Won't Start
- Check Docker daemon: `docker ps`
- Verify NVIDIA Container Toolkit: `nvidia-ctk --version`
- Check GPU access: `docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi`

### ROS2 Not Found
- Ensure entrypoint script is executable: `chmod +x Docker/entrypoint.sh`
- Check ROS2 installation: `ls /opt/ros/jazzy/`

### Isaac Sim Not Found
- Verify volume mount exists: `ls /ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0`
- Check Isaac Sim build: `ls /ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0/IsaacSim/_build`

### Display Issues
- Set DISPLAY: `export DISPLAY=:0`
- For X11 forwarding: `xhost +local:docker`
- Check X11 socket: `ls /tmp/.X11-unix`

## Customization

### Changing ROS Domain ID
Edit `compose.yaml`:
```yaml
environment:
  - ROS_DOMAIN_ID=42  # Change this value
```

### Adding Environment Variables
Edit `compose.yaml`:
```yaml
environment:
  - ROS_DOMAIN_ID=42
  - YOUR_VAR=value
```

### Modifying Container User
Edit `compose.yaml`:
```yaml
user: "1000:1000"  # Change UID:GID
```

## Build Cache

The build script uses Docker BuildKit cache for faster rebuilds:
- Cache location: `.buildx-cache/` (in project root)
- Cache type: Local filesystem cache
- Cache mode: Maximum (saves all layers)

To clear cache:
```bash
rm -rf .buildx-cache
```
