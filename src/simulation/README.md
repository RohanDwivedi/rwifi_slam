# Simulation

This directory contains the Isaac Sim simulation environment for WiLD-SLAM.

## Overview

The simulation environment uses NVIDIA Isaac Sim 6.0 to provide high-fidelity robot simulation with GPU-accelerated physics and rendering. The simulation integrates with ROS2 for robot control and sensor data.

## Directory Structure

```
simulation/
├── isaac_sim_dev/          # Isaac Sim development environment
│   ├── main.py             # Main simulation entry point
│   ├── assets/             # USD assets (robots, scenes)
│   │   ├── robots/        # Robot USD files
│   │   └── scenes/        # Scene/environment USD files
│   ├── configs/            # Robot configuration files
│   └── isaac-sim-6.0/     # Isaac Sim source/build (gitignored)
└── COLCON_IGNORE          # Prevents colcon from building this directory
```

## Prerequisites

- NVIDIA GPU with CUDA support
- Isaac Sim 6.0 (built locally or mounted)
- ROS2 Jazzy (in development container)

## Running the Simulation

### Using the Alias (Recommended)

After building Isaac Sim, an alias is created:
```bash
isaac-wildslam --robot nova_carter --scene warehouse
```

### Direct Execution

From within the development container:
```bash
/ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0/IsaacSim/_build/linux-x86_64/release/python.sh \
  /ros2_ws/src/simulation/isaac_sim_dev/main.py \
  --robot nova_carter \
  --scene warehouse
```

### Command Line Arguments

- `--robot <name>`: Robot to simulate (default: `nova_carter`)
- `--scene <name>`: Scene/environment to load (default: `warehouse`)
- `--headless`: Run simulation without GUI (for headless servers)

**Examples:**
```bash
# Run with GUI
isaac-wildslam --robot nova_carter --scene warehouse

# Run headless
isaac-wildslam --robot nova_carter --scene warehouse --headless

# Use different robot/scene
isaac-wildslam --robot my_robot --scene my_scene
```

## Available Robots

### nova_carter
- **Asset**: `assets/robots/nova_carter.usda`
- **Config**: `configs/nova_carter.yaml`
- **Spawn Point**: Configured in YAML file

### Adding New Robots

1. **Create Robot USD Asset**
   - Place robot USD/USDA file in `assets/robots/<robot_name>.usda`
   - Ensure proper USD structure with joints, links, etc.

2. **Create Configuration File**
   - Create `configs/<robot_name>.yaml`
   - Define spawn point:
     ```yaml
     spawn_point:
       position:
         x: 0.0
         y: 0.0
         z: 0.0
       rotation:
         roll: 0.0
         pitch: 0.0
         yaw: 0.0
     ```

3. **Test**
   ```bash
   isaac-wildslam --robot <robot_name> --scene warehouse
   ```

## Available Scenes

### warehouse
- **Asset**: `assets/scenes/warehouse.usda`
- Default warehouse environment

### Adding New Scenes

1. **Create Scene USD Asset**
   - Place scene USD/USDA file in `assets/scenes/<scene_name>.usda`
   - Ensure scene has proper structure (ground plane, obstacles, etc.)

2. **Test**
   ```bash
   isaac-wildslam --robot nova_carter --scene <scene_name>
   ```

## Configuration Files

Robot configuration files are YAML files located in `configs/`. They define:
- **Spawn Point**: Initial position and orientation of the robot
  - `position`: x, y, z coordinates (meters)
  - `rotation`: roll, pitch, yaw angles (degrees)

**Example:**
```yaml
spawn_point:
  position:
    x: 0.0
    y: 0.0
    z: 0.5
  rotation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

## Isaac Sim Integration

### Isaac Sim Location

Isaac Sim is expected at:
```
/ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0/
```

This directory is:
- Mounted as a volume in Docker containers
- Gitignored (not committed to repository)
- Built locally using `build-isaac-sim-local.sh`

### Building Isaac Sim

See [scripts/README.md](../../scripts/README.md) for building instructions:
```bash
./scripts/build-isaac-sim-local.sh
```

## ROS2 Integration

The simulation can integrate with ROS2 for:
- Robot control (joint commands, velocity commands)
- Sensor data publishing (LiDAR, cameras, IMU)
- TF transforms
- ROS2 topics and services

**Note**: ROS2 integration is configured separately in the ROS2 packages.

## Troubleshooting

### Simulation Won't Start
- Check Isaac Sim build: `ls isaac-sim-6.0/IsaacSim/_build`
- Verify GPU access: `nvidia-smi`
- Check USD assets exist: `ls assets/robots/` and `ls assets/scenes/`

### Robot Not Visible
- Verify robot USD file loads correctly
- Check spawn point configuration
- Ensure robot is within scene bounds

### Scene Not Loading
- Verify scene USD file exists
- Check USD file structure
- Review console output for errors

### Headless Mode Issues
- Ensure `--headless` flag is set
- Check DISPLAY variable (should not be set in headless)
- Verify GPU is accessible in container

### Extension Errors
- Check Isaac Sim version compatibility
- Verify extensions are available in Isaac Sim 6.0
- Review extension enable logs

## Development

### Adding Features

1. **Modify `main.py`**: Add new functionality to simulation loop
2. **Create Assets**: Add USD files to `assets/` directories
3. **Update Configs**: Add configuration files as needed
4. **Test**: Run simulation and verify behavior

### Debugging

- Enable debug output in `main.py`
- Use Isaac Sim's built-in debug visualization
- Check USD stage structure: `print(stage.ExportToString())`
- Review Isaac Sim logs

## Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [USD Documentation](https://openusd.org/)
- [Omniverse Python API](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/)
