# Simulation

NVIDIA Isaac Sim 6.0 environment for rwifi_slam: a warehouse scene, a Nova Carter robot
with IMU, RGB-D camera, and a Wi-Fi antenna, and five Wi-Fi access points. On first run it
precomputes the line-of-sight map that the RTT simulator in
[`wifi_rtt_interface`](../wifi_rtt_interface) uses.

This directory is not a ROS package; `COLCON_IGNORE` keeps colcon out of it.

## Layout

```
simulation/isaac_sim_dev/
├── main.py                       # Entry point: load config, spawn scene/robot/APs, run the sim loop
├── include/precompute_nlos.py    # Raycast-based NLOS map generation and visualisation
├── configs/
│   ├── isaac_sim_warehouse_config.yaml   # Robot, scene, and access point configuration
│   └── warehouse_nlos_map.pkl            # Cached NLOS map for the warehouse
├── assets/
│   ├── robots/nova_carter_ROS_rgbd.usda  # Nova Carter with RGB-D camera and Wi-Fi antenna
│   └── sensors/wifi_access_point/        # Access point model
└── isaac-sim-6.0/                # Local Isaac Sim build (git-ignored)
```

## Building Isaac Sim

Run this inside the development container:

```bash
/ros2_ws/scripts/build-isaac-sim-local.sh
```

The script clones Isaac Sim `v6.0.0-dev` into `isaac_sim_dev/isaac-sim-6.0/IsaacSim` and builds
it. Use `--fast-rebuild` or `--rebuild` to rebuild, and `--help` for all options.

## Running

The usual way is the SLAM launch file, which starts the simulator for you:

```bash
ros2 launch rwifi_slam_ros2 rwifi_slam.launch.py           # with the Isaac Sim GUI
ros2 launch rwifi_slam_ros2 rwifi_slam.launch.py headless:=true
```

To run the simulator on its own:

```bash
cd /ros2_ws/src/simulation/isaac_sim_dev
./isaac-sim-6.0/IsaacSim/_build/linux-x86_64/release/python.sh main.py \
  --config configs/isaac_sim_warehouse_config.yaml
```

| Option | Description |
|---|---|
| `--config PATH` | Simulation config (default: `configs/isaac_sim_warehouse_config.yaml`) |
| `--headless` | Run without the GUI |
| `--visualize-nlos` | Draw the NLOS map in the viewport (green = line of sight to every AP, red = to none) |

## Configuration

```yaml
robot:
  name: nova_carter
  usd_path: "/ros2_ws/src/simulation/isaac_sim_dev/assets/robots/nova_carter_ROS_rgbd.usda"
  spawn_point:
    position: [0.0, 0.0, 0.0]
    orientation: [1.0, 0.0, 0.0, 0.0]   # quaternion [w, x, y, z]
  antenna_height: 0.6                   # z of the NLOS grid; must match base_link -> chassis_wifi

scene:
  name: warehouse                       # also names the cached map: configs/<name>_nlos_map.pkl
  usd_path: "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
  resolution: 0.25                      # NLOS grid cell size (m)

access_points:
  - name: AP1
    position: [-4.5, -23.24, 3.2]
    orientation: [0.70, -0.70, 0.0, 0.0] # quaternion [w, x, y, z]
    mac_address: "00:00:00:00:00:01"
```

`usd_path` can be a local file, a path relative to `isaac_sim_dev/`, or a path on the Isaac
Sim asset server.

> [!IMPORTANT]
> The RTT simulator computes ranges from the robot's position in the `odom` frame, but
> access point positions are in world coordinates. The two agree only when the robot spawns
> at the origin with identity orientation `[1, 0, 0, 0]`. The shipped config uses
> `[0, 0, 0, 1]`, a 180° yaw in Isaac's `[w, x, y, z]` convention. This is tracked as
> [M4.2](../../docs/ROADMAP.md#m4-simulation-fidelity) in the roadmap.

## NLOS map

On start-up, `main.py` loads `configs/<scene name>_nlos_map.pkl` if it exists; otherwise it
computes the map and saves it. Computing it raycasts from every access point to every grid
cell, which takes a while.

> [!WARNING]
> The cached map is reused even if you change the access points or the scene. Delete the
> `.pkl` file after editing the config to regenerate it.

## Adding a robot or scene

1. Put the USD file under `assets/`, or point at an asset-server path.
2. Copy `configs/isaac_sim_warehouse_config.yaml` and edit `robot`, `scene`, and
   `access_points`.
3. Pass the new file with `--config`, or to the launch file with `config:=<path>`.

A new robot must publish the same topics and frames as the Nova Carter asset
(`/chassis/odom`, `/chassis/imu`, `/camera_depth/depth_pcl`, and TF for `chassis_imu` and
`chassis_wifi`), or the SLAM node's parameters must be changed to match.

## Troubleshooting

- **The simulator doesn't start.** Check that `isaac-sim-6.0/IsaacSim/_build/linux-x86_64/release/python.sh`
  exists, and that `nvidia-smi` works inside the container.
- **The RTT publisher waits for the NLOS map.** Isaac Sim is still computing it on first run.
  The publisher waits up to five minutes.
- **No GUI from the container.** Allow local X11 clients on the host with `xhost +local:`,
  and check that `DISPLAY` is set.
