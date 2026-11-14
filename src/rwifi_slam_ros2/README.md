# rwifi_slam_ros2

ROS 2 lifecycle node for Wi-Fi RTT SLAM. It fuses wheel odometry and IMU in an EKF, adds a
keyframe on each RTT scan, detects loop closures from range-vector similarity, optimises a
g2o pose graph, and builds an OctoMap from depth clouds.

See the [top-level README](../../README.md) for an overview, and the
[roadmap](../../docs/ROADMAP.md) for known issues.

## Code layout

| Module | Responsibility |
|---|---|
| [`rwifi_slam_ros2_node.py`](rwifi_slam_ros2/rwifi_slam_ros2_node.py) | Lifecycle node: subscriptions, keyframing, publishing, TF |
| [`include/front_end/pose_estimator.py`](rwifi_slam_ros2/include/front_end/pose_estimator.py) | 8-state EKF `[x, y, vx, vy, yaw, b_ax, b_ay, b_gz]`; IMU predict, odometry update |
| [`include/front_end/loop_closure.py`](rwifi_slam_ros2/include/front_end/loop_closure.py) | Range-vector matching with threshold, ratio test, and near-neighbour exclusion |
| [`include/backend/pose_graph.py`](rwifi_slam_ros2/include/backend/pose_graph.py) | g2o SE(3) pose graph with odometry and robust (DCS) loop-closure edges |
| [`include/backend/octomap.py`](rwifi_slam_ros2/include/backend/octomap.py) | OctoMap insertion and 2D occupancy-grid projection |
| [`common/`](rwifi_slam_ros2/common) | Parameter declarations, QoS profiles, SE(3) and quaternion helpers |

## Interface

### Subscribed topics

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/chassis/odom` | `nav_msgs/Odometry` | best effort | EKF update |
| `/chassis/imu` | `sensor_msgs/Imu` | best effort | EKF predict; transformed into `base_frame` via TF |
| `/wifi_rtt_scan` | `wifi_rtt_msgs/APRangeObservation` | best effort | Keyframes and loop closure |
| `/camera_depth/depth_pcl` | `sensor_msgs/PointCloud2` | best effort, depth 1 | Mapping; processed on a worker thread |

### Published topics

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/odom_filtered` | `nav_msgs/Odometry` | reliable, transient local | EKF state in `odom_frame` |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | reliable | Latest keyframe pose in `map_frame` |
| `/trajectory` | `nav_msgs/Path` | reliable, transient local | Last `trajectory_keep_history` keyframes |
| `/map` | `nav_msgs/OccupancyGrid` | reliable, transient local | 2D slice of the OctoMap |

Topic names are fixed for now; remap them at launch if needed.

### TF

| Transform | Role |
|---|---|
| `map_frame → odom_frame` | Broadcast by this node |
| `base_frame → imu_frame` | Required (looked up once and cached) |
| `map_frame → <depth cloud frame>` | Required for mapping; the frame comes from the cloud's header |

## Parameters

The values below are those in [`config/rwifi_slam.parameters.yaml`](config/rwifi_slam.parameters.yaml),
which the launch file loads. The coded defaults in
[`common/parameters.py`](rwifi_slam_ros2/common/parameters.py) differ for a few entries.

### Frames and keyframes

| Parameter | Value | Description |
|---|---|---|
| `map_frame` | `map` | Global frame of the pose graph and map |
| `odom_frame` | `odom` | Odometry frame of the EKF |
| `base_frame` | `base_link` | Robot base frame |
| `imu_frame` | `chassis_imu` | IMU frame, transformed into `base_frame` |
| `depth_frame` | `camera_pseudo_depth` | Declared but unused; the cloud header's frame is used |
| `wifi_frame` | `chassis_wifi` | Declared but unused |
| `distance_interval` | `0.1` | Minimum travel (m) between keyframes |
| `yaw_interval` | `0.0174533` | Minimum rotation (rad) between keyframes |
| `trajectory_keep_history` | `500` | Keyframes in `/trajectory` (0 = all) |

### `pose_estimator.*`

| Parameter | Value | Description |
|---|---|---|
| `initial_covar_mat_diag` | `[0.01, 0.01, 0.01, 0.01, 0.27, 0.04, 0.04, 0.0025]` | Initial covariance of `[x, y, vx, vy, yaw, b_ax, b_ay, b_gz]` |
| `imu.gyro_noise` | `1.0` | Gyroscope white noise (rad/s/√Hz) |
| `imu.accel_noise` | `0.5` | Accelerometer white noise (m/s²/√Hz) |
| `imu.gyro_random_walk` | `0.0` | Gyroscope bias random walk (rad/s²/√Hz) |
| `imu.accel_random_walk` | `0.0` | Accelerometer bias random walk (m/s³/√Hz) |
| `odometry.enabled` | `true` | Fuse wheel odometry |
| `odometry.meas_noise_covar_diag` | `[0.01, 0.01, 0.01, 0.01, 0.05]` | `[x, y, vx, vy, yaw]` noise, used when the message has no valid covariance |

### `loop_closure.*`

| Parameter | Value | Description |
|---|---|---|
| `distance_method` | `euclidean` | Range-vector distance (only `euclidean`) |
| `clustering_threshold` | `1.0` | Maximum distance (m) for a match |
| `use_ratio_test` | `true` | Require the best match to beat the runner-up |
| `ratio_threshold` | `0.8` | Maximum best / runner-up ratio |
| `exclude_near_neighbors` | `true` | Ignore the most recent keyframes |
| `near_neighbor_threshold` | `20` | Number of recent keyframes to ignore |

### `pose_graph.*`

| Parameter | Value | Description |
|---|---|---|
| `robustness_threshold` | `0.5` | DCS kernel parameter for loop-closure edges |
| `optimization_iterations` | `10` | Levenberg–Marquardt iterations per loop closure |
| `verbose` | `false` | g2o optimiser output |

### `octomap.*`

| Parameter | Value | Description |
|---|---|---|
| `enabled` | `true` | Insert depth clouds and publish `/map` |
| `resolution` | `0.1` | Voxel and grid-cell size (m) |
| `height_min`, `height_max` | `0.0`, `0.3` | The 2D grid samples the slice at their midpoint |
| `margin_m` | `1.0` | Margin (m) around observed space in the 2D grid |
| `max_range` | `10.0` | Maximum ray length (m); `-1` for unlimited |
| `use_decay` | `false` | Decay old occupancy before each insert (dynamic scenes) |
| `logodd_decay_value` | `-0.2` | Log-odds decay per insert when `use_decay` is true |
| `downsample_step` | `16` | Insert every Nth point |
| `depth_frame_interval` | `4` | Process every Nth depth message |
| `map_publish_interval_ms` | `5000.0` | Minimum time between `/map` publishes (0 = every insert) |

## Launch files

| File | Starts |
|---|---|
| [`rwifi_slam.launch.py`](launch/rwifi_slam.launch.py) | `wifi_bringup.launch.py`, the SLAM node, `nav2_lifecycle_manager`, and RViz |
| [`wifi_bringup.launch.py`](launch/wifi_bringup.launch.py) | Isaac Sim (optional), the RTT publisher, and static TFs for `odom → base_link` and `base_link → chassis_wifi` |

```bash
ros2 launch rwifi_slam_ros2 rwifi_slam.launch.py headless:=true use_rviz:=false
```

`rwifi_slam.launch.py` arguments: `backend`, `simulation_type`, `headless`, `launch_simulation`,
`autostart`, `use_rviz`, `log_level`. `wifi_bringup.launch.py` also accepts `config` (Isaac Sim
YAML), `nlos_map_file`, `wifi_rtt_config_path`, `visualize_nlos`, and `antenna_height`.

To run the node on its own, configure and activate it yourself:

```bash
ros2 run rwifi_slam_ros2 rwifi_slam_node --ros-args \
  --params-file "$(ros2 pkg prefix rwifi_slam_ros2)/share/rwifi_slam_ros2/config/rwifi_slam.parameters.yaml"
ros2 lifecycle set /rwifi_slam_node configure
ros2 lifecycle set /rwifi_slam_node activate
```

## Dependencies

ROS dependencies are declared in [`package.xml`](package.xml) and installed by `rosdep`.
`g2opy` and `pyoctomap` have no rosdep keys, so install them with pip:

```bash
python3 -m pip install --break-system-packages -r src/rwifi_slam_ros2/requirements.txt
```

CI tests both ends of the ranges in [`requirements.txt`](requirements.txt). `pyoctomap` is held
below 1.0, because 1.x removed the call that `octomap.use_decay` relies on.

## Tests

```bash
colcon test --packages-select rwifi_slam_ros2 --event-handlers console_direct+
```

The tests in [`test/`](test) are plain pytest unit tests and need no running ROS graph.
