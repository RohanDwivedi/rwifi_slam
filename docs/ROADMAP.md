# Roadmap

rwifi_slam is a research prototype. The full pipeline runs in Isaac Sim, but the
pose-graph back end has known correctness issues, and there is no quantitative
evaluation yet. This page is the plan for getting it to a state where results
can be trusted. It is ordered by dependency: each milestone makes the next one
measurable.

Items reference code as `path:line` at the time of writing. When you pick
something up, open an issue (or comment on the linked one) so work isn't
duplicated.

| Milestone | Goal | Status |
|---|---|---|
| [M0](#m0-project-infrastructure) | CI, tests, docs, releases | Done |
| [M1](#m1-slam-correctness) | Pose-graph corrections that persist | Not started |
| [M2](#m2-evaluation) | Measure SLAM against ground truth | Not started |
| [M3](#m3-loop-closure-detection) | Loop closures that fire on real revisits | Not started |
| [M4](#m4-simulation-fidelity) | A simulator that produces realistic RTT data | Not started |
| [M5](#m5-estimator-mapping-and-node) | EKF, mapping, and lifecycle polish | Not started |
| [M6](#m6-real-hardware) | Run on a real robot | Not started |

---

## M0: Project infrastructure

- [x] CI runs pre-commit (Black, isort, flake8, pydocstyle, yamllint, actionlint) and a
      colcon build + test in a `ros:jazzy-ros-base` container. Closes #15.
- [x] Unit tests for transforms, SE(3) helpers, pose graph, loop closure, OctoMap, EKF,
      and the NLOS map schema.
- [x] Python dependencies install cleanly on Jazzy: pip no longer tries to replace
      Debian-managed packages, and `g2opy`/`pyoctomap` are pinned to versions whose API
      the code uses.
- [x] Release image builds the workspace at image build time; releases are tag-driven;
      Docker images are build-checked on PRs that touch them. Addresses #24.
- [x] Documentation rewritten for the actual layout and commands. Addresses #19.

Follow-ups:

- [ ] **Port to `g2opy` ≥ 2.3.** 2.3.0 changed the `Isometry3d` bindings and
      `set_translation` rejects the 1-D arrays passed in
      [`pose_graph.py`](../src/rwifi_slam_ros2/rwifi_slam_ros2/include/backend/pose_graph.py).
      Moot if M1.3 moves the graph to SE(2).
- [ ] **Port to `pyoctomap` ≥ 1.0.** 1.x removed `OcTree.decayAndInsertPointCloud`, used when
      `octomap.use_decay` is true.
- [ ] Protect `main`: require the `CI` workflow to pass before merging.
- [ ] Give every package the same version if the project is released through bloom, which
      requires it.

---

## M1: SLAM correctness

Today a loop-closure correction is lost one keyframe after it is applied, so the SLAM
output is close to raw odometry. These items are small and touch two files, and they
unblock everything after them.

### M1.1 Keep the odom and map frames separate in the pose graph

**Problem.** New keyframes are initialised from the raw EKF odometry pose
(`rwifi_slam_ros2_node.py:361`), but the odometry edge is measured against the previous
node's *optimised* pose (`pose_graph.py:130-134`). After an optimisation moves node *N*,
the edge *N → N+1* records the whole map/odom offset as if the robot had driven it,
and node *N+1* is back in odometry coordinates. `has_robot_moved` also compares an odom
pose against an optimised one (`rwifi_slam_ros2_node.py:354`).

**Recommendation.** Store each keyframe's raw odometry pose alongside the optimised
vertex. Then:

- odometry edge measurement: `Z = inv(odom[k-1]) @ odom[k]`
- initial estimate of the new vertex: `x[k] = x[k-1] @ Z`
- keyframe selection: compare `odom[now]` against `odom[k]`

**Done when** a unit test shows that after a loop closure shifts node *N*, the next node
keeps the shift and its edge equals the odometry delta.

### M1.2 Compute map → odom from the keyframe's own odometry pose

**Problem.** `rwifi_slam_ros2_node.py:426-429` computes
`T_map_odom = x[last] @ inv(odom[now])`, and it runs on every odometry message. Between
keyframes the robot therefore appears frozen in the map frame at the last keyframe. Depth
clouds are projected with that stale pose: at 1 rad/s and 5 Hz RTT, that's up to ~11° of
heading error, or ~1 m at 5 m range.

**Recommendation.** `T_map_odom = x[k] @ inv(odom[k])`, using the same keyframe *k* for both
terms.

### M1.3 Use a planar SE(2) pose graph

**Problem.** `adjoint_SE3` (`common/utils.py:26-42`) orders the tangent space
rotation-first, while ROS and g2o covariances are translation-first.
`relative_pose_covariance` (`utils.py:129`) also matches neither the left- nor the
right-perturbation convention. As a result, translation variance leaks into the edge's yaw
variance, scaled by the squared distance from the odom origin: 20 m from the start, it adds
~4 rad², leaving odometry edges with almost no heading information.

**Recommendation.** The robot is planar. Replace `VertexSE3`/`EdgeSE3` with
`VertexSE2`/`EdgeSE2`, and derive each edge's covariance from the EKF motion between
keyframes, not by differencing two absolute covariances (those are strongly correlated).
This also removes the near-singular z/roll/pitch blocks that are currently regularised to
an information of 1e8.

### M1.4 Constrain only translation in loop closures

**Problem.** Loop-closure edges use an identity measurement and identity information
(`pose_graph.py:155`), which pulls the two headings together. RTT ranges carry no heading
information.

**Recommendation.** Zero the rotational information. Size the translational information
from data (M2/M3), not as a fixed 1 m standard deviation.

### M1.5 Key RTT observations by MAC address

**Problem.** Range vectors are compared by position in the list. A scan with a different
number of access points, such as an empty scan published when the simulator's TF lookup
fails (`simulation_backend.py:219-222`) or any real-world AP dropout, makes
`loop_closure.py:130-132` raise. The exception escapes the executor and stops the node.

**Recommendation.** Store `{mac: range}` per keyframe and compare on the APs two keyframes
have in common, with a minimum overlap. Skip empty scans.

### M1.6 Keep optimisation off the sensor path

**Problem.** `Graph.optimize()` runs inside the Wi-Fi callback, which shares a callback group
with the IMU and odometry callbacks, so the EKF stalls while the graph optimises.

**Recommendation.** Run optimisation in its own callback group or worker thread, and swap
results in under a lock.

---

## M2: Evaluation

Nothing can be tuned until SLAM can be compared with ground truth. As far as we know,
Isaac Sim's odometry is computed from the true chassis pose, so it doesn't drift and there
is currently nothing for SLAM to correct.

- [ ] **M2.1** Publish the ground-truth `world → base_link` pose from Isaac Sim on its own
      topic.
- [ ] **M2.2** Add configurable drift and noise to wheel odometry (scale error, heading bias,
      and random walk).
- [ ] **M2.3** Record benchmark bags with several loops through the warehouse (`/chassis/*`,
      `/wifi_rtt_scan`, ground truth).
- [ ] **M2.4** Add an evaluation script (for example with [evo](https://github.com/MichaelGrupp/evo))
      that reports ATE/RPE for wheel odometry, the EKF, and SLAM. Run it in CI on a short bag
      to catch regressions.

---

## M3: Loop-closure detection

The approach follows [rWiFiSLAM](https://arxiv.org/abs/2212.08418): range vectors are
treated as fingerprints, and similar ones are clustered into loop closures.

- [ ] **M3.1 Average scans per keyframe.** Under the simulator's noise model
      (`0.4·t(3) + 0.2` m per AP), two single scans from the *same* spot differ by a median
      of 1.66 m, while `clustering_threshold` is 1.0. Only ~13% of true revisits pass.
      Use the median of several consecutive 5 Hz scans per keyframe; the median also handles
      the heavy-tailed noise.
- [ ] **M3.2 Make the ratio test neighbourhood-aware.** Keyframes are a few centimetres apart,
      so the runner-up match is almost always the keyframe next to the best one, from the
      same earlier pass. The ratio is then ≈ 1 and real revisits are rejected
      (`loop_closure.py:97-104`). Compare the best candidate against the best one *outside*
      ±k nodes of it.
- [ ] **M3.3 Choose thresholds from data.** Use the M2 ground truth to set
      `clustering_threshold`, `ratio_threshold` and the loop-closure information.
- [ ] **M3.4 Document the ratio test's contract.** It returns at most one candidate, while the
      threshold path can return several. Either document this or allow several candidates.

---

## M4: Simulation fidelity

- [ ] **M4.1 Remove duplicate TF publishers.** `wifi_bringup.launch.py` publishes a static
      identity `odom → base_link` (line 160) and a static `base_link → chassis_wifi` (line 191),
      but the robot USD already has Isaac Sim publish `chassis_wifi`. Isaac's Nova Carter
      graph normally publishes `odom → base_link` too. Check the tree with
      `ros2 run tf2_tools view_frames` and keep one publisher per transform.
- [ ] **M4.2 Simulate ranges from the world-frame pose.** `simulation_backend.py:219` uses the
      robot position in `odom`, but the access point positions are in world coordinates.
      That is only correct when the robot spawns at the world origin with identity
      orientation. The spawn quaternion `[0, 0, 0, 1]`
      (`configs/isaac_sim_warehouse_config.yaml:7`) is a 180° yaw in Isaac's `wxyz`
      convention.
- [ ] **M4.3 Spatially correlated NLOS bias.** The bias is drawn independently for every
      0.25 m cell (`precompute_nlos.py:162`), so it changes by ~1 m per blocking object
      between neighbouring cells. Real multipath bias varies smoothly. Use a correlated
      field, for example by smoothing the per-cell draws or sampling a Gaussian process.
- [ ] **M4.4 Invalidate the cached NLOS map** when the access point configuration changes:
      `main.py:133-135` reuses any existing pickle. Store a hash of the config with the map.
- [ ] **M4.5 Use a released robot asset.** The robot USD references NVIDIA's
      `omniverse-content-staging` bucket, which can change or disappear.
- [ ] **M4.6 Use one clock.** Isaac Sim stamps messages with simulation time, but the RTT
      publisher (`wifi_bringup.launch.py:138`), SLAM node and RViz all use wall time. Set
      `use_sim_time` everywhere and publish `/clock`.
- [ ] **M4.7 Small fixes:** `simulation_isaac_sim.grid_cell_resolution` is read but unused, and
      the backend publishes an empty observation when its TF lookup fails instead of
      skipping the scan.

---

## M5: Estimator, mapping and node

- [ ] **M5.1 EKF.**
  - The Jacobian is evaluated after the state has already been propagated
    (`pose_estimator.py:396-398`).
  - IMU gaps over 1 s skip prediction without re-initialising the filter.
  - The defaults (`gyro_noise: 1.0` rad/s/√Hz, zero bias random walk) make the filter
    effectively ignore the gyroscope.
  - Consider fusing odometry velocities instead of the absolute odometry pose, which today
    makes the EKF track wheel odometry almost exactly.
- [ ] **M5.2 IMU orientation.** `transform_imu_with_tf` composes the orientation in the wrong
      order (`transforms.py:127`). The EKF doesn't use the orientation yet, so this is latent.
- [ ] **M5.3 OctoMap.**
  - Project the whole `[height_min, height_max]` band, not one voxel-thick slice at
    mid-height.
  - Account for pruned leaves that are larger than a grid cell.
  - Build the grid inside the lock but publish it outside.
  - Rebuild or re-anchor the map after pose-graph optimisation.
- [ ] **M5.4 Lifecycle.**
  - Don't publish, or broadcast TF, while the node is inactive.
  - Stop the depth worker on shutdown.
  - Expose a service to reset the graph.
- [ ] **M5.5 Interface.**
  - Make topic names parameters, with defaults matching today's names.
  - Either use the `depth_frame` and `wifi_frame` parameters or remove them. Note that
    `camera_pseudo_depth` doesn't match the actual frame `Camera_Pseudo_Depth`.

---

## M6: Real hardware

- [ ] **M6.1** Android Wi-Fi RTT backend for `wifi_rtt_interface` (#11, #26).
- [ ] **M6.2** Hardware platform and sensor setup (#10).
- [ ] **M6.3** Validate the simulated RTT error model against a real dataset.
