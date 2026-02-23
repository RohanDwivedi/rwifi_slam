# rwifi_slam_ros2 — Recommendations

This document summarizes recommendations from a code review of the Range Wi-Fi SLAM ROS 2 package. Items are grouped by priority and type.

---

## 1. High priority

### 1.1 Make topic names configurable

**Current state:** Subscription and publication topic names are hardcoded in `rwifi_slam_ros2_node.py` (e.g. `/chassis/odom`, `/chassis/imu`, `/wifi_rtt_scan`, `/camera_depth/depth_pcl`, `/pose`, `/trajectory`, `/odom_filtered`, `/map`).

**Recommendation:**

- Declare string parameters for all input and output topics in `parameters.py` (e.g. `odom_topic`, `imu_topic`, `wifi_rtt_topic`, `depth_topic`, `pose_topic`, `trajectory_topic`, `odom_filtered_topic`, `map_topic`).
- Use these parameters in `_init_subscribers()` and `_init_publishers()` instead of literal strings.
- Add default values in `rwifi_slam.parameters.yaml` matching current behavior.

**Benefit:** Enables remapping and reuse across robots and namespaces without code changes.

---

### 1.2 Guard publishing on lifecycle state

**Current state:** Callbacks publish as soon as messages arrive; if the node is configured but not yet activated, lifecycle publishers may not deliver messages as expected.

**Recommendation:**

- Either document in the package README that the launch flow must activate the node before publishing meaningful data, **or**
- Guard all `publish()` calls with a check that the node is in the Active state (e.g. `self.get_current_state().id == State.PRIMARY_STATE_ACTIVE`) and skip publishing when inactive.

**Benefit:** Predictable behavior and no stray publishes before activation.

---

### 1.3 EKF behavior on large time gaps

**Current state:** When IMU `dt > 1.0` s, the predict step is skipped and only `_last_imu_stamp` is advanced. The next update uses a normal `dt`, which can leave the filter in an inconsistent state after long gaps.

**Recommendation:**

- For large gaps (e.g. `dt > 1.0` s or a configurable `max_predict_dt`), consider one of:
  - **Reinitialize** the filter using the next odometry message (position, velocity, yaw) and initial covariance, or
  - **Cap** `dt` to a maximum (e.g. 1.0 s) for the predict step and document that very large gaps are handled conservatively.
- Add a short note in the pose estimator docstring or parameters describing behavior after timestamp gaps.

**Benefit:** More robust behavior after pauses or replay with gaps.

---

## 2. Medium priority

### 2.1 Shorten octomap lock during map publish

**Current state:** When `_map_publish_interval_ms > 0`, the depth worker holds `_octomap_lock` for the full duration of `_publish_map_impl()` (including `to_occupancy_grid_2d()` and `publish()`), which can block the next `insert_pointcloud`.

**Recommendation:**

- Inside the lock: build the 2D occupancy grid (or the data needed to construct it) and copy it into a local variable or message.
- Release the lock, then call `publish()` (and any message construction that does not touch the octomap) outside the lock.

**Benefit:** Shorter critical section and lower latency under high depth rate.

---

### 2.2 Document or extend loop-closure ratio test

**Current state:** With `use_ratio_test=True`, at most one loop-closure candidate is returned per keyframe; without ratio test, multiple candidates can be returned.

**Recommendation:**

- Document in `loop_closure.py` (module or `get_loop_closure_indexes` docstring) that ratio test intentionally returns at most one candidate for stability, **or**
- Extend the ratio test to allow multiple candidates when several pass the ratio and distance thresholds (e.g. with a configurable max count) for consistency with the non–ratio-test path.

**Benefit:** Clear contract for downstream and consistent behavior across modes.

---

### 2.3 Use `depth_frame` parameter in depth pipeline (optional)

**Current state:** `depth_frame` is declared in parameters but the depth callback uses `msg.header.frame_id` for TF lookups.

**Recommendation:**

- If you need to enforce or override the expected depth frame (e.g. for validation or multi-camera setups), use the `depth_frame` parameter in the depth callback (e.g. validate `msg.header.frame_id == self._depth_frame` or use `depth_frame` for lookup when appropriate). Otherwise, document that the frame is taken from the message header.

**Benefit:** Clearer semantics and optional validation.

---

## 3. Low priority / polish

### 3.1 Parameter description string

**File:** `rwifi_slam_ros2/common/parameters.py` (odometry measurement-noise covariance descriptor).

**Recommendation:** Use a single string for the description, e.g.  
`"Odometry measurement-noise covariance diagonal [X, Y, VX, VY, YAW]. Must be non-negative, length 5."`  
instead of concatenating two string literals.

---

### 3.2 Expose frame IDs and topics in one place

**Recommendation:** Consider a small table in the package README or in this document listing all configurable frames and topics (and their default values) so operators can see the full interface at a glance.

---

### 3.3 Unit tests

**Recommendation:** Add a `test/` package and use the existing `extras_require["test"]` in `setup.py` to run tests (e.g. `pytest`). Suggested targets:

- `has_robot_moved`, `wrap_angle`, `relative_pose_covariance` in `common/utils.py`
- Minimal pose graph add/optimize in `include/backend/pose_graph.py`
- Optional: EKF predict/update with synthetic IMU and odom

---

### 3.4 Graph reset at runtime

**Recommendation:** If you need to clear the map and restart SLAM without restarting the node, consider exposing `Graph.reset()` (and any related state in the node) via a service or a parameter that triggers reset when changed. Document lifecycle (e.g. “reset only when inactive” or “reset and re-initialize first node on next keyframe”).

---

## 4. Implementation checklist

| # | Recommendation                               | File(s) to touch              | Effort |
|---|---------------------------------------------|--------------------------------|--------|
| 1.1 | Topic names configurable                     | parameters.py, node, config   | Medium |
| 1.2 | Guard publishing on lifecycle state         | rwifi_slam_ros2_node.py       | Low    |
| 1.3 | EKF large time gap handling                 | pose_estimator.py             | Medium |
| 2.1 | Shorten octomap lock during map publish     | rwifi_slam_ros2_node.py       | Low    |
| 2.2 | Document or extend ratio test                | loop_closure.py               | Low    |
| 2.3 | Use depth_frame in depth pipeline (optional)| rwifi_slam_ros2_node.py      | Low    |
| 3.1 | Parameter description string                | parameters.py                 | Trivial|
| 3.2 | README table for frames/topics              | README or this doc            | Trivial|
| 3.3 | Unit tests                                  | test/, setup.py              | Medium |
| 3.4 | Graph reset at runtime                      | node, pose_graph, (service)   | Medium |

---

*Generated from code review of rwifi_slam_ros2. Last updated: February 2025.*
