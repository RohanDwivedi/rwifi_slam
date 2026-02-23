"""ROS 2 lifecycle node for Range Wi-Fi SLAM.

Orchestrator for the Range Wi-Fi SLAM.

Subscribes to IMU, odometry, Wi-Fi RTT, and depth sensors,
processes the data through the :class:`PoseEstimator` EKF,
and publishes fused odometry and trajectory outputs from
the wifi based posegraph SLAM module.
"""

from __future__ import annotations

import queue
import threading
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleNode
from rclpy.logging import get_logger_effective_level, set_logger_level
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rwifi_slam_ros2.common import (
    DEPTH_QOS_PROFILE,
    MAP_QOS_PROFILE,
    POSE_QOS_PROFILE,
    SENSOR_QOS_PROFILE,
    declare_parameters,
    has_robot_moved,
    inv_SE3,
    quaternion_to_rotation_matrix,
    se3_from_pose,
    transform_imu_with_tf,
    yaw_to_quaternion,
)
from rwifi_slam_ros2.include.backend import Graph, Octomap
from rwifi_slam_ros2.include.front_end import LoopClosure, PoseEstimator
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener, TransformStamped
from wifi_rtt_msgs.msg import APRangeObservation


class WifiSlamNode(LifecycleNode):
    """Lifecycle node for Range Wi-Fi SLAM: EKF pose estimation, pose graph, loop closure."""

    def __init__(self) -> None:
        """Initialize node, declare parameters, and create pose estimator, graph, and loop closure."""
        super().__init__("rwifi_slam_node")

        declare_parameters(self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cache_static_tf: TransformStamped | None = None

        self._last_map_publish_ns: int = 0
        self._depth_frame_count: int = 0

        self._distance_interval: float = self.get_parameter("distance_interval").value
        self._yaw_interval: float = self.get_parameter("yaw_interval").value
        self._map_frame: str = self.get_parameter("map_frame").value
        self._odom_frame: str = self.get_parameter("odom_frame").value
        self._base_frame: str = self.get_parameter("base_frame").value
        self._imu_frame: str = self.get_parameter("imu_frame").value
        self._trajectory_keep_history: int = self.get_parameter("trajectory_keep_history").value

        pose_estimator_params: dict[str, Parameter] = self.get_parameters_by_prefix("pose_estimator")
        base_frame = self.get_parameter("base_frame")
        odom_frame = self.get_parameter("odom_frame")
        pose_estimator_params["base_frame"] = base_frame
        pose_estimator_params["odom_frame"] = odom_frame
        self._pose_estimator = PoseEstimator(parameters=pose_estimator_params)

        pose_graph_params: dict[str, Parameter] = self.get_parameters_by_prefix("pose_graph")
        self._pose_graph = Graph(parameters=pose_graph_params)

        loop_closure_params: dict[str, Parameter] = self.get_parameters_by_prefix("loop_closure")
        self._loop_closure = LoopClosure(parameters=loop_closure_params, graph=self._pose_graph)

        octomap_params: dict[str, Parameter] = self.get_parameters_by_prefix("octomap")
        octomap_params["map_frame"] = self.get_parameter("map_frame")
        self._octomap = Octomap(parameters=octomap_params)

        self._depth_callback_group = MutuallyExclusiveCallbackGroup()
        self._mapping_enabled: bool = self.get_parameter("octomap.enabled").value
        self._octomap_downsample_step: int = self.get_parameter("octomap.downsample_step").value
        self._map_publish_interval_ms: float = self.get_parameter("octomap.map_publish_interval_ms").value
        self._octomap_depth_frame_interval: int = self.get_parameter("octomap.depth_frame_interval").value

        self._depth_queue: queue.Queue[tuple[str, Any, np.ndarray] | None] = queue.Queue(maxsize=1)
        self._depth_worker_stop = threading.Event()
        self._depth_worker_thread: threading.Thread | None = None
        self._octomap_lock = threading.Lock()

        self._propagate_log_level()

    def _propagate_log_level(self) -> None:
        """Propagate the node's log level to all class and module loggers used by this package."""
        node_level = get_logger_effective_level(self.get_logger().name)
        if node_level is None:
            return
        child_logger_names = [
            "rwifi_slam_ros2.include.front_end.pose_estimator",
            "rwifi_slam_ros2.include.front_end.loop_closure",
            "rwifi_slam_ros2.include.backend.pose_graph",
            "rwifi_slam_ros2.common.transforms",
            "rwifi_slam_ros2.common.utils",
        ]
        for name in child_logger_names:
            set_logger_level(name, node_level)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Create subscriptions and publishers. Return SUCCESS or FAILURE on exception."""
        try:
            self._init_subscribers()
            self._init_publishers()

            self.get_logger().info("Node configured successfully.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate node and start depth worker thread if mapping is enabled."""
        self.get_logger().info("Activating.")
        if self._mapping_enabled:
            self._depth_worker_stop.clear()
            self._depth_worker_thread = threading.Thread(
                target=self._depth_worker,
                name="depth_mapping_worker",
                daemon=True,
            )
            self._depth_worker_thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate node and stop the depth worker thread."""
        self.get_logger().info("Deactivating.")
        if self._depth_worker_thread is not None:
            self._depth_worker_stop.set()
            try:
                self._depth_queue.put_nowait(None)
            except queue.Full:
                pass
            self._depth_worker_thread.join(timeout=2.0)
            self._depth_worker_thread = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy subscriptions and publishers created in on_configure."""
        self.get_logger().info("Cleaning up.")
        self._destroy_subscribers()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release subscriptions and publishers on shutdown from any state."""
        self.get_logger().info("Shutting down.")
        self._destroy_subscribers()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def _init_subscribers(self) -> None:
        """Create subscriptions for odometry, IMU, Wi-Fi RTT, and depth."""
        self._odometry_sub: Subscription = self.create_subscription(
            Odometry,
            "/chassis/odom",
            self._odometry_callback,
            SENSOR_QOS_PROFILE,
        )
        self._imu_sub: Subscription = self.create_subscription(
            Imu,
            "/chassis/imu",
            self._imu_callback,
            SENSOR_QOS_PROFILE,
        )
        self._wifi_rtt_sub: Subscription = self.create_subscription(
            APRangeObservation,
            "/wifi_rtt_scan",
            self._wifi_rtt_callback,
            SENSOR_QOS_PROFILE,
        )
        self._depth_sub: Subscription = self.create_subscription(
            PointCloud2,
            "/camera_depth/depth_pcl",
            self._depth_callback,
            DEPTH_QOS_PROFILE,
            callback_group=self._depth_callback_group,
        )

    def _init_publishers(self) -> None:
        """Create all lifecycle publishers and the TF broadcaster."""
        self.pose_with_covariance_pub = self.create_lifecycle_publisher(
            PoseWithCovarianceStamped,
            "/pose",
            POSE_QOS_PROFILE,
        )
        self.trajectory_pub = self.create_lifecycle_publisher(
            Path,
            "/trajectory",
            MAP_QOS_PROFILE,
        )
        self.odom_filtered_pub = self.create_lifecycle_publisher(
            Odometry,
            "/odom_filtered",
            MAP_QOS_PROFILE,
        )
        self.map_pub = self.create_lifecycle_publisher(
            OccupancyGrid,
            "/map",
            MAP_QOS_PROFILE,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

    def _destroy_subscribers(self) -> None:
        """Destroy odometry, IMU, Wi-Fi RTT, and depth subscriptions."""
        if hasattr(self, "_odometry_sub"):
            self.destroy_subscription(self._odometry_sub)
        if hasattr(self, "_imu_sub"):
            self.destroy_subscription(self._imu_sub)
        if hasattr(self, "_wifi_rtt_sub"):
            self.destroy_subscription(self._wifi_rtt_sub)
        if hasattr(self, "_depth_sub"):
            self.destroy_subscription(self._depth_sub)

    def _destroy_publishers(self) -> None:
        """Destroy pose, trajectory, odom_filtered, and map publishers."""
        if hasattr(self, "pose_with_covariance_pub"):
            self.destroy_publisher(self.pose_with_covariance_pub)
        if hasattr(self, "trajectory_pub"):
            self.destroy_publisher(self.trajectory_pub)
        if hasattr(self, "odom_filtered_pub"):
            self.destroy_publisher(self.odom_filtered_pub)
        if hasattr(self, "map_pub"):
            self.destroy_publisher(self.map_pub)

    def _odometry_callback(self, msg: Odometry) -> None:
        """Forward wheel odometry to the pose estimator and publish filtered odom."""
        self._pose_estimator.push_odometry_msg(msg)
        self._publish_odom_filtered()
        self._publish_map_to_odom_tf()

    def _imu_callback(self, msg: Imu) -> None:
        """Transform IMU to base_frame, feed to pose estimator, and publish filtered odom."""
        if self._cache_static_tf is None:
            try:
                self._cache_static_tf = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._imu_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0),
                )
            except TransformException as e:
                self.get_logger().warning(f"IMU transform not yet available: {e}")
                return

        msg = transform_imu_with_tf(msg, self._cache_static_tf, self._base_frame)
        self._pose_estimator.push_imu_msg(msg)
        self._publish_odom_filtered()

    def _depth_callback(self, msg: PointCloud2) -> None:
        """Enqueue depth for mapping; worker thread handles TF, octomap insert, and map publish."""
        if not self._mapping_enabled:
            return
        self._depth_frame_count += 1
        if (self._depth_frame_count - 1) % self._octomap_depth_frame_interval != 0:
            return
        points_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_struct = np.fromiter(
            points_iter,
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if points_struct.size == 0:
            return
        points = np.column_stack((points_struct["x"], points_struct["y"], points_struct["z"])).astype(np.float64)
        try:
            self._depth_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._depth_queue.put_nowait((msg.header.frame_id, msg.header.stamp, points))
        except queue.Full:
            pass

    def _depth_worker(self) -> None:
        """Consume depth from queue, transform to map frame, insert into octomap, publish map."""
        while not self._depth_worker_stop.is_set():
            try:
                item = self._depth_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None or self._depth_worker_stop.is_set():
                continue
            frame_id, stamp, points = item
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    frame_id,
                    stamp,
                )
            except TransformException as e:
                self.get_logger().debug(f"Depth transform not available: {e}")
                continue
            rotation = quaternion_to_rotation_matrix(tf.transform.rotation)
            translation = np.array(
                [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                ],
                dtype=np.float64,
            )
            points_map = (rotation @ points.T).T + translation
            sensor_origin_map = translation
            if self._octomap_downsample_step > 1:
                points_map = points_map[:: self._octomap_downsample_step]
            try:
                with self._octomap_lock:
                    self._octomap.insert_pointcloud(points_map, sensor_origin_map)
            except ValueError as e:
                self.get_logger().debug(f"Octomap insert skipped: {e}")
                continue
            if self._map_publish_interval_ms <= 0:
                self._publish_map()
            else:
                with self._octomap_lock:
                    now_ns = self.get_clock().now().nanoseconds
                    if now_ns - self._last_map_publish_ns >= int(self._map_publish_interval_ms * 1e6):
                        self._last_map_publish_ns = now_ns
                        self._publish_map_impl()

    def _wifi_rtt_callback(self, msg: APRangeObservation) -> None:
        """Add keyframes and loop closures from Wi-Fi RTT observations."""
        if self._pose_graph.is_empty():
            self._pose_graph.add_node(
                pose=self._pose_estimator.current_odom_filtered_pose.pose,
                observations=np.array([obs.measured_distance for obs in msg.observations], dtype=np.float64),
                timestamp=msg.header.stamp,
            )
            self._publish_slam_pose()
            self._publish_trajectory()
            self._publish_map_to_odom_tf()
            return

        if has_robot_moved(
            self._pose_estimator.current_odom_filtered_pose.pose,
            self._pose_graph.nodes[-1].pose,
            self._distance_interval,
            self._yaw_interval,
        ):
            prev_index = self._pose_graph.nodes_length - 1
            self._pose_graph.add_node(
                pose=self._pose_estimator.current_odom_filtered_pose.pose,
                observations=np.array([obs.measured_distance for obs in msg.observations], dtype=np.float64),
                timestamp=msg.header.stamp,
            )
            self._pose_graph.add_odometry_edge(from_index=prev_index, to_index=self._pose_graph.nodes_length - 1)
            self._publish_slam_pose()
            self._publish_trajectory()
            self._publish_map_to_odom_tf()

            loop_closure_indexes = self._loop_closure.get_loop_closure_indexes()
            if loop_closure_indexes is not None:
                current_index = self._pose_graph.nodes_length - 1
                self.get_logger().info(
                    f"Loop closure detected: node {current_index} matches previous nodes {loop_closure_indexes}"
                )
                for loop_idx in loop_closure_indexes:
                    self._pose_graph.add_loop_closure_constraint(from_index=loop_idx, to_index=current_index)

                self._pose_graph.optimize()
                self._publish_trajectory()
                self._publish_map_to_odom_tf()
                self._publish_slam_pose()

    def _publish_odom_filtered(self) -> None:
        """Publish the latest EKF-filtered odometry."""
        odom_msg = self._pose_estimator.current_odom_filtered_pose
        self.odom_filtered_pub.publish(odom_msg)

    def _publish_slam_pose(self) -> None:
        """Publish the latest SLAM pose (last graph node) with covariance on /pose."""
        if self._pose_graph.is_empty():
            return
        last_node = self._pose_graph.nodes[-1]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = last_node.timestamp
        msg.header.frame_id = self._map_frame
        msg.pose = last_node.pose
        self.pose_with_covariance_pub.publish(msg)

    def _publish_trajectory(self) -> None:
        """Publish the SLAM trajectory as a nav_msgs/Path (collection of pose-graph poses, last N nodes)."""
        if self._pose_graph.is_empty():
            return
        keep = self._trajectory_keep_history
        nodes = self._pose_graph.nodes
        if keep <= 0:
            node_views = list(nodes)
        else:
            node_views = list(nodes[-keep:])
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._map_frame
        for node_view in node_views:
            pose = PoseStamped()
            pose.header.stamp = node_view.timestamp
            pose.header.frame_id = self._map_frame
            pose.pose = node_view.pose.pose
            path.poses.append(pose)
        self.trajectory_pub.publish(path)

    def _publish_map_to_odom_tf(self) -> None:
        """Broadcast the map -> odom transform derived from the optimized graph."""
        if self._pose_graph.is_empty():
            return
        odom_pose = self._pose_estimator.current_odom_filtered_pose
        T_map_robot = se3_from_pose(self._pose_graph.nodes[-1].pose)
        T_odom_robot = se3_from_pose(odom_pose.pose)
        T_map_odom = T_map_robot @ inv_SE3(T_odom_robot)

        yaw = float(np.arctan2(T_map_odom[1, 0], T_map_odom[0, 0]))

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = odom_pose.header.stamp
        map_to_odom.header.frame_id = self._map_frame
        map_to_odom.child_frame_id = self._odom_frame
        map_to_odom.transform.translation.x = float(T_map_odom[0, 3])
        map_to_odom.transform.translation.y = float(T_map_odom[1, 3])
        map_to_odom.transform.translation.z = float(T_map_odom[2, 3])
        map_to_odom.transform.rotation = yaw_to_quaternion(yaw)
        self._tf_broadcaster.sendTransform(map_to_odom)

    def _publish_map(self) -> None:
        """Publish the latest 2D occupancy grid from the octomap (thread-safe)."""
        with self._octomap_lock:
            self._publish_map_impl()

    def _publish_map_impl(self) -> None:
        """Build and publish the 2D occupancy grid. Caller must hold _octomap_lock."""
        grid = self._octomap.to_occupancy_grid_2d(
            stamp=self.get_clock().now().to_msg(),
        )
        self.map_pub.publish(grid)


def main(args: list[str] | None = None) -> int:
    """Entry point for rwifi_slam_node. Uses MultiThreadedExecutor for depth in a separate thread."""
    rclpy.init(args=args)
    node = WifiSlamNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
