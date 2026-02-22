"""ROS 2 lifecycle node for Range Wi-Fi SLAM.

Orchestrator for the Range Wi-Fi SLAM.

Subscribes to IMU, odometry, Wi-Fi RTT, and depth sensors,
processes the data through the :class:`PoseEstimator` EKF,
and publishes fused odometry and trajectory outputs from
the wifi based posegraph SLAM module.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleNode
from rclpy.logging import get_logger_effective_level, set_logger_level
from rclpy.subscription import Subscription
from rwifi_slam_ros2.common import (
    MAP_QOS_PROFILE,
    POSE_QOS_PROFILE,
    SENSOR_QOS_PROFILE,
    declare_parameters,
    get_static_transform,
    has_robot_moved,
    inv_SE3,
    se3_from_pose,
    transform_imu_with_tf,
    yaw_to_quaternion,
)
from rwifi_slam_ros2.include.backend import Graph
from rwifi_slam_ros2.include.front_end import LoopClosure, PoseEstimator
from sensor_msgs.msg import Imu, PointCloud2
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

        self._distance_interval = self.get_parameter("distance_interval").value
        self._yaw_interval = self.get_parameter("yaw_interval").value

        params = self.get_parameters_by_prefix("pose_estimator")
        base_frame = self.get_parameter("base_frame")
        odom_frame = self.get_parameter("odom_frame")
        params["base_frame"] = base_frame
        params["odom_frame"] = odom_frame
        self._pose_estimator = PoseEstimator(parameters=params)

        robustness_threshold = self.get_parameter("pose_graph.robustness_threshold").value
        pose_graph_verbose = self.get_parameter("pose_graph.verbose").value
        optimization_iterations = self.get_parameter("pose_graph.optimization_iterations").value
        self._pose_graph = Graph(
            robustness_threshold=robustness_threshold,
            verbose=pose_graph_verbose,
            optimization_iterations=optimization_iterations,
        )

        params = self.get_parameters_by_prefix("loop_closure")
        self._loop_closure = LoopClosure(parameters=params, graph=self._pose_graph)

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
        """Create subscriptions, publishers, services, and timers.

        Args:
            state: Current lifecycle state.

        Returns:
            ``SUCCESS`` when all resources are created, ``FAILURE`` on
            any exception.
        """
        try:
            self._init_subscribers()
            self._init_publishers()

            self.get_logger().info("Node configured successfully.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate lifecycle publishers and subscriptions.

        Args:
            state: Current lifecycle state.

        Returns:
            Result of the parent ``on_activate``.
        """
        self.get_logger().info("Activating.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate lifecycle publishers and subscriptions.

        Args:
            state: Current lifecycle state.

        Returns:
            Result of the parent ``on_deactivate``.
        """
        self.get_logger().info("Deactivating.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy resources created during ``on_configure``.

        Args:
            state: Current lifecycle state.

        Returns:
            ``SUCCESS`` after cleanup completes.
        """
        self.get_logger().info("Cleaning up.")
        self._destroy_subscribers()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle shutdown from any state, releasing resources if present.

        Args:
            state: Current lifecycle state.

        Returns:
            ``SUCCESS`` after shutdown completes.
        """
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
            SENSOR_QOS_PROFILE,
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
        """Forward wheel odometry to the pose estimator and publish filtered odom.

        Args:
            msg: Odometry message from the chassis.
        """
        self._pose_estimator.push_odometry_msg(msg)
        self._publish_odom_filtered()
        self._publish_map_to_odom_tf()

    def _imu_callback(self, msg: Imu) -> None:
        """Transform IMU to base_frame and feed to pose estimator; publish filtered odom.

        Args:
            msg: IMU message in imu_frame.
        """
        from_frame = self.get_parameter("imu_frame").value
        to_frame = self.get_parameter("base_frame").value

        if self._cache_static_tf is None:
            try:
                self._cache_static_tf = get_static_transform(
                    self._tf_buffer,
                    from_frame,
                    to_frame,
                )
            except TransformException as e:
                self.get_logger().warning(f"IMU transform not yet available: {e}")
                return

        msg = transform_imu_with_tf(msg, self._cache_static_tf, to_frame)
        self._pose_estimator.push_imu_msg(msg)
        self._publish_odom_filtered()

    def _depth_callback(self, msg: PointCloud2) -> None:
        """Handle depth point cloud (reserved for future use).

        Args:
            msg: Depth point cloud message.
        """
        return

    def _wifi_rtt_callback(self, msg: APRangeObservation) -> None:
        """Add keyframes and loop closures from Wi-Fi RTT observations.

        Args:
            msg: Wi-Fi RTT scan with per-AP range observations.
        """
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
        """Publish the latest SLAM pose with covariance (from pose graph, not EKF).

        The trajectory is the collection of poses from the SLAM pose graph; it can
        be updated after optimization (including previous poses). This method
        always publishes the latest value of the SLAM pose (last node in the
        trajectory) with covariance on /pose. Covariance comes from the pose graph
        (stored per node at keyframe creation; see pose_graph._covariances).
        """
        if self._pose_graph.is_empty():
            return
        last_node = self._pose_graph.nodes[-1]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = last_node.timestamp
        msg.header.frame_id = self.get_parameter("map_frame").value
        # last_node.pose is PoseWithCovariance: .pose (position + orientation) and .covariance (36-element)
        msg.pose = last_node.pose
        self.pose_with_covariance_pub.publish(msg)

    def _publish_trajectory(self) -> None:
        """Publish the SLAM trajectory as a nav_msgs/Path (collection of pose-graph poses, last N nodes)."""
        if self._pose_graph.is_empty():
            return
        keep = int(self.get_parameter("trajectory_keep_history").value)
        nodes = self._pose_graph.nodes
        if keep <= 0:
            node_views = list(nodes)
        else:
            node_views = list(nodes[-keep:])
        map_frame = self.get_parameter("map_frame").value
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = map_frame
        for node_view in node_views:
            pose = PoseStamped()
            pose.header.stamp = node_view.timestamp
            pose.header.frame_id = map_frame
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

        t = TransformStamped()
        t.header.stamp = odom_pose.header.stamp
        t.header.frame_id = self.get_parameter("map_frame").value
        t.child_frame_id = self.get_parameter("odom_frame").value
        t.transform.translation.x = float(T_map_odom[0, 3])
        t.transform.translation.y = float(T_map_odom[1, 3])
        t.transform.translation.z = float(T_map_odom[2, 3])
        t.transform.rotation = yaw_to_quaternion(yaw)
        self._tf_broadcaster.sendTransform(t)

    def _publish_map(self) -> None:
        """Publish the latest map (reserved for future use)."""
        return


def main(args: list[str] | None = None) -> int:
    """Entry point for the ``rwifi_slam_node`` executable.

    Args:
        args: Command-line arguments forwarded to ``rclpy.init``.

    Returns:
        Exit code (always ``0``).
    """
    rclpy.init(args=args)
    node = WifiSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
