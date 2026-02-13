"""ROS 2 lifecycle node for Range WiFi SLAM.

Orchestrator for the Range WiFi SLAM.

Subscribes to IMU, odometry, Wi-Fi RTT, and depth sensors,
processes the data through the :class:`PoseEstimator` EKF,
and publishes fused odometry and trajectory outputs from
the wifi based posegraph SLAM module.
"""

from __future__ import annotations

from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleNode
from rclpy.subscription import Subscription
from rwifi_slam_ros2.common import (
    MAP_QOS_PROFILE,
    POSE_QOS_PROFILE,
    SENSOR_QOS_PROFILE,
    declare_parameters,
    get_static_transform,
    transform_imu_with_tf,
)
from rwifi_slam_ros2.include.front_end import PoseEstimator
from rwifi_slam_ros2.include.front_end.wifi_rtt_clustering import WifiRttClustering
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener, TransformStamped
from wifi_rtt_msgs.msg import APRangeObservation


class WifiSlamNode(LifecycleNode):
    """Lifecycle node that runs the Range WiFi SLAM pipeline."""

    def __init__(self) -> None:
        """Initialize the Range WiFi SLAM node."""
        super().__init__("rwifi_slam_node")

        declare_parameters(self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cache_static_tf: TransformStamped | None = None

        self._pose_history: deque[PoseStamped] = deque(maxlen=self.get_parameter("trajectory_keep_history").value)

        # Initialize the pose estimator
        params = self.get_parameters_by_prefix("pose_estimator")
        base_frame = self.get_parameter("base_frame")
        odom_frame = self.get_parameter("odom_frame")
        params["base_frame"] = base_frame
        params["odom_frame"] = odom_frame
        self._pose_estimator = PoseEstimator(parameters=params)

        # Initialize the Wi-Fi RTT clustering module
        params = self.get_parameters_by_prefix("wifi_rtt_clustering")
        self._wifi_rtt_clustering = WifiRttClustering(parameters=params, pose_estimator=self._pose_estimator)

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Subscription / publisher setup
    # ------------------------------------------------------------------

    def _init_subscribers(self) -> None:
        """Create all sensor subscriptions."""
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
        """Create all lifecycle publishers."""
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

    def _destroy_subscribers(self) -> None:
        """Destroy all sensor subscriptions."""
        if hasattr(self, "_odometry_sub"):
            self.destroy_subscription(self._odometry_sub)
        if hasattr(self, "_imu_sub"):
            self.destroy_subscription(self._imu_sub)
        if hasattr(self, "_wifi_rtt_sub"):
            self.destroy_subscription(self._wifi_rtt_sub)
        if hasattr(self, "_depth_sub"):
            self.destroy_subscription(self._depth_sub)

    def _destroy_publishers(self) -> None:
        """Destroy all lifecycle publishers."""
        if hasattr(self, "pose_with_covariance_pub"):
            self.destroy_publisher(self.pose_with_covariance_pub)
        if hasattr(self, "trajectory_pub"):
            self.destroy_publisher(self.trajectory_pub)
        if hasattr(self, "odom_filtered_pub"):
            self.destroy_publisher(self.odom_filtered_pub)
        if hasattr(self, "map_pub"):
            self.destroy_publisher(self.map_pub)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _odometry_callback(self, msg: Odometry) -> None:
        """Forward wheel-odometry to the pose estimator."""
        self._pose_estimator.push_odometry_msg(msg)
        self._publish_odom_filtered()

    def _imu_callback(self, msg: Imu) -> None:
        """Transform IMU into ``base_frame`` and forward to the estimator."""
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
        """Handle depth point-cloud (not yet implemented)."""
        return

    def _wifi_rtt_callback(self, msg: APRangeObservation) -> None:
        """Handle Wi-Fi RTT range observation."""
        self._wifi_rtt_clustering.push_wifi_rtt_msg(msg)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_odom_filtered(self) -> None:
        """Publish the latest fused odometry."""
        try:
            odom_msg = self._pose_estimator.current_odom_filtered_pose
            self.odom_filtered_pub.publish(odom_msg)
        except ValueError as e:
            self.get_logger().warning(f"Failed to publish filtered odometry: {e}")

    def _publish_map(self) -> None:
        """Publish the latest map (not yet implemented)."""
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
