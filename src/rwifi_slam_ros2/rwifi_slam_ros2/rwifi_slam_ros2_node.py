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
    transform_imu,
)
from rwifi_slam_ros2.include.front_end import PoseEstimator
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from wifi_rtt_msgs.msg import APRangeObservation


class WifiSlamNode(LifecycleNode):
    """Lifecycle node that runs the Range WiFi SLAM pipeline."""

    def __init__(self) -> None:
        """Initialize the Range WiFi SLAM node."""
        super().__init__("rwifi_slam_node")

        declare_parameters(self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose_history: deque[PoseStamped] = deque(maxlen=self.get_parameter("trajectory_keep_history").value)

        # Initialize the pose estimator (EKF-based 2-D pose estimator fusing IMU and wheel odometry)
        params = self.get_parameters_by_prefix("pose_estimator")
        self._pose_estimator = PoseEstimator(parameters=params)

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
        for sub in (
            self._odometry_sub,
            self._imu_sub,
            self._wifi_rtt_sub,
            self._depth_sub,
        ):
            self.destroy_subscription(sub)

    def _destroy_publishers(self) -> None:
        """Destroy all lifecycle publishers."""
        for pub in (
            self.pose_with_covariance_pub,
            self.trajectory_pub,
            self.odom_filtered_pub,
            self.map_pub,
        ):
            self.destroy_publisher(pub)

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
        try:
            msg = transform_imu(
                self._tf_buffer,
                msg,
                from_frame,
                to_frame,
                timeout=rclpy.duration.Duration(seconds=0),
                time=rclpy.time.Time(),
            )
        except TransformException as e:
            self.get_logger().debug(f"IMU transform not yet available: {e}")
            return
        self._pose_estimator.push_imu_msg(msg)
        self._publish_odom_filtered()

    def _depth_callback(self, msg: PointCloud2) -> None:
        """Handle depth point-cloud (not yet implemented)."""
        return

    def _wifi_rtt_callback(self, msg: APRangeObservation) -> None:
        """Handle Wi-Fi RTT range observation (not yet implemented)."""
        return

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_odom_filtered(self) -> None:
        """Publish the latest fused odometry."""
        odom_msg = self._pose_estimator.current_odom_filtered_pose

        # Set timestamp if missing or invalid
        if odom_msg.header.stamp is None:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
        elif odom_msg.header.stamp.sec == 0 and odom_msg.header.stamp.nanosec == 0:
            odom_msg.header.stamp = self.get_clock().now().to_msg()

        odom_msg.header.frame_id = self.get_parameter("odom_frame").value
        odom_msg.child_frame_id = self.get_parameter("base_frame").value

        self.odom_filtered_pub.publish(odom_msg)

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
