"""Wi-Fi RTT clustering module for the Range WiFi SLAM."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common import (
    Keyframe,
    KeyframeCollection,
    quaternion_to_yaw,
    stamp_to_sec,
    wrap_angle,
    zero_timestamp,
)
from rwifi_slam_ros2.include.front_end.pose_estimator import PoseEstimator
from wifi_rtt_msgs.msg import APRangeObservation


class WifiRttClustering:
    """Wi-Fi RTT clustering module for the Range WiFi SLAM."""

    def __init__(self, parameters: dict[str, Parameter], pose_estimator: PoseEstimator) -> None:
        """Initialize the Wi-Fi RTT clustering module."""
        self._logger = get_logger(self.__class__.__name__)

        self._parameters = parameters

        self._distance_interval: float = self._parameters["distance_interval"].value
        self._yaw_interval: float = self._parameters["yaw_interval"].value
        self._distance_method: str = self._parameters["distance_method"].value
        self._clustering_threshold: float = self._parameters["clustering_threshold"].value
        self._use_ratio_test: bool = self._parameters["use_ratio_test"].value
        self._ratio_threshold: float = self._parameters["ratio_threshold"].value
        self._exclude_near_neighbors: bool = self._parameters["exclude_near_neighbors"].value
        self._near_neighbor_threshold: float = self._parameters["near_neighbor_threshold"].value

        self._last_pose: Odometry | None = None
        self._current_pose: Odometry | None = None

        self._keyframe_collection: KeyframeCollection = KeyframeCollection(keyframes=[])
        self._keyframe_stamp: Time = zero_timestamp()

        self._pose_estimator = pose_estimator

    @property
    def keyframes_array(self) -> np.ndarray:
        """Get all keyframe observations as a 2D numpy array."""
        return self._keyframe_collection.keyframes_array

    @property
    def timestamps_array(self) -> np.ndarray:
        """Get all keyframe timestamps as a numpy array."""
        return self._keyframe_collection.timestamps_array

    def _add_keyframe(self, msg: APRangeObservation) -> None:
        """Create a keyframe from the Wi-Fi RTT message."""
        if not msg.observations:
            self._logger.warning("Received empty observation list; skipping keyframe.")
            return

        self._keyframe_stamp = msg.header.stamp
        timestamp_sec: float = stamp_to_sec(self._keyframe_stamp)

        observation_vector: np.ndarray = np.array(
            [observation.measured_distance for observation in msg.observations], dtype=np.float64
        )

        keyframe = Keyframe(observations=observation_vector, timestamp=timestamp_sec)
        self._keyframe_collection.keyframes.append(keyframe)

        self._last_pose = self._pose_estimator.current_odom_filtered_pose

    def _has_robot_moved(self) -> bool:
        """Check if the robot has moved since the last keyframe."""
        try:
            self._current_pose = self._pose_estimator.current_odom_filtered_pose
        except (ValueError, AttributeError) as e:
            self._logger.warning(f"Could not get current pose: {e}")
            return False

        if self._last_pose is None:
            return True

        current_pos = np.array([self._current_pose.pose.pose.position.x, self._current_pose.pose.pose.position.y])
        last_pos = np.array([self._last_pose.pose.pose.position.x, self._last_pose.pose.pose.position.y])

        current_yaw = quaternion_to_yaw(self._current_pose.pose.pose.orientation)
        last_yaw = quaternion_to_yaw(self._last_pose.pose.pose.orientation)

        distance_magnitude = abs(np.linalg.norm(current_pos - last_pos))

        yaw_diff = wrap_angle(current_yaw - last_yaw)
        yaw_magnitude = abs(yaw_diff)

        return distance_magnitude > self._distance_interval or yaw_magnitude > self._yaw_interval

    def _is_loop_closure(self, current_keyframe: Keyframe) -> tuple[bool, list[int] | None]:
        """Check if the current keyframe represents a loop closure.

        Args:
            current_keyframe: The current keyframe to check for loop closure.

        Returns:
            Tuple of (is_loop_closure, matched_indexes). If loop closure is detected,
            matched_indexes contains the indices of matching keyframes. Otherwise,
            matched_indexes is None.
        """
        if len(self._keyframe_collection.keyframes) < 2:
            self._logger.warning("Not enough keyframes to check for loop closure.")
            return False, None

        if self._distance_method == "euclidean":
            self._logger.debug("Checking for loop closure with Euclidean distance method")
            previous_keyframes = self._keyframe_collection.keyframes_array[:-1]
            distances = np.linalg.norm(current_keyframe.observations - previous_keyframes, axis=1)
        else:
            raise ValueError(f"Invalid distance method: {self._distance_method}")

        if self._exclude_near_neighbors:
            current_idx = len(distances)
            k = int(self._near_neighbor_threshold)
            keep_mask = np.abs(np.arange(len(distances)) - current_idx) > k
            distances = np.where(keep_mask, distances, np.inf)

        if self._use_ratio_test:
            finite_mask = np.isfinite(distances)
            if np.sum(finite_mask) < 2:
                return False, None

            best_idx = int(np.argmin(distances))
            best_distance = float(distances[best_idx])
            if not np.isfinite(best_distance):
                return False, None

            finite_distances = np.sort(distances[finite_mask])
            second_best_distance = float(finite_distances[1])
            if second_best_distance <= 0.0:
                return False, None

            ratio = best_distance / second_best_distance
            if ratio < self._ratio_threshold and best_distance < self._clustering_threshold:
                self._logger.debug(f"Loop closure detected at index {best_idx}")
                return True, [best_idx]

            return False, None

        valid_mask = distances < self._clustering_threshold
        loop_closure_indexes = np.where(valid_mask)[0].tolist()

        if loop_closure_indexes:
            self._logger.debug(f"Loop closure detected at indexes {loop_closure_indexes}")
            return True, loop_closure_indexes

        return False, None

    def push_wifi_rtt_msg(self, msg: APRangeObservation) -> None:
        """Push the Wi-Fi RTT message to the clustering module.

        Adds the current keyframe to the keyframe collection and checks for loop closures
        if the robot has moved sufficiently since the last keyframe.
        """
        if not self._has_robot_moved():
            return

        self._add_keyframe(msg)

        is_loop_closure, matched_indexes = self._is_loop_closure(self._keyframe_collection.keyframes[-1])
        if is_loop_closure:
            self._logger.info(f"Loop closure detected at indexes {matched_indexes}")
