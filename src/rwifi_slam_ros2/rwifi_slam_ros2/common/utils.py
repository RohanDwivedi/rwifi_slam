"""Common utilities: motion helpers and covariance propagation."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from rclpy.logging import get_logger
from rwifi_slam_ros2.common.transforms import quaternion_to_yaw, wrap_angle

_logger = get_logger(__name__)


def skew(v: np.ndarray) -> np.ndarray:
    """Build the 3x3 skew-symmetric matrix from a 3-vector.

    Args:
        v: 3D vector (x, y, z).

    Returns:
        3x3 skew-symmetric matrix such that skew(v) @ w = v x w.
    """
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def adjoint_SE3(T: np.ndarray) -> np.ndarray:
    """Compute the 6x6 adjoint matrix of an SE(3) transform.

    Args:
        T: 4x4 homogeneous transformation matrix in SE(3).

    Returns:
        6x6 adjoint matrix Ad(T) for Lie algebra covariance propagation.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Adj = np.zeros((6, 6), dtype=float)
    Adj[:3, :3] = R
    Adj[3:, 3:] = R
    Adj[:3, 3:] = np.zeros((3, 3), dtype=float)
    Adj[3:, :3] = skew(t) @ R
    return Adj


def inv_SE3(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 SE(3) transformation matrix.

    Args:
        T: 4x4 homogeneous transformation matrix in SE(3).

    Returns:
        4x4 inverse transformation T^{-1}.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def relative_pose_covariance(
    T1: np.ndarray,
    Sigma1: np.ndarray,
    T2: np.ndarray,
    Sigma2: np.ndarray,
    Sigma12: np.ndarray | None = None,
) -> np.ndarray:
    """Compute 6x6 covariance of the relative pose T1^{-1} @ T2.

    Uses adjoint propagation; optionally subtracts cross-covariance Sigma12.

    Args:
        T1: 4x4 pose of first frame.
        Sigma1: 6x6 covariance of first pose.
        T2: 4x4 pose of second frame.
        Sigma2: 6x6 covariance of second pose.
        Sigma12: Optional 6x6 cross-covariance between poses. If None, poses
            are treated as independent.

    Returns:
        6x6 covariance matrix of the relative pose (symmetric).
    """
    T1_inv = inv_SE3(T1)
    Adj_T1_inv = adjoint_SE3(T1_inv)

    if Sigma12 is None:
        Sigma_rel = Sigma1 + Adj_T1_inv @ Sigma2 @ Adj_T1_inv.T
    else:
        Sigma_rel = Sigma1 + Adj_T1_inv @ Sigma2 @ Adj_T1_inv.T - Adj_T1_inv @ Sigma12 - Sigma12.T @ Adj_T1_inv.T

    Sigma_rel = 0.5 * (Sigma_rel + Sigma_rel.T)
    return Sigma_rel


def se3_from_pose(pose: PoseWithCovariance) -> np.ndarray:
    """Convert a PoseWithCovariance to a 4x4 SE(3) matrix.

    Uses 2D yaw from the quaternion and full 3D position.

    Args:
        pose: ROS PoseWithCovariance message.

    Returns:
        4x4 homogeneous transformation matrix (rotation from yaw, translation
        from position).
    """
    yaw = quaternion_to_yaw(pose.pose.orientation)
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4, dtype=np.float64)
    T[0:2, 0:2] = np.array([[c, -s], [s, c]])
    T[0:3, 3] = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    return T


def has_robot_moved(
    current_pose: PoseWithCovariance,
    last_pose: PoseWithCovariance,
    distance_interval: float,
    yaw_interval: float,
) -> bool:
    """Return True if the robot moved beyond distance or yaw thresholds.

    Args:
        current_pose: Current robot pose.
        last_pose: Previous pose to compare against.
        distance_interval: Minimum Euclidean (xy) distance to count as moved.
        yaw_interval: Minimum absolute yaw change (rad) to count as moved.

    Returns:
        True if either distance or yaw change exceeds the given intervals.
    """
    current_pos = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
    last_pos = np.array([last_pose.pose.position.x, last_pose.pose.position.y])

    current_yaw = quaternion_to_yaw(current_pose.pose.orientation)
    last_yaw = quaternion_to_yaw(last_pose.pose.orientation)

    distance_magnitude = np.linalg.norm(current_pos - last_pos)

    yaw_diff = wrap_angle(current_yaw - last_yaw)
    yaw_magnitude = abs(yaw_diff)

    moved = distance_magnitude > distance_interval or yaw_magnitude > yaw_interval
    if moved:
        _logger.debug(
            f"Robot moved: dist={float(distance_magnitude):.4f} (threshold={distance_interval:.4f}), "
            f"yaw_diff={float(yaw_magnitude):.4f} (threshold={yaw_interval:.4f})"
        )
    return moved
