"""Shared fixtures and builders for rwifi_slam_ros2 unit tests."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common.transforms import yaw_to_quaternion


def make_params(**values: object) -> dict[str, Parameter]:
    """Build a parameter dict the way ``Node.get_parameters_by_prefix`` returns it.

    Args:
        **values: Parameter names (dots replaced by ``__``) mapped to values.

    Returns:
        Dict of parameter name to ``rclpy.parameter.Parameter``.
    """
    params = {}
    for key, value in values.items():
        name = key.replace("__", ".")
        params[name] = Parameter(name, value=value)
    return params


def make_pose(x: float = 0.0, y: float = 0.0, yaw: float = 0.0, var: float = 0.01) -> PoseWithCovariance:
    """Build a planar PoseWithCovariance shaped like the EKF output (x, y, yaw variances only).

    Args:
        x: Position x (m).
        y: Position y (m).
        yaw: Heading (rad).
        var: Variance placed on x, y, and yaw.

    Returns:
        PoseWithCovariance message.
    """
    cov = np.zeros((6, 6))
    cov[0, 0] = cov[1, 1] = cov[5, 5] = var
    return PoseWithCovariance(
        pose=Pose(position=Point(x=float(x), y=float(y), z=0.0), orientation=yaw_to_quaternion(yaw)),
        covariance=cov.ravel().tolist(),
    )


def stamp(seconds: float) -> Time:
    """Build a builtin_interfaces/Time from float seconds.

    Args:
        seconds: Time in seconds.

    Returns:
        Time message.
    """
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def graph_params() -> dict[str, Parameter]:
    """Return default pose-graph parameters."""
    return make_params(robustness_threshold=0.5, verbose=False, optimization_iterations=10)
