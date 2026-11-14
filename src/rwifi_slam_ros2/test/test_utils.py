"""Unit tests for SE(3) and motion helpers in ``common.utils``."""

import math

import numpy as np
import pytest
from conftest import make_pose
from rwifi_slam_ros2.common.transforms import quaternion_to_yaw
from rwifi_slam_ros2.common.utils import has_robot_moved, inv_SE3, pose_from_se3, se3_from_pose


def test_inv_se3_is_inverse():
    """T @ inv(T) is the identity."""
    T = se3_from_pose(make_pose(x=3.0, y=-2.0, yaw=1.1))
    np.testing.assert_allclose(T @ inv_SE3(T), np.eye(4), atol=1e-12)
    np.testing.assert_allclose(inv_SE3(T) @ T, np.eye(4), atol=1e-12)


def test_pose_se3_round_trip_keeps_pose_and_covariance():
    """se3_from_pose and pose_from_se3 are inverses for planar poses."""
    pose = make_pose(x=1.5, y=0.25, yaw=-2.0, var=0.3)
    restored = pose_from_se3(se3_from_pose(pose), covariance=list(pose.covariance))
    assert restored.pose.position.x == pytest.approx(1.5)
    assert restored.pose.position.y == pytest.approx(0.25)
    assert quaternion_to_yaw(restored.pose.orientation) == pytest.approx(-2.0)
    assert list(restored.covariance) == list(pose.covariance)


@pytest.mark.parametrize(
    "current, moved",
    [
        (make_pose(x=0.05), False),
        (make_pose(x=0.2), True),
        (make_pose(yaw=0.01), False),
        (make_pose(yaw=0.05), True),
    ],
)
def test_has_robot_moved_thresholds(current, moved):
    """Movement is reported only when distance or yaw exceeds its interval."""
    assert has_robot_moved(current, make_pose(), distance_interval=0.1, yaw_interval=0.0175) == moved


def test_has_robot_moved_wraps_yaw():
    """A small heading change across +/-pi is not a large rotation."""
    current = make_pose(yaw=math.pi - 0.001)
    last = make_pose(yaw=-math.pi + 0.001)
    assert not has_robot_moved(current, last, distance_interval=0.1, yaw_interval=0.0175)
