"""Unit tests for quaternion and angle helpers in ``common.transforms``."""

import math

import numpy as np
import pytest
from rwifi_slam_ros2.common.transforms import (
    quaternion_to_rotation_matrix,
    quaternion_to_yaw,
    wrap_angle,
    yaw_to_quaternion,
)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, math.pi - 1e-6, -math.pi + 1e-6])
def test_yaw_quaternion_round_trip(yaw):
    """Yaw survives conversion to a quaternion and back."""
    assert quaternion_to_yaw(yaw_to_quaternion(yaw)) == pytest.approx(yaw, abs=1e-9)


@pytest.mark.parametrize(
    "angle, expected",
    [(0.0, 0.0), (2 * math.pi, 0.0), (1.5 * math.pi, -0.5 * math.pi), (-1.5 * math.pi, 0.5 * math.pi)],
)
def test_wrap_angle_known_values(angle, expected):
    """Angles are wrapped to their equivalent in [-pi, pi]."""
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-9)


def test_wrap_angle_stays_in_range():
    """Wrapped angles never leave [-pi, pi]."""
    for angle in np.linspace(-20.0, 20.0, 401):
        assert -math.pi <= wrap_angle(float(angle)) <= math.pi


def test_rotation_matrix_matches_yaw():
    """A yaw-only quaternion maps to a rotation about Z."""
    yaw = 0.7
    expected = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(quaternion_to_rotation_matrix(yaw_to_quaternion(yaw)), expected, atol=1e-12)
