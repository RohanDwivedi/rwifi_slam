"""Unit tests for the IMU + odometry EKF."""

import pytest
from conftest import make_params, stamp
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common.transforms import quaternion_to_yaw, yaw_to_quaternion
from rwifi_slam_ros2.include.front_end import PoseEstimator
from sensor_msgs.msg import Imu


def _estimator() -> PoseEstimator:
    """Build an estimator with the packaged default noise settings."""
    params = make_params(
        initial_covar_mat_diag=[0.01, 0.01, 0.01, 0.01, 0.27, 0.04, 0.04, 0.0025],
        imu__gyro_noise=1.0,
        imu__accel_noise=0.5,
        imu__gyro_random_walk=0.0,
        imu__accel_random_walk=0.0,
        odometry__enabled=True,
        odometry__meas_noise_covar_diag=[0.01, 0.01, 0.01, 0.01, 0.05],
    )
    params["base_frame"] = Parameter("base_frame", value="base_link")
    params["odom_frame"] = Parameter("odom_frame", value="odom")
    return PoseEstimator(parameters=params)


def _odom(t: float, x: float, y: float, yaw: float) -> Odometry:
    """Build an Odometry message with no covariance (so the default noise is used)."""
    msg = Odometry()
    msg.header.stamp = stamp(t)
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = yaw_to_quaternion(yaw)
    return msg


def test_output_frames():
    """The filtered odometry is expressed in odom -> base_link."""
    msg = _estimator().current_odom_filtered_pose
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"


def test_converges_to_repeated_odometry():
    """Repeated odometry measurements pull the state onto the measured pose."""
    estimator = _estimator()
    for i in range(100):
        estimator.push_odometry_msg(_odom(1.0 + 0.01 * i, x=1.0, y=2.0, yaw=0.5))
    pose = estimator.current_odom_filtered_pose.pose.pose
    assert pose.position.x == pytest.approx(1.0, abs=0.05)
    assert pose.position.y == pytest.approx(2.0, abs=0.05)
    assert quaternion_to_yaw(pose.orientation) == pytest.approx(0.5, abs=0.05)


def test_stale_odometry_is_ignored():
    """An odometry message older than the last one does not change the state."""
    estimator = _estimator()
    estimator.push_odometry_msg(_odom(2.0, x=1.0, y=0.0, yaw=0.0))
    before = estimator.current_odom_filtered_pose.pose.pose.position.x
    estimator.push_odometry_msg(_odom(1.0, x=50.0, y=0.0, yaw=0.0))
    assert estimator.current_odom_filtered_pose.pose.pose.position.x == pytest.approx(before)


def test_imu_in_wrong_frame_is_ignored():
    """IMU samples that were not transformed into base_frame are skipped."""
    estimator = _estimator()
    for i in range(3):
        imu = Imu()
        imu.header.stamp = stamp(1.0 + 0.01 * i)
        imu.header.frame_id = "chassis_imu"
        imu.linear_acceleration.x = 5.0
        estimator.push_imu_msg(imu)
    assert estimator.current_odom_filtered_pose.twist.twist.linear.x == pytest.approx(0.0)
