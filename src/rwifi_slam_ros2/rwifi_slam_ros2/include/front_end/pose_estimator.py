"""Pose estimator using an Extended Kalman Filter (EKF).

State vector (8-D):
    [X, Y, VX, VY, YAW, BIAS_ACC_X, BIAS_ACC_Y, BIAS_GYR_Z]

The predict step fuses IMU measurements (gyroscope + accelerometer). Update
steps fuse odometry.
"""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common import (
    is_zero_timestamp,
    quaternion_to_yaw,
    stamp_to_sec,
    wrap_angle,
    yaw_to_quaternion,
    zero_timestamp,
)
from sensor_msgs.msg import Imu

_STATE_DIM = 8
_I_X, _I_Y, _I_VX, _I_VY, _I_YAW = 0, 1, 2, 3, 4
_I_BIAS_AX, _I_BIAS_AY, _I_BIAS_GZ = 5, 6, 7


class PoseEstimator:
    """Extended Kalman Filter for pose estimation using IMU and odometry."""

    def __init__(self, parameters: dict[str, Parameter]) -> None:
        """Initialize the pose estimator with given parameters.

        Args:
            parameters: Dictionary of ROS parameters for the pose estimator.
        """
        self._logger = get_logger(self.__class__.__name__)

        self._gyro_rw: float = parameters["imu.gyro_random_walk"].value
        self._accel_rw: float = parameters["imu.accel_random_walk"].value
        self._gyro_noise: float = parameters["imu.gyro_noise"].value
        self._accel_noise: float = parameters["imu.accel_noise"].value
        self._initial_covar_diag: list[float] = parameters["initial_covar_mat_diag"].value
        self._meas_noise_odom_diag: list[float] = parameters["odometry.meas_noise_covar_diag"].value
        self._odometry_enabled: bool = parameters["odometry.enabled"].value

        self._base_frame: str = parameters["base_frame"].value
        self._odom_frame: str = parameters["odom_frame"].value

        self._last_imu_stamp = zero_timestamp()
        self._last_odom_stamp = zero_timestamp()
        self._last_update_stamp = zero_timestamp()

        self._init_filter()

    def _init_filter(self) -> None:
        """Initialize the EKF state and covariance matrices."""
        self._state = np.zeros(_STATE_DIM, dtype=np.float64)
        self._state_covar = np.diag(self._initial_covar_diag).astype(np.float64)
        self._meas_noise_odom_covar = np.diag(self._meas_noise_odom_diag).astype(np.float64)
        self._last_accel_body = np.zeros(2, dtype=np.float64)

        self._logger.info("Filter initialised.")
        self._logger.debug(f"State: {self._state}")
        self._logger.debug(f"State covariance: {self._state_covar}")
        self._logger.debug(f"Measurement noise covariance: {self._meas_noise_odom_covar}")

    @property
    def current_odom_filtered_pose(self) -> Odometry:
        """Get the current filtered pose as an Odometry message.

        Returns:
            Odometry message containing the current filtered pose and velocity.
        """
        return self._build_odom_msg()

    def push_imu_msg(self, msg: Imu) -> None:
        """Process an IMU message and update the filter prediction step.

        Args:
            msg: IMU message containing accelerometer and gyroscope data.
        """
        if self._is_stale_timestamp(msg.header.stamp, self._last_imu_stamp):
            self._logger.warning("IMU stamp older than last IMU; skipping.")
            return

        if not self._in_base_frame(msg):
            self._logger.warning("IMU message is not in base frame; skipping.")
            return

        self._predict(msg)

    def push_odometry_msg(self, msg: Odometry) -> None:
        """Process an odometry message and update the filter measurement step.

        Args:
            msg: Odometry message containing pose and velocity measurements.
        """
        if not self._odometry_enabled:
            self._logger.warning("Odometry disabled; skipping update.")
            return

        if self._is_stale_timestamp(msg.header.stamp, self._last_odom_stamp):
            self._logger.warning("Odometry stamp older than last odometry; skipping.")
            return

        self._update_from_odometry(msg)

    def _build_odom_msg(self) -> Odometry:
        """Build an Odometry message from the current filter state.

        Returns:
            Odometry message containing pose, twist, and their covariances.
        """
        msg = Odometry()
        msg.header.stamp.sec = self._last_update_stamp.sec
        msg.header.stamp.nanosec = self._last_update_stamp.nanosec
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        msg.pose.pose.position = Point(x=float(self._state[_I_X]), y=float(self._state[_I_Y]), z=0.0)
        msg.pose.pose.orientation = yaw_to_quaternion(float(self._state[_I_YAW]))

        msg.twist.twist.linear.x = float(self._state[_I_VX])
        msg.twist.twist.linear.y = float(self._state[_I_VY])
        msg.twist.twist.angular.z = 0.0

        pose_cov = np.zeros((6, 6), dtype=np.float64)
        ekf_idx = [_I_X, _I_Y, _I_YAW]
        ros_idx = [0, 1, 5]
        pose_cov[np.ix_(ros_idx, ros_idx)] = self._state_covar[np.ix_(ekf_idx, ekf_idx)]
        msg.pose.covariance = pose_cov.ravel().tolist()

        twist_cov = np.zeros((6, 6), dtype=np.float64)
        ekf_tw = [_I_VX, _I_VY]
        ros_tw = [0, 1]
        twist_cov[np.ix_(ros_tw, ros_tw)] = self._state_covar[np.ix_(ekf_tw, ekf_tw)]
        msg.twist.covariance = twist_cov.ravel().tolist()

        return msg

    def _is_stale_timestamp(self, stamp: Time, last_stamp: Time) -> bool:
        """Check if timestamp is older than the last processed timestamp.

        Args:
            stamp: Timestamp to check.
            last_stamp: Last processed timestamp to compare against.

        Returns:
            True if stamp is older than last_stamp.
        """
        if is_zero_timestamp(last_stamp):
            return False
        return stamp_to_sec(stamp) < stamp_to_sec(last_stamp)

    def _check_state_health(self, label: str) -> None:
        """Check for NaN, Inf, or invalid covariance in the filter state.

        Args:
            label: Label for logging context.
        """
        if np.any(np.isnan(self._state)) or np.any(np.isinf(self._state)):
            self._logger.error(f"[{label}] NaN/Inf in state: {self._state}")

        if np.any(np.isnan(self._state_covar)) or np.any(np.isinf(self._state_covar)):
            self._logger.error(f"[{label}] NaN/Inf in covariance diag: {np.diag(self._state_covar)}")

        eig = np.linalg.eigvalsh(self._state_covar)
        if np.any(eig <= 0):
            self._logger.error(f"[{label}] Covariance not positive-definite: min_eigval={np.min(eig):.6e}")

    def _in_base_frame(self, msg: Imu) -> bool:
        """Check if IMU message is in the base frame.

        Args:
            msg: IMU message to check.

        Returns:
            True if message frame_id matches base_frame.
        """
        return msg.header.frame_id == self._base_frame

    def _advance_time(self, cur_time: Time, last_time: Time) -> float | None:
        """Compute time delta and validate timestamps.

        Args:
            cur_time: Current timestamp.
            last_time: Previous timestamp.

        Returns:
            Time delta in seconds, or None if invalid.
        """
        dt = stamp_to_sec(cur_time) - stamp_to_sec(last_time)
        if dt <= 0.0:
            self._logger.warning(f"dt={dt:.6f}s <= 0; skipping predict.")
            return None
        if dt > 1.0:
            self._logger.warning(f"dt={dt:.6f}s > 1s; possible timestamp gap.")
            self._last_imu_stamp = cur_time
            return None

        self._last_imu_stamp = cur_time
        return dt

    def _update_state(self, dt: float, msg: Imu) -> None:
        """Update the filter state using IMU measurements.

        Args:
            dt: Time delta in seconds.
            msg: IMU message containing accelerometer and gyroscope data.
        """
        accel: Vector3 = msg.linear_acceleration
        gyro: Vector3 = msg.angular_velocity

        accel_b = np.array([accel.x, accel.y, accel.z], dtype=np.float64)
        accel_b[0] -= self._state[_I_BIAS_AX]
        accel_b[1] -= self._state[_I_BIAS_AY]

        yaw = float(self._state[_I_YAW])
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        accel_w = np.array(
            [
                accel_b[0] * cos_yaw - accel_b[1] * sin_yaw,
                accel_b[0] * sin_yaw + accel_b[1] * cos_yaw,
                accel_b[2],
            ],
            dtype=np.float64,
        )

        gyro_z = float(gyro.z) - float(self._state[_I_BIAS_GZ])

        self._last_accel_body = accel_b[:2].copy()

        self._state[_I_X] += self._state[_I_VX] * dt + 0.5 * accel_w[0] * dt**2
        self._state[_I_Y] += self._state[_I_VY] * dt + 0.5 * accel_w[1] * dt**2
        self._state[_I_VX] += accel_w[0] * dt
        self._state[_I_VY] += accel_w[1] * dt
        self._state[_I_YAW] += gyro_z * dt
        self._state[_I_YAW] = wrap_angle(float(self._state[_I_YAW]))

    def _update_process_noise_covar(self, dt: float) -> np.ndarray:
        """Compute process noise covariance matrix for the prediction step.

        Args:
            dt: Time delta in seconds.

        Returns:
            Process noise covariance matrix.
        """
        accel_noise_sq: float = float(self._accel_noise) ** 2
        gyro_noise_sq: float = float(self._gyro_noise) ** 2
        gyro_rw_sq: float = float(self._gyro_rw) ** 2
        accel_rw_sq: float = float(self._accel_rw) ** 2

        q_pp: float = 0.25 * accel_noise_sq * dt**3
        q_pv: float = 0.5 * accel_noise_sq * dt**2
        q_vv: float = accel_noise_sq * dt
        q_yaw: float = gyro_noise_sq * dt
        q_bax: float = accel_rw_sq * dt
        q_bay: float = accel_rw_sq * dt
        q_bgz: float = gyro_rw_sq * dt

        process_noise_covar = np.zeros((_STATE_DIM, _STATE_DIM), dtype=np.float64)

        process_noise_covar[_I_X, _I_X] = q_pp
        process_noise_covar[_I_X, _I_VX] = q_pv
        process_noise_covar[_I_VX, _I_X] = q_pv
        process_noise_covar[_I_VX, _I_VX] = q_vv

        process_noise_covar[_I_Y, _I_Y] = q_pp
        process_noise_covar[_I_Y, _I_VY] = q_pv
        process_noise_covar[_I_VY, _I_Y] = q_pv
        process_noise_covar[_I_VY, _I_VY] = q_vv

        process_noise_covar[_I_YAW, _I_YAW] = q_yaw

        process_noise_covar[_I_BIAS_AX, _I_BIAS_AX] = q_bax
        process_noise_covar[_I_BIAS_AY, _I_BIAS_AY] = q_bay
        process_noise_covar[_I_BIAS_GZ, _I_BIAS_GZ] = q_bgz

        return process_noise_covar

    def _update_state_jacobian(self, dt: float) -> np.ndarray:
        """Compute state transition Jacobian matrix.

        Args:
            dt: Time delta in seconds.

        Returns:
            State transition Jacobian matrix.
        """
        state_jacobian = np.eye(_STATE_DIM, dtype=np.float64)
        state_jacobian[_I_X, _I_VX] = dt
        state_jacobian[_I_Y, _I_VY] = dt

        yaw = float(self._state[_I_YAW])
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))

        state_jacobian[_I_VX, _I_BIAS_AX] = -cos_yaw * dt
        state_jacobian[_I_VX, _I_BIAS_AY] = sin_yaw * dt
        state_jacobian[_I_VY, _I_BIAS_AX] = -sin_yaw * dt
        state_jacobian[_I_VY, _I_BIAS_AY] = -cos_yaw * dt
        state_jacobian[_I_X, _I_BIAS_AX] = -cos_yaw * 0.5 * dt**2
        state_jacobian[_I_X, _I_BIAS_AY] = sin_yaw * 0.5 * dt**2
        state_jacobian[_I_Y, _I_BIAS_AX] = -sin_yaw * 0.5 * dt**2
        state_jacobian[_I_Y, _I_BIAS_AY] = -cos_yaw * 0.5 * dt**2

        state_jacobian[_I_YAW, _I_BIAS_GZ] = -dt

        ab = self._last_accel_body
        dax_dyaw = -ab[0] * sin_yaw - ab[1] * cos_yaw
        day_dyaw = ab[0] * cos_yaw - ab[1] * sin_yaw
        state_jacobian[_I_VX, _I_YAW] = dax_dyaw * dt
        state_jacobian[_I_VY, _I_YAW] = day_dyaw * dt
        state_jacobian[_I_X, _I_YAW] = dax_dyaw * 0.5 * dt**2
        state_jacobian[_I_Y, _I_YAW] = day_dyaw * 0.5 * dt**2

        return state_jacobian

    def _compute_innovation(
        self, measured_state: np.ndarray, predicted_state: np.ndarray, yaw_index: int
    ) -> np.ndarray:
        """Compute innovation (measurement residual).

        Args:
            measured_state: Measured state vector.
            predicted_state: Predicted state vector.
            yaw_index: Index of yaw angle in state vector.

        Returns:
            Innovation vector with wrapped yaw angle.
        """
        innovation = measured_state - predicted_state
        innovation[yaw_index] = wrap_angle(float(innovation[yaw_index]))
        return innovation

    def _compute_kalman_gain(self, innovation_covar: np.ndarray, observation_jacobian: np.ndarray) -> np.ndarray:
        """Compute Kalman gain matrix.

        Args:
            innovation_covar: Innovation covariance matrix.
            observation_jacobian: Observation Jacobian matrix.

        Returns:
            Kalman gain matrix.
        """
        return np.linalg.solve(innovation_covar, observation_jacobian @ self._state_covar).T

    def _update_state_from_observation(self, innovation: np.ndarray, kalman_gain: np.ndarray) -> None:
        """Update state vector using innovation and Kalman gain.

        Args:
            innovation: Innovation vector.
            kalman_gain: Kalman gain matrix.
        """
        self._state += kalman_gain @ innovation
        self._state[_I_YAW] = wrap_angle(float(self._state[_I_YAW]))

    def _update_state_covar_from_observation(
        self, meas_noise_covar: np.ndarray, kalman_gain: np.ndarray, observation_jacobian: np.ndarray
    ) -> None:
        """Update state covariance matrix using Kalman filter update equation.

        Args:
            meas_noise_covar: Measurement noise covariance matrix.
            kalman_gain: Kalman gain matrix.
            observation_jacobian: Observation Jacobian matrix.
        """
        I_KH = np.eye(_STATE_DIM, dtype=np.float64) - kalman_gain @ observation_jacobian
        self._state_covar = I_KH @ self._state_covar @ I_KH.T + kalman_gain @ meas_noise_covar @ kalman_gain.T
        self._state_covar = 0.5 * (self._state_covar + self._state_covar.T)

        eigvals = np.linalg.eigvalsh(self._state_covar)
        min_eigval = float(np.min(eigvals))
        if min_eigval <= 0.0:
            regularization = abs(min_eigval) + 1e-8
            self._state_covar += np.eye(_STATE_DIM, dtype=np.float64) * regularization
            self._logger.warning(
                f"Covariance update produced non-positive-definite matrix (min_eigval={min_eigval:.6e}), "
                f"added regularization {regularization:.6e}"
            )

    def _predict(self, msg: Imu) -> None:
        """Perform EKF prediction step using IMU measurements.

        Args:
            msg: IMU message containing accelerometer and gyroscope data.
        """
        if is_zero_timestamp(self._last_imu_stamp):
            self._last_imu_stamp = msg.header.stamp
            self._last_update_stamp = msg.header.stamp
            return

        dt = self._advance_time(cur_time=msg.header.stamp, last_time=self._last_imu_stamp)
        if dt is None:
            return

        self._update_state(dt, msg)
        process_noise_covar = self._update_process_noise_covar(dt)
        state_jacobian = self._update_state_jacobian(dt)
        self._state_covar = state_jacobian @ self._state_covar @ state_jacobian.T + process_noise_covar
        self._last_update_stamp = msg.header.stamp
        self._check_state_health("predict")

    def _extract_odom_covariance(self, msg: Odometry) -> np.ndarray:
        """Extract and validate odometry covariance from message.

        Args:
            msg: Odometry message.

        Returns:
            Measurement noise covariance matrix.
        """
        meas_cov = np.diag(self._meas_noise_odom_diag).astype(np.float64)

        pose_cov_6x6 = np.array(list(msg.pose.covariance), dtype=np.float64).reshape(6, 6)
        ros_pose_idx = [0, 1, 5]
        pose_cov_3x3 = pose_cov_6x6[np.ix_(ros_pose_idx, ros_pose_idx)]

        if (
            not np.any(np.isnan(pose_cov_3x3))
            and not np.any(np.isinf(pose_cov_3x3))
            and not np.any(np.diag(pose_cov_3x3) < 0)
            and np.all(np.linalg.eigvalsh(pose_cov_3x3) > 0)
        ):
            map_idx = [0, 1, 4]
            meas_cov[np.ix_(map_idx, map_idx)] = pose_cov_3x3

        twist_cov_6x6 = np.array(list(msg.twist.covariance), dtype=np.float64).reshape(6, 6)
        ros_twist_idx = [0, 1]
        twist_cov_2x2 = twist_cov_6x6[np.ix_(ros_twist_idx, ros_twist_idx)]

        if (
            not np.any(np.isnan(twist_cov_2x2))
            and not np.any(np.isinf(twist_cov_2x2))
            and not np.any(np.diag(twist_cov_2x2) < 0)
            and np.all(np.linalg.eigvalsh(twist_cov_2x2) > 0)
        ):
            vel_idx = [2, 3]
            meas_cov[np.ix_(vel_idx, vel_idx)] = twist_cov_2x2

        return meas_cov

    def _update_from_odometry(self, msg: Odometry) -> None:
        """Perform EKF update step using odometry measurements.

        Args:
            msg: Odometry message containing pose and velocity measurements.
        """
        self._last_odom_stamp = msg.header.stamp

        x_meas: float = float(msg.pose.pose.position.x)
        y_meas: float = float(msg.pose.pose.position.y)
        yaw_meas: float = float(quaternion_to_yaw(msg.pose.pose.orientation))

        vx_body: float = float(msg.twist.twist.linear.x)
        vy_body: float = float(msg.twist.twist.linear.y)
        cos_yaw = float(np.cos(yaw_meas))
        sin_yaw = float(np.sin(yaw_meas))
        vx_world: float = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world: float = vx_body * sin_yaw + vy_body * cos_yaw

        measured_state = np.array([x_meas, y_meas, vx_world, vy_world, yaw_meas], dtype=np.float64)
        predicted_state = np.array(
            [self._state[_I_X], self._state[_I_Y], self._state[_I_VX], self._state[_I_VY], self._state[_I_YAW]],
            dtype=np.float64,
        )

        observation_jacobian = np.zeros((5, _STATE_DIM), dtype=np.float64)
        observation_jacobian[0, _I_X] = 1.0
        observation_jacobian[1, _I_Y] = 1.0
        observation_jacobian[2, _I_VX] = 1.0
        observation_jacobian[3, _I_VY] = 1.0
        observation_jacobian[4, _I_YAW] = 1.0

        meas_noise_covar = self._extract_odom_covariance(msg)
        innovation = self._compute_innovation(measured_state, predicted_state, yaw_index=4)
        innovation_covar = observation_jacobian @ self._state_covar @ observation_jacobian.T + meas_noise_covar
        kalman_gain = self._compute_kalman_gain(innovation_covar, observation_jacobian)

        self._update_state_from_observation(innovation, kalman_gain)
        self._update_state_covar_from_observation(meas_noise_covar, kalman_gain, observation_jacobian)

        self._last_update_stamp = msg.header.stamp
        self._check_state_health("update")
