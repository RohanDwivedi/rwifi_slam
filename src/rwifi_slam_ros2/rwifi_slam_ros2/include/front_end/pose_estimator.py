"""Pose estimator using an Extended Kalman Filter (EKF).

State vector (8-D):
    [X, Y, VX, VY, YAW, BIAS_GYR_Z, BIAS_ACC_X, BIAS_ACC_Y]

The predict step fuses IMU measurements (gyroscope + accelerometer with
gravity compensation).  Update steps fuse odometry.
"""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common import quaternion_to_rotation_matrix, quaternion_to_yaw, wrap_angle, yaw_to_quaternion
from sensor_msgs.msg import Imu

# Indices into the 8-D state vector.
_IX, _IY, _IVX, _IVY, _IYAW = 0, 1, 2, 3, 4
_IBIAS_GZ, _IBIAS_AX, _IBIAS_AY = 5, 6, 7
_STATE_DIM = 8


def _stamp_to_sec(stamp: Time) -> float:
    """Convert a ROS ``Time`` stamp to seconds as a float."""
    return stamp.sec + stamp.nanosec * 1e-9


class PoseEstimator:
    """EKF-based 2-D pose estimator fusing IMU and wheel odometry.

    Attributes:
        current_odom_filtered_pose: Latest fused pose as a ROS Odometry
            message (header fields left blank for the caller to fill).
    """

    def __init__(self, parameters: dict[str, Parameter]) -> None:
        """Initialise the estimator from ROS parameters.

        Args:
            parameters: Parameter dict keyed by suffix (e.g.
                ``"imu.gravity"``, ``"odometry.enabled"``).  Obtained via
                ``Node.get_parameters_by_prefix("pose_estimator")``.
        """
        self._logger = get_logger(self.__class__.__name__)

        self._gravity: float = parameters["imu.gravity"].value
        self._gyro_rw: float = parameters["imu.gyro_random_walk"].value
        self._accel_rw: float = parameters["imu.accel_random_walk"].value
        self._gyro_noise: float = parameters["imu.gyro_noise"].value
        self._accel_noise: float = parameters["imu.accel_noise"].value
        self._initial_covar_diag: list[float] = parameters["initial_covar_mat_diag"].value
        self._meas_noise_odom_diag: list[float] = parameters["odometry.meas_noise_covar_diag"].value
        self._odometry_enabled: bool = parameters["odometry.enabled"].value

        self._init_filter()

    # ------------------------------------------------------------------
    # Filter initialisation
    # ------------------------------------------------------------------

    def _init_filter(self) -> None:
        """Reset state, covariance, and timestamps to initial values."""
        self._last_imu_stamp: Time | None = None
        self._cur_imu_stamp: Time | None = None
        self._last_update_stamp: Time | None = None

        # State: [X, Y, VX, VY, YAW, b_gz, b_ax, b_ay]
        self._state: np.ndarray = np.zeros(_STATE_DIM)

        # P (8x8) -- initial covariance
        self._covar: np.ndarray = np.diag(self._initial_covar_diag)

        # R_odom (3x3) for [X, Y, YAW] measurements
        self._R_odom: np.ndarray = np.diag(self._meas_noise_odom_diag)

        self._logger.debug("Filter reset to initial state.")

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def current_odom_filtered_pose(self) -> Odometry:
        """Return the latest fused pose as a ``nav_msgs/Odometry`` message.

        The caller is responsible for filling ``header.stamp``,
        ``header.frame_id``, and ``child_frame_id``.
        """
        return self._build_odom_msg()

    def push_imu_msg(self, msg: Imu) -> None:
        """Run an EKF predict step with the given IMU measurement.

        Args:
            msg: IMU message already transformed into ``base_frame``.
        """
        self._predict(msg)

    def push_odometry_msg(self, msg: Odometry) -> None:
        """Run an EKF update step with the given wheel-odometry measurement.

        Silently drops the message when odometry is disabled or when its
        timestamp is older than the last processed IMU sample.

        Args:
            msg: Wheel-odometry message.
        """
        if not self._odometry_enabled:
            self._logger.warning("Odometry disabled; skipping update.")
            return

        if self._is_stale(msg.header.stamp):
            self._logger.warning("Odometry stamp older than last IMU; skipping.")
            return

        self._update_from_odometry(msg)

    # ------------------------------------------------------------------
    # Message building
    # ------------------------------------------------------------------

    def _build_odom_msg(self) -> Odometry:
        """Build an ``Odometry`` message from the current state and covariance.

        Returns:
            Odometry message with pose, twist, and their 6x6 covariances
            populated
        """
        msg = Odometry()
        msg.header.stamp = self._last_update_stamp
        msg.pose.pose.position = Point(x=self._state[_IX], y=self._state[_IY], z=0.0)
        msg.pose.pose.orientation = yaw_to_quaternion(self._state[_IYAW])
        msg.twist.twist.linear.x = self._state[_IVX]
        msg.twist.twist.linear.y = self._state[_IVY]
        msg.twist.twist.angular.z = 0.0

        # 6x6 pose covariance -- ROS layout: (x, y, z, roll, pitch, yaw).
        # Map EKF indices [0,1,4] -> ROS indices [0,1,5].
        pose_cov = np.zeros((6, 6))
        ekf_idx = [_IX, _IY, _IYAW]
        ros_idx = [0, 1, 5]
        pose_cov[np.ix_(ros_idx, ros_idx)] = self._covar[np.ix_(ekf_idx, ekf_idx)]
        msg.pose.covariance = pose_cov.ravel().tolist()

        # 6x6 twist covariance -- ROS layout: (vx, vy, vz, wx, wy, wz).
        # Map EKF indices [2,3] -> ROS indices [0,1].
        twist_cov = np.zeros((6, 6))
        ekf_tw = [_IVX, _IVY]
        ros_tw = [0, 1]
        twist_cov[np.ix_(ros_tw, ros_tw)] = self._covar[np.ix_(ekf_tw, ekf_tw)]
        msg.twist.covariance = twist_cov.ravel().tolist()

        return msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _is_stale(self, stamp: Time) -> bool:
        """Return ``True`` if *stamp* predates the last processed IMU stamp.

        Args:
            stamp: Timestamp to compare.

        Returns:
            Whether the stamp is strictly older than the last IMU stamp.
        """
        if self._last_imu_stamp is None:
            return False
        return _stamp_to_sec(stamp) < _stamp_to_sec(self._last_imu_stamp)

    def _check_state_health(self, label: str) -> None:
        """Log errors/warnings if state or covariance are unhealthy.

        Args:
            label: Short label for log context (e.g. ``"predict"``).
        """
        if np.any(np.isnan(self._state)) or np.any(np.isinf(self._state)):
            self._logger.error(f"[{label}] NaN/Inf in state: {self._state}")
        if np.any(np.isnan(self._covar)) or np.any(np.isinf(self._covar)):
            self._logger.error(f"[{label}] NaN/Inf in covariance diag: {np.diag(self._covar)}")
        eig = np.linalg.eigvalsh(self._covar)
        if np.any(eig <= 0):
            self._logger.warning(f"[{label}] Covariance not positive-definite: " f"min_eigval={np.min(eig):.6e}")

    # ------------------------------------------------------------------
    # EKF predict
    # ------------------------------------------------------------------

    def _predict(self, msg: Imu) -> None:
        """EKF predict step driven by a single IMU sample.

        Uses a semi-implicit Euler integrator.  Gravity is compensated by
        projecting the world-frame gravity vector into the body frame via
        the IMU-reported orientation (already in ``base_frame``).

        Args:
            msg: IMU message in ``base_frame``.
        """
        # Bootstrap: first message only records the timestamp.
        if self._last_imu_stamp is None:
            self._last_imu_stamp = msg.header.stamp
            self._last_update_stamp = msg.header.stamp
            return

        self._cur_imu_stamp = msg.header.stamp
        dt = _stamp_to_sec(self._cur_imu_stamp) - _stamp_to_sec(self._last_imu_stamp)
        self._last_imu_stamp = self._cur_imu_stamp
        self._last_update_stamp = self._cur_imu_stamp

        if dt <= 0:
            self._logger.warning(f"dt={dt:.6f}s <= 0; skipping predict.")
            return
        if dt > 1.0:
            self._logger.warning(f"dt={dt:.6f}s > 1s; possible timestamp gap.")

        # --- State prediction ---
        a = msg.linear_acceleration
        w = msg.angular_velocity
        bax, bay = self._state[_IBIAS_AX], self._state[_IBIAS_AY]
        bgz = self._state[_IBIAS_GZ]

        # Gravity compensation: g_body = R^T @ [0, 0, g]
        # where R is the body-to-world rotation from IMU orientation.
        R = quaternion_to_rotation_matrix(msg.orientation)
        g_body = R.T @ np.array([0.0, 0.0, self._gravity])

        # Bias-corrected, gravity-compensated body-frame acceleration
        ax_b = (a.x - g_body[0]) - bax
        ay_b = (a.y - g_body[1]) - bay

        yaw = self._state[_IYAW]
        c, s = np.cos(yaw), np.sin(yaw)

        # Body -> world rotation for planar acceleration
        ax_w = c * ax_b - s * ay_b
        ay_w = s * ax_b + c * ay_b

        gz = w.z - bgz

        # Semi-implicit Euler integration
        vx = self._state[_IVX] + ax_w * dt
        vy = self._state[_IVY] + ay_w * dt

        self._state[_IX] += vx * dt
        self._state[_IY] += vy * dt
        self._state[_IVX] = vx
        self._state[_IVY] = vy
        self._state[_IYAW] += gz * dt

        # --- Process noise Q (8x8) ---
        # Continuous-time diagonal approximation discretised with dt:
        #   Position  : sigma_a^2 * dt^4 / 4  (double-integrated accel)
        #   Velocity  : sigma_a^2 * dt^2       (single-integrated accel)
        #   Yaw       : sigma_g^2 * dt^2       (integrated gyro)
        #   Gyro bias : sigma_grw^2 * dt       (random walk)
        #   Accel bias: sigma_arw^2 * dt       (random walk)
        dt2 = dt * dt
        sa2 = self._accel_noise**2
        sg2 = self._gyro_noise**2

        yaw_q = sg2 * dt2

        Q = np.diag(
            [
                sa2 * dt2 * dt2 / 4.0,  # X
                sa2 * dt2 * dt2 / 4.0,  # Y
                sa2 * dt2,  # VX
                sa2 * dt2,  # VY
                yaw_q,  # YAW (velocity-scaled)
                self._gyro_rw**2 * dt,  # BIAS_GYR_Z
                self._accel_rw**2 * dt,  # BIAS_ACC_X
                self._accel_rw**2 * dt,  # BIAS_ACC_Y
            ]
        )

        # --- Jacobian F = d(state_next)/d(state_current) ---
        # Semi-implicit Euler:
        #   vx' = vx + ax*dt,  vy' = vy + ay*dt
        #   x'  = x + vx'*dt = x + vx*dt + ax*dt^2
        #   y'  = y + vy'*dt = y + vy*dt + ay*dt^2
        #   yaw'= yaw + gz*dt
        F = np.eye(_STATE_DIM)

        # Velocity rows
        F[_IVX, _IYAW] = dt * (-s * ax_b - c * ay_b)
        F[_IVY, _IYAW] = dt * (c * ax_b - s * ay_b)
        F[_IVX, _IBIAS_AX] = -dt * c
        F[_IVX, _IBIAS_AY] = dt * s
        F[_IVY, _IBIAS_AX] = -dt * s
        F[_IVY, _IBIAS_AY] = -dt * c

        # Position rows
        F[_IX, _IVX] = dt
        F[_IY, _IVY] = dt
        F[_IX, _IYAW] = dt2 * (-s * ax_b - c * ay_b)
        F[_IY, _IYAW] = dt2 * (c * ax_b - s * ay_b)
        F[_IX, _IBIAS_AX] = -dt2 * c
        F[_IX, _IBIAS_AY] = dt2 * s
        F[_IY, _IBIAS_AX] = -dt2 * s
        F[_IY, _IBIAS_AY] = -dt2 * c

        # Yaw row
        F[_IYAW, _IBIAS_GZ] = -dt

        # Covariance propagation: P = F P F^T + Q
        self._covar = F @ self._covar @ F.T + Q

        self._check_state_health("predict")

    # ------------------------------------------------------------------
    # EKF update (odometry)
    # ------------------------------------------------------------------

    def _update_from_odometry(self, msg: Odometry) -> None:
        """EKF update using a wheel-odometry pose measurement.

        Measurement model::

            z = [x, y, yaw]        (3-D)
            h(x) = [X, Y, YAW]    (linear observation of state)

        Uses the Joseph-form covariance update for numerical stability.

        Args:
            msg: Odometry message providing (x, y, yaw) observation.
        """
        x_m = float(msg.pose.pose.position.x)
        y_m = float(msg.pose.pose.position.y)
        yaw_m = float(quaternion_to_yaw(msg.pose.pose.orientation))

        z = np.array([x_m, y_m, yaw_m])
        z_hat = np.array(
            [
                self._state[_IX],
                self._state[_IY],
                self._state[_IYAW],
            ]
        )

        # Innovation with yaw-angle wrapping
        y_res = z - z_hat
        y_res[2] = wrap_angle(y_res[2])

        # Observation Jacobian H (3x8) -- linear, so constant
        H = np.zeros((3, _STATE_DIM))
        H[0, _IX] = 1.0
        H[1, _IY] = 1.0
        H[2, _IYAW] = 1.0

        R = self._R_odom

        # Innovation covariance: S = H P H^T + R
        S = H @ self._covar @ H.T + R

        det_S = np.linalg.det(S)
        if abs(det_S) < 1e-10:
            self._logger.warning(f"Near-singular innovation covariance: det(S)={det_S:.6e}")

        # Kalman gain: K = P H^T S^{-1}
        K = np.linalg.solve(S.T, (self._covar @ H.T).T).T

        # State update
        self._state += K @ y_res
        self._state[_IYAW] = wrap_angle(float(self._state[_IYAW]))

        # Joseph-form covariance update: P = (I-KH) P (I-KH)^T + K R K^T
        I_KH = np.eye(_STATE_DIM) - K @ H
        self._covar = I_KH @ self._covar @ I_KH.T + K @ R @ K.T

        # Update timestamp to reflect this filter update
        self._last_update_stamp = msg.header.stamp

        self._check_state_health("update")
