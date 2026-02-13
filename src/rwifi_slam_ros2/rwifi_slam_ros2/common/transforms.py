"""TF2 and quaternion transformations helpers for the Range WiFi SLAM node."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformStamped


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle to a ``geometry_msgs/Quaternion``.

    Assumes zero roll and pitch (rotation about the Z-axis only).

    Args:
        yaw: Yaw angle in radians.

    Returns:
        Quaternion with ``x=0, y=0`` and ``z, w`` set from *yaw*.
    """
    return Quaternion(
        x=0.0,
        y=0.0,
        z=float(np.sin(yaw / 2)),
        w=float(np.cos(yaw / 2)),
    )


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract the yaw angle (rotation about Z) from a quaternion.

    Works for arbitrary 3D orientations;

    Args:
        q: Input quaternion (w, x, y, z).

    Returns:
        Yaw angle in radians on ``(-pi, pi]``.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return float(yaw)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval ``[-pi, pi]``.

    Args:
        angle: Angle in radians.

    Returns:
        Equivalent angle in ``[-pi, pi]``.
    """
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def quaternion_to_rotation_matrix(q: Quaternion) -> np.ndarray:
    """Convert a ``geometry_msgs/Quaternion`` to a 3x3 rotation matrix.

    The resulting matrix represents the body-to-world rotation.

    Args:
        q: Input quaternion (scalar-last: x, y, z, w).

    Returns:
        3x3 rotation matrix as a NumPy array.
    """
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def rotate_vector_by_quaternion(q: Quaternion, v: Vector3) -> Vector3:
    """Rotate a 3-D vector by a unit quaternion.

    Args:
        q: Unit quaternion defining the rotation.
        v: Vector to rotate.

    Returns:
        Rotated vector as a ``geometry_msgs/Vector3``.
    """
    r = Rotation.from_quat([q.x, q.y, q.z, q.w])
    v_rot = r.apply([v.x, v.y, v.z])
    return Vector3(x=v_rot[0], y=v_rot[1], z=v_rot[2])


def quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Compose two quaternions (Hamilton product ``q1 * q2``).

    Args:
        q1: Left quaternion.
        q2: Right quaternion.

    Returns:
        Product quaternion as a ``geometry_msgs/Quaternion``.
    """
    r1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
    r2 = Rotation.from_quat([q2.x, q2.y, q2.z, q2.w])
    xyzw = (r1 * r2).as_quat()
    return Quaternion(x=xyzw[0], y=xyzw[1], z=xyzw[2], w=xyzw[3])


def transform_imu_with_tf(msg: Imu, tf: TransformStamped, to_frame: str) -> Imu:
    """Transform an IMU message with a TF2 transform.

    Args:
        msg: Input IMU message.
        tf: TF2 transform.
        to_frame: Target frame.

    Returns:
        Transformed IMU message.
    """
    rot = tf.transform.rotation
    rot_q = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)

    out = Imu()

    out.header.stamp.sec = msg.header.stamp.sec
    out.header.stamp.nanosec = msg.header.stamp.nanosec
    out.header.frame_id = to_frame
    out.orientation = quaternion_multiply(rot_q, msg.orientation)
    out.orientation_covariance = msg.orientation_covariance
    out.angular_velocity = rotate_vector_by_quaternion(rot_q, msg.angular_velocity)
    out.angular_velocity_covariance = msg.angular_velocity_covariance
    out.linear_acceleration = rotate_vector_by_quaternion(rot_q, msg.linear_acceleration)
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance

    return out


def get_static_transform(
    tf_buffer: Buffer,
    from_frame: str,
    to_frame: str,
) -> TransformStamped:
    """Get a static transform between two frames.

    Args:
        tf_buffer: TF2 buffer to look up transforms.
        from_frame: Source TF frame.
        to_frame: Target TF frame.

    Returns:
        Transform between the frames.

    Raises:
        TransformException: If the transform is unavailable.
    """
    try:
        transform = tf_buffer.lookup_transform(
            to_frame,
            from_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0),
        )
    except TransformException as e:
        raise TransformException(f"Transform {from_frame} -> {to_frame} unavailable: {e}") from e

    return transform
