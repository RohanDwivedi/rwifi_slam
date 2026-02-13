"""ROS2 timestamp utility functions."""

from builtin_interfaces.msg import Time


def stamp_to_sec(stamp: Time) -> float:
    """Convert ROS2 timestamp to seconds as float.

    Args:
        stamp: ROS2 timestamp.

    Returns:
        Timestamp in seconds as float.
    """
    return stamp.sec + stamp.nanosec * 1e-9


def zero_timestamp() -> Time:
    """Create a zero timestamp.

    Returns:
        Time message with sec=0 and nanosec=0.
    """
    return Time(sec=0, nanosec=0)


def is_zero_timestamp(stamp: Time) -> bool:
    """Check if timestamp is zero.

    Args:
        stamp: ROS2 timestamp.

    Returns:
        True if timestamp is zero, False otherwise.
    """
    return stamp.sec == 0 and stamp.nanosec == 0
