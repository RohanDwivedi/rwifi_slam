"""QoS profiles for the Range Wi-Fi SLAM node.

SENSOR_QOS_PROFILE: Best-effort, keep-last 10 (IMU, odometry, Wi-Fi, depth).
POSE_QOS_PROFILE: Reliable, keep-last 1 (pose output).
MAP_QOS_PROFILE: Reliable, transient-local, keep-last 10 (map, trajectory).
"""

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

SENSOR_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

POSE_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

MAP_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
