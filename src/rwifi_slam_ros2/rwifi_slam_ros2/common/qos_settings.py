"""Quality-of-service profiles declarations for the Range WiFi SLAM node."""

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Best-effort QoS for high-frequency sensor streams (IMU, odometry).
SENSOR_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Reliable QoS for the latest pose output.
POSE_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Reliable, transient-local QoS for map and trajectory topics.
MAP_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
