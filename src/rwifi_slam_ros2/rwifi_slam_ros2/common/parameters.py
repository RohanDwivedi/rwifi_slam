"""Parameter declarations for the Range WiFi SLAM node.

Each parameter is declared with a default value and a
:class:`ParameterDescriptor` that documents its purpose, type, and
valid range.
"""

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, ParameterType
from rclpy.node import Node


def declare_parameters(node: Node) -> None:
    """Declare all ROS parameters on the given node.

    Parameters are grouped into three sections:

    * **Global** -- frame IDs, trajectory history length.
    * **Pose estimator / IMU** -- noise and bias characteristics.
    * **Pose estimator / Odometry** -- measurement noise covariance.

    Args:
        node: The ROS 2 node on which to declare the parameters.
    """
    # -- Global --------------------------------------------------------
    node.declare_parameter(
        "map_frame",
        "map",
        ParameterDescriptor(
            description="TF frame used for the map origin.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "base_frame",
        "base_link",
        ParameterDescriptor(
            description="TF frame attached to the robot base.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "odom_frame",
        "odom",
        ParameterDescriptor(
            description="TF frame used for odometry.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "imu_frame",
        "chassis_imu",
        ParameterDescriptor(
            description="TF frame of the IMU sensor.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "depth_frame",
        "camera_pseudo_depth",
        ParameterDescriptor(
            description="TF frame of the depth camera.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "wifi_frame",
        "chassis_wifi",
        ParameterDescriptor(
            description="TF frame of the Wi-Fi antenna.",
            type=ParameterType.PARAMETER_STRING,
        ),
    )
    node.declare_parameter(
        "trajectory_keep_history",
        100,
        ParameterDescriptor(
            description=("Maximum number of poses retained in the trajectory."),
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=0, to_value=1000, step=1)],
        ),
    )

    # -- Pose estimator: initial covariance ----------------------------
    node.declare_parameter(
        "pose_estimator.initial_covar_mat_diag",
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        ParameterDescriptor(
            description=(
                "Initial covariance diagonal "
                "[X, Y, VX, VY, YAW, b_ax, b_ay, b_gz]. "
                "Must be a non-negative array of length 8."
            ),
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            array_length=8,
        ),
    )

    # -- Pose estimator: IMU -------------------------------------------
    node.declare_parameter(
        "pose_estimator.imu.gyro_noise",
        0.001,
        ParameterDescriptor(
            description="Gyroscope white-noise density (rad/s/sqrt(Hz)).",
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "pose_estimator.imu.accel_noise",
        0.001,
        ParameterDescriptor(
            description=("Accelerometer white-noise density (m/s^2/sqrt(Hz))."),
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "pose_estimator.imu.gyro_random_walk",
        0.001,
        ParameterDescriptor(
            description="Gyroscope bias random-walk (rad/s^2/sqrt(Hz)).",
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "pose_estimator.imu.accel_random_walk",
        0.001,
        ParameterDescriptor(
            description=("Accelerometer bias random-walk (m/s^3/sqrt(Hz))."),
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )

    # -- Pose estimator: odometry --------------------------------------
    node.declare_parameter(
        "pose_estimator.odometry.meas_noise_covar_diag",
        [0.01, 0.01, 0.01, 0.01, 0.01],
        ParameterDescriptor(
            description=(
                "Odometry measurement-noise covariance diagonal " "[X, Y, VX, VY, YAW]. Must be non-negative, length 5."
            ),
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            array_length=5,
        ),
    )
    node.declare_parameter(
        "pose_estimator.odometry.enabled",
        True,
        ParameterDescriptor(
            description="Enable odometry measurements in the EKF.",
            type=ParameterType.PARAMETER_BOOL,
        ),
    )

    # -- Wi-Fi RTT clustering -------------------------------------------
    node.declare_parameter(
        "wifi_rtt_clustering.distance_interval",
        0.1,
        ParameterDescriptor(
            description=("Minimum distance (in meters) the robot must move before creating a new Wi-Fi RTT keyframe."),
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.yaw_interval",
        0.0174533,  # 1 degree
        ParameterDescriptor(
            description=(
                "Minimum yaw angle change (in radians) the robot must rotate "
                "before creating a new Wi-Fi RTT keyframe."
            ),
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.distance_method",
        "euclidean",
        ParameterDescriptor(
            description="Distance method for Wi-Fi RTT clustering",
            type=ParameterType.PARAMETER_STRING,
            additional_constraints="Options: euclidean",
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.clustering_threshold",
        1.0,
        ParameterDescriptor(
            description=("Threshold for Wi-Fi RTT clustering"),
            type=ParameterType.PARAMETER_DOUBLE,
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.use_ratio_test",
        True,
        ParameterDescriptor(
            description="Use ratio test for Wi-Fi RTT clustering",
            type=ParameterType.PARAMETER_BOOL,
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.ratio_threshold",
        0.8,
        ParameterDescriptor(
            description="Ratio threshold for Wi-Fi RTT clustering",
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)],
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.exclude_near_neighbors",
        True,
        ParameterDescriptor(
            description="Exclude near neighbors for Wi-Fi RTT clustering",
            type=ParameterType.PARAMETER_BOOL,
        ),
    )
    node.declare_parameter(
        "wifi_rtt_clustering.near_neighbor_threshold",
        10,
        ParameterDescriptor(
            description="Threshold for excluding near neighbors for Wi-Fi RTT clustering",
            type=ParameterType.PARAMETER_INTEGER,
        ),
    )
