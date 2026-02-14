"""Top-level launch file for the Range WiFi SLAM stack.

Includes the Wi-Fi RTT bringup launch (simulation + publisher), starts the
``rwifi_slam_node`` lifecycle node, and uses a ``nav2_lifecycle_manager`` to
automatically transition the node through configure -> activate.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the full rwifi_slam stack.

    Includes ``wifi_bringup.launch.py`` (simulation + Wi-Fi RTT publisher)
    and launches the ``rwifi_slam_node`` with parameters from
    ``rwifi_slam.parameters.yaml``.  A ``nav2_lifecycle_manager`` handles
    lifecycle transitions automatically.

    Returns:
        Complete ``LaunchDescription`` with arguments and actions.
    """
    pkg_share = get_package_share_directory("rwifi_slam_ros2")

    wifi_bringup_launch = os.path.join(pkg_share, "launch", "wifi_bringup.launch.py")
    rwifi_slam_params = os.path.join(pkg_share, "config", "rwifi_slam.parameters.yaml")

    # -- Launch arguments (forwarded to wifi_bringup) ------------------
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="simulation",
        description=("Backend for Wi-Fi RTT: 'simulation' or 'android'."),
    )
    simulation_type_arg = DeclareLaunchArgument(
        "simulation_type",
        default_value="isaac_sim",
        description=("Simulation type: 'isaac_sim' or 'gazebo' (only when backend='simulation')."),
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Isaac Sim in headless mode.",
    )
    launch_sim_arg = DeclareLaunchArgument(
        "launch_simulation",
        default_value="true",
        description="If true, start the simulation process.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=("Automatically configure and activate lifecycle nodes."),
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz visualisation.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description=("Log level for nodes: 'debug', 'info', 'warn', 'error', 'fatal'."),
    )

    # -- Include wifi_bringup launch -----------------------------------
    wifi_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wifi_bringup_launch),
        launch_arguments={
            "backend": LaunchConfiguration("backend"),
            "simulation_type": LaunchConfiguration("simulation_type"),
            "headless": LaunchConfiguration("headless"),
            "launch_simulation": LaunchConfiguration("launch_simulation"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # -- rwifi_slam_node (pose estimator) ------------------------------
    rwifi_slam_node = LifecycleNode(
        package="rwifi_slam_ros2",
        executable="rwifi_slam_node",
        name="rwifi_slam_node",
        namespace="",
        output="screen",
        parameters=[rwifi_slam_params],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # -- Nav2 lifecycle manager (auto configure -> activate) -----------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {
                "autostart": LaunchConfiguration("autostart"),
                "node_names": ["rwifi_slam_node"],
                "bond_timeout": 0.0,
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # -- RViz ----------------------------------------------------------
    rviz_config_path = os.path.join(pkg_share, "rviz", "rwifi_slam.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_path,
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            backend_arg,
            simulation_type_arg,
            headless_arg,
            launch_sim_arg,
            autostart_arg,
            use_rviz_arg,
            log_level_arg,
            wifi_bringup,
            rwifi_slam_node,
            lifecycle_manager,
            rviz_node,
        ]
    )
