"""Launch Wi-Fi RTT publisher with optional simulation backend.

Supports Isaac Sim simulation (default).  Android and Gazebo backends are
declared but not yet implemented.
"""

from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch_ros.actions import Node

_ISAAC_SIM_PYTHON_SH = os.path.join(
    "isaac-sim-6.0",
    "IsaacSim",
    "_build",
    "linux-x86_64",
    "release",
    "python.sh",
)


def _get_simulation_dir() -> str:
    """Return the absolute path to ``simulation/isaac_sim_dev``.

    Resolves relative to the workspace root when running from the install
    space (path contains ``/install/``), otherwise relative to this
    launch file.

    Returns:
        Absolute path to the simulation directory.
    """
    launch_file = os.path.abspath(__file__)
    if "/install/" in launch_file:
        workspace = launch_file.split("/install/")[0]
        return os.path.join(workspace, "src", "simulation", "isaac_sim_dev")
    launch_dir = os.path.dirname(launch_file)
    return os.path.join(launch_dir, "..", "..", "simulation", "isaac_sim_dev")


def _launch_isaac_sim(
    context: LaunchContext,
    *_args: Any,
    **_kwargs: Any,
) -> list[ExecuteProcess]:
    """Optionally start the Isaac Sim process.

    Args:
        context: Launch context used to read configuration values.
        *_args: Unused positional arguments.
        **_kwargs: Unused keyword arguments.

    Returns:
        A single-element list with the ``ExecuteProcess`` action, or an
        empty list if ``launch_simulation`` is not ``"true"``.
    """
    if context.launch_configurations.get("launch_simulation", "true") != "true":
        return []

    sim_dir = os.path.abspath(_get_simulation_dir())
    python_sh = os.path.join(sim_dir, _ISAAC_SIM_PYTHON_SH)
    main_py = os.path.join(sim_dir, "main.py")

    cmd: list[str] = [
        python_sh,
        main_py,
        "--config",
        context.launch_configurations["config"],
    ]
    if context.launch_configurations.get("headless", "false") == "true":
        cmd.append("--headless")
    if context.launch_configurations.get("visualize_nlos", "false") == "true":
        cmd.append("--visualize-nlos")

    return [
        ExecuteProcess(
            cmd=cmd,
            cwd=sim_dir,
            additional_env={"ISAAC_SIM_PATH": sim_dir},
            shell=False,
        )
    ]


def _launch_simulation_and_wifi_node(
    context: LaunchContext,
    *args: Any,
    **kwargs: Any,
) -> list[ExecuteProcess | Node]:
    """Launch the simulation (if enabled) and the Wi-Fi RTT publisher node.

    Reads ``backend`` and ``simulation_type`` from launch configurations.
    Currently only ``backend='simulation'`` with
    ``simulation_type='isaac_sim'`` is supported.

    Args:
        context: Launch context providing configuration values.
        *args: Forwarded to ``_launch_isaac_sim``.
        **kwargs: Forwarded to ``_launch_isaac_sim``.

    Returns:
        List of launch actions (Isaac Sim process, Wi-Fi RTT publisher
        node, and static transform publishers).

    Raises:
        NotImplementedError: If backend is ``'android'`` or simulation
            type is ``'gazebo'``.
        ValueError: For unsupported backend / simulation-type
            combinations.
    """
    backend = context.launch_configurations.get("backend", "simulation")
    sim_type = context.launch_configurations.get("simulation_type", "isaac_sim")

    if backend == "android":
        raise NotImplementedError("Android backend is not yet supported.")
    if backend == "simulation" and sim_type == "gazebo":
        raise NotImplementedError("Gazebo simulation is not yet supported.")
    if backend != "simulation" or sim_type != "isaac_sim":
        raise ValueError(
            f"Unsupported backend/simulation_type: "
            f"'{backend}'/'{sim_type}'. Only "
            f"'simulation'/'isaac_sim' is currently supported."
        )

    actions: list[ExecuteProcess | Node] = []

    actions.extend(_launch_isaac_sim(context, *args, **kwargs))

    backend_type = "simulation_isaac_sim"
    wifi_rtt_config = context.launch_configurations["wifi_rtt_config_path"]

    node_params: dict[str, Any] = {
        "use_sim_time": False,
        "backend_type": backend_type,
        "simulation_isaac_sim.nlos_map_file": (context.launch_configurations["nlos_map_file"]),
    }

    actions.append(
        Node(
            package="wifi_rtt_interface",
            executable="wifi_rtt_publisher",
            name="wifi_rtt_publisher",
            output="screen",
            parameters=[wifi_rtt_config, node_params],
        )
    )

    # Static TF: odom -> base_link (identity)
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "odom",
                "--child-frame-id",
                "base_link",
            ],
        )
    )

    # Static TF: base_link -> chassis_wifi (antenna height offset)
    antenna_z = str(float(context.launch_configurations.get("antenna_height", "0.6")))
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_chassis_wifi",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                antenna_z,
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "chassis_wifi",
            ],
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for Wi-Fi RTT bringup.

    Declares launch arguments and registers an ``OpaqueFunction`` that
    starts the simulation (if applicable) and the publisher node.

    Returns:
        Complete ``LaunchDescription`` with all arguments.
    """
    sim_dir = os.path.abspath(_get_simulation_dir())
    default_config = os.path.join(sim_dir, "configs", "isaac_sim_warehouse_config.yaml")
    default_nlos_map = os.path.join(sim_dir, "configs", "warehouse_nlos_map.pkl")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="simulation",
                description=("Backend for Wi-Fi RTT: 'simulation' or 'android'."),
            ),
            DeclareLaunchArgument(
                "simulation_type",
                default_value="isaac_sim",
                description=("Simulation type: 'isaac_sim' or 'gazebo' " "(only when backend='simulation')."),
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description=("Path to Isaac Sim config YAML (simulation_type=isaac_sim)."),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Isaac Sim in headless mode.",
            ),
            DeclareLaunchArgument(
                "visualize_nlos",
                default_value="false",
                description=("Visualise NLOS map (AP coverage) in Isaac Sim."),
            ),
            DeclareLaunchArgument(
                "launch_simulation",
                default_value="true",
                description="If true, start the simulation process.",
            ),
            DeclareLaunchArgument(
                "nlos_map_file",
                default_value=default_nlos_map,
                description=("Path to NLOS-map pickle for Wi-Fi RTT (Isaac Sim)."),
            ),
            DeclareLaunchArgument(
                "wifi_rtt_config_path",
                default_value=os.path.join(
                    get_package_share_directory("wifi_rtt_interface"),
                    "config",
                    "isaac_sim_warehouse_config.yaml",
                ),
                description=("Path to Wi-Fi RTT config YAML (simulation + isaac_sim)."),
            ),
            DeclareLaunchArgument(
                "antenna_height",
                default_value="0.6",
                description=("Z-offset of chassis_wifi from base_link (metres)."),
            ),
            OpaqueFunction(function=_launch_simulation_and_wifi_node),
        ]
    )
