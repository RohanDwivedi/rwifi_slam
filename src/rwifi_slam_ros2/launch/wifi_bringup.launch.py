"""Launch Wi-Fi RTT publisher with optional simulation or Android backend."""

from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


def _get_simulation_dir() -> str:
    """Return the path to the simulation/isaac_sim_dev directory.

    When the launch file is run from the install space (e.g. path contains
    '/install/'), resolves relative to the workspace root. Otherwise resolves
    relative to this launch file.

    Returns:
        Absolute path to the simulation/isaac_sim_dev directory.
    """
    launch_file = os.path.abspath(__file__)
    if "/install/" in launch_file:
        workspace = launch_file.split("/install/")[0]
        return os.path.join(workspace, "src", "simulation", "isaac_sim_dev")
    launch_dir = os.path.dirname(launch_file)
    return os.path.join(launch_dir, "..", "..", "simulation", "isaac_sim_dev")


_ISAAC_SIM_PYTHON_SH = os.path.join("isaac-sim-6.0", "IsaacSim", "_build", "linux-x86_64", "release", "python.sh")


def _launch_isaac_sim(
    context: LaunchContext,
    *args: Any,
    **kwargs: Any,
) -> list[ExecuteProcess]:
    """Launch Isaac Sim (main.py) when backend is simulation and type is isaac_sim.

    Args:
        context: Launch context; used to read config, headless, and visualize_nlos.
        *args: Unused.
        **kwargs: Unused.

    Returns:
        List containing one ExecuteProcess that runs Isaac Sim, or an empty list
        if launch_simulation is not 'true'.
    """
    if context.launch_configurations.get("launch_simulation", "true") != "true":
        return []
    sim_dir = os.path.abspath(_get_simulation_dir())
    python_sh = os.path.join(sim_dir, _ISAAC_SIM_PYTHON_SH)
    main_py = os.path.join(sim_dir, "main.py")
    cmd = [
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


def _launch_gazebo(
    context: LaunchContext,
    *args: Any,
    **kwargs: Any,
) -> None:
    """Launch Gazebo when simulation_type is gazebo.

    Not implemented; raises NotImplementedError.

    Args:
        context: Launch context (unused).
        *args: Unused.
        **kwargs: Unused.

    Raises:
        NotImplementedError: Gazebo is not supported.
    """
    raise NotImplementedError("Gazebo simulation type is not implemented yet")


def _launch_simulation_and_wifi_node(
    context: LaunchContext,
    *args: Any,
    **kwargs: Any,
) -> list[ExecuteProcess | Node]:
    """Launch the simulation process (if enabled) and the Wi-Fi RTT publisher node.

    Reads backend and simulation_type from launch_configurations. For
    backend=simulation and simulation_type=isaac_sim, optionally starts Isaac Sim,
    then adds the wifi_rtt_publisher node and a static_transform_publisher
    (odom -> base_link).

    Args:
        context: Launch context; used to read launch_configurations.
        *args: Unused.
        **kwargs: Unused.

    Returns:
        List of launch actions: optionally one ExecuteProcess for Isaac Sim, then
        Node for wifi_rtt_publisher, then Node for static_transform_publisher.

    Raises:
        NotImplementedError: If backend is 'android' or simulation_type is
            'gazebo'.
    """
    backend = context.launch_configurations.get("backend", "simulation")
    simulation_type = context.launch_configurations.get("simulation_type", "isaac_sim")

    if backend == "android":
        raise NotImplementedError("Android backend is not implemented yet")
    if backend == "simulation" and simulation_type == "gazebo":
        raise NotImplementedError("Gazebo simulation type is not implemented yet")

    actions = []

    if backend == "simulation":
        if simulation_type == "isaac_sim":
            actions.extend(_launch_isaac_sim(context, *args, **kwargs))
        elif simulation_type == "gazebo":
            actions.extend(_launch_gazebo(context, *args, **kwargs))

    assert backend == "simulation" and simulation_type == "isaac_sim"
    backend_type = "simulation_isaac_sim"
    use_sim_time = False

    wifi_rtt_config_path = context.launch_configurations["wifi_rtt_config_path"]

    node_param_overrides = {
        "use_sim_time": use_sim_time,
        "backend_type": backend_type,
    }
    if backend_type == "simulation_isaac_sim":
        node_param_overrides["simulation_isaac_sim.nlos_map_file"] = context.launch_configurations["nlos_map_file"]

    actions.append(
        Node(
            package="wifi_rtt_interface",
            executable="wifi_rtt_publisher",
            name="wifi_rtt_publisher",
            output="screen",
            parameters=[wifi_rtt_config_path, node_param_overrides],
        )
    )
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

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for Wi-Fi RTT bringup.

    Declares launch arguments (backend, simulation_type, config, headless,
    visualize_nlos, launch_simulation, nlos_map_file, wifi_rtt_config_path) and
    registers an OpaqueFunction that invokes _launch_simulation_and_wifi_node
    to start the simulation (if applicable) and the publisher node.

    Returns:
        LaunchDescription with all arguments and the opaque function.
    """
    sim_dir = os.path.abspath(_get_simulation_dir())
    default_config = os.path.join(sim_dir, "configs", "isaac_sim_warehouse_config.yaml")
    default_nlos_map = os.path.join(sim_dir, "configs", "warehouse_nlos_map.pkl")

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="simulation",
        description=("Backend for Wi-Fi RTT: 'simulation' (default) or 'android'"),
    )
    simulation_type_arg = DeclareLaunchArgument(
        "simulation_type",
        default_value="isaac_sim",
        description=("When backend is simulation: 'isaac_sim' (default) or 'gazebo'"),
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to Isaac Sim config YAML (simulation_type=isaac_sim)",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Isaac Sim in headless mode",
    )
    visualize_nlos_arg = DeclareLaunchArgument(
        "visualize_nlos",
        default_value="false",
        description="Visualize NLOS map (AP coverage) in Isaac Sim viewport",
    )
    launch_sim_arg = DeclareLaunchArgument(
        "launch_simulation",
        default_value="true",
        description="If true, start the simulation process.",
    )
    nlos_map_arg = DeclareLaunchArgument(
        "nlos_map_file",
        default_value=default_nlos_map,
        description="Path to NLOS map pickle for Wi-Fi RTT (Isaac Sim).",
    )
    wifi_rtt_config_path_arg = DeclareLaunchArgument(
        "wifi_rtt_config_path",
        default_value=os.path.join(
            get_package_share_directory("wifi_rtt_interface"),
            "config",
            "isaac_sim_warehouse_config.yaml",
        ),
        description=("Path to Wi-Fi RTT config YAML (simulation + isaac_sim)."),
    )

    return LaunchDescription(
        [
            backend_arg,
            simulation_type_arg,
            config_arg,
            headless_arg,
            visualize_nlos_arg,
            launch_sim_arg,
            nlos_map_arg,
            wifi_rtt_config_path_arg,
            OpaqueFunction(function=_launch_simulation_and_wifi_node),
        ]
    )
