"""Entrypoint for Isaac Sim Environment for wild_slam_ros2 simulation."""

import argparse
import os

import yaml

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
CONFIG_DIR = os.path.join(os.path.dirname(__file__), "configs")

arguments = argparse.ArgumentParser()
arguments.add_argument(
    "--config",
    type=str,
    default=os.path.join(CONFIG_DIR, "isaac_sim_config.yaml"),
    help="Path to the Isaac Sim configuration file",
)
arguments.add_argument(
    "--headless", action="store_true", help="Run the simulation in headless mode"
)
args = arguments.parse_args()

config_file: str = args.config
headless: bool = args.headless

robot: dict | None = None
scene: dict | None = None
access_points: list[dict] | None = None

if not os.path.isfile(config_file):
    raise FileNotFoundError(f"Config file not found: {config_file}")

with open(config_file, "r") as file:
    config = yaml.safe_load(file)
    if "robot" not in config or "scene" not in config or "access_points" not in config:
        raise KeyError("One or more required keys not found in config file")
    robot = config["robot"]
    scene = config["scene"]
    access_points = config["access_points"]

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless})

import numpy as np
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.rotations import euler_angles_to_quat

enable_extension("isaacsim.ros2.bridge")

world = World(stage_units_in_meters=1.0)


def _parse_orientation(orientation: list[float]) -> np.ndarray:
    orientation_np = np.asarray(orientation, dtype=np.float64)
    if orientation_np.shape == (4,):
        if np.max(np.abs(orientation_np)) <= 1.0 + 1e-3:
            # Heuristic: treat as xyzw if last component is dominant (common in configs).
            if np.argmax(np.abs(orientation_np)) == 3:
                return orientation_np[[3, 0, 1, 2]]
            return orientation_np
        if np.allclose(orientation_np[:3], 0.0):
            return euler_angles_to_quat(
                np.array([0.0, 0.0, orientation_np[3]]), degrees=True
            )
        raise ValueError(
            f"Orientation {orientation} must be a quaternion (w,x,y,z) or yaw degrees in the last element."
        )
    if orientation_np.shape == (3,):
        return euler_angles_to_quat(orientation_np, degrees=True)
    raise ValueError(
        f"Orientation {orientation} must be length 3 (Euler degrees) or 4 (quaternion)."
    )

# Spawn the scene
scene_path = get_assets_root_path() + scene["usd_path"]
scene_prim_path = "/World/" + scene["name"]
add_reference_to_stage(usd_path=scene_path, path=scene_prim_path)

# Spawn the robot
robot_path = get_assets_root_path() + robot["usd_path"]
robot_prim_path = "/World/" + robot["name"]

robot_spawn = robot.get("spawn_point", {})
position = np.array(robot_spawn.get("position", [0.0, 0.0, 0.0]), dtype=np.float64)
orientation = _parse_orientation(robot_spawn.get("orientation", [1.0, 0.0, 0.0, 0.0]))

add_reference_to_stage(usd_path=robot_path, path=robot_prim_path)
robot_articulation = Articulation(prim_paths_expr=robot_prim_path, name=robot["name"])
world.scene.add(robot_articulation)
robot_articulation.set_world_poses(
    positions=np.array([position], dtype=np.float32),
    orientations=np.array([orientation], dtype=np.float32),
)

# Spawn the access points
for access_point in access_points:
    access_point_path = os.path.join(
        ASSETS_DIR, "sensors", "wifi_access_point", "unify_ap.usda"
    )
    access_point_prim_path = "/World/" + access_point["name"]
    add_reference_to_stage(usd_path=access_point_path, path=access_point_prim_path)
    access_point_prim = SingleXFormPrim(
        prim_path=access_point_prim_path,
        name=access_point["name"],
        position=np.array(access_point["position"], dtype=np.float64),
        orientation=_parse_orientation(access_point["orientation"]),
        scale=np.array([0.1, 0.1, 0.1], dtype=np.float64),
    )
    world.scene.add(access_point_prim)

omni.timeline.get_timeline_interface().play()
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
