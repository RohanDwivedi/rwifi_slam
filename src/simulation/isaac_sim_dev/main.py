"""
Entrypoint for Isaac Sim environment for Wi-Fi RTT / wild_slam_ros2 simulation.

Loads config, spawns scene/robot/APs, precomputes NLOS map, and runs the sim loop.
"""

import argparse
import os

import yaml

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
CONFIG_DIR = os.path.join(os.path.dirname(__file__), "configs")
SIMULATION_WARMUP_TICKS = 100

arguments = argparse.ArgumentParser()
arguments.add_argument(
    "--config",
    type=str,
    default=os.path.join(CONFIG_DIR, "isaac_sim_warehouse_config.yaml"),
    help="Path to the Isaac Sim configuration file",
)
arguments.add_argument(
    "--headless",
    action="store_true",
    help="Run the simulation in headless mode",
)
arguments.add_argument(
    "--visualize-nlos",
    action="store_true",
    help="Visualize the NLOS map (AP coverage) in the viewport",
)

args = arguments.parse_args()

config_file: str = args.config
headless: bool = args.headless
visualize_nlos: bool = args.visualize_nlos

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

# Isaac Sim requires SimulationApp before any other isaacsim/omni imports.
from isaacsim import SimulationApp  # noqa: E402, I100

simulation_app = SimulationApp({"headless": headless})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from include.precompute_nlos import NLOSMapFile, PrecomputeNLOS  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.experimental.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import Articulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

enable_extension("isaacsim.ros2.bridge")

world = World(stage_units_in_meters=1.0)

scene_path = get_assets_root_path() + scene["usd_path"]
scene_prim_path = "/World/" + scene["name"]
scene_prim = add_reference_to_stage(usd_path=scene_path, path=scene_prim_path)

robot_path = get_assets_root_path() + robot["usd_path"]
robot_prim_path = "/World/" + robot["name"]

robot_spawn = robot.get("spawn_point", {})
position = np.array(robot_spawn.get("position", [0.0, 0.0, 0.0]), dtype=np.float64)
orientation = np.array(
    robot_spawn.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=np.float64
)
positions = position[None, :]
orientations = orientation[None, :]

robot_prim = add_reference_to_stage(usd_path=robot_path, path=robot_prim_path)
robot_articulation = Articulation(prim_paths_expr=robot_prim_path, name=robot["name"])
world.scene.add(robot_articulation)
robot_articulation.set_world_poses(
    positions=positions,
    orientations=orientations,
)

for access_point in access_points:
    access_point_path = os.path.join(
        ASSETS_DIR, "sensors", "wifi_access_point", "unify_ap.usda"
    )
    access_point_prim_path = "/World/" + access_point["name"]
    add_reference_to_stage(usd_path=access_point_path, path=access_point_prim_path)
    access_point_prim = SingleXFormPrim(
        access_point_prim_path,
        name=access_point["name"],
        position=np.array(access_point["position"], dtype=np.float64),
        orientation=np.array(access_point["orientation"], dtype=np.float64),
        scale=np.array([0.1, 0.1, 0.1], dtype=np.float64),
    )
    world.scene.add(access_point_prim)


def precompute_nlos_map() -> None:
    """Load or compute the NLOS map.

    Loads from CONFIG_DIR/<scene_name>_nlos_map.pkl if it exists; otherwise
    computes via PrecomputeNLOS and saves. Recompute by deleting the pickle.
    """
    nlos_map_path = os.path.join(CONFIG_DIR, scene["name"] + "_nlos_map.pkl")
    if os.path.exists(nlos_map_path):
        nlos_map_file = NLOSMapFile.load(nlos_map_path)
    else:
        print("Computing NLOS map... Patience is a virtue...")
        precompute_nlos = PrecomputeNLOS(
            access_points,
            scene_prim,
            float(robot["antenna_height"]),
            float(scene["resolution"]),
            simulation_app,
        )
        nlos_map_file = precompute_nlos.precompute_nlos_map()
        nlos_map_file.save(nlos_map_path)

    if visualize_nlos:
        PrecomputeNLOS.draw_all_aps_field(nlos_map_file)


omni.timeline.get_timeline_interface().play()

for _ in range(SIMULATION_WARMUP_TICKS):
    simulation_app.update()

precompute_nlos_map()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
