import argparse
import math
import os
import sys
import threading
from typing import Any, Dict, List, Optional, Tuple

import omni
import omni.usd
import rclpy
import yaml
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import open_stage
from pxr import Gf, Usd, UsdGeom
from rclpy.node import Node


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
    """Convert roll, pitch, yaw (degrees) to quaternion (x, y, z, w)."""
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    # Calculate quaternion from Euler angles
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Isaac Sim bootstrap: load scene, spawn robot, init ROS."
    )

    p.add_argument("--robot", required=True, help="Robot name (e.g., nova_carter)")
    p.add_argument("--scene", required=True, help="Scene name (e.g., warehouse)")
    p.add_argument(
        "--headless",
        type=str,
        default="false",
        help="Run in headless mode (true/false)",
    )

    return p.parse_args()


def _set_prim_pose(
    stage: Usd.Stage, prim_path: str, pos_xyz: List[float], quat_xyzw: List[float]
) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found at path: {prim_path}")

    xform = UsdGeom.Xformable(prim)

    # Clear existing ops for deterministic pose
    for op in xform.GetOrderedXformOps():
        xform.RemoveXformOp(op)

    t_op = xform.AddTranslateOp()
    o_op = xform.AddOrientOp()

    t_op.Set(Gf.Vec3d(pos_xyz[0], pos_xyz[1], pos_xyz[2]))

    # pxr Gf.Quatf is (w, x, y, z)
    qx, qy, qz, qw = quat_xyzw
    o_op.Set(Gf.Quatf(float(qw), float(qx), float(qy), float(qz)))


class MinimalRosNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_sim_bootstrap")
        self.get_logger().info("rclpy initialized (RMW from env).")


def _start_ros_spin() -> Tuple[Optional[MinimalRosNode], Optional[threading.Thread]]:
    rclpy.init(args=None)
    node = MinimalRosNode()

    def _spin():
        try:
            rclpy.spin(node)
        except Exception:
            node.get_logger().exception("ROS spin loop error")
        finally:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return node, th


def main() -> None:
    args = _parse_args()

    # Load robot config file
    config_path = f"/workspace/src/simulation/isaac_sim/configs/{args.robot}.yaml"
    if not os.path.exists(config_path):
        raise RuntimeError(f"Robot config file not found: {config_path}")

    robot_cfg = _load_yaml(config_path)
    spawn_point = robot_cfg.get("spawn_point", {})
    position_cfg = spawn_point.get("position", {})
    rotation_cfg = spawn_point.get("rotation", {})

    # Get spawn position
    pos = [
        float(position_cfg.get("x", 0.0)),
        float(position_cfg.get("y", 0.0)),
        float(position_cfg.get("z", 0.0)),
    ]

    # Get spawn rotation and convert to quaternion
    roll = float(rotation_cfg.get("roll", 0.0))
    pitch = float(rotation_cfg.get("pitch", 0.0))
    yaw = float(rotation_cfg.get("yaw", 0.0))
    quat = _rpy_to_quat(roll, pitch, yaw)

    # Construct USD paths (can be overridden via environment variables)
    scene_usd = os.getenv(
        "ISAAC_SIM_SCENE_USD",
        f"/workspace/src/simulation/isaac_sim/assets/scenes/{args.scene}.usd",
    )
    robot_usd = os.getenv(
        "ISAAC_SIM_ROBOT_USD",
        f"/workspace/src/simulation/isaac_sim/assets/robots/{args.robot}.usd",
    )
    prim_path = os.getenv(
        "ISAAC_SIM_PRIM_PATH", f"/World/{args.robot.replace('_', '').title()}"
    )

    # Set headless mode from argument
    if args.headless.lower() in ("true", "1", "yes"):
        os.environ["OMNI_ISAAC_SIM_HEADLESS"] = "1"

    # Initialize ROS
    ros_node, _ = _start_ros_spin()

    # Load scene
    if not os.path.exists(scene_usd):
        raise RuntimeError(f"Scene USD file not found: {scene_usd}")
    open_stage(scene_usd)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Failed to acquire USD stage from omni.usd context.")

    # Create robot prim by referencing its USD
    if not os.path.exists(robot_usd):
        raise RuntimeError(f"Robot USD file not found: {robot_usd}")
    create_prim(
        prim_path=prim_path,
        usd_path=robot_usd,
        translation=pos,
        orientation=quat,
    )

    # Ensure pose is set deterministically (some USDs carry their own transform ops)
    _set_prim_pose(stage, prim_path, pos, quat)

    # Keep Kit running
    app = omni.kit.app.get_app()
    while app.is_running():
        app.update()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # Print to stderr so it shows up in container logs.
        print(f"[FATAL] main.py error: {e}", file=sys.stderr)
        raise
