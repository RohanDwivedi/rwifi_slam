from __future__ import annotations

import pickle
from dataclasses import dataclass, field

import carb
import numpy as np
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.util.debug_draw import _debug_draw as debug_draw
from omni.physx import get_physx_scene_query_interface
from pxr import Usd

# fixed seed for reproducibility
SEED = 42
rng = np.random.default_rng(seed=SEED)
MU_NLOS = 2.0
SIGMA_NLOS = 0.7


@dataclass
class NLOSObservation:
    los: bool = True
    nlos_bias: float = 0.0
    noise: float = 0.2
    grid_cell_center: np.ndarray | None = None


@dataclass
class NLOSMap:
    access_point_mac_address: str
    acess_point_hardware_bias: float  # moved here
    nlos_observations: list[NLOSObservation] = field(default_factory=list)


@dataclass
class NLOSMapFile:
    nlos_map: list[NLOSMap] = field(default_factory=list)

    @classmethod
    def load(cls, path: str) -> "NLOSMapFile":
        with open(path, "rb") as f:
            data = pickle.load(f)
        if not isinstance(data, cls):
            raise TypeError(f"Expected {cls.__name__} in pickle, got {type(data)}")
        print(f"Loaded {cls.__name__} from {path}")
        return data

    def save(self, path: str) -> None:
        with open(path, "wb") as f:
            pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"Saved {self.__class__.__name__} to {path}")


class PrecomputeNLOS:
    def __init__(
        self,
        acess_points: list[dict],
        scene_prim: Usd.Prim,
        antenna_height: float,
        resolution: float,
        simulation_app,
    ):
        self.nlos_map_file: NLOSMapFile | None = None
        self.acess_points = acess_points
        self.scene_prim = scene_prim
        self.antenna_height = antenna_height
        self.resolution = resolution
        self.simulation_app = simulation_app

    def _compute_bbox(self, prim: Usd.Prim) -> np.ndarray:
        bb_cache = create_bbox_cache()
        bounds = compute_aabb(bb_cache, prim.GetPath())
        return np.array(bounds, dtype=np.float64)

    def _compute_grid_centers(
        self, scene_bbox: np.ndarray, resolution: float
    ) -> np.ndarray:
        if resolution <= 0:
            raise ValueError("resolution must be > 0")

        bmin = scene_bbox[0:3]
        bmax = scene_bbox[3:6]
        z = float(self.antenna_height)

        x_centers = np.arange(
            bmin[0] + resolution / 2, bmax[0] - resolution / 2 + 1e-12, resolution
        )
        y_centers = np.arange(
            bmin[1] + resolution / 2, bmax[1] - resolution / 2 + 1e-12, resolution
        )

        xx, yy = np.meshgrid(x_centers, y_centers, indexing="ij")
        centers = np.column_stack(
            (xx.ravel(), yy.ravel(), np.full(xx.size, z, dtype=np.float64))
        ).astype(np.float64)

        return centers

    def _is_los(
        self,
        grid_center: np.ndarray,
        access_point_position: np.ndarray,
        true_distance: float,
    ) -> bool:
        if true_distance <= 1e-12:
            return True

        origin = access_point_position
        # Normalize direction
        ray_direction = (grid_center - access_point_position) / true_distance

        # We use a radius (e.g., 0.2m) to simulate the 'thickness' of the signal path.
        # This catches edges of racks/walls that a single ray might miss.
        hit = get_physx_scene_query_interface().raycast_closest(
            origin, ray_direction, true_distance
        )

        return not bool(hit["hit"])

    @staticmethod
    def draw_all_aps_stacked(
        nlos_map_file,
        point_size=15.0,
        vmax=5.0,
        alpha=0.9,
        z_step=0.08,
        z_offset=0.02,
    ):
        """
        Draws all AP NLOS maps at once.
        For each grid cell center, draws one point per AP, stacked in Z.
        Color encodes nlos_bias (blue->red).
        """
        dd = debug_draw.acquire_debug_draw_interface()
        dd.clear_points()

        all_pts = []
        all_rgba = []
        all_sizes = []

        for ap_idx, ap_map in enumerate(nlos_map_file.nlos_map):
            # Extract positions and values for this AP
            pts_np = np.array(
                [obs.grid_cell_center for obs in ap_map.nlos_observations],
                dtype=np.float64,
            )
            vals = np.array(
                [float(obs.nlos_bias) for obs in ap_map.nlos_observations],
                dtype=np.float64,
            )

            # Stack layer in Z so APs don't overlap perfectly
            pts_np[:, 2] += z_offset + ap_idx * z_step

            # Normalize 0..1 using fixed vmax so colors comparable across APs
            t = np.clip(vals / float(vmax), 0.0, 1.0)

            # Blue->red RGBA
            rgba = np.stack(
                [t, np.zeros_like(t), 1.0 - t, np.full_like(t, alpha)], axis=-1
            )

            all_pts.extend(pts_np.tolist())
            all_rgba.extend(rgba.tolist())
            all_sizes.extend((np.ones(len(vals)) * point_size).tolist())

        dd.draw_points(all_pts, all_rgba, all_sizes)

    def precompute_nlos_map(self) -> NLOSMapFile:
        self.nlos_map_file = NLOSMapFile()

        scene_bbox = self._compute_bbox(self.scene_prim)
        grid_centers = self._compute_grid_centers(scene_bbox, self.resolution)

        for access_point in self.acess_points:
            mac = access_point["mac_address"]
            access_point_position = np.array(access_point["position"], dtype=np.float64)

            # generate once per AP
            hardware_bias = float(rng.normal(0.0, 0.3))

            ap_map = NLOSMap(
                access_point_mac_address=mac,
                acess_point_hardware_bias=hardware_bias,
            )

            for grid_center in grid_centers:
                true_distance = float(
                    np.linalg.norm(grid_center - access_point_position)
                )
                los = self._is_los(grid_center, access_point_position, true_distance)

                if not los:
                    nlos_bias = max(
                        0.0,
                        float(rng.normal(MU_NLOS, SIGMA_NLOS)) + (0.01 * true_distance),
                    )
                    noise = 0.4
                else:
                    nlos_bias = 0.2 + (0.01 * true_distance)
                    noise = 0.2

                obs = NLOSObservation(
                    grid_cell_center=grid_center.copy(),
                    los=los,
                    nlos_bias=nlos_bias,
                    noise=noise,
                )
                ap_map.nlos_observations.append(obs)

            self.nlos_map_file.nlos_map.append(ap_map)

        return self.nlos_map_file
