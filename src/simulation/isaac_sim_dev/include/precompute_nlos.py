"""Precompute NLOS map over a 2D grid for Wi-Fi RTT simulation in Isaac Sim.

Raycasts from each access point to each grid cell; counts obstructions and
computes environmental bias. Measurement noise is applied at runtime, not stored.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.util.debug_draw import _debug_draw as debug_draw
from omni.physx import get_physx_scene_query_interface
from pxr import Usd

from wifi_rtt_interface.nlos_map_schema import (NLOSMap, NLOSMapFile,
                                                NLOSObservation)
from wifi_rtt_interface.rng import RNG

MU_NLOS = 2.0
SIGMA_NLOS = 0.7
LINEAR_WEIGHT = 0.02


class PrecomputeNLOS:
    """Builds an NLOS map by raycasting over a 2D grid for each access point."""

    def __init__(
        self,
        access_points: list[dict[str, Any]],
        scene_prim: Usd.Prim,
        antenna_height: float,
        resolution: float,
        simulation_app: Any,
    ) -> None:
        """Initialize the precomputer with scene and grid parameters.

        Args:
            access_points: List of dicts with 'mac_address' and 'position' (3D).
            scene_prim: USD prim for the scene; used to compute axis-aligned bounds.
            antenna_height: Z coordinate (meters) for grid cell centers.
            resolution: Grid cell size (meters).
            simulation_app: Isaac Sim application handle.
        """
        self.nlos_map_file: NLOSMapFile | None = None
        self.access_points = access_points
        self.scene_prim = scene_prim
        self.antenna_height = antenna_height
        self.resolution = resolution
        self.simulation_app = simulation_app

    def _compute_bbox(self, prim: Usd.Prim) -> np.ndarray:
        """Compute axis-aligned bounding box for the given prim.

        Args:
            prim: USD prim to query.

        Returns:
            Array of shape (6,) [xmin, ymin, zmin, xmax, ymax, zmax].
        """
        bb_cache = create_bbox_cache()
        bounds = compute_aabb(bb_cache, prim.GetPath())
        return np.array(bounds, dtype=np.float64)

    def _compute_grid_centers(
        self, scene_bbox: np.ndarray, resolution: float
    ) -> np.ndarray:
        """Compute (N, 3) grid cell centers at z = antenna_height.

        Args:
            scene_bbox: (6,) array [xmin, ymin, zmin, xmax, ymax, zmax].
            resolution: Grid cell size (meters).

        Returns:
            Array of shape (N, 3) with x, y, z for each cell center.

        Raises:
            ValueError: If resolution is not positive.
        """
        if resolution <= 0:
            raise ValueError("resolution must be > 0")

        bmin, bmax = scene_bbox[0:3], scene_bbox[3:6]
        z = float(self.antenna_height)

        x_centers = np.arange(
            bmin[0] + resolution / 2,
            bmax[0] - resolution / 2 + 1e-12,
            resolution,
        )
        y_centers = np.arange(
            bmin[1] + resolution / 2,
            bmax[1] - resolution / 2 + 1e-12,
            resolution,
        )

        xx, yy = np.meshgrid(x_centers, y_centers, indexing="ij")
        return np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, z))).astype(
            np.float64
        )

    def _count_obstructions(
        self,
        origin: np.ndarray,
        direction: np.ndarray,
        distance: float,
    ) -> int:
        """Raycast from origin along direction and count unique rigid bodies hit.

        Args:
            origin: (3,) array, ray origin.
            direction: (3,) array, unit direction.
            distance: Maximum ray length.

        Returns:
            Number of unique rigid bodies intersecting the ray.
        """
        hit_bodies: list[Any] = []

        def report_hit(hit: Any) -> bool:
            if hit.rigid_body:
                hit_bodies.append(hit.rigid_body)
            return True

        get_physx_scene_query_interface().raycast_all(
            origin, direction, distance, report_hit, bothSides=True
        )
        return len(sorted(set(hit_bodies)))

    def precompute_nlos_map(self) -> NLOSMapFile:
        """Build the NLOS map for all access points over the grid.

        For each AP and each grid cell: raycasts to determine LOS/NLOS and
        obstruction count, then computes environmental bias (and optional
        distance-based drift). Hardware bias is sampled once per AP. Measurement
        noise is not stored; it is sampled at runtime in the simulation backend.

        Returns:
            Filled NLOSMapFile with one NLOSMap per access point.
        """
        self.nlos_map_file = NLOSMapFile()
        scene_bbox = self._compute_bbox(self.scene_prim)
        grid_centers = self._compute_grid_centers(scene_bbox, self.resolution)

        for access_point in self.access_points:
            mac = access_point["mac_address"]
            ap_pos = np.array(access_point["position"], dtype=np.float64)
            hardware_bias = float(RNG.normal(0.0, 0.3))

            ap_map = NLOSMap(
                access_point_mac_address=mac,
                access_point_hardware_bias=hardware_bias,
                access_point_position=ap_pos,
            )

            for grid_center in grid_centers:
                true_dist = float(np.linalg.norm(grid_center - ap_pos))

                if true_dist < 1e-6:
                    is_los, n_blocks = True, 0
                else:
                    direction = (grid_center - ap_pos) / true_dist
                    n_blocks = self._count_obstructions(ap_pos, direction, true_dist)
                    is_los = n_blocks == 0

                if is_los:
                    bias_val = 0.0
                else:
                    cumulative_delay = sum(
                        RNG.normal(MU_NLOS, SIGMA_NLOS) for _ in range(n_blocks)
                    )
                    bias_val = float(cumulative_delay) + (LINEAR_WEIGHT * true_dist)

                obs = NLOSObservation(
                    grid_cell_center=grid_center.copy(),
                    los=is_los,
                    nlos_bias=bias_val,
                )
                ap_map.nlos_observations.append(obs)

            self.nlos_map_file.nlos_map.append(ap_map)

        return self.nlos_map_file

    @staticmethod
    def draw_all_aps_field(
        nlos_map_file: NLOSMapFile,
        point_size: int = 30,
        alpha: float = 0.9,
    ) -> None:
        """Draw the grid in the Isaac Sim viewport (green = LOS, red = NLOS).

        For each cell, color is blended by the fraction of APs that see NLOS
        at that cell.

        Args:
            nlos_map_file: Loaded NLOS map to visualize.
            point_size: Size of each point in pixels.
            alpha: Alpha for point color.
        """
        dd = debug_draw.acquire_debug_draw_interface()
        dd.clear_points()

        if not nlos_map_file.nlos_map:
            return

        num_aps = len(nlos_map_file.nlos_map)
        num_cells = len(nlos_map_file.nlos_map[0].nlos_observations)

        all_pts: list[list[float]] = []
        all_rgba: list[list[float]] = []

        for cell_idx in range(num_cells):
            pt = nlos_map_file.nlos_map[0].nlos_observations[cell_idx].grid_cell_center
            all_pts.append([float(pt[0]), float(pt[1]), 0.1])

            nlos_count = sum(
                1
                for ap_map in nlos_map_file.nlos_map
                if not ap_map.nlos_observations[cell_idx].los
            )
            fraction_nlos = nlos_count / num_aps
            all_rgba.append([fraction_nlos, 1.0 - fraction_nlos, 0.0, alpha])

        dd.draw_points(all_pts, all_rgba, [point_size] * num_cells)
