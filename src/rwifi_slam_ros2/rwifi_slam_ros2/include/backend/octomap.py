"""Octomap backend for 3D occupancy mapping with depth and Wi-Fi RTT SLAM.

Uses the pyoctomap (OctoMap) library to maintain a 3D voxel map and project
it to a 2D nav_msgs/OccupancyGrid for use with navigation stacks.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import pyoctomap
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter


class Octomap:
    """3D occupancy octree with 2D grid projection for Wi-Fi RTT SLAM.

    Inserts point clouds in map frame and exports a 2D slice as an
    OccupancyGrid (unknown=-1, free=0, occupied=100).
    """

    def __init__(self, parameters: dict[str, Parameter]) -> None:
        """Initialize the octree and 2D grid parameters.

        Args:
            parameters: ROS parameters (height_min, height_max, resolution,
                map_frame, optional margin_m for dynamic grid bounds).
        """
        self._height_min: float = parameters["height_min"].value
        self._height_max: float = parameters["height_max"].value
        self._resolution: float = parameters["resolution"].value
        self._map_frame: str = parameters["map_frame"].value
        # Margin (m) around observed bounds when computing dynamic grid; ensures world origin (0,0) is included
        self._margin_m: float = parameters["margin_m"].value if "margin_m" in parameters else 1.0
        # Batch insertion: limit ray length (performance guide recommends e.g. 50 m)
        self._max_range: float = parameters["max_range"].value if "max_range" in parameters else -1.0
        # Dynamic environments: decay old occupancy before insert (reduces ghost artifacts)
        self._use_decay: bool = parameters["use_decay"].value if "use_decay" in parameters else False
        self._logodd_decay_value: float = (
            parameters["logodd_decay_value"].value if "logodd_decay_value" in parameters else -0.2
        )

        # Resolution: 0.1–0.2 m is a good compromise for general-purpose robotics (performance guide)
        self._octomap = pyoctomap.OcTree(self._resolution)

    def insert_pointcloud(self, points: np.ndarray, sensor_origin: np.ndarray) -> None:
        """Insert a point cloud into the octree and update occupancy.

        Uses batch insertion (insertPointCloud with lazy_eval, then a single
        updateInnerOccupancy). Optionally applies decay in the scan bbox before
        insert for dynamic environments (moving sensors). See pyoctomap
        performance guide: use batch ops, realistic resolution, max_range.

        Args:
            points: Nx3 array of point positions, shape (N, 3), dtype float64.
            sensor_origin: Sensor position (3,) or (1, 3), dtype float64.
        """
        points = np.asarray(points, dtype=np.float64)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        if points.size == 0:
            raise ValueError("empty point cloud")
        sensor_origin = np.asarray(sensor_origin, dtype=np.float64).ravel()
        if self._use_decay:
            self._octomap.decayAndInsertPointCloud(
                points,
                sensor_origin,
                logodd_decay_value=self._logodd_decay_value,
                max_range=self._max_range if self._max_range > 0 else -1.0,
                update_inner_occupancy=True,
            )
        else:
            self._octomap.insertPointCloud(
                points,
                sensor_origin,
                max_range=self._max_range if self._max_range > 0 else -1.0,
                lazy_eval=True,
            )
            self._octomap.updateInnerOccupancy()

    def to_occupancy_grid_2d(self, stamp: Optional[Time] = None) -> OccupancyGrid:
        """Project the 3D octree to a 2D occupancy grid.

        Grid bounds use the octree's metric min/max (O(1)); the grid always
        includes the world origin (0, 0). Size grows with observed data.

        Args:
            stamp: Optional ROS time for grid header.stamp (e.g. from
                get_clock().now().to_msg()). If None, header.stamp is not set.

        Returns:
            OccupancyGrid message with frame_id, resolution, origin, and data.
        """
        height_min_m = self._height_min
        height_max_m = self._height_max
        z_sample_m = (height_min_m + height_max_m) / 2.0
        half_slab = max(self._resolution * 0.5, 1e-6)
        margin_m = max(self._margin_m, self._resolution)

        # Use octree metric bounds (O(1)) instead of iterating all leaves (~5M+ → ~4.7s saved)
        min_bound = self._octomap.getMetricMin()
        max_bound = self._octomap.getMetricMax()
        has_any = min_bound[0] < max_bound[0] and min_bound[1] < max_bound[1]
        if has_any:
            min_x = float(min_bound[0])
            min_y = float(min_bound[1])
            max_x = float(max_bound[0])
            max_y = float(max_bound[1])
            # Expand to include world origin (0, 0), then add margin
            bbox_min_x = min(0.0, min_x) - margin_m
            bbox_min_y = min(0.0, min_y) - margin_m
            bbox_max_x = max(0.0, max_x) + margin_m
            bbox_max_y = max(0.0, max_y) + margin_m
            grid_origin_x = bbox_min_x
            grid_origin_y = bbox_min_y
            grid_width_m = bbox_max_x - bbox_min_x
            grid_height_m = bbox_max_y - bbox_min_y
        else:
            # Empty octomap: small grid around world origin
            grid_origin_x = -1.0
            grid_origin_y = -1.0
            grid_width_m = 2.0
            grid_height_m = 2.0

        width_cells = int(round(grid_width_m / self._resolution))
        height_cells = int(round(grid_height_m / self._resolution))
        if width_cells <= 0 or height_cells <= 0:
            grid = OccupancyGrid()
            grid.header.frame_id = self._map_frame
            if stamp is not None:
                grid.header.stamp = stamp
            grid.info.resolution = self._resolution
            grid.info.width = 0
            grid.info.height = 0
            grid.info.origin.position.x = grid_origin_x
            grid.info.origin.position.y = grid_origin_y
            grid.info.origin.position.z = 0.0
            grid.info.origin.orientation.w = 1.0
            grid.data = []
            return grid

        occupancy_data = np.full((height_cells, width_cells), -1, dtype=np.int8)
        bbx_min = np.array([grid_origin_x, grid_origin_y, z_sample_m - half_slab], dtype=np.float64)
        bbx_max = np.array(
            [
                grid_origin_x + grid_width_m,
                grid_origin_y + grid_height_m,
                z_sample_m + half_slab,
            ],
            dtype=np.float64,
        )
        for leaf in self._octomap.begin_leafs_bbx(bbx_min, bbx_max):
            coord = leaf.getCoordinate()
            col_index = int((coord[0] - grid_origin_x) / self._resolution)
            row_index = int((coord[1] - grid_origin_y) / self._resolution)
            if 0 <= row_index < height_cells and 0 <= col_index < width_cells:
                occupancy_data[row_index, col_index] = 100 if self._octomap.isNodeOccupied(leaf) else 0

        grid = OccupancyGrid()
        grid.header.frame_id = self._map_frame

        if stamp is not None:
            grid.header.stamp = stamp
        grid.info.resolution = self._resolution
        grid.info.width = width_cells
        grid.info.height = height_cells
        grid.info.origin.position.x = grid_origin_x
        grid.info.origin.position.y = grid_origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = occupancy_data.ravel().tolist()
        return grid
