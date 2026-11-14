"""Unit tests for the OctoMap backend and its 2D occupancy-grid projection."""

import numpy as np
import pytest
from conftest import make_params
from rwifi_slam_ros2.include.backend import Octomap


def _octomap(use_decay: bool = False) -> Octomap:
    """Build an Octomap with a 0.1 m grid sliced around z = 0.15 m."""
    return Octomap(
        parameters=make_params(
            height_min=0.0,
            height_max=0.3,
            resolution=0.1,
            map_frame="map",
            margin_m=1.0,
            max_range=-1.0,
            use_decay=use_decay,
            logodd_decay_value=-0.2,
        )
    )


def _wall(x: float = 2.0, z: float = 0.15) -> np.ndarray:
    """Return points on a vertical wall at the given x, spanning y in [-1, 1]."""
    ys = np.arange(-1.0, 1.0, 0.05)
    return np.column_stack((np.full_like(ys, x), ys, np.full_like(ys, z)))


def test_empty_map_is_unknown():
    """An empty octree projects to a small grid of unknown cells."""
    grid = _octomap().to_occupancy_grid_2d()
    assert grid.header.frame_id == "map"
    assert grid.info.width * grid.info.height == len(grid.data)
    assert set(grid.data) == {-1}


def test_empty_point_cloud_is_rejected():
    """Inserting no points raises instead of corrupting the map."""
    with pytest.raises(ValueError):
        _octomap().insert_pointcloud(np.empty((0, 3)), np.zeros(3))


@pytest.mark.parametrize("use_decay", [False, True])
def test_wall_projects_to_occupied_and_free_cells(use_decay):
    """A scanned wall marks occupied cells at the wall and free cells along the rays."""
    octomap = _octomap(use_decay=use_decay)
    octomap.insert_pointcloud(_wall(), sensor_origin=np.array([0.0, 0.0, 0.15]))
    grid = octomap.to_occupancy_grid_2d()

    assert grid.info.resolution == pytest.approx(0.1)
    assert grid.info.width * grid.info.height == len(grid.data)
    cells = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)

    def cell(x: float, y: float) -> int:
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        return int(cells[row, col])

    assert cell(2.05, 0.05) == 100
    assert cell(1.0, 0.05) == 0
    assert cell(-0.5, -0.5) == -1
