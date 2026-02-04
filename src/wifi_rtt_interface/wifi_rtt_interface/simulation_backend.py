"""Simulation backend for Wi-Fi RTT: serves observations from a precomputed NLOS map.

Resolves the robot pose via TF, looks up NLOS bias and LOS flag from the map at the
nearest grid cell, adds runtime measurement noise, and publishes APRangeObservation
messages.
"""

from __future__ import annotations

import os
import time
from typing import TYPE_CHECKING

import numpy as np
import rclpy.duration
import rclpy.time
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock
from rclpy.node import Node
from scipy.spatial import KDTree
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from wifi_rtt_interface.nlos_map_schema import NLOSMapFile
from wifi_rtt_interface.rng import RNG
from wifi_rtt_msgs.msg import APRange, APRangeObservation

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


WAIT_TIMEOUT_SEC = 300.0
POLL_INTERVAL_SEC = 2.0
LOG_INTERVAL_SEC = 10.0
LOAD_ATTEMPTS = 5


class SimulationBackend:
    """Serves RTT observations from a precomputed NLOS map (e.g. from Isaac Sim).

    All access points share a single 2D grid; one KDTree lookup finds the nearest
    cell for the robot position, then each AP's observation at that cell index is used.
    """

    def __init__(
        self,
        simulation_type: str,
        clock: Clock,
        node: Node,
        publisher: Publisher,
    ) -> None:
        """Initialize the backend with TF listener and load the NLOS map.

        Reads world_frame, base_link_frame, and simulation_isaac_sim.* parameters
        from the node. For simulation_isaac_sim, loads the pickle at
        simulation_isaac_sim.nlos_map_file and builds the grid index. If the file
        is missing, waits up to WAIT_TIMEOUT_SEC for it (e.g. while Isaac Sim
        is still writing it).

        Args:
            simulation_type: Backend identifier, e.g. 'simulation_isaac_sim'.
            clock: ROS clock used for message header stamps.
            node: ROS 2 node for parameters and logging.
            publisher: Publisher for APRangeObservation messages.

        Raises:
            ValueError: If simulation_type is not supported or required parameters
                are missing.
            FileNotFoundError: If the NLOS map file is not found within the wait
                timeout.
        """
        self.simulation_type = simulation_type
        self.clock = clock
        self.node = node
        self._publisher = publisher

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self._grid_kdtree: KDTree | None = None
        self._grid_cell_centers: np.ndarray | None = None

        self.world_frame = node.get_parameter("world_frame").value
        self.base_link_frame = node.get_parameter("base_link_frame").value
        self.grid_cell_resolution = float(node.get_parameter("simulation_isaac_sim.grid_cell_resolution").value)
        self.antenna_height = float(node.get_parameter("simulation_isaac_sim.antenna_height").value)

        if self.simulation_type == "simulation_isaac_sim":
            nlos_map_path = node.get_parameter("simulation_isaac_sim.nlos_map_file").value
            self._load_nlos_map(nlos_map_path)
        else:
            raise ValueError(f"Invalid simulation type: {self.simulation_type}")

    def _load_nlos_map(self, nlos_map_path: str) -> None:
        """Load the NLOS map from a pickle file and build the grid index.

        If the file does not exist, polls until it appears (up to WAIT_TIMEOUT_SEC)
        or retries on EOFError (file still being written). Then builds a single
        KDTree over the shared grid cell centers.

        Args:
            nlos_map_path: Absolute or relative path to the NLOS map pickle file.

        Raises:
            FileNotFoundError: If the file is still missing after the timeout.
            EOFError: If the file remains incomplete after LOAD_ATTEMPTS retries.
        """
        waited = 0.0

        try:
            self.nlos_map_file = NLOSMapFile.load(nlos_map_path)
        except FileNotFoundError as e:
            self.node.get_logger().error(f"Failed to load NLOS map file: {e}")

            while not os.path.exists(nlos_map_path):
                if waited >= WAIT_TIMEOUT_SEC:
                    self.node.get_logger().error(
                        f"NLOS map not found after {WAIT_TIMEOUT_SEC:.0f}s: " f"{nlos_map_path}"
                    )
                    raise FileNotFoundError(f"NLOS map file not found: {nlos_map_path}")

                self.node.get_logger().warning(
                    f"Waiting for NLOS map (Isaac Sim may be computing): " f"{nlos_map_path}",
                    throttle_duration_sec=LOG_INTERVAL_SEC,
                )
                time.sleep(POLL_INTERVAL_SEC)
                waited += POLL_INTERVAL_SEC

            for attempt in range(LOAD_ATTEMPTS):
                try:
                    self.nlos_map_file = NLOSMapFile.load(nlos_map_path)
                    break
                except EOFError:
                    if attempt == LOAD_ATTEMPTS - 1:
                        raise
                    self.node.get_logger().warning(
                        f"NLOS map file incomplete (still being written?), "
                        f"retrying... ({attempt + 1}/{LOAD_ATTEMPTS})"
                    )
                    time.sleep(POLL_INTERVAL_SEC)

        self._build_grid_index()

    def _build_grid_index(self) -> None:
        """Build a KDTree over the shared 2D grid cell centers.

        Uses the first NLOSMap's nlos_observations to obtain (x, y) cell centers;
        all APs share the same grid, so one index is used for every AP at runtime.
        """
        if not hasattr(self, "nlos_map_file") or not self.nlos_map_file.nlos_map:
            return

        first_map = self.nlos_map_file.nlos_map[0]
        if not first_map.nlos_observations:
            return

        self._grid_cell_centers = np.array([obs.grid_cell_center[:2] for obs in first_map.nlos_observations])
        self._grid_kdtree = KDTree(self._grid_cell_centers)

        n_aps = len(self.nlos_map_file.nlos_map)
        n_cells = len(self._grid_cell_centers)
        self.node.get_logger().info(f"Built grid index: {n_cells} cells, {n_aps} AP(s)")

    def _get_transform(self, from_frame_id: str, to_frame_id: str) -> TransformStamped | None:
        """Look up the latest transform from source frame to target frame.

        Args:
            from_frame_id: Source TF frame id.
            to_frame_id: Target TF frame id.

        Returns:
            The latest TransformStamped, or None if the lookup fails (e.g. no
            transform available or timeout).
        """
        try:
            return self.tf_buffer.lookup_transform(
                from_frame_id,
                to_frame_id,
                rclpy.time.Time(seconds=0, nanoseconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.node.get_logger().error(f"Transform exception: {e}")
            return None

    def _get_nearest_grid_cell_index(self, position: np.ndarray) -> int | None:
        """Return the index of the nearest grid cell to the given (x, y) position.

        The grid is shared by all APs; this index is used to look up each AP's
        NLOS observation at that cell.

        Args:
            position: Shape (2,) array with x, y in the world frame.

        Returns:
            The row-major grid cell index, or None if the grid is not built.

        Raises:
            ValueError: If position does not have shape (2,).
        """
        if self._grid_kdtree is None:
            return None
        if np.shape(position) != (2,):
            raise ValueError(f"Position must be a 2D array, got {np.shape(position)}")
        _, index = self._grid_kdtree.query(position)
        return int(index)

    def _get_rtt_observations(self) -> list[APRange]:
        """Compute RTT observations for the current base_link position.

        Looks up world -> base_link, finds the nearest grid cell, then for each
        AP builds an APRange with true/measured distance, biases, and LOS flag
        from the precomputed map, plus runtime measurement noise (heavy-tailed).

        Returns:
            List of APRange messages, one per access point, or an empty list
            if the transform or grid lookup fails.
        """
        observations: list[APRange] = []

        transform = self._get_transform(self.world_frame, self.base_link_frame)
        if transform is None:
            self.node.get_logger().error("Failed to get transform from world to base_link")
            return []

        robot_position = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ]
        )
        nearest_grid_cell_index = self._get_nearest_grid_cell_index(robot_position)
        if nearest_grid_cell_index is None:
            return []

        for nlos_map in self.nlos_map_file.nlos_map:
            mac_address = nlos_map.access_point_mac_address
            access_point_position = nlos_map.access_point_position
            obs = nlos_map.nlos_observations[nearest_grid_cell_index]
            nlos_bias = obs.nlos_bias
            los = obs.los
            hardware_bias = nlos_map.access_point_hardware_bias

            measurement_noise = (RNG.standard_t(df=3.0) * 0.4) + 0.2

            robot_3d = np.append(robot_position, self.antenna_height)
            true_distance = float(np.linalg.norm(access_point_position - robot_3d))
            measured_distance = true_distance + nlos_bias + measurement_noise + hardware_bias

            observations.append(
                APRange(
                    mac_address=mac_address,
                    measured_distance=measured_distance,
                    true_distance=true_distance,
                    nlos_bias=nlos_bias,
                    measurement_noise=measurement_noise,
                    hardware_bias=hardware_bias,
                    los=los,
                )
            )

        return observations

    def publish_observation(self) -> None:
        """Fetch the current RTT observation, wrap it in a header, and publish.

        Uses the node clock for the header stamp and world_frame for frame_id.
        """
        observations = self._get_rtt_observations()
        header = Header()
        header.stamp = self.clock.now().to_msg()
        header.frame_id = self.world_frame
        msg = APRangeObservation(header=header, observations=observations)
        self._publisher.publish(msg)
