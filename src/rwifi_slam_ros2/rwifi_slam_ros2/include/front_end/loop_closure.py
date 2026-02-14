"""Loop closure detection for the Range WiFi SLAM pose graph."""

from __future__ import annotations

import numpy as np
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rwifi_slam_ros2.common import Graph


class LoopClosure:
    """Loop closure detection for the pose graph using Wi-Fi observation distances."""

    def __init__(self, parameters: dict[str, Parameter], graph: Graph) -> None:
        """Initialize the loop closure detector.

        Args:
            parameters: ROS parameters (distance_method, clustering_threshold,
                use_ratio_test, ratio_threshold, exclude_near_neighbors,
                near_neighbor_threshold).
            graph: Pose graph containing nodes and observations.
        """
        self._logger = get_logger(self.__class__.__name__)

        self._parameters = parameters
        self._distance_method: str = self._parameters["distance_method"].value
        self._clustering_threshold: float = self._parameters["clustering_threshold"].value
        self._use_ratio_test: bool = self._parameters["use_ratio_test"].value
        self._ratio_threshold: float = self._parameters["ratio_threshold"].value
        self._exclude_near_neighbors: bool = self._parameters["exclude_near_neighbors"].value
        self._near_neighbor_threshold: int = self._parameters["near_neighbor_threshold"].value

        self._graph = graph

        self._logger.debug(
            f"LoopClosure init: distance_method={self._distance_method}, "
            f"clustering_threshold={self._clustering_threshold:.4f}, use_ratio_test={self._use_ratio_test}, "
            f"ratio_threshold={self._ratio_threshold:.4f}, exclude_near_neighbors={self._exclude_near_neighbors}, "
            f"near_neighbor_threshold={self._near_neighbor_threshold}"
        )

    def _calculate_distance(self, current_observations: np.ndarray, previous_observations: np.ndarray) -> np.ndarray:
        """Compute per-node distances from current to each previous observation vector.

        Args:
            current_observations: Observation vector of the current node.
            previous_observations: Array of observation vectors (one per previous node).

        Returns:
            1D array of distances, one per previous node.

        Raises:
            ValueError: If distance_method is not supported.
        """
        if self._distance_method == "euclidean":
            return np.linalg.norm(current_observations - previous_observations, axis=1)
        else:
            raise ValueError(f"Invalid distance method: {self._distance_method}")

    def _exclude_last_k_neighbors(self, distances: np.ndarray, near_neighbor_threshold: int) -> np.ndarray:
        """Exclude the last k graph neighbors from loop-closure candidates.

        Sets distances for the most recent k nodes to inf so they are not
        selected as loop closures.

        Args:
            distances: 1D array of distances from current node to each previous node.
            near_neighbor_threshold: Number of recent nodes to exclude (k).

        Returns:
            Copy of distances with the last k entries set to inf.
        """
        k = int(near_neighbor_threshold)
        n = len(distances)
        keep_mask = np.arange(n) < n - k
        return np.where(keep_mask, distances, np.inf)

    def _perform_ratio_test(self, distances: np.ndarray) -> int | None:
        """Apply ratio test: best match must be sufficiently better than second-best.

        Args:
            distances: 1D array of distances to previous nodes.

        Returns:
            Index of the best loop-closure candidate if ratio and threshold pass,
            otherwise None.
        """
        finite_mask = np.isfinite(distances)
        if np.sum(finite_mask) < 2:
            return None

        best_idx = int(np.argmin(distances))
        best_distance = float(distances[best_idx])
        if not np.isfinite(best_distance):
            return None

        finite_distances = np.sort(distances[finite_mask])
        second_best_distance = float(finite_distances[1])
        if second_best_distance <= 0.0:
            return None

        ratio = best_distance / second_best_distance
        if ratio < self._ratio_threshold and best_distance < self._clustering_threshold:
            return best_idx
        return None

    @property
    def loop_closure_indexes(self) -> list[int] | None:
        """Indices of previous nodes that form a loop closure with the current node.

        Returns:
            List of node indices, or None if no loop closure detected.
        """
        loop_closure_indexes = self._find_loop_closure_indexes()
        if loop_closure_indexes:
            return loop_closure_indexes
        return None

    def _find_loop_closure_indexes(self) -> list[int] | None:
        """Find graph nodes that close a loop with the current (last) node.

        Returns:
            List of node indices that match the current node under the configured
            distance and ratio criteria, or None if none found or graph too small.
        """
        if len(self._graph.nodes) < 2:
            self._logger.warning("Not enough nodes to check for loop closure.")
            return None

        current_observations = self._graph.nodes[-1].observations
        previous_observations = np.array([node.observations for node in self._graph.nodes[:-1]])

        distances = self._calculate_distance(current_observations, previous_observations)
        min_d = float(np.min(distances)) if len(distances) > 0 else float("nan")
        max_d = float(np.max(distances)) if len(distances) > 0 else float("nan")
        self._logger.debug(
            f"Distances to {len(distances)} previous nodes: min={min_d:.4f}, "
            f"max={max_d:.4f}, threshold={self._clustering_threshold:.4f}"
        )

        if self._exclude_near_neighbors:
            distances = self._exclude_last_k_neighbors(distances, self._near_neighbor_threshold)

        if self._use_ratio_test:
            loop_closure_index = self._perform_ratio_test(distances)
            if loop_closure_index is not None:
                self._logger.debug(f"Loop closure detected at index {loop_closure_index}")
                return [loop_closure_index]
            return None

        valid_mask = distances < self._clustering_threshold
        loop_closure_indexes = np.where(valid_mask)[0].tolist()
        if len(loop_closure_indexes) > 0:
            self._logger.debug(f"Loop closure detected at indexes {loop_closure_indexes}")
            return loop_closure_indexes
        return None
