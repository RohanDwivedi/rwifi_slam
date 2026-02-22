"""Pose graph for Wi-Fi RTT SLAM, backed by a single g2o graph.

g2o holds poses and edges. We keep three parallel lists for what g2o
doesn't store: observations, timestamps, covariances (same index = same node).
"""

from __future__ import annotations

from typing import Iterator

import g2opy as g2o
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovariance
from rclpy.logging import get_logger
from rwifi_slam_ros2.common.utils import inv_SE3, pose_from_se3, relative_pose_covariance, se3_from_pose

_logger = get_logger(__name__)


def _to_isometry(transform_matrix: np.ndarray) -> g2o.Isometry3d:
    """Convert a 4x4 SE(3) matrix to a g2o Isometry3d.

    Args:
        transform_matrix: 4x4 homogeneous transformation matrix.

    Returns:
        g2o Isometry3d with the same rotation and translation.
    """
    isometry = g2o.Isometry3d()
    isometry.set_rotation(transform_matrix[:3, :3].copy())
    isometry.set_translation(transform_matrix[:3, 3].copy())
    return isometry


def _from_isometry(iso: g2o.Isometry3d) -> np.ndarray:
    """Convert a g2o Isometry3d to a 4x4 SE(3) matrix.

    Args:
        iso: g2o Isometry3d (e.g. from vertex.estimate()).

    Returns:
        4x4 homogeneous transformation matrix as a numpy array.
    """
    return np.asarray(iso.matrix(), dtype=np.float64)


class Graph:
    """Pose graph backed by g2o.

    Poses live in g2o; observations, timestamps, and covariances are stored
    in parallel lists keyed by node index.
    """

    def __init__(
        self,
        robustness_threshold: float = 0.5,
        verbose: bool = False,
        optimization_iterations: int = 10,
    ) -> None:
        """Initialize the pose graph and g2o optimizer.

        Args:
            robustness_threshold: Delta for the DCS robust kernel on loop-closure
                edges. Defaults to 0.5.
            verbose: If True, enable g2o optimizer verbose output. Defaults to False.
            optimization_iterations: Number of Levenberg-Marquardt iterations for
                optimize(). Defaults to 10.
        """
        self._robustness_threshold = robustness_threshold
        self._verbose = verbose
        self._optimization_iterations = optimization_iterations
        self._observations: list[np.ndarray] = []
        self._timestamps: list[Time] = []
        self._covariances: list[list[float]] = []
        self._edge_id = 0

        self._opt = g2o.SparseOptimizer()
        self._opt.set_algorithm(g2o.OptimizationAlgorithmLevenberg(g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())))
        self._opt.set_verbose(verbose)

    @property
    def nodes_length(self) -> int:
        """Return the number of nodes in the graph."""
        return len(self._observations)

    @property
    def nodes(self) -> _NodesView:
        """Read-only list-like view over graph nodes.

        Each element supports .index, .pose, .observations, .timestamp.
        """
        return _NodesView(self)

    def is_empty(self) -> bool:
        """Return True if the graph has no nodes."""
        return self.nodes_length == 0

    def add_node(
        self,
        pose: PoseWithCovariance,
        observations: np.ndarray,
        timestamp: Time,
    ) -> None:
        """Add a new node with the given pose, Wi-Fi observations, and timestamp.

        Args:
            pose: Pose and covariance for the node.
            observations: 1D array of Wi-Fi range observations.
            timestamp: ROS time for this keyframe.
        """
        node_index = self.nodes_length
        vertex = g2o.VertexSE3()
        vertex.set_id(node_index)
        vertex.set_estimate(_to_isometry(se3_from_pose(pose)))
        if node_index == 0:
            vertex.set_fixed(True)
        self._opt.add_vertex(vertex)
        self._observations.append(observations)
        self._timestamps.append(timestamp)
        self._covariances.append(list(pose.covariance))
        _logger.debug(f"Added node {node_index}, observations={len(observations)}")

    def add_odometry_edge(self, from_index: int, to_index: int) -> None:
        """Add an odometry edge between two consecutive nodes.

        Args:
            from_index: Index of the source node.
            to_index: Index of the target node.

        Raises:
            IndexError: If indices are equal or out of range.
        """
        if from_index == to_index or from_index >= self.nodes_length or to_index >= self.nodes_length:
            raise IndexError(f"Invalid edge indices {from_index} -> {to_index}")

        pose_from = _from_isometry(self._opt.vertex(from_index).estimate())
        pose_to = _from_isometry(self._opt.vertex(to_index).estimate())
        covariance_from = np.array(self._covariances[from_index], dtype=float).reshape(6, 6)
        covariance_to = np.array(self._covariances[to_index], dtype=float).reshape(6, 6)
        relative_pose = inv_SE3(pose_from) @ pose_to
        cov_rel = relative_pose_covariance(pose_from, covariance_from, pose_to, covariance_to)
        information_matrix = np.linalg.inv(cov_rel)

        edge = self._edge(from_index, to_index, relative_pose, information_matrix)
        self._opt.add_edge(edge)
        _logger.debug(f"Added odom edge {from_index} -> {to_index}")

    def add_loop_closure_constraint(self, from_index: int, to_index: int) -> None:
        """Add a loop-closure constraint edge with a robust kernel.

        Args:
            from_index: Index of the earlier node.
            to_index: Index of the later node (current).

        Raises:
            IndexError: If indices are equal or out of range.
        """
        if from_index == to_index or from_index >= self.nodes_length or to_index >= self.nodes_length:
            raise IndexError(f"Invalid loop closure indices {from_index} -> {to_index}")

        edge = self._edge(from_index, to_index, np.eye(4, dtype=np.float64), np.eye(6, dtype=np.float64))
        robust_kernel = g2o.RobustKernelDCS()
        robust_kernel.set_delta(self._robustness_threshold)
        edge.set_robust_kernel(robust_kernel)
        self._opt.add_edge(edge)
        _logger.debug(f"Added loop closure {from_index} -> {to_index}")

    def optimize(self) -> None:
        """Run g2o optimization over the pose graph.

        Uses the iteration count passed to the constructor (optimization_iterations).
        """
        if self.nodes_length < 2:
            return
        self._opt.initialize_optimization()
        self._opt.optimize(self._optimization_iterations)
        _logger.info(f"Optimized pose graph ({self.nodes_length} nodes, {self._optimization_iterations} iters)")

    def reset(self) -> None:
        """Clear all nodes and edges; reinitialize the g2o optimizer."""
        self._observations.clear()
        self._timestamps.clear()
        self._covariances.clear()
        self._edge_id = 0
        self._opt.clear()
        self._opt.set_algorithm(g2o.OptimizationAlgorithmLevenberg(g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())))
        self._opt.set_verbose(self._verbose)

    def _edge(
        self,
        from_index: int,
        to_index: int,
        relative_pose: np.ndarray,
        information: np.ndarray,
    ) -> g2o.EdgeSE3:
        """Build a g2o SE3 edge between two vertices.

        Args:
            from_index: Source vertex index.
            to_index: Target vertex index.
            relative_pose: 4x4 relative pose measurement.
            information: 6x6 information matrix.

        Returns:
            Configured g2o EdgeSE3 (not yet added to the optimizer).
        """
        edge = g2o.EdgeSE3()
        edge.set_id(self._edge_id)
        self._edge_id += 1
        edge.set_vertex(0, self._opt.vertex(from_index))
        edge.set_vertex(1, self._opt.vertex(to_index))
        edge.set_measurement(_to_isometry(relative_pose))
        edge.set_information(information)
        return edge


class _NodesView:
    """List-like view over graph nodes.

    Each item has .index, .pose, .observations, .timestamp.
    """

    __slots__ = ("_graph",)

    def __init__(self, graph: Graph) -> None:
        """Initialize the view with the given graph."""
        self._graph = graph

    def __len__(self) -> int:
        """Return the number of nodes."""
        return self._graph.nodes_length

    def __getitem__(self, key: int | slice) -> _NodeRef | list[_NodeRef]:
        """Return a single node or a list of nodes for a slice."""
        if isinstance(key, slice):
            start, stop, step = key.indices(len(self))
            return [_NodeRef(self._graph, node_index) for node_index in range(start, stop, step)]
        node_index = key if key >= 0 else key + len(self)
        if node_index < 0 or node_index >= len(self):
            raise IndexError(key)
        return _NodeRef(self._graph, node_index)

    def __iter__(self) -> Iterator[_NodeRef]:
        """Iterate over all nodes in order."""
        for node_index in range(len(self)):
            yield _NodeRef(self._graph, node_index)


class _NodeRef:
    """Single-node view: pose from g2o, observations/timestamp/covariance from parallel lists."""

    __slots__ = ("_graph", "_index")

    def __init__(self, graph: Graph, index: int) -> None:
        """Initialize the reference to the node at the given index."""
        self._graph = graph
        self._index = index

    @property
    def index(self) -> int:
        """Return the node index in the graph."""
        return self._index

    @property
    def pose(self) -> PoseWithCovariance:
        transform_matrix = _from_isometry(self._graph._opt.vertex(self._index).estimate())
        return pose_from_se3(transform_matrix, covariance=self._graph._covariances[self._index])

    @property
    def observations(self) -> np.ndarray:
        """Return the Wi-Fi observation vector for this node."""
        return self._graph._observations[self._index]

    @property
    def timestamp(self) -> Time:
        """Return the ROS timestamp for this node."""
        return self._graph._timestamps[self._index]
