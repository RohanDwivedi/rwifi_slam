"""Node data structures for Wi-Fi RTT SLAM."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovariance
from rclpy.logging import get_logger
from rwifi_slam_ros2.common.utils import inv_SE3, relative_pose_covariance, se3_from_pose

_logger = get_logger(__name__)


@dataclass
class Node:
    """A single pose-graph node with pose, Wi-Fi observations, and timestamp.

    Attributes:
        index: Unique node index in the graph.
        pose: Pose with 6x6 covariance (ROS PoseWithCovariance).
        observations: Wi-Fi RTT observation vector (e.g. distances to APs).
        timestamp: ROS time of the node.
    """

    index: int = 0
    pose: PoseWithCovariance = field(default_factory=lambda: PoseWithCovariance())
    observations: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float64))
    timestamp: Time = field(default_factory=lambda: Time(sec=0, nanosec=0))


@dataclass
class Edge:
    """Odometric edge between two nodes: relative SE(3) transform and covariance.

    Attributes:
        from_index: Source node index.
        to_index: Target node index.
        T_from_to: 4x4 relative pose from source to target.
        covariance: 6x6 covariance of the relative pose.
    """

    from_index: int = 0
    to_index: int = 0
    T_from_to: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    covariance: np.ndarray = field(default_factory=lambda: np.eye(6, dtype=np.float64) * 1e-2)


@dataclass
class LoopClosureConstraint:
    """Loop-closure constraint between two nodes: relative SE(3) and covariance.

    Attributes:
        from_index: Source (older) node index.
        to_index: Target (current) node index.
        T_from_to: 4x4 relative pose from source to target.
        covariance: 6x6 covariance of the relative pose.
    """

    from_index: int = 0
    to_index: int = 0
    T_from_to: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    covariance: np.ndarray = field(default_factory=lambda: np.eye(6, dtype=np.float64) * 1e-2)


@dataclass
class Graph:
    """Pose graph: nodes (poses + observations), odometry edges, and loop-closure edges.

    Attributes:
        nodes: List of graph nodes.
        edges: Odometry edges between consecutive nodes.
        loop_closure_edges: Loop-closure constraints between non-consecutive nodes.
    """

    nodes: list[Node] = field(default_factory=list)
    edges: list[Edge] = field(default_factory=list)
    loop_closure_edges: list[LoopClosureConstraint] = field(default_factory=list)

    def add_node(self, node: Node) -> None:
        """Append a node with the next index; the provided node is copied, not mutated.

        Args:
            node: Node data (pose, observations, timestamp). Index is set automatically.
        """
        index = len(self.nodes)
        new_node = Node(
            index=index,
            pose=node.pose,
            observations=node.observations,
            timestamp=node.timestamp,
        )
        self.nodes.append(new_node)
        _logger.debug(f"Added node index={index}, observations={len(new_node.observations)}")

    def add_edge(self, from_index: int, to_index: int) -> None:
        """Add an odometry edge between two nodes.

        Args:
            from_index: Source node index.
            to_index: Target node index.
        """
        edge = self.make_edge(from_index, to_index)
        self.edges.append(edge)
        _logger.debug(f"Added odometry edge {from_index} -> {to_index}")

    def add_loop_closure(self, from_index: int, to_index: int) -> None:
        """Add a loop-closure constraint between two nodes.

        Args:
            from_index: Source (older) node index.
            to_index: Target (current) node index.
        """
        loop_closure_edge = self.make_loop_closure_edge(from_index, to_index)
        self.loop_closure_edges.append(loop_closure_edge)
        _logger.debug(f"Added loop closure edge {from_index} -> {to_index}")

    def is_empty(self) -> bool:
        """Return True if the graph has no nodes.

        Returns:
            True if nodes list is empty.
        """
        return len(self.nodes) == 0

    def make_edge(self, from_index: int, to_index: int) -> Edge:
        """Build an odometry edge from node poses and covariances.

        Args:
            from_index: Source node index.
            to_index: Target node index.

        Returns:
            Edge with relative pose and propagated covariance.
        """
        from_pose = se3_from_pose(self.nodes[from_index].pose)
        to_pose = se3_from_pose(self.nodes[to_index].pose)
        Sigma_from = np.array(self.nodes[from_index].pose.covariance, dtype=float).reshape(6, 6)
        Sigma_to = np.array(self.nodes[to_index].pose.covariance, dtype=float).reshape(6, 6)

        T_from_to = inv_SE3(from_pose) @ to_pose
        covariance = relative_pose_covariance(from_pose, Sigma_from, to_pose, Sigma_to)

        _logger.debug(
            f"make_edge {from_index} -> {to_index}: translation norm={float(np.linalg.norm(T_from_to[:3, 3])):.4f}"
        )
        return Edge(
            from_index=from_index,
            to_index=to_index,
            T_from_to=T_from_to,
            covariance=covariance,
        )

    def make_loop_closure_edge(self, from_index: int, to_index: int) -> LoopClosureConstraint:
        """Build a loop-closure constraint from node poses and covariances.

        Args:
            from_index: Source node index.
            to_index: Target node index.

        Returns:
            LoopClosureConstraint with relative pose and propagated covariance.
        """
        from_pose = se3_from_pose(self.nodes[from_index].pose)
        to_pose = se3_from_pose(self.nodes[to_index].pose)
        Sigma_from = np.array(self.nodes[from_index].pose.covariance, dtype=float).reshape(6, 6)
        Sigma_to = np.array(self.nodes[to_index].pose.covariance, dtype=float).reshape(6, 6)

        T_from_to = inv_SE3(from_pose) @ to_pose
        covariance = relative_pose_covariance(from_pose, Sigma_from, to_pose, Sigma_to)

        _logger.debug(
            f"make_loop_closure_edge {from_index} -> {to_index}: "
            f"translation norm={float(np.linalg.norm(T_from_to[:3, 3])):.4f}"
        )
        return LoopClosureConstraint(
            from_index=from_index,
            to_index=to_index,
            T_from_to=T_from_to,
            covariance=covariance,
        )
