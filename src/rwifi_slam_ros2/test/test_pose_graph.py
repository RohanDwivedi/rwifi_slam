"""Unit tests for the g2o-backed pose graph."""

import numpy as np
import pytest
from conftest import graph_params, make_pose, stamp
from rwifi_slam_ros2.common.transforms import quaternion_to_yaw
from rwifi_slam_ros2.include.backend import Graph


def _chain(graph: Graph, poses: list[tuple[float, float, float]]) -> None:
    """Add nodes for the given (x, y, yaw) poses, linked by odometry edges."""
    for index, (x, y, yaw) in enumerate(poses):
        graph.add_node(pose=make_pose(x, y, yaw), observations=np.array([float(index)]), timestamp=stamp(index))
        if index > 0:
            graph.add_odometry_edge(from_index=index - 1, to_index=index)


def test_empty_graph():
    """A new graph has no nodes."""
    graph = Graph(parameters=graph_params())
    assert graph.is_empty()
    assert graph.nodes_length == 0
    assert len(graph.nodes) == 0


def test_nodes_view_indexing_and_slicing():
    """The nodes view supports negative indexing, slicing, and iteration."""
    graph = Graph(parameters=graph_params())
    _chain(graph, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.5, 0.3)])

    assert graph.nodes[-1].index == 2
    assert graph.nodes[-1].pose.pose.position.x == pytest.approx(2.0)
    assert quaternion_to_yaw(graph.nodes[-1].pose.pose.orientation) == pytest.approx(0.3)
    assert [node.index for node in graph.nodes[1:]] == [1, 2]
    assert [node.timestamp.sec for node in graph.nodes] == [0, 1, 2]
    np.testing.assert_array_equal(graph.nodes[1].observations, [1.0])
    with pytest.raises(IndexError):
        graph.nodes[3]


@pytest.mark.parametrize("from_index, to_index", [(0, 0), (0, 5), (5, 0)])
def test_odometry_edge_rejects_invalid_indices(from_index, to_index):
    """Edges between a node and itself or missing nodes are rejected."""
    graph = Graph(parameters=graph_params())
    _chain(graph, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    with pytest.raises(IndexError):
        graph.add_odometry_edge(from_index=from_index, to_index=to_index)
    with pytest.raises(IndexError):
        graph.add_loop_closure_constraint(from_index=from_index, to_index=to_index)


def test_optimizing_consistent_chain_keeps_poses():
    """Optimizing a graph whose edges match the initial poses leaves the poses in place."""
    poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.2), (1.8, 0.6, 0.6), (2.3, 1.5, 1.0)]
    graph = Graph(parameters=graph_params())
    _chain(graph, poses)
    graph.optimize()
    for node, (x, y, yaw) in zip(graph.nodes, poses):
        assert node.pose.pose.position.x == pytest.approx(x, abs=1e-6)
        assert node.pose.pose.position.y == pytest.approx(y, abs=1e-6)
        assert quaternion_to_yaw(node.pose.pose.orientation) == pytest.approx(yaw, abs=1e-6)


def test_first_node_is_fixed_during_optimization():
    """The first node anchors the graph when a loop closure pulls on it."""
    graph = Graph(parameters=graph_params())
    _chain(graph, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.2, 1.0, 0.0)])
    graph.add_loop_closure_constraint(from_index=0, to_index=3)
    graph.optimize()
    first = graph.nodes[0].pose.pose.position
    assert (first.x, first.y) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_reset_clears_graph():
    """reset() removes all nodes and allows rebuilding."""
    graph = Graph(parameters=graph_params())
    _chain(graph, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    graph.reset()
    assert graph.is_empty()
    _chain(graph, [(5.0, 5.0, 0.0)])
    assert graph.nodes[0].pose.pose.position.x == pytest.approx(5.0)
