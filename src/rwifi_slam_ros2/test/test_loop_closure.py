"""Unit tests for Wi-Fi RTT loop-closure detection."""

import numpy as np
import pytest
from conftest import graph_params, make_params, make_pose, stamp
from rwifi_slam_ros2.include.backend import Graph
from rwifi_slam_ros2.include.front_end import LoopClosure


def _detector(observations: list[list[float]], **overrides: object) -> LoopClosure:
    """Build a graph with one node per observation vector and a detector over it."""
    graph = Graph(parameters=graph_params())
    for index, obs in enumerate(observations):
        graph.add_node(pose=make_pose(x=float(index)), observations=np.array(obs), timestamp=stamp(index))
    settings = {
        "distance_method": "euclidean",
        "clustering_threshold": 1.0,
        "use_ratio_test": False,
        "ratio_threshold": 0.8,
        "exclude_near_neighbors": False,
        "near_neighbor_threshold": 0,
    }
    settings.update(overrides)
    return LoopClosure(parameters=make_params(**settings), graph=graph)


def test_single_node_has_no_loop_closure():
    """A graph with one node cannot close a loop."""
    assert _detector([[0.0, 0.0]]).get_loop_closure_indexes() is None


def test_threshold_mode_returns_every_match():
    """Without the ratio test, every previous node under the threshold is returned."""
    detector = _detector([[0.0, 0.0], [5.0, 5.0], [0.1, 0.0], [9.0, 9.0], [0.0, 0.1]])
    assert detector.get_loop_closure_indexes() == [0, 2]


def test_threshold_mode_returns_none_when_nothing_matches():
    """No loop closure is reported when all previous nodes are far away."""
    assert _detector([[5.0, 5.0], [9.0, 9.0], [0.0, 0.0]]).get_loop_closure_indexes() is None


def test_recent_neighbors_are_excluded():
    """The most recent k previous nodes are never loop-closure candidates."""
    detector = _detector(
        [[0.0, 0.0], [5.0, 5.0], [0.1, 0.0], [0.0, 0.05], [0.0, 0.1]],
        exclude_near_neighbors=True,
        near_neighbor_threshold=2,
    )
    assert detector.get_loop_closure_indexes() == [0]


def test_ratio_test_accepts_distinctive_match():
    """The ratio test keeps a match that is much better than the runner-up."""
    detector = _detector([[0.0, 0.0], [4.0, 4.0], [8.0, 8.0], [0.1, 0.1]], use_ratio_test=True)
    assert detector.get_loop_closure_indexes() == [0]


def test_ratio_test_rejects_ambiguous_match():
    """The ratio test drops a match when the runner-up is almost as close."""
    detector = _detector([[0.0, 0.0], [0.05, 0.0], [8.0, 8.0], [0.0, 0.1]], use_ratio_test=True)
    assert detector.get_loop_closure_indexes() is None


def test_unknown_distance_method_raises():
    """An unsupported distance method is reported instead of silently ignored."""
    detector = _detector([[0.0], [1.0]], distance_method="cosine")
    with pytest.raises(ValueError):
        detector.get_loop_closure_indexes()
