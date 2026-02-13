"""Keyframe data structures for Wi-Fi RTT SLAM."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Keyframe:
    """Represents a single Wi-Fi RTT keyframe with observations and timestamp."""

    observations: np.ndarray
    timestamp: float

    def __post_init__(self) -> None:
        """Ensure observations is a numpy array."""
        if not isinstance(self.observations, np.ndarray):
            self.observations = np.array(self.observations, dtype=np.float64)


@dataclass
class KeyframeCollection:
    """Collection of Wi-Fi RTT keyframes with array accessors."""

    keyframes: list[Keyframe]

    @property
    def keyframes_array(self) -> np.ndarray:
        """Get all keyframe observations as a 2D numpy array."""
        if not self.keyframes:
            return np.empty((0, 0), dtype=np.float64)
        return np.vstack([kf.observations for kf in self.keyframes])

    @property
    def timestamps_array(self) -> np.ndarray:
        """Get all keyframe timestamps as a numpy array."""
        if not self.keyframes:
            return np.empty(0, dtype=np.float64)
        return np.array([kf.timestamp for kf in self.keyframes], dtype=np.float64)
