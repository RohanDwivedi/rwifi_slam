"""Shared NLOS map dataclasses and pickle serialization.

No Isaac Sim dependencies; used by both the ROS 2 interface and the Isaac Sim
precompute script. Handles cross-environment pickle loading (include.* and
numpy.core / numpy._core).
"""

from __future__ import annotations

import pickle
from dataclasses import dataclass, field
from typing import Any

import numpy as np

_SCHEMA_CLASSES = ("NLOSMapFile", "NLOSMap", "NLOSObservation")


class _NLOSUnpickler(pickle.Unpickler):
    """Custom unpickler for NLOS maps produced in Isaac Sim or with different NumPy.

    Resolves classes serialized under 'include.*' (Isaac Sim) to this module's
    schema classes, and maps numpy.core.* <-> numpy._core.* for NumPy 1.x vs 2.x
    compatibility.
    """

    def find_class(self, module: str, name: str) -> Any:
        """Resolve module/class to a loadable type in this process.

        Args:
            module: Serialized module name (e.g. 'include.precompute_nlos').
            name: Class name (e.g. 'NLOSMapFile').

        Returns:
            The class object to use for deserialization.
        """
        if (module == "include" or module.startswith("include.")) and name in _SCHEMA_CLASSES:
            return getattr(
                __import__(__name__, fromlist=[name]),
                name,
            )
        if module.startswith("numpy._core."):
            alt = "numpy.core." + module[len("numpy._core.") :]
            try:
                return super().find_class(alt, name)
            except (ModuleNotFoundError, AttributeError):
                pass
        if module.startswith("numpy.core."):
            alt = "numpy._core." + module[len("numpy.core.") :]
            try:
                return super().find_class(alt, name)
            except (ModuleNotFoundError, AttributeError):
                pass
        return super().find_class(module, name)


@dataclass
class NLOSObservation:
    """Observation at a single grid cell: LOS/NLOS, bias, and cell center.

    Attributes:
        grid_cell_center: (3,) array [x, y, z] of the cell center in world frame.
        los: True if line of sight from AP to cell is unblocked.
        nlos_bias: Non-negative bias (meters) added when NLOS; 0 when LOS.
        measurement_noise: Reserved for per-cell noise; often 0 (noise at runtime).
    """

    grid_cell_center: np.ndarray
    los: bool = True
    nlos_bias: float = 0.0
    measurement_noise: float = 0.0


@dataclass
class NLOSMap:
    """Per–access-point map of NLOS observations over a shared 2D grid.

    The grid is row-major; cell order must match across all NLOSMaps in an
    NLOSMapFile.

    Attributes:
        access_point_mac_address: MAC address string for the AP.
        access_point_hardware_bias: Constant device bias (meters).
        access_point_position: (3,) array [x, y, z] in world frame.
        nlos_observations: One NLOSObservation per grid cell, same order for all APs.
    """

    access_point_mac_address: str
    access_point_hardware_bias: float
    access_point_position: np.ndarray
    nlos_observations: list[NLOSObservation] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Validate access_point_position shape."""
        if np.shape(self.access_point_position) != (3,):
            raise ValueError("Access point position must be a 3D array, got " f"{np.shape(self.access_point_position)}")


@dataclass
class NLOSMapFile:
    """Container of per-AP NLOS maps; load/save via pickle.

    Attributes:
        nlos_map: List of NLOSMap, one per access point.
    """

    nlos_map: list[NLOSMap] = field(default_factory=list)

    @classmethod
    def load(cls, path: str) -> NLOSMapFile:
        """Load an NLOSMapFile from a pickle file.

        Uses _NLOSUnpickler so that pickles created in Isaac Sim (include.*)
        or with a different NumPy version load correctly.

        Args:
            path: Path to the pickle file.

        Returns:
            The loaded NLOSMapFile instance.

        Raises:
            TypeError: If the pickle root object is not an NLOSMapFile.
        """
        with open(path, "rb") as f:
            data = _NLOSUnpickler(f).load()
        if not isinstance(data, cls):
            raise TypeError(f"Expected {cls.__name__} in pickle, got {type(data)}")
        print(f"Loaded {cls.__name__} from {path}")
        return data

    def save(self, path: str) -> None:
        """Serialize this NLOSMapFile to a pickle file.

        Args:
            path: Output path for the pickle file.
        """
        with open(path, "wb") as f:
            pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"Saved {self.__class__.__name__} to {path}")
