"""Unit tests for the NLOS map schema and its cross-environment pickle loading."""

import pickle
import sys
import types

import numpy as np
import pytest
from wifi_rtt_interface import nlos_map_schema
from wifi_rtt_interface.nlos_map_schema import NLOSMap, NLOSMapFile, NLOSObservation


def _sample_map() -> NLOSMapFile:
    """Build a two-cell map for one access point."""
    ap_map = NLOSMap(
        access_point_mac_address="00:00:00:00:00:01",
        access_point_hardware_bias=0.1,
        access_point_position=np.array([1.0, 2.0, 3.0]),
        nlos_observations=[
            NLOSObservation(grid_cell_center=np.array([0.0, 0.0, 0.6])),
            NLOSObservation(grid_cell_center=np.array([0.25, 0.0, 0.6]), los=False, nlos_bias=2.1),
        ],
    )
    return NLOSMapFile(nlos_map=[ap_map])


def _assert_sample(loaded: NLOSMapFile) -> None:
    """Check that a loaded map matches ``_sample_map``."""
    assert isinstance(loaded, NLOSMapFile)
    ap_map = loaded.nlos_map[0]
    assert ap_map.access_point_mac_address == "00:00:00:00:00:01"
    np.testing.assert_array_equal(ap_map.access_point_position, [1.0, 2.0, 3.0])
    assert [obs.los for obs in ap_map.nlos_observations] == [True, False]
    assert ap_map.nlos_observations[1].nlos_bias == pytest.approx(2.1)


def test_save_load_round_trip(tmp_path):
    """A saved map loads back unchanged."""
    path = tmp_path / "map.pkl"
    _sample_map().save(str(path))
    _assert_sample(NLOSMapFile.load(str(path)))


def test_access_point_position_must_be_3d():
    """A 2D access-point position is rejected."""
    with pytest.raises(ValueError):
        NLOSMap(
            access_point_mac_address="00:00:00:00:00:01",
            access_point_hardware_bias=0.0,
            access_point_position=np.array([1.0, 2.0]),
        )


def test_load_rejects_other_pickles(tmp_path):
    """Loading a pickle whose root is not an NLOSMapFile raises TypeError."""
    path = tmp_path / "other.pkl"
    path.write_bytes(pickle.dumps({"not": "a map"}))
    with pytest.raises(TypeError):
        NLOSMapFile.load(str(path))


def test_load_remaps_isaac_sim_module_names(tmp_path, monkeypatch):
    """Maps pickled inside Isaac Sim (module ``include.precompute_nlos``) load in ROS."""
    path = tmp_path / "isaac.pkl"
    with monkeypatch.context() as patch:
        fake = types.ModuleType("include.precompute_nlos")
        patch.setitem(sys.modules, "include", types.ModuleType("include"))
        patch.setitem(sys.modules, "include.precompute_nlos", fake)
        for name in ("NLOSMapFile", "NLOSMap", "NLOSObservation"):
            cls = getattr(nlos_map_schema, name)
            patch.setattr(cls, "__module__", "include.precompute_nlos")
            setattr(fake, name, cls)
        path.write_bytes(pickle.dumps(_sample_map()))

    assert b"include.precompute_nlos" in path.read_bytes()
    _assert_sample(NLOSMapFile.load(str(path)))
