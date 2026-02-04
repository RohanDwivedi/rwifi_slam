"""Workspace-wide RNG for reproducible Wi-Fi RTT simulation."""

import numpy as np

SEED: int = 42
RNG: np.random.Generator = np.random.default_rng(seed=SEED)
