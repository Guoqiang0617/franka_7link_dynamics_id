"""Reject ambiguous historical or unrelated parameter files at workflow boundaries."""
from pathlib import Path

import numpy as np

from .inertial import cad_prior_vector, inertial_origin_to_com


def load_parameter_vectors(path: Path, config: dict | None = None):
    with np.load(path, allow_pickle=False) as saved:
        expected = {"parameter_layout": "rigid70_friction14", "inertia_reference": "link_origin"}
        for key, value in expected.items():
            if key not in saved or str(saved[key].item()) != value:
                raise ValueError(
                    f"{path}: missing/incompatible {key}. Run this project's identification script; "
                    "historical reference files and external-object estimates cannot be substituted."
                )
        prior = np.asarray(saved["phi_prior"], dtype=float).reshape(-1)
        identified = np.asarray(saved["phi_identified"], dtype=float).reshape(-1)
    if prior.shape != (84,) or identified.shape != (84,):
        raise ValueError("Expected 70 link-origin inertial plus 14 joint friction parameters.")
    if not np.isfinite(prior).all() or not np.isfinite(identified).all():
        raise ValueError("Parameter arrays must contain only finite values.")
    if config is not None and not np.allclose(prior, cad_prior_vector(config), atol=1e-9, rtol=1e-7):
        raise ValueError("The parameter prior does not match this FR3 model configuration.")
    for joint in range(7):
        inertial_origin_to_com(identified[joint*10:(joint+1)*10])
    if np.any(identified[70:] < 0):
        raise ValueError("Joint friction coefficients must be nonnegative.")
    return prior, identified
