from __future__ import annotations

import os
from pathlib import Path


def package_root() -> Path:
    """Return the package source root when running from a checkout."""
    return Path(__file__).resolve().parents[1]


def data_root() -> Path:
    configured = os.environ.get("FRANKA_7LINK_ID_DATA")
    if configured:
        return Path(configured).expanduser().resolve()
    candidate = package_root() / "data"
    if (candidate / "raw").is_dir():
        return candidate
    raise FileNotFoundError(
        "Project data/raw is missing. Run offline analysis from the copied project folder "
        "or set FRANKA_7LINK_ID_DATA to the package data directory."
    )


def model_config_path() -> Path:
    return package_root() / "config" / "fr3_model.yaml"
