"""Locate package resources in a source tree or an installed ROS 2 workspace."""

from __future__ import annotations

from pathlib import Path


def package_share_directory() -> Path:
    """Return the directory containing config, arena assets, and RViz files."""
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "config" / "default.yaml").is_file():
        return source_root

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise RuntimeError(
            "mujoco_crawler resources were not found; run from the source tree "
            "or source the ROS 2 install workspace"
        ) from error
    return Path(get_package_share_directory("mujoco_crawler"))
