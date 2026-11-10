"""Ray-cast LiDAR with a deterministic Livox Mid-360-like scan pattern."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
import yaml


def load_ray_pattern(path: str | Path) -> tuple[np.ndarray, dict]:
    """Load the package-independent crawler_mujoco_lidar_pattern/v1 format."""
    path_text = str(path)
    if path_text.startswith("package://"):
        package_and_relative = path_text[len("package://"):].split("/", 1)
        if len(package_and_relative) != 2:
            raise ValueError(f"invalid package URI: {path_text}")
        package, relative = package_and_relative
        try:
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory(package))
        except (ImportError, LookupError):
            root = Path(__file__).resolve().parents[2] / package
        path = root / relative
    path = Path(path).expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("format") != "crawler_mujoco_lidar_pattern/v1":
        raise ValueError(f"unsupported LiDAR pattern format: {path}")
    csv_path = Path(config["pattern_csv"])
    if not csv_path.is_absolute():
        csv_path = path.parent / csv_path
    table = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                          encoding="utf-8")
    azimuth = np.radians(table["azimuth_deg"].astype(np.float64))
    elevation = np.radians(table["elevation_deg"].astype(np.float64))
    horizontal = np.cos(elevation)
    return np.column_stack((horizontal * np.cos(azimuth),
                            horizontal * np.sin(azimuth),
                            np.sin(elevation))), config


class LivoxMid360Lidar:
    """Generate a non-repeating 360 degree scan and intersect MuJoCo geoms.

    The actual Livox firing sequence is proprietary. This model follows the
    published 360 x 59 degree field of view while using incommensurate angular
    increments to obtain non-repetitive coverage.
    """

    def __init__(self, model: mujoco.MjModel, cfg: dict,
                 site_name: str = "lidar"):
        self.model = model
        self.cfg = cfg
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"MuJoCo site {site_name!r} is missing")
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "crawler")
        self.sequence = 0
        self.measured_directions = None
        self.pattern_points_per_scan = None
        pattern_file = str(cfg.get("pattern_file", "")).strip()
        if pattern_file:
            self.measured_directions, pattern = load_ray_pattern(pattern_file)
            self.pattern_points_per_scan = int(pattern.get(
                "points_per_scan", len(self.measured_directions)))

    def directions(self, count: int) -> np.ndarray:
        """Return unit rays in lidar_link (x forward, y left, z up)."""
        if self.measured_directions is not None:
            # Preserve one native scan period even when points_per_scan is
            # reduced for simulation performance.
            step = self.pattern_points_per_scan / max(count, 1)
            offsets = np.floor(np.arange(count) * step).astype(np.int64)
            indices = (self.sequence + offsets) % len(self.measured_directions)
            self.sequence = int(
                (self.sequence + self.pattern_points_per_scan)
                % len(self.measured_directions))
            return self.measured_directions[indices]
        indices = np.arange(self.sequence, self.sequence + count, dtype=np.float64)
        self.sequence += count
        azimuth = np.remainder(indices * math.pi * (3.0 - math.sqrt(5.0)),
                               2.0 * math.pi)
        phase = np.remainder(indices * (math.sqrt(3.0) - 1.0), 1.0)
        minimum = math.radians(float(self.cfg.get("vertical_min_deg", -7.0)))
        maximum = math.radians(float(self.cfg.get("vertical_max_deg", 52.0)))
        triangle = 1.0 - np.abs(2.0 * phase - 1.0)
        elevation = minimum + (maximum - minimum) * triangle
        horizontal = np.cos(elevation)
        return np.column_stack((horizontal * np.cos(azimuth),
                                horizontal * np.sin(azimuth),
                                np.sin(elevation)))

    def scan(self, data: mujoco.MjData) -> np.ndarray:
        """Return valid XYZ hits in lidar_link coordinates."""
        count = max(1, int(self.cfg.get("points_per_scan", 2000)))
        minimum = max(0.0, float(self.cfg.get("min_range", 0.1)))
        maximum = float(self.cfg.get("max_range", 40.0))
        local_rays = self.directions(count)
        rotation = np.asarray(data.site_xmat[self.site_id]).reshape(3, 3)
        origin = np.asarray(data.site_xpos[self.site_id], dtype=np.float64)
        points = []
        geom_id = np.empty(1, dtype=np.int32)
        for local_ray in local_rays:
            distance = mujoco.mj_ray(self.model, data, origin,
                                     rotation @ local_ray, None, True,
                                     self.body_id, geom_id)
            if minimum <= distance <= maximum:
                points.append(local_ray * distance)
        return np.asarray(points, dtype=np.float32).reshape(-1, 3)
