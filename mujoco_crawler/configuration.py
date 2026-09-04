"""Load the MuJoCo configuration and resolve its external robot section."""

from pathlib import Path

import yaml


def load_config(path: str | Path) -> dict:
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    robot_config = config.get("robot_config")
    if robot_config:
        robot_path = Path(robot_config).expanduser()
        if not robot_path.is_absolute():
            robot_path = path.parent / robot_path
        with robot_path.resolve().open(encoding="utf-8") as stream:
            external = yaml.safe_load(stream) or {}
        robot = external.get("robot", external)
        if not isinstance(robot, dict):
            raise ValueError(f"robot configuration must be a mapping: {robot_path}")
        # An optional inline robot section can override a shared robot file.
        robot.update(config.get("robot", {}))
        config["robot"] = robot

    if "robot" not in config:
        raise KeyError(f"robot or robot_config is required in {path}")
    return config
