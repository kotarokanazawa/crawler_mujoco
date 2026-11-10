"""Load the MuJoCo configuration and resolve its external robot section."""

from pathlib import Path
import copy

import yaml


def expand_composite_sensors(config: dict) -> None:
    """Expand each Mid-360 frame into colocated LiDAR and six-axis IMU."""
    sensors = config.setdefault("sensors", {})
    mid360 = sensors.get("mid360", {})
    if not isinstance(mid360, dict) or not bool(mid360.get("enabled", False)):
        return
    frames = mid360.get("frames", [])
    if not isinstance(frames, list) or not frames:
        raise ValueError("sensors.mid360.frames requires at least one frame")

    lidar = copy.deepcopy(sensors.get("lidar", {}))
    existing_lidar_frames = copy.deepcopy(lidar.get("frames", []))
    lidar.update(copy.deepcopy(mid360.get("lidar", {})))
    lidar["enabled"] = True
    lidar_frames = []

    imu = copy.deepcopy(sensors.get("imu", {}))
    existing_imu_frames = copy.deepcopy(imu.get("frames", []))
    imu.update(copy.deepcopy(mid360.get("imu", {})))
    imu["enabled"] = True
    imu_frames = []

    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"sensors.mid360.frames[{index}] must be a mapping")
        name = str(frame.get("name", f"mid360_{index}"))
        frame_id = str(frame.get("frame_id", f"{name}_link"))
        position = copy.deepcopy(frame.get("position", (0.0, 0.0, 0.0)))
        rpy = copy.deepcopy(frame.get("rpy_deg", (0.0, 0.0, 0.0)))
        lidar_frame = {
            "site_name": str(frame.get("lidar_site_name", f"{name}_lidar")),
            "frame_id": frame_id,
            "topic": str(frame.get(
                "pointcloud_topic", f"/mid360/{name}/points")),
            "position": position,
            "rpy_deg": rpy,
        }
        lidar_frame.update(copy.deepcopy(frame.get("lidar", {})))
        lidar_frames.append(lidar_frame)

        imu_frame = {
            "site_name": str(frame.get("imu_site_name", f"{name}_imu")),
            "sensor_name": str(frame.get("imu_sensor_name", f"{name}_imu")),
            "frame_id": frame_id,
            "topic": str(frame.get("imu_topic", f"/mid360/{name}/imu")),
            "position": position,
            "rpy_deg": rpy,
        }
        if "imu_aliases" in frame:
            imu_frame["aliases"] = copy.deepcopy(frame["imu_aliases"])
        imu_frame.update(copy.deepcopy(frame.get("imu", {})))
        imu_frames.append(imu_frame)

    lidar["frames"] = [*existing_lidar_frames, *lidar_frames]
    imu["frames"] = [*existing_imu_frames, *imu_frames]
    sensors["lidar"], sensors["imu"] = lidar, imu


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
    expand_composite_sensors(config)
    return config
