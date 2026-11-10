from __future__ import annotations

import copy
from pathlib import Path
import unittest

import mujoco
import numpy as np

from crawler_mujoco import simulator as crawler
from crawler_mujoco.arena import resolve_arena_path
from crawler_mujoco.lidar import LivoxMid360Lidar


ROOT = Path(__file__).resolve().parents[1]


class ModelSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = crawler.load_config(ROOT / "config" / "default.yaml")

    def test_default_is_standalone(self) -> None:
        self.assertEqual(self.config["environment"]["arena_yaml"], "")

    def test_all_grouser_shapes_compile(self) -> None:
        for shape in crawler.SHAPES:
            with self.subTest(shape=shape):
                model = mujoco.MjModel.from_xml_string(
                    crawler.build_xml(copy.deepcopy(self.config), shape))
                self.assertGreater(model.nbody, 1)
                self.assertGreater(model.nu, 0)

    def test_configured_six_axis_imu_sensors_compile(self) -> None:
        model = mujoco.MjModel.from_xml_string(
            crawler.build_xml(copy.deepcopy(self.config), "none"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for prefix in ("body", "front_imu", "rear_imu"):
            self.assertEqual(len(data.sensor(f"{prefix}_gyro").data), 3)
            self.assertEqual(len(data.sensor(f"{prefix}_acceleration").data), 3)

    def test_mid360_expands_to_colocated_lidar_and_imu(self) -> None:
        lidar = self.config["sensors"]["lidar"]["frames"]
        imu = self.config["sensors"]["imu"]["frames"]
        for name in ("front", "rear"):
            lidar_frame = next(x for x in lidar if x["site_name"] == f"{name}_lidar")
            imu_frame = next(x for x in imu if x["site_name"] == f"{name}_imu")
            self.assertEqual(lidar_frame["frame_id"], imu_frame["frame_id"])
            self.assertEqual(lidar_frame["position"], imu_frame["position"])
            self.assertEqual(lidar_frame["rpy_deg"], imu_frame["rpy_deg"])

    def test_all_bundled_arenas_compile(self) -> None:
        arena_root = ROOT / "assets" / "arenas"
        for arena in sorted(arena_root.rglob("*.yaml")):
            with self.subTest(arena=arena.relative_to(arena_root)):
                self.assertEqual(resolve_arena_path(arena.name if arena.parent == arena_root
                                                    else arena.relative_to(arena_root)),
                                 arena.resolve())
                config = copy.deepcopy(self.config)
                config["environment"]["arena_yaml"] = str(arena)
                model = mujoco.MjModel.from_xml_string(
                    crawler.build_xml(config, "none"))
                self.assertGreater(model.ngeom, 1)

    def test_livox_lidar_pattern_and_raycast(self) -> None:
        config = copy.deepcopy(self.config)
        config["sensors"]["lidar"]["enabled"] = True
        config["sensors"]["lidar"]["points_per_scan"] = 128
        # Exercise the analytic fallback independently of any local pattern asset.
        config["sensors"]["lidar"]["pattern_file"] = ""
        model = mujoco.MjModel.from_xml_string(
            crawler.build_xml(config, "none"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        sensor = LivoxMid360Lidar(
            model, config["sensors"]["lidar"], "front_lidar")
        directions = sensor.directions(1000)
        self.assertTrue(np.allclose(np.linalg.norm(directions, axis=1), 1.0))
        elevations = np.degrees(np.arcsin(directions[:, 2]))
        self.assertGreaterEqual(elevations.min(), -7.0 - 1e-6)
        self.assertLessEqual(elevations.max(), 52.0 + 1e-6)
        # The bundled floor is visible to the downward portion of the scan.
        self.assertGreater(len(sensor.scan(data)), 0)
        rear = LivoxMid360Lidar(
            model, config["sensors"]["lidar"], "rear_lidar")
        self.assertGreater(len(rear.scan(data)), 0)
        # Match ROS fixed-axis RPY: Rz(yaw) @ Ry(pitch) @ Rx(roll).
        rear_site = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "rear_lidar")
        rotation = np.asarray(data.site_xmat[rear_site]).reshape(3, 3)
        pitch = np.radians(20.0)
        expected = np.array([
            [-np.cos(pitch), 0.0, -np.sin(pitch)],
            [0.0, -1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)],
        ])
        self.assertTrue(np.allclose(rotation, expected, atol=1e-7))


if __name__ == "__main__":
    unittest.main()
