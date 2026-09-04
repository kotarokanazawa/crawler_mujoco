from __future__ import annotations

import copy
from pathlib import Path
import unittest

import mujoco

from crawler_mujoco import simulator as crawler
from crawler_mujoco.arena import resolve_arena_path


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


if __name__ == "__main__":
    unittest.main()
