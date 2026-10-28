from pathlib import Path
import tempfile
import unittest

import yaml

from crawler_mujoco.environment_editor import arena_payload, arena_payload_from_config, objects_to_arena


ROOT = Path(__file__).resolve().parents[1]


class EnvironmentEditorTest(unittest.TestCase):
    def test_existing_arena_round_trip_and_edit(self) -> None:
        payload = arena_payload(ROOT / "assets" / "arenas" / "benchmark.yaml")
        self.assertGreater(len(payload["objects"]), 1)
        payload["objects"][0]["x"] = 12.5

        arena = objects_to_arena(payload["objects"], ROOT / "assets" / "models")
        first = payload["objects"][0]["name"]
        self.assertEqual(arena["robocup_arena"]["objects"][first]["x"], 12.5)

        with tempfile.TemporaryDirectory() as directory:
            saved = Path(directory) / "edited-copy.yaml"
            saved.write_text(yaml.safe_dump(arena, sort_keys=False), encoding="utf-8")
            reopened = arena_payload(saved)
        self.assertEqual(reopened["stem"], "edited-copy")
        self.assertEqual(reopened["objects"][0]["x"], 12.5)

    def test_unknown_model_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown model"):
            objects_to_arena([{"name": "missing_1", "model": "missing"}],
                             ROOT / "assets" / "models")

    def test_yaml_text_payload_defaults_missing_values(self) -> None:
        payload = arena_payload_from_config(
            {"robocup_arena": {"objects": {"wall50single_1": {"x": 2}}}},
            path="editor", stem="editor")
        self.assertEqual(payload["objects"][0]["x"], 2.0)
        self.assertEqual(payload["objects"][0]["sz"], 1.0)


if __name__ == "__main__":
    unittest.main()
