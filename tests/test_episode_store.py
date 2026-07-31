#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "data_collection"))

from episode_store import (
    create_episode,
    list_episodes,
    move_episode_to_trash,
    paths_for_episode,
    safe_slug,
    update_episode,
)


class EpisodeStoreTest(unittest.TestCase):
    def test_safe_slug(self) -> None:
        self.assertEqual(safe_slug(" Shirt folding / Level 2 "), "shirt-folding-level-2")
        self.assertEqual(safe_slug(""), "task")

    def test_create_update_list_and_trash(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths = create_episode(
                root,
                task="shirt folding",
                metadata={
                    "task": "shirt folding",
                    "instruction": "fold",
                    "outcome": "not_assessed",
                    "ros_domain_id": 88,
                    "recorded_topics": ["/topic"],
                },
            )
            self.assertTrue(paths.episode_json.is_file())
            self.assertFalse(
                paths.bag.exists(),
                "ros2 bag record must receive a non-existent output directory",
            )
            update_episode(
                paths,
                status="recorded",
                outcome="success",
                operator_disposition="usable",
                notes="clean fold",
            )
            paths.validation_json.write_text(
                json.dumps({"status": "pass"}),
                encoding="utf-8",
            )
            episodes = list_episodes(root)
            self.assertEqual(len(episodes), 1)
            self.assertEqual(episodes[0]["status"], "recorded")
            self.assertEqual(episodes[0]["outcome"], "success")
            self.assertEqual(episodes[0]["_validation_status"], "pass")

            reconstructed = paths_for_episode(paths.root)
            self.assertEqual(reconstructed.episode_json, paths.episode_json)
            trashed = move_episode_to_trash(paths.root, root)
            self.assertTrue(trashed.is_dir())
            self.assertEqual(list_episodes(root), [])

    def test_trash_rejects_outside_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outside = root / "nested" / "episode_bad"
            outside.mkdir(parents=True)
            (outside / "episode.json").write_text("{}", encoding="utf-8")
            with self.assertRaises(ValueError):
                move_episode_to_trash(outside, root)


if __name__ == "__main__":
    unittest.main()
