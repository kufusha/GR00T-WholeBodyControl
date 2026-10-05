import json
from datetime import datetime

import pytest

from gear_sonic.utils.data_collection.episode_metadata import (
    build_dataset_name,
    build_processed_episode_metadata,
    update_episode_metadata_file,
)


def test_update_episode_metadata_preserves_standard_fields(tmp_path):
    path = tmp_path / "meta" / "episodes.jsonl"
    path.parent.mkdir()
    path.write_text(
        json.dumps({"episode_index": 0, "tasks": ["turn"], "length": 4}) + "\n",
        encoding="utf-8",
    )

    update_episode_metadata_file(
        path, 0, {"hand_mode": "two_hand", "angle_success": True}
    )

    saved = json.loads(path.read_text(encoding="utf-8").strip())
    assert saved == {
        "episode_index": 0,
        "tasks": ["turn"],
        "length": 4,
        "hand_mode": "two_hand",
        "angle_success": True,
    }


def test_update_episode_metadata_rejects_standard_field_overwrite(tmp_path):
    path = tmp_path / "episodes.jsonl"
    path.write_text('{"episode_index": 0, "tasks": [], "length": 1}\n')

    with pytest.raises(ValueError, match="standard episode fields"):
        update_episode_metadata_file(path, 0, {"length": 2})


def test_processed_episode_keeps_valve_metadata():
    original = {
        "episode_index": 8,
        "tasks": ["turn"],
        "length": 10,
        "hand_mode": "left_only",
        "angle_success": False,
    }

    rewritten = build_processed_episode_metadata(original, episode_index=0, length=7)

    assert rewritten == {
        "episode_index": 0,
        "tasks": ["turn"],
        "length": 7,
        "hand_mode": "left_only",
        "angle_success": False,
    }


def test_dataset_name_uses_timestamp_and_task_prompt():
    name = build_dataset_name(
        "Grasp & turn the circular valve clockwise!",
        now=datetime(2026, 10, 5, 14, 3, 9),
    )

    assert name == "2026-10-05-14-03-09-grasp-turn-the-circular-valve-clockwise"


def test_dataset_name_preserves_non_latin_task_words():
    name = build_dataset_name(
        "円形バルブを回す",
        now=datetime(2026, 10, 5, 14, 3, 9),
    )

    assert name == "2026-10-05-14-03-09-円形バルブを回す"
