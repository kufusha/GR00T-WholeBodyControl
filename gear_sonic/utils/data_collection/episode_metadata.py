"""Pure helpers for preserving extended LeRobot v2.1 episode metadata."""

from datetime import datetime
import json
from pathlib import Path
import re


_STANDARD_EPISODE_FIELDS = {"episode_index", "tasks", "length"}


def build_dataset_name(task_prompt: str, now: datetime | None = None) -> str:
    """Build a filesystem-safe automatic name from collection time and task."""
    timestamp = (now or datetime.now()).strftime("%Y-%m-%d-%H-%M-%S")
    task_slug = re.sub(r"[\W_]+", "-", task_prompt.lower()).strip("-")
    return f"{timestamp}-{task_slug[:64] or 'task'}"


def update_episode_metadata_file(path: Path, episode_index: int, extra: dict) -> None:
    overlap = _STANDARD_EPISODE_FIELDS.intersection(extra)
    if overlap:
        raise ValueError(f"Cannot replace standard episode fields: {sorted(overlap)}")
    json.dumps(extra)
    entries = [json.loads(line) for line in path.read_text().splitlines() if line]
    matches = [entry for entry in entries if entry["episode_index"] == episode_index]
    if len(matches) != 1:
        raise ValueError(f"Expected one episode {episode_index}, found {len(matches)}")
    matches[0].update(extra)
    temp_path = path.with_suffix(".jsonl.tmp")
    with temp_path.open("w", encoding="utf-8") as stream:
        for entry in entries:
            stream.write(json.dumps(entry) + "\n")
    temp_path.replace(path)


def build_processed_episode_metadata(
    original: dict, episode_index: int, length: int
) -> dict:
    return {
        **original,
        "episode_index": episode_index,
        "tasks": original.get("tasks", []),
        "length": length,
    }
