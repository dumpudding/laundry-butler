#!/usr/bin/env python3
"""Episode directory and sidecar management for Laundry Butler."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

SCHEMA_VERSION = "1.1"
EPISODE_PREFIX = "episode_"
TRASH_DIR_NAME = ".trash"


@dataclass(frozen=True)
class EpisodePaths:
    root: Path
    bag: Path
    episode_json: Path
    validation_json: Path
    recorder_log: Path

    @property
    def episode_id(self) -> str:
        return self.root.name


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def safe_slug(value: str, fallback: str = "task") -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_-]+", "-", value.strip()).strip("-_")
    return cleaned.lower()[:48] or fallback


def paths_for_episode(root: Path) -> EpisodePaths:
    root = root.expanduser().resolve()
    return EpisodePaths(
        root=root,
        bag=root / "bag",
        episode_json=root / "episode.json",
        validation_json=root / "validation.json",
        recorder_log=root / "recorder.log",
    )


def create_episode(
    output_root: Path,
    *,
    task: str,
    metadata: dict[str, Any],
) -> EpisodePaths:
    output_root = output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    episode_id = f"{EPISODE_PREFIX}{stamp}_{safe_slug(task)}"
    episode_root = output_root / episode_id
    episode_root.mkdir(parents=True, exist_ok=False)
    # ros2 bag record requires its -o directory not to exist yet; it creates bag/.
    paths = paths_for_episode(episode_root)

    payload = {
        "schema_version": SCHEMA_VERSION,
        "episode_id": episode_id,
        "created_at": utc_now_iso(),
        "status": "recording",
        "source_mcap_immutable": True,
        **metadata,
    }
    write_json_atomic(paths.episode_json, payload)
    return paths


def update_episode(paths: EpisodePaths, **updates: Any) -> dict[str, Any]:
    payload = read_json(paths.episode_json)
    payload.update(updates)
    payload["schema_version"] = SCHEMA_VERSION
    payload["last_edited_at"] = utc_now_iso()
    write_json_atomic(paths.episode_json, payload)
    return payload


def move_episode_to_trash(episode_root: Path, output_root: Path) -> Path:
    """Move an episode into output/.trash instead of permanently deleting it."""
    output_root = output_root.expanduser().resolve()
    episode_root = episode_root.expanduser().resolve()
    if episode_root.parent != output_root:
        raise ValueError("Refusing to move an episode outside the configured output root")
    if not episode_root.name.startswith(EPISODE_PREFIX):
        raise ValueError("Refusing to move a directory that is not an episode")
    if not (episode_root / "episode.json").is_file():
        raise ValueError("Episode metadata is missing")

    trash_root = output_root / TRASH_DIR_NAME
    trash_root.mkdir(parents=True, exist_ok=True)
    destination = trash_root / episode_root.name
    if destination.exists():
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        destination = trash_root / f"{episode_root.name}_{stamp}"
    shutil.move(str(episode_root), str(destination))
    return destination


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def git_revision(repository_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repository_root), "rev-parse", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    revision = result.stdout.strip()
    return revision if result.returncode == 0 and revision else None


def list_episodes(output_root: Path) -> list[dict[str, Any]]:
    output_root = output_root.expanduser()
    if not output_root.is_dir():
        return []

    episodes: list[dict[str, Any]] = []
    for child in output_root.iterdir():
        if not child.is_dir() or not child.name.startswith(EPISODE_PREFIX):
            continue
        episode_path = child / "episode.json"
        if not episode_path.is_file():
            continue
        try:
            payload = read_json(episode_path)
        except (OSError, ValueError, json.JSONDecodeError):
            continue

        validation_path = child / "validation.json"
        validation_status = "not run"
        if validation_path.is_file():
            try:
                validation_status = str(
                    read_json(validation_path).get("status", "unknown")
                )
            except (OSError, ValueError, json.JSONDecodeError):
                validation_status = "invalid"

        payload["_path"] = str(child)
        payload["_validation_status"] = validation_status
        episodes.append(payload)

    episodes.sort(
        key=lambda item: str(item.get("created_at", "")),
        reverse=True,
    )
    return episodes


def topic_manifest(topics: Iterable[str]) -> list[str]:
    result = []
    seen = set()
    for topic in topics:
        topic = str(topic).strip()
        if not topic or topic in seen:
            continue
        seen.add(topic)
        result.append(topic)
    return result
