#!/usr/bin/env python3
"""Post-recording MCAP validation without modifying the source bag."""

from __future__ import annotations

import re
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from episode_store import write_json_atomic
from preflight import CAMERA_TOPICS, REQUIRED_TOPICS

CAMERA_RATE_RANGE = (24.0, 36.0)
ARM_RATE_RANGE = (160.0, 240.0)


@dataclass(frozen=True)
class TopicStats:
    count: int
    first_timestamp_ns: int | None
    last_timestamp_ns: int | None

    @property
    def duration_seconds(self) -> float:
        if self.first_timestamp_ns is None or self.last_timestamp_ns is None:
            return 0.0
        return max(0.0, (self.last_timestamp_ns - self.first_timestamp_ns) / 1e9)

    @property
    def rate_hz(self) -> float:
        duration = self.duration_seconds
        if self.count < 2 or duration <= 0.0:
            return 0.0
        return (self.count - 1) / duration

    def to_dict(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "duration_seconds": self.duration_seconds,
            "rate_hz": self.rate_hz,
        }


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def validate_episode(
    bag_dir: Path,
    validation_path: Path,
    recorder_log: Path,
) -> dict[str, Any]:
    indexed_by = "rosbag2_py"
    try:
        stats = index_with_rosbag2_py(bag_dir)
    except Exception as exc:
        indexed_by = "ros2 bag info fallback"
        stats = index_with_bag_info(bag_dir)
        index_warning = f"rosbag2_py indexing unavailable: {exc}"
    else:
        index_warning = None

    checks: list[dict[str, str]] = []
    missing = sorted(set(REQUIRED_TOPICS) - set(stats))
    if missing:
        checks.append(
            {
                "name": "Required topics",
                "status": "fail",
                "detail": "Missing: " + ", ".join(missing),
            }
        )
    else:
        checks.append(
            {
                "name": "Required topics",
                "status": "pass",
                "detail": f"All {len(REQUIRED_TOPICS)} topics recorded",
            }
        )

    rate_failures: list[str] = []
    rate_warnings: list[str] = []
    for topic in REQUIRED_TOPICS:
        topic_stats = stats.get(topic)
        if topic_stats is None:
            continue
        low, high = CAMERA_RATE_RANGE if topic in CAMERA_TOPICS else ARM_RATE_RANGE
        rate = topic_stats.rate_hz
        if rate == 0.0:
            rate_failures.append(f"{topic}=0 Hz")
        elif rate < low * 0.8 or rate > high * 1.2:
            rate_failures.append(f"{topic}={rate:.1f} Hz")
        elif rate < low or rate > high:
            rate_warnings.append(f"{topic}={rate:.1f} Hz")

    if rate_failures:
        checks.append(
            {
                "name": "Topic rates",
                "status": "fail",
                "detail": "; ".join(rate_failures),
            }
        )
    elif rate_warnings:
        checks.append(
            {
                "name": "Topic rates",
                "status": "warn",
                "detail": "; ".join(rate_warnings),
            }
        )
    else:
        checks.append(
            {
                "name": "Topic rates",
                "status": "pass",
                "detail": "Camera topics near 30 Hz; arm topics near 200 Hz",
            }
        )

    overlap = timestamp_overlap(stats)
    if overlap is None:
        checks.append(
            {
                "name": "Timestamp overlap",
                "status": "warn",
                "detail": "Per-topic timestamps unavailable in fallback index",
            }
        )
    else:
        overlap_seconds, coverage = overlap
        if coverage >= 0.90:
            status = "pass"
        elif coverage >= 0.75:
            status = "warn"
        else:
            status = "fail"
        checks.append(
            {
                "name": "Timestamp overlap",
                "status": status,
                "detail": (
                    f"{overlap_seconds:.3f} s common overlap "
                    f"({coverage * 100:.1f}% of union)"
                ),
            }
        )

    log_status, log_detail = inspect_recorder_log(recorder_log)
    checks.append(
        {
            "name": "Recorder log",
            "status": log_status,
            "detail": log_detail,
        }
    )

    if index_warning:
        checks.append(
            {
                "name": "Indexer",
                "status": "warn",
                "detail": index_warning,
            }
        )

    status = aggregate(check["status"] for check in checks)
    payload = {
        "schema_version": "1.0",
        "generated_at": utc_now_iso(),
        "status": status,
        "indexed_by": indexed_by,
        "bag_directory": str(bag_dir),
        "checks": checks,
        "topics": {
            topic: topic_stats.to_dict()
            for topic, topic_stats in sorted(stats.items())
        },
    }
    write_json_atomic(validation_path, payload)
    return payload


def aggregate(statuses) -> str:
    values = set(statuses)
    if "fail" in values:
        return "fail"
    if "warn" in values:
        return "warn"
    return "pass"


def index_with_rosbag2_py(bag_dir: Path) -> dict[str, TopicStats]:
    import rosbag2_py  # type: ignore

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="mcap",
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    mutable: dict[str, list[int | None]] = {}
    while reader.has_next():
        topic, _data, timestamp = reader.read_next()
        if topic not in mutable:
            mutable[topic] = [0, None, None]
        entry = mutable[topic]
        entry[0] = int(entry[0] or 0) + 1
        if entry[1] is None:
            entry[1] = int(timestamp)
        entry[2] = int(timestamp)

    return {
        topic: TopicStats(
            count=int(values[0] or 0),
            first_timestamp_ns=(
                int(values[1]) if values[1] is not None else None
            ),
            last_timestamp_ns=(
                int(values[2]) if values[2] is not None else None
            ),
        )
        for topic, values in mutable.items()
    }


def index_with_bag_info(bag_dir: Path) -> dict[str, TopicStats]:
    result = subprocess.run(
        ["ros2", "bag", "info", "--yaml", str(bag_dir)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=30.0,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stdout.strip() or "ros2 bag info failed")

    try:
        import yaml  # type: ignore

        payload = yaml.safe_load(result.stdout)
        information = payload.get("rosbag2_bagfile_information", payload)
        topics = information.get("topics_with_message_count", [])
        result_stats: dict[str, TopicStats] = {}
        for item in topics:
            metadata = item.get("topic_metadata", {})
            name = metadata.get("name")
            if not name:
                continue
            result_stats[str(name)] = TopicStats(
                count=int(item.get("message_count", 0)),
                first_timestamp_ns=None,
                last_timestamp_ns=None,
            )
        return result_stats
    except Exception:
        return parse_bag_info_text(result.stdout)


def parse_bag_info_text(text: str) -> dict[str, TopicStats]:
    stats: dict[str, TopicStats] = {}
    for match in re.finditer(
        r"Topic:\s*(?P<topic>/\S+).*?Count:\s*(?P<count>\d+)",
        text,
        flags=re.DOTALL,
    ):
        stats[match.group("topic")] = TopicStats(
            count=int(match.group("count")),
            first_timestamp_ns=None,
            last_timestamp_ns=None,
        )
    if not stats:
        raise RuntimeError("Could not parse ros2 bag info output")
    return stats


def timestamp_overlap(
    stats: dict[str, TopicStats],
) -> tuple[float, float] | None:
    selected = [
        stats[topic]
        for topic in REQUIRED_TOPICS
        if topic in stats
        and stats[topic].first_timestamp_ns is not None
        and stats[topic].last_timestamp_ns is not None
    ]
    if len(selected) != len(REQUIRED_TOPICS):
        return None

    common_start = max(int(item.first_timestamp_ns or 0) for item in selected)
    common_end = min(int(item.last_timestamp_ns or 0) for item in selected)
    union_start = min(int(item.first_timestamp_ns or 0) for item in selected)
    union_end = max(int(item.last_timestamp_ns or 0) for item in selected)
    overlap_seconds = max(0.0, (common_end - common_start) / 1e9)
    union_seconds = max(0.0, (union_end - union_start) / 1e9)
    coverage = overlap_seconds / union_seconds if union_seconds > 0.0 else 0.0
    return overlap_seconds, coverage


def inspect_recorder_log(path: Path) -> tuple[str, str]:
    if not path.is_file():
        return "warn", "Recorder log missing"
    text = path.read_text(encoding="utf-8", errors="replace")
    suspicious = [
        line.strip()
        for line in text.splitlines()
        if re.search(
            r"\b(error|failed|lost messages|dropped messages)\b",
            line,
            flags=re.I,
        )
    ]
    if suspicious:
        return "warn", "Review: " + " | ".join(suspicious[-3:])
    return "pass", "No obvious recorder error or loss message"
