#!/usr/bin/env python3
"""Read-only preflight checks for synchronized Laundry Butler recording."""

from __future__ import annotations

import concurrent.futures
import re
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

CAMERA_NODES = {
    "/camera_f/camera_f",
    "/camera_l/camera_l",
    "/camera_r/camera_r",
}
ARM_NODES = {
    "/piper_left_ctrl_node",
    "/piper_right_ctrl_node",
}

CAMERA_TOPICS = (
    "/camera_f/color/image_raw",
    "/camera_f/color/camera_info",
    "/camera_l/color/image_raw",
    "/camera_l/color/camera_info",
    "/camera_r/color/image_raw",
    "/camera_r/color/camera_info",
)
ARM_TOPICS = (
    "/puppet/joint_left",
    "/puppet/joint_right",
    "/puppet/end_pose_left",
    "/puppet/end_pose_right",
    "/piper_left_ctrl_node/arm_status",
    "/piper_right_ctrl_node/arm_status",
)
REQUIRED_TOPICS = CAMERA_TOPICS + ARM_TOPICS

LIGHTWEIGHT_LIVENESS_TOPICS = (
    "/camera_f/color/camera_info",
    "/camera_l/color/camera_info",
    "/camera_r/color/camera_info",
    "/puppet/joint_left",
    "/puppet/joint_right",
    "/piper_left_ctrl_node/arm_status",
    "/piper_right_ctrl_node/arm_status",
)

ISOLATED_COMMAND_TOPICS = (
    "/laundry_butler/observation_only/left/pos_cmd",
    "/laundry_butler/observation_only/left/joint_ctrl_single",
    "/laundry_butler/observation_only/left/enable_flag",
    "/laundry_butler/observation_only/right/pos_cmd",
    "/laundry_butler/observation_only/right/joint_ctrl_single",
    "/laundry_butler/observation_only/right/enable_flag",
)
ISOLATED_ENABLE_SERVICES = (
    "/laundry_butler/observation_only/left/enable_srv",
    "/laundry_butler/observation_only/right/enable_srv",
)
DEFAULT_ENABLE_SERVICES = (
    "/piper_left_ctrl_node/enable_srv",
    "/piper_right_ctrl_node/enable_srv",
)


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class Report:
    status: str
    checks: tuple[Check, ...]

    @property
    def passed(self) -> bool:
        return self.status != "fail"

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "checks": [check.to_dict() for check in self.checks],
        }


def run_preflight(output_root: Path, minimum_free_gb: float = 20.0) -> Report:
    checks = [
        check_dependencies(),
        check_nodes(),
        check_topics(),
        check_topic_liveness(),
        check_can("can_left"),
        check_can("can_right"),
        check_command_isolation(),
        check_storage(output_root, minimum_free_gb),
    ]
    status = aggregate_status(checks)
    return Report(status=status, checks=tuple(checks))


def aggregate_status(checks: Iterable[Check]) -> str:
    statuses = {check.status for check in checks}
    if "fail" in statuses:
        return "fail"
    if "warn" in statuses:
        return "warn"
    return "pass"


def run(
    command: list[str],
    *,
    timeout: float = 4.0,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )


def lines(command: list[str], timeout: float = 4.0) -> set[str]:
    try:
        result = run(command, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return set()
    if result.returncode != 0:
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}



def check_dependencies() -> Check:
    missing = [
        executable
        for executable in ("ros2", "ip", "timeout")
        if shutil.which(executable) is None
    ]
    if missing:
        return Check(
            "Recorder dependencies",
            "fail",
            "Missing executables: " + ", ".join(missing),
        )

    try:
        result = run(
            ["ros2", "pkg", "prefix", "rosbag2_storage_mcap"],
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return Check("Recorder dependencies", "fail", str(exc))
    if result.returncode != 0:
        return Check(
            "Recorder dependencies",
            "fail",
            "ROS 2 MCAP storage plugin is unavailable",
        )
    return Check(
        "Recorder dependencies",
        "pass",
        "ROS 2 CLI, CAN tools, and MCAP storage plugin available",
    )

def check_nodes() -> Check:
    nodes = lines(["ros2", "node", "list"])
    expected = CAMERA_NODES | ARM_NODES
    missing = sorted(expected - nodes)
    if missing:
        return Check(
            "ROS nodes",
            "fail",
            "Missing: " + ", ".join(missing),
        )
    return Check("ROS nodes", "pass", "All camera and arm nodes present")


def check_topics() -> Check:
    topics = lines(["ros2", "topic", "list"])
    missing = sorted(set(REQUIRED_TOPICS) - topics)
    if missing:
        return Check(
            "Required topics",
            "fail",
            "Missing: " + ", ".join(missing),
        )
    return Check("Required topics", "pass", f"All {len(REQUIRED_TOPICS)} topics present")


def _topic_alive(topic: str) -> tuple[str, bool, str]:
    try:
        result = run(
            ["timeout", "3", "ros2", "topic", "echo", "--once", topic],
            timeout=4.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return topic, False, str(exc)
    return topic, result.returncode == 0, result.stdout.strip()[-160:]


def check_topic_liveness() -> Check:
    failures: list[str] = []
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=len(LIGHTWEIGHT_LIVENESS_TOPICS)
    ) as executor:
        futures = [
            executor.submit(_topic_alive, topic)
            for topic in LIGHTWEIGHT_LIVENESS_TOPICS
        ]
        for future in concurrent.futures.as_completed(futures):
            topic, alive, _detail = future.result()
            if not alive:
                failures.append(topic)

    if failures:
        return Check(
            "Topic liveness",
            "fail",
            "No sample within 3 s: " + ", ".join(sorted(failures)),
        )
    return Check(
        "Topic liveness",
        "pass",
        "Camera info, joint, and arm-status samples received",
    )


def check_can(interface: str) -> Check:
    try:
        result = run(
            ["ip", "-details", "-statistics", "link", "show", "dev", interface],
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return Check(f"CAN {interface}", "fail", str(exc))

    text = result.stdout
    if result.returncode != 0:
        return Check(f"CAN {interface}", "fail", text.strip() or "Interface missing")

    up = bool(re.search(r"\bstate\s+UP\b", text))
    error_active = "ERROR-ACTIVE" in text
    bitrate = re.search(r"\bbitrate\s+(\d+)", text)
    bitrate_ok = bool(bitrate and bitrate.group(1) == "1000000")
    bus_off = bool(re.search(r"\bbus[-_ ]?off\s+[1-9]\d*", text, re.I))
    tx_errors = _counter(text, "tx_errors")
    rx_errors = _counter(text, "rx_errors")

    problems = []
    if not up:
        problems.append("link is not UP")
    if not error_active:
        problems.append("CAN state is not ERROR-ACTIVE")
    if not bitrate_ok:
        problems.append("bitrate is not 1000000")
    if bus_off:
        problems.append("bus-off counter is non-zero")
    if tx_errors not in (None, 0):
        problems.append(f"tx_errors={tx_errors}")
    if rx_errors not in (None, 0):
        problems.append(f"rx_errors={rx_errors}")

    if problems:
        return Check(f"CAN {interface}", "fail", "; ".join(problems))
    return Check(
        f"CAN {interface}",
        "pass",
        "UP, ERROR-ACTIVE, 1 Mbit/s",
    )


def _counter(text: str, name: str) -> int | None:
    match = re.search(rf"\b{re.escape(name)}\s+(\d+)", text, re.I)
    return int(match.group(1)) if match else None


def topic_counts(topic: str) -> tuple[int | None, int | None, str]:
    try:
        result = run(["ros2", "topic", "info", "-v", topic], timeout=3.0)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, None, str(exc)
    if result.returncode != 0:
        return None, None, result.stdout.strip()

    publishers = re.search(r"Publisher count:\s*(\d+)", result.stdout)
    subscriptions = re.search(r"Subscription count:\s*(\d+)", result.stdout)
    return (
        int(publishers.group(1)) if publishers else None,
        int(subscriptions.group(1)) if subscriptions else None,
        result.stdout,
    )


def check_command_isolation() -> Check:
    failures: list[str] = []
    for topic in ISOLATED_COMMAND_TOPICS:
        publishers, subscriptions, _ = topic_counts(topic)
        if publishers != 0 or subscriptions != 1:
            failures.append(
                f"{topic} publishers={publishers} subscribers={subscriptions}"
            )

    services = lines(["ros2", "service", "list"])
    for service in ISOLATED_ENABLE_SERVICES:
        if service not in services:
            failures.append(f"missing isolated service {service}")
    for service in DEFAULT_ENABLE_SERVICES:
        if service in services:
            failures.append(f"default enable service exposed: {service}")

    if failures:
        return Check("Command isolation", "fail", "; ".join(failures))
    return Check(
        "Command isolation",
        "pass",
        "No command publishers; isolated subscribers/services verified",
    )


def check_storage(output_root: Path, minimum_free_gb: float) -> Check:
    output_root.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(output_root)
    free_gb = usage.free / (1024**3)
    if free_gb < minimum_free_gb:
        return Check(
            "Free storage",
            "fail",
            f"{free_gb:.1f} GiB available; require {minimum_free_gb:.1f} GiB",
        )
    status = "warn" if free_gb < minimum_free_gb * 2 else "pass"
    return Check(
        "Free storage",
        status,
        f"{free_gb:.1f} GiB available",
    )
