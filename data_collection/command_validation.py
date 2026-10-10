"""Streaming verification of command values against recorded CAN provenance."""

from __future__ import annotations

import json
import math

from demonstration_commands import (
    COMMAND_MAX_AGE_SECONDS, COMMAND_TOPICS, COMMAND_TRACE_TOPICS,
    JOINT_NAMES, sample_from_trace,
)


class CommandAudit:
    def __init__(self) -> None:
        self.pending = {side: {} for side in ("left", "right")}
        self.previous_stamp = {side: 0 for side in self.pending}
        self.matched = {side: 0 for side in self.pending}
        self.first_match = {side: None for side in self.pending}
        self.last_match = {side: None for side in self.pending}
        self.failures: list[str] = []

    def fail(self, detail: str) -> None:
        if len(self.failures) < 8:
            self.failures.append(detail)

    def add(self, topic: str, data: bytes, bag_timestamp_ns: int) -> None:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        if topic not in COMMAND_TOPICS + COMMAND_TRACE_TOPICS:
            return
        side = "left" if topic.endswith("_left") else "right"
        is_trace = topic in COMMAND_TRACE_TOPICS
        try:
            if is_trace:
                payload = json.loads(deserialize_message(data, String).data)
                sample = sample_from_trace(payload)
                if payload.get("interface") != f"can_{side}":
                    raise ValueError("Wrong source CAN interface")
                stamp = sample.timestamp_ns
                oldest_joint = min(frame.timestamp_ns for frame in sample.frames[:3])
                if not 0 <= (bag_timestamp_ns - oldest_joint) / 1e9 <= COMMAND_MAX_AGE_SECONDS:
                    raise ValueError("Joint source frames were stale at recording time")
                if stamp <= self.previous_stamp[side]:
                    raise ValueError("Source timestamps repeated or moved backward")
                self.previous_stamp[side] = stamp
                value = (sample.positions, sample.efforts)
            else:
                message = deserialize_message(data, JointState)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if tuple(message.name) != JOINT_NAMES or message.header.frame_id != f"piper_control_can_{side}":
                    raise ValueError("Wrong command layout or source marker")
                if len(message.position) != 7 or len(message.effort) != 7:
                    raise ValueError("Command must contain 6 joint targets and 1 gripper target")
                if not all(math.isfinite(value) for value in (*message.position, *message.effort)):
                    raise ValueError("Non-finite command values")
                if not 0 <= (bag_timestamp_ns - stamp) / 1e9 <= COMMAND_MAX_AGE_SECONDS:
                    raise ValueError("Command timestamp stale at recording time")
                value = (tuple(message.position), tuple(message.effort))
            entry = self.pending[side].setdefault(stamp, {})
            key = "trace" if is_trace else "command"
            if key in entry:
                raise ValueError("Duplicate command/source timestamp")
            entry[key] = value
            if len(entry) == 2:
                actual, expected = entry["command"], entry["trace"]
                if any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-10)
                       for actual_values, expected_values in zip(actual, expected)
                       for a, b in zip(actual_values, expected_values)):
                    self.fail(f"{side}: target values differ from CAN source frames")
                else:
                    self.matched[side] += 1
                    self.first_match[side] = stamp if self.first_match[side] is None else min(stamp, self.first_match[side])
                    self.last_match[side] = stamp if self.last_match[side] is None else max(stamp, self.last_match[side])
                del self.pending[side][stamp]
            # Keep memory bounded if an entire counterpart topic is absent.
            if len(self.pending[side]) > 1000:
                self.fail(f"{side}: commands/source frames missing counterparts")
                self.pending[side].pop(next(iter(self.pending[side])))
        except (ValueError, TypeError, KeyError, OverflowError, AttributeError) as exc:
            self.fail(f"{side}: {exc}")

    def result(self) -> dict[str, str]:
        boundary_warnings = []
        for side in self.pending:
            first, last = self.first_match[side], self.last_match[side]
            for stamp in self.pending[side]:
                # Independent DDS subscriptions may begin/end one message
                # apart. Only a 50 ms edge is tolerated, explicitly excluded
                # from the verified interval; any interior mismatch fails.
                if first is not None and last is not None and (
                    0 < first - stamp <= 50_000_000 or 0 < stamp - last <= 50_000_000
                ):
                    boundary_warnings.append(side)
                else:
                    self.fail(f"{side}: unmatched command/source sample within recording")
            if self.matched[side] < 2:
                self.fail(f"{side}: fewer than two verified commands")
        details = ", ".join(f"{side}: {count} targets match actual CAN frames" for side, count in self.matched.items())
        if boundary_warnings:
            details += f"; exclude {len(boundary_warnings)} unpaired boundary samples outside command_verified_interval_ns"
        return {
            "name": "Command provenance",
            "status": "fail" if self.failures else "warn" if boundary_warnings else "pass",
            "detail": "; ".join(self.failures) if self.failures else details,
        }

    def verified_interval(self) -> list[int] | None:
        if any(self.first_match[side] is None or self.last_match[side] is None for side in self.pending):
            return None
        return [max(self.first_match.values()), min(self.last_match.values())]
