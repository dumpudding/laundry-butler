"""Receive-only Piper CAN target capture, independent of measured joint state.

The Piper V2 protocol carries joint pairs in 0x155/156/157 (0.001 degrees)
and gripper targets in 0x159 (0.001 mm, effort 0.001 N m). No SDK connection
is used: SDK initialization can transmit frames even for an observer.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import socket
import struct
import threading
import time
from typing import Callable

COMMAND_TOPICS = (
    "/laundry_butler/demonstration/command_left",
    "/laundry_butler/demonstration/command_right",
)
COMMAND_TRACE_TOPICS = (
    "/laundry_butler/demonstration/source_left",
    "/laundry_butler/demonstration/source_right",
)
JOINT_NAMES = tuple(f"joint{i}" for i in range(7))
JOINT_IDS = (0x155, 0x156, 0x157)
CONTROL_IDS = JOINT_IDS + (0x159,)
COMMAND_MAX_AGE_SECONDS = 0.5
JOINT_CYCLE_MAX_SPAN_SECONDS = 0.1
CAN_FRAME = struct.Struct("=IB3x8s")
# Linux SO_TIMESTAMPNS, including platforms whose Python omits the symbol.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)


@dataclass(frozen=True)
class CommandFrame:
    can_id: int
    data: bytes
    timestamp_ns: int

    def to_dict(self) -> dict[str, object]:
        return {"can_id": self.can_id, "data_hex": self.data.hex(), "timestamp_ns": self.timestamp_ns}


def decode_frame(frame: CommandFrame) -> tuple[float, ...]:
    if len(frame.data) != 8 or frame.timestamp_ns <= 0:
        raise ValueError("Control frames require 8 bytes and a receive timestamp")
    if frame.can_id in JOINT_IDS:
        values = tuple(math.radians(value / 1000.0) for value in struct.unpack(">ii", frame.data))
        if any(abs(value) > math.tau for value in values):
            raise ValueError("Joint target outside supported Piper angle range")
        return values
    if frame.can_id == 0x159:
        opening, effort, status, set_zero = struct.unpack(">iHBB", frame.data)
        if not 0 <= opening <= 100000 or status not in (0, 1, 2, 3) or set_zero != 0:
            raise ValueError("Invalid gripper target or gripper zeroing in progress")
        return opening / 1e6, effort / 1000.0
    raise ValueError("Not a joint/gripper target frame")


@dataclass(frozen=True)
class CommandSample:
    frames: tuple[CommandFrame, ...]

    @property
    def timestamp_ns(self) -> int:
        return max(frame.timestamp_ns for frame in self.frames)

    @property
    def positions(self) -> tuple[float, ...]:
        return tuple(value for frame in self.frames[:3] for value in decode_frame(frame)) + (decode_frame(self.frames[3])[0],)

    @property
    def efforts(self) -> tuple[float, ...]:
        return (0.0,) * 6 + (decode_frame(self.frames[3])[1],)

    def trace(self, interface: str) -> dict[str, object]:
        return {
            "source": "socketcan_control_frames_v1",
            "interface": interface,
            "timestamp_ns": self.timestamp_ns,
            "frames": [frame.to_dict() for frame in self.frames],
        }


def sample_from_trace(payload: dict[str, object]) -> CommandSample:
    """Validate persisted provenance before interpreting targets as actions."""
    if payload.get("source") != "socketcan_control_frames_v1":
        raise ValueError("Unknown command source")
    frames = tuple(CommandFrame(int(item["can_id"]), bytes.fromhex(item["data_hex"]), int(item["timestamp_ns"]))
                   for item in payload["frames"])
    if tuple(frame.can_id for frame in frames) != CONTROL_IDS:
        raise ValueError("Incomplete command source frames")
    for frame in frames:
        decode_frame(frame)
    sample = CommandSample(frames)
    if sample.timestamp_ns != int(payload["timestamp_ns"]):
        raise ValueError("Command timestamp differs from source frames")
    stamps = [frame.timestamp_ns for frame in frames[:3]]
    if (max(stamps) - min(stamps)) / 1e9 > JOINT_CYCLE_MAX_SPAN_SECONDS:
        raise ValueError("Joint target frames do not form a coherent cycle")
    if (sample.timestamp_ns - min(stamps)) / 1e9 > COMMAND_MAX_AGE_SECONDS:
        raise ValueError("Stale joint target frames")
    return sample


class CommandAssembler:
    """Publish only new complete target cycles or real gripper changes.

    A gripper target is held until its next control frame. Missing initial
    gripper data is never filled from feedback. CAN reconnection, an invalid
    frame, or a command gap invalidates all cached components.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.pending: dict[int, CommandFrame] = {}
        self.dirty: set[int] = set()
        self.joints: tuple[CommandFrame, ...] = ()
        self.gripper: CommandFrame | None = None
        self.last_sample: CommandSample | None = None
        self.last_joint_arrival = 0.0
        self.error = "Waiting for teleoperation joint and gripper commands"

    def feed(self, frame: CommandFrame, now: float) -> CommandSample | None:
        if frame.can_id not in CONTROL_IDS:
            return None
        if self.last_joint_arrival and now - self.last_joint_arrival > COMMAND_MAX_AGE_SECONDS:
            self.reset()
        try:
            decode_frame(frame)
        except ValueError as exc:
            self.reset()
            self.error = str(exc)
            return None
        if frame.can_id in JOINT_IDS:
            self.pending[frame.can_id] = frame
            self.dirty.add(frame.can_id)
            if not all(can_id in self.dirty for can_id in JOINT_IDS):
                return None
            cycle = tuple(self.pending[can_id] for can_id in JOINT_IDS)
            stamps = [item.timestamp_ns for item in cycle]
            if (max(stamps) - min(stamps)) / 1e9 > JOINT_CYCLE_MAX_SPAN_SECONDS:
                self.error = "Joint target frames are incomplete or too far apart"
                return None
            self.joints = cycle
            self.dirty.clear()
            self.last_joint_arrival = now
        else:
            self.gripper = frame
        if not self.joints or self.gripper is None:
            return None
        candidate = CommandSample(self.joints + (self.gripper,))
        try:
            sample_from_trace(candidate.trace(""))
        except ValueError as exc:
            self.error = str(exc)
            return None
        if self.last_sample and candidate.timestamp_ns <= self.last_sample.timestamp_ns:
            return None
        self.last_sample = candidate
        self.error = ""
        return candidate

    def problem(self, now: float) -> str | None:
        if self.error:
            return self.error
        if not self.last_sample or now - self.last_joint_arrival > COMMAND_MAX_AGE_SECONDS:
            return "Teleoperation joint commands stopped (more than 0.5 s)"
        return None


class PassiveCANReader:
    """Raw SocketCAN receive loop: no send method and no arm initialization."""

    def __init__(self, interface: str, on_frame: Callable[[CommandFrame, float], None], on_error: Callable[[str], None]) -> None:
        self.interface = interface
        self.on_frame = on_frame
        self.on_error = on_error
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"command-tap-{interface}", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
                    # Exact standard data frames: reject RTR and extended frames.
                    filters = b"".join(struct.pack("=II", can_id, 0xC00007FF) for can_id in CONTROL_IDS)
                    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
                    sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
                    sock.bind((self.interface,))
                    sock.settimeout(0.1)
                    while not self.stop_event.is_set():
                        try:
                            packet, ancillary, flags, _address = sock.recvmsg(CAN_FRAME.size, 128)
                        except socket.timeout:
                            continue
                        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or len(packet) != CAN_FRAME.size:
                            raise OSError("Incomplete CAN frame or timestamp")
                        timestamp_ns = None
                        for level, kind, data in ancillary:
                            if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
                                seconds, nanoseconds = struct.unpack("@ll", data[:struct.calcsize("@ll")])
                                timestamp_ns = seconds * 1_000_000_000 + nanoseconds
                        if timestamp_ns is None:
                            raise OSError("CAN kernel receive timestamps unavailable")
                        # Kernel timestamps prevent a delayed receive queue from
                        # turning old CAN frames into apparently fresh commands.
                        age = (time.time_ns() - timestamp_ns) / 1e9
                        if not 0.0 <= age <= COMMAND_MAX_AGE_SECONDS:
                            self.on_error("Delayed CAN command frames; waiting for fresh targets")
                            continue
                        can_id, length, data = CAN_FRAME.unpack(packet)
                        if can_id in CONTROL_IDS:
                            self.on_frame(CommandFrame(can_id, data[:length], timestamp_ns), time.monotonic() - age)
            except OSError as exc:
                self.on_error(f"{self.interface}: {exc}")
                self.stop_event.wait(1.0)


def command_metadata() -> dict[str, object]:
    return {
        "source": "socketcan_control_frames_v1",
        "capture_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "topics": list(COMMAND_TOPICS),
        "provenance_topics": list(COMMAND_TRACE_TOPICS),
        "message_type": "sensor_msgs/msg/JointState",
        "layout": "[left joint0..5, left gripper, right joint0..5, right gripper]",
        "joint_unit": "radian",
        "gripper_unit": "metre",
        "gripper_effort_unit": "N m",
        "timestamp": "kernel CAN receive time; no periodic cached republishing",
        "gripper_policy": "hold last observed target until a new gripper command; never substitute feedback",
        "maximum_joint_gap_seconds": COMMAND_MAX_AGE_SECONDS,
        "maximum_joint_cycle_span_seconds": JOINT_CYCLE_MAX_SPAN_SECONDS,
        "protocol": "Piper V2 0x155/0x156/0x157 joint targets, 0x159 gripper target",
    }
