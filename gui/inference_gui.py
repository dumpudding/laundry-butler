#!/usr/bin/env python3
"""Laundry Butler real-robot inference GUI.

This GUI deliberately keeps policy serving in a separate process. It provides:
- live reduced-rate previews for all three cameras;
- live left/right joint rate, age, and state monitoring;
- CAN and policy-server health checks;
- managed CAN/camera/observation-arm startup controls;
- one-shot dry inference with no robot publishing;
- guarded full-task rollout using the proven dual-Piper command endpoints;
- per-replan action/state diagnostics and JSONL logging;
- observation snapshots for debugging.

The policy output is the physical 14-D absolute representation:
[left joint0..5, left gripper, right joint0..5, right gripper].
"""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from piper_msgs.srv import Enable
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from openpi_client import websocket_client_policy
from PIL import Image as PILImage

from PyQt5.QtCore import QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QFontDatabase, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


APP_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = APP_DIR.parent
LOG_ROOT = REPOSITORY_ROOT / "logs" / "inference"
SNAPSHOT_ROOT = LOG_ROOT / "snapshots"
RUNTIME_ROOT = LOG_ROOT / "runtime"

CAMERA_LAUNCH = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_CAMERA_LAUNCH",
        str(REPOSITORY_ROOT / "cameras" / "multi_camera_rgb.launch.py"),
    )
).expanduser()
ARM_LAUNCH = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_ARM_LAUNCH",
        str(REPOSITORY_ROOT / "arms" / "dual_arm_observe.launch.py"),
    )
).expanduser()

CAMERAS = (
    ("left", "Left wrist", "/camera_l/color/image_raw"),
    ("front", "Front / top", "/camera_f/color/image_raw"),
    ("right", "Right wrist", "/camera_r/color/image_raw"),
)
CAMERA_POLICY_KEYS = {
    "front": "cam_high",
    "left": "cam_left_wrist",
    "right": "cam_right_wrist",
}
JOINT_TOPICS = {
    "left": "/puppet/joint_left",
    "right": "/puppet/joint_right",
}
COMMAND_TOPICS = {
    "left": "/laundry_butler/observation_only/left/joint_ctrl_single",
    "right": "/laundry_butler/observation_only/right/joint_ctrl_single",
}
ENABLE_SERVICES = {
    "left": "/laundry_butler/observation_only/left/enable_srv",
    "right": "/laundry_butler/observation_only/right/enable_srv",
}
CAMERA_NODES = {
    "/camera_f/camera_f",
    "/camera_l/camera_l",
    "/camera_r/camera_r",
}
ARM_NODES = {
    "/piper_left_ctrl_node",
    "/piper_right_ctrl_node",
}

PREVIEW_MAX_FPS = 8.0
RATE_WINDOW_SECONDS = 2.5

ARM_IDX = np.array([0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12], dtype=int)
GRIP_IDX = np.array([6, 13], dtype=int)
DIMENSION_NAMES = [
    "L j0",
    "L j1",
    "L j2",
    "L j3",
    "L j4",
    "L j5",
    "L grip",
    "R j0",
    "R j1",
    "R j2",
    "R j3",
    "R j4",
    "R j5",
    "R grip",
]

# Manufacturer arm limits used by the successful rollout script.
SINGLE_LO = np.array([-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944], dtype=float)
SINGLE_HI = np.array([2.618, 3.14, 0.0, 1.745, 1.22, 2.0944], dtype=float)
JOINT_LIMIT_MARGIN = 0.05


class Bridge(QObject):
    log = pyqtSignal(str)
    rollout_state = pyqtSignal(str, str)
    rollout_metrics = pyqtSignal(object)
    action_debug = pyqtSignal(object)
    worker_error = pyqtSignal(str, str)
    server_metadata = pyqtSignal(object)
    snapshot_saved = pyqtSignal(str)


def trim(arrivals: deque[float], now: float) -> None:
    cutoff = now - RATE_WINDOW_SECONDS
    while arrivals and arrivals[0] < cutoff:
        arrivals.popleft()


def rate(arrivals: deque[float]) -> float:
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        return 0.0
    return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])


def to_qimage(message: Image) -> QImage:
    formats = {
        "rgb8": QImage.Format_RGB888,
        "bgr8": QImage.Format_BGR888,
        "rgba8": QImage.Format_RGBA8888,
        "bgra8": QImage.Format_ARGB32,
        "mono8": QImage.Format_Grayscale8,
    }
    image_format = formats.get(message.encoding.lower())
    if image_format is None:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    image = QImage(
        bytes(message.data),
        int(message.width),
        int(message.height),
        int(message.step),
        image_format,
    )
    if image.isNull():
        raise ValueError("Could not construct camera image")
    return image.copy()


def refresh_style(widget: QWidget) -> None:
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def health(label: QLabel, text: str, status: str) -> None:
    label.setText(text)
    label.setProperty("health", status)
    refresh_style(label)


def tcp_server_up(host: str, port: int, timeout: float = 0.15) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except OSError:
        return False


def can_status(interface: str) -> tuple[str, str]:
    try:
        result = subprocess.run(
            ["/usr/sbin/ip", "-details", "-statistics", "link", "show", "dev", interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=1.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "fail", str(exc)

    text = result.stdout
    if result.returncode != 0:
        return "fail", text.strip() or "interface missing"

    up = bool(re.search(r"\bstate\s+UP\b", text))
    error_active = "ERROR-ACTIVE" in text
    bitrate = re.search(r"\bbitrate\s+(\d+)", text)
    bitrate_ok = bool(bitrate and bitrate.group(1) == "1000000")

    # Parse RX/TX packet counters from the standard ip -statistics layout when possible.
    rx_packets = None
    tx_packets = None
    rx_match = re.search(r"RX:\s+bytes\s+packets.*?\n\s*\d+\s+(\d+)", text, re.S)
    tx_match = re.search(r"TX:\s+bytes\s+packets.*?\n\s*\d+\s+(\d+)", text, re.S)
    if rx_match:
        rx_packets = int(rx_match.group(1))
    if tx_match:
        tx_packets = int(tx_match.group(1))

    parts = []
    if up:
        parts.append("UP")
    else:
        parts.append("DOWN")
    parts.append("ERROR-ACTIVE" if error_active else "CAN state bad")
    parts.append("1 Mbit/s" if bitrate_ok else "bitrate bad")
    if rx_packets is not None:
        parts.append(f"RX {rx_packets}")
    if tx_packets is not None:
        parts.append(f"TX {tx_packets}")

    return ("pass" if up and error_active and bitrate_ok else "fail", " · ".join(parts))


def configure_can_interfaces() -> str:
    command = r'''
set -Eeuo pipefail
for iface in can_left can_right; do
    /usr/sbin/ip link show dev "$iface" >/dev/null 2>&1 || {
        echo "Missing SocketCAN interface: $iface" >&2
        exit 20
    }
done
for iface in can_left can_right; do
    /usr/sbin/ip link set dev "$iface" down 2>/dev/null || true
    /usr/sbin/ip link set dev "$iface" type can bitrate 1000000
    /usr/sbin/ip link set dev "$iface" txqueuelen 1000
    /usr/sbin/ip link set dev "$iface" up
    echo "Configured $iface"
done
'''
    result = subprocess.run(
        ["pkexec", "/bin/bash", "-c", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=45.0,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stdout.strip() or "CAN configuration failed")
    return result.stdout.strip()


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    log_handle: object
    log_path: Path


@dataclass
class RolloutConfig:
    prompt: str = "Fold the shirt."
    host: str = "127.0.0.1"
    port: int = 8000
    rate_hz: float = 30.0
    exec_steps: int = 50
    max_seconds: float = 60.0
    max_joint_step: float = 0.08
    max_gripper_step: float = 0.015
    hard_raw_joint_jump: float = 0.35
    command_speed: int = 30
    gripper_effort: float = 0.5
    observation_age_limit: float = 1.0
    arm_age_limit: float = 0.5
    observation_recovery_seconds: float = 2.0
    feedback_recovery_seconds: float = 5.0
    auto_enable: bool = True


class JsonlLogger:
    def __init__(self, mode: str, config: RolloutConfig) -> None:
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = LOG_ROOT / f"{mode}-{stamp}.jsonl"
        self._lock = threading.Lock()
        self.write("run_start", mode=mode, config=asdict(config))

    def write(self, event: str, **payload: object) -> None:
        record = {
            "time": datetime.now().astimezone().isoformat(),
            "monotonic": time.monotonic(),
            "event": event,
            **payload,
        }
        text = json.dumps(record, separators=(",", ":"), allow_nan=False)
        with self._lock:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(text + "\n")


class RobotNode(Node):
    """ROS monitor/control node.

    Camera callbacks deliberately only store the latest ROS Image. Converting images
    for policy inference or GUI preview happens outside the ROS callback threads.
    """

    def __init__(self) -> None:
        super().__init__("laundry_butler_inference_gui")
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.cb_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()

        self.images: dict[str, Image] = {}
        self.joints: dict[str, np.ndarray] = {}
        self.camera_times: dict[str, float] = {}
        self.joint_times: dict[str, float] = {}
        self.camera_arrivals = {key: deque() for key, _title, _topic in CAMERAS}
        self.joint_arrivals = {side: deque() for side in JOINT_TOPICS}
        self._subscriptions = []

        for key, _title, topic in CAMERAS:
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, k=key: self._image_cb(k, msg),
                    qos_profile_sensor_data,
                    callback_group=self.cb_group,
                )
            )

        for side, topic in JOINT_TOPICS.items():
            self._subscriptions.append(
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, s=side: self._joint_cb(s, msg),
                    qos_profile_sensor_data,
                    callback_group=self.cb_group,
                )
            )

        self.enable_clients = {
            side: self.create_client(Enable, service, callback_group=self.service_group)
            for side, service in ENABLE_SERVICES.items()
        }
        self.command_publishers: dict[str, object] = {}

    def _image_cb(self, key: str, msg: Image) -> None:
        now = time.monotonic()
        with self.lock:
            self.images[key] = msg
            self.camera_times[key] = now
            arrivals = self.camera_arrivals[key]
            arrivals.append(now)
            trim(arrivals, now)

    def _joint_cb(self, side: str, msg: JointState) -> None:
        if len(msg.position) < 7:
            return
        now = time.monotonic()
        values = np.asarray(msg.position[:7], dtype=np.float32)
        with self.lock:
            self.joints[side] = values
            self.joint_times[side] = now
            arrivals = self.joint_arrivals[side]
            arrivals.append(now)
            trim(arrivals, now)

    def monitor_snapshot(self) -> dict[str, object]:
        now = time.monotonic()
        with self.lock:
            cameras = {}
            for key, arrivals in self.camera_arrivals.items():
                trim(arrivals, now)
                last = self.camera_times.get(key, 0.0)
                cameras[key] = {
                    "rate": rate(arrivals),
                    "age": now - last if last else float("inf"),
                    "message": self.images.get(key),
                }
            joints = {}
            for side, arrivals in self.joint_arrivals.items():
                trim(arrivals, now)
                last = self.joint_times.get(side, 0.0)
                joints[side] = {
                    "rate": rate(arrivals),
                    "age": now - last if last else float("inf"),
                    "state": self.joints.get(side).copy() if side in self.joints else None,
                }
        return {"cameras": cameras, "joints": joints}

    def ros_nodes(self) -> set[str]:
        result = set()
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                namespace = namespace.rstrip("/")
                full = f"{namespace}/{name}" if namespace else f"/{name}"
                result.add(full)
        except Exception:
            return set()
        return result

    def observation_ready(self, max_age: float = 1.0) -> bool:
        snap = self.monitor_snapshot()
        return (
            all(float(v["age"]) < max_age for v in snap["cameras"].values())
            and all(float(v["age"]) < max_age for v in snap["joints"].values())
        )

    def arms_fresh(self, max_age: float = 0.5) -> bool:
        snap = self.monitor_snapshot()["joints"]
        return all(float(v["age"]) < max_age for v in snap.values())

    def state(self) -> np.ndarray:
        with self.lock:
            if "left" not in self.joints or "right" not in self.joints:
                raise RuntimeError("Both arm joint states are not available")
            return np.concatenate([self.joints["left"], self.joints["right"]]).astype(np.float32)

    def observation(self, prompt: str) -> dict[str, object]:
        snap = self.monitor_snapshot()
        if not self.observation_ready():
            raise RuntimeError("Robot/camera observation is stale")
        state = self.state()
        raw_images = {key: snap["cameras"][key]["message"] for key, _title, _topic in CAMERAS}
        images = {}
        for gui_key, message in raw_images.items():
            if message is None:
                raise RuntimeError(f"Missing camera: {gui_key}")
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            images[CAMERA_POLICY_KEYS[gui_key]] = np.ascontiguousarray(
                rgb.transpose(2, 0, 1), dtype=np.uint8
            )
        return {"state": state, "images": images, "prompt": prompt}

    def preview_qimage(self, key: str) -> Optional[QImage]:
        snap = self.monitor_snapshot()
        message = snap["cameras"].get(key, {}).get("message")
        if message is None:
            return None
        try:
            return to_qimage(message)
        except ValueError:
            return None

    def age_report(self) -> dict[str, float]:
        snap = self.monitor_snapshot()
        result = {}
        for key, value in snap["cameras"].items():
            result[f"cam_{key}"] = round(float(value["age"]), 3)
        for side, value in snap["joints"].items():
            result[f"joint_{side}"] = round(float(value["age"]), 3)
        return result

    def ensure_command_interfaces(self) -> None:
        if self.command_publishers:
            return
        self.command_publishers = {
            side: self.create_publisher(JointState, topic, 1)
            for side, topic in COMMAND_TOPICS.items()
        }

    def release_command_interfaces(self) -> None:
        for publisher in list(self.command_publishers.values()):
            try:
                self.destroy_publisher(publisher)
            except Exception:
                pass
        self.command_publishers.clear()

    def command_subscription_counts(self) -> dict[str, int]:
        return {
            side: int(pub.get_subscription_count())
            for side, pub in self.command_publishers.items()
        }

    def enable_arm(self, side: str, timeout: float = 7.0) -> None:
        client = self.enable_clients[side]
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{side} enable service unavailable")
        req = Enable.Request()
        req.enable_request = True
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"{side} enable timed out")
            time.sleep(0.03)
        result = future.result()
        if result is None or not result.enable_response:
            raise RuntimeError(f"{side} failed to enable")

    @staticmethod
    def make_joint_command(target7: np.ndarray, speed: int, gripper_effort: float, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
        msg.position = [float(x) for x in target7]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(speed)]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(gripper_effort)]
        return msg

    def publish_target(self, action: np.ndarray, speed: int, gripper_effort: float) -> None:
        if set(self.command_publishers) != {"left", "right"}:
            raise RuntimeError("Command interfaces are not armed")
        stamp = self.get_clock().now().to_msg()
        self.command_publishers["left"].publish(
            self.make_joint_command(action[:7], speed, gripper_effort, stamp)
        )
        self.command_publishers["right"].publish(
            self.make_joint_command(action[7:], speed, gripper_effort, stamp)
        )

    def hold_current(self, speed: int = 20, gripper_effort: float = 0.5) -> None:
        if not self.command_publishers:
            return
        try:
            state = self.state().astype(float)
        except Exception:
            return
        state[6] = np.clip(state[6], 0.0, 0.08)
        state[13] = np.clip(state[13], 0.0, 0.08)
        for _ in range(5):
            try:
                self.publish_target(state, speed, gripper_effort)
            except Exception:
                break
            time.sleep(1.0 / 30.0)

    def save_snapshot(self, prompt: str) -> Path:
        SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)
        root = SNAPSHOT_ROOT / datetime.now().strftime("snapshot-%Y%m%d-%H%M%S")
        root.mkdir(parents=True, exist_ok=False)
        snap = self.monitor_snapshot()
        state = self.state()
        metadata = {
            "time": datetime.now().astimezone().isoformat(),
            "prompt": prompt,
            "state": state.tolist(),
            "ages": self.age_report(),
            "rates_hz": {
                "cameras": {k: float(v["rate"]) for k, v in snap["cameras"].items()},
                "joints": {k: float(v["rate"]) for k, v in snap["joints"].items()},
            },
        }
        (root / "snapshot.json").write_text(json.dumps(metadata, indent=2) + "\n")
        for key, _title, _topic in CAMERAS:
            message = snap["cameras"][key]["message"]
            if message is None:
                continue
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            PILImage.fromarray(rgb).save(root / f"{key}.png")
        return root


def sanitize_actions(
    raw_actions: np.ndarray,
    current: np.ndarray,
    cfg: RolloutConfig,
) -> tuple[np.ndarray, dict[str, float]]:
    actions = np.asarray(raw_actions, dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] != 14:
        raise RuntimeError(f"Bad action shape: {actions.shape}")
    if actions.shape[0] < cfg.exec_steps:
        raise RuntimeError(
            f"Policy returned only {actions.shape[0]} actions; need {cfg.exec_steps}"
        )
    if not np.isfinite(actions).all():
        raise RuntimeError("NaN/Inf action received")

    actions = actions.copy()
    actions[:, 6] = np.clip(actions[:, 6], 0.0, 0.08)
    actions[:, 13] = np.clip(actions[:, 13], 0.0, 0.08)

    current_safe = current.astype(np.float64).copy()
    current_safe[6] = np.clip(current_safe[6], 0.0, 0.08)
    current_safe[13] = np.clip(current_safe[13], 0.0, 0.08)

    raw_seq = np.vstack([current_safe, actions[: cfg.exec_steps]])
    raw_jumps = np.diff(raw_seq, axis=0)
    raw_max_joint = float(np.max(np.abs(raw_jumps[:, ARM_IDX])))
    raw_max_gripper = float(np.max(np.abs(raw_jumps[:, GRIP_IDX])))

    if raw_max_joint > cfg.hard_raw_joint_jump:
        raise RuntimeError(
            f"Raw policy discontinuity {raw_max_joint:.4f} rad exceeds hard abort "
            f"{cfg.hard_raw_joint_jump:.4f} rad"
        )

    published = actions.copy()
    prev = current_safe.copy()
    for i in range(cfg.exec_steps):
        target = published[i].copy()
        delta = target - prev
        target[ARM_IDX] = prev[ARM_IDX] + np.clip(
            delta[ARM_IDX], -cfg.max_joint_step, cfg.max_joint_step
        )
        target[GRIP_IDX] = prev[GRIP_IDX] + np.clip(
            delta[GRIP_IDX], -cfg.max_gripper_step, cfg.max_gripper_step
        )
        target[6] = np.clip(target[6], 0.0, 0.08)
        target[13] = np.clip(target[13], 0.0, 0.08)
        published[i] = target
        prev = target

    lo = np.tile(SINGLE_LO, 2) - JOINT_LIMIT_MARGIN
    hi = np.tile(SINGLE_HI, 2) + JOINT_LIMIT_MARGIN
    arm_values = published[: cfg.exec_steps, ARM_IDX]
    if np.any(arm_values < lo):
        raise RuntimeError("Published joint target below allowed range")
    if np.any(arm_values > hi):
        raise RuntimeError("Published joint target above allowed range")

    smooth_seq = np.vstack([current_safe, published[: cfg.exec_steps]])
    smooth_jumps = np.diff(smooth_seq, axis=0)
    smooth_max_joint = float(np.max(np.abs(smooth_jumps[:, ARM_IDX])))
    smooth_max_gripper = float(np.max(np.abs(smooth_jumps[:, GRIP_IDX])))

    return published, {
        "raw_max_joint_step": raw_max_joint,
        "published_max_joint_step": smooth_max_joint,
        "raw_max_gripper_step": raw_max_gripper,
        "published_max_gripper_step": smooth_max_gripper,
    }


class RolloutRunner:
    def __init__(
        self,
        node: RobotNode,
        bridge: Bridge,
        cfg: RolloutConfig,
        stop_event: threading.Event,
        dry_run: bool,
    ) -> None:
        self.node = node
        self.bridge = bridge
        self.cfg = cfg
        self.stop_event = stop_event
        self.dry_run = dry_run
        self.logger = JsonlLogger("dry-inference" if dry_run else "rollout", cfg)

    def emit_log(self, text: str) -> None:
        self.bridge.log.emit(text)

    def wait_for_observation(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while not self.node.observation_ready(self.cfg.observation_age_limit):
            if self.stop_event.is_set():
                raise InterruptedError("Stop requested")
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "Observation did not recover; ages=" + repr(self.node.age_report())
                )
            time.sleep(0.02)

    def connect_client(self):
        if not tcp_server_up(self.cfg.host, self.cfg.port, timeout=0.5):
            raise RuntimeError(
                f"Policy server is not reachable at {self.cfg.host}:{self.cfg.port}"
            )
        client = websocket_client_policy.WebsocketClientPolicy(
            host=self.cfg.host,
            port=self.cfg.port,
        )
        self.bridge.server_metadata.emit(client.get_server_metadata())
        return client

    def run(self) -> None:
        status = "finished"
        detail = ""
        command_armed = False
        try:
            self.bridge.rollout_state.emit("checking", "Checking live observations")
            self.wait_for_observation(10.0)
            client = self.connect_client()
            self.emit_log(f"Policy server connected: {self.cfg.host}:{self.cfg.port}")
            self.emit_log(f"Run log: {self.logger.path}")

            if self.dry_run:
                self.bridge.rollout_state.emit("dry", "Running one dry inference")
                self._one_inference(client, cycle=1, elapsed=0.0, publish=False)
                self.logger.write("run_end", status="dry_success")
                detail = "Dry inference complete"
                return

            self.node.ensure_command_interfaces()
            command_armed = True
            deadline = time.monotonic() + 3.0
            while True:
                counts = self.node.command_subscription_counts()
                if counts.get("left", 0) >= 1 and counts.get("right", 0) >= 1:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Piper command subscriber missing: {counts}")
                time.sleep(0.05)
            self.emit_log(f"Command subscribers: {counts}")

            if self.cfg.auto_enable:
                self.bridge.rollout_state.emit("enabling", "Enabling both arms")
                self.node.enable_arm("left")
                self.emit_log("LEFT: ENABLED")
                self.node.enable_arm("right")
                self.emit_log("RIGHT: ENABLED")
                self.wait_for_observation(self.cfg.feedback_recovery_seconds)
                self.emit_log("Feedback recovered after enable")

            self.bridge.rollout_state.emit("running", "Full task rollout running")
            self.logger.write("motion_start")
            start = time.monotonic()
            cycle = 0
            while time.monotonic() - start < self.cfg.max_seconds:
                if self.stop_event.is_set():
                    status = "stopped"
                    detail = "Stop requested"
                    break
                cycle += 1
                self.wait_for_observation(self.cfg.observation_recovery_seconds)
                elapsed = time.monotonic() - start
                self._one_inference(client, cycle=cycle, elapsed=elapsed, publish=True)

            if status == "finished":
                detail = f"{self.cfg.max_seconds:.1f} s backstop reached"
            self.logger.write("run_end", status=status, detail=detail)

        except InterruptedError as exc:
            status = "stopped"
            detail = str(exc)
            self.logger.write("run_end", status=status, detail=detail)
        except BaseException as exc:
            status = "aborted"
            detail = str(exc)
            self.logger.write(
                "run_end",
                status=status,
                detail=detail,
                traceback=traceback.format_exc(),
            )
            self.bridge.worker_error.emit("Rollout aborted", detail)
        finally:
            if command_armed:
                try:
                    self.bridge.rollout_state.emit("holding", "Commanding current-position hold")
                    self.node.hold_current(self.cfg.command_speed, self.cfg.gripper_effort)
                except Exception as exc:
                    self.emit_log(f"Hold failed: {exc}")
                try:
                    self.node.release_command_interfaces()
                except Exception:
                    pass
            if self.dry_run:
                if status == "aborted":
                    self.bridge.rollout_state.emit("idle", f"Dry inference aborted: {detail}")
                else:
                    self.bridge.rollout_state.emit("idle", detail or "Dry inference complete")
            else:
                label = {
                    "finished": "Run finished",
                    "stopped": "Stopped / holding",
                    "aborted": "Aborted / holding",
                }.get(status, status)
                self.bridge.rollout_state.emit("idle", f"{label}: {detail}".strip())

    def _one_inference(self, client, cycle: int, elapsed: float, publish: bool) -> None:
        if self.stop_event.is_set():
            raise InterruptedError("Stop requested")

        obs = self.node.observation(self.cfg.prompt)
        infer_start = time.monotonic()
        result = client.infer(obs)
        inference_seconds = time.monotonic() - infer_start

        if not self.node.arms_fresh(self.cfg.arm_age_limit):
            raise RuntimeError(
                "Arm feedback stale after inference; ages=" + repr(self.node.age_report())
            )

        current = self.node.state()
        raw = np.asarray(result["actions"], dtype=np.float64)
        published, step_metrics = sanitize_actions(raw, current, self.cfg)
        metrics = {
            "cycle": cycle,
            "elapsed": elapsed,
            "inference_ms": inference_seconds * 1000.0,
            "action_count": int(raw.shape[0]),
            **step_metrics,
        }

        debug = {
            "current": current.copy(),
            "raw_first": raw[0].copy(),
            "published_first": published[0].copy(),
            "raw_last": raw[min(self.cfg.exec_steps, len(raw)) - 1].copy(),
            "published_last": published[min(self.cfg.exec_steps, len(published)) - 1].copy(),
        }
        self.bridge.rollout_metrics.emit(metrics)
        self.bridge.action_debug.emit(debug)
        self.emit_log(
            f"REPLAN {cycle:03d}  {elapsed:5.1f}s  "
            f"infer {inference_seconds * 1000.0:6.1f} ms  "
            f"joint {step_metrics['raw_max_joint_step']:.5f} -> "
            f"{step_metrics['published_max_joint_step']:.5f} rad  "
            f"grip {step_metrics['raw_max_gripper_step']:.5f} -> "
            f"{step_metrics['published_max_gripper_step']:.5f} m"
        )
        self.logger.write(
            "replan",
            metrics=metrics,
            ages=self.node.age_report(),
            state=current.tolist(),
            raw_actions=raw.tolist(),
            published_actions=published[: self.cfg.exec_steps].tolist(),
        )

        if not publish:
            return

        period = 1.0 / self.cfg.rate_hz
        next_tick = time.monotonic()
        max_lag = 0.0
        for i in range(self.cfg.exec_steps):
            if self.stop_event.is_set():
                raise InterruptedError("Stop requested")
            if not self.node.arms_fresh(self.cfg.arm_age_limit):
                raise RuntimeError(
                    "Arm feedback lost during motion; ages=" + repr(self.node.age_report())
                )
            self.node.publish_target(
                published[i], self.cfg.command_speed, self.cfg.gripper_effort
            )
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                max_lag = max(max_lag, -delay)

        metrics = dict(metrics)
        metrics["max_publish_lag_ms"] = max_lag * 1000.0
        self.bridge.rollout_metrics.emit(metrics)
        self.logger.write("chunk_published", cycle=cycle, max_publish_lag_ms=max_lag * 1000.0)


class PreviewLabel(QLabel):
    def __init__(self) -> None:
        super().__init__("Waiting for camera")
        self.source: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(190, 125)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#111;color:#aaa;border:1px solid #444")

    def set_image(self, image: QImage) -> None:
        self.source = QPixmap.fromImage(image)
        self._rescale()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self) -> None:
        if self.source is not None:
            self.setPixmap(
                self.source.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )


class CameraPane(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        heading = QLabel(title)
        heading.setProperty("sectionHeading", True)
        self.preview = PreviewLabel()
        self.health = QLabel("No signal")
        self.health.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(heading)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.health)


class ArmPane(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        heading = QLabel(title)
        heading.setProperty("sectionHeading", True)
        self.health = QLabel("No signal")
        self.values = QLabel("Waiting for joint feedback")
        self.values.setWordWrap(True)
        self.values.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 7)
        header = QHBoxLayout()
        header.addWidget(heading)
        header.addStretch(1)
        header.addWidget(self.health)
        layout.addLayout(header)
        layout.addWidget(self.values)


class Window(QMainWindow):
    def __init__(self, node: RobotNode, bridge: Bridge) -> None:
        super().__init__()
        self.node = node
        self.bridge = bridge
        self.settings = QSettings("LaundryButler", "Inference")
        self.managed: dict[str, ManagedProcess] = {}
        self.rollout_stop = threading.Event()
        self.rollout_thread: Optional[threading.Thread] = None
        self.last_metrics: dict[str, object] = {}
        self.last_action_debug: dict[str, np.ndarray] = {}
        self.last_preview = {key: 0.0 for key, _title, _topic in CAMERAS}

        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)
        RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)

        self.setWindowTitle("Laundry Butler inference")
        self.setMinimumSize(1150, 760)
        self.resize(1700, 980)
        geometry = self.settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        self.setStatusBar(QStatusBar())

        self.camera_panes = {key: CameraPane(title) for key, title, _topic in CAMERAS}
        camera_splitter = QSplitter(Qt.Horizontal)
        for key, _title, _topic in CAMERAS:
            camera_splitter.addWidget(self.camera_panes[key])
        camera_splitter.setChildrenCollapsible(False)
        camera_splitter.setSizes([1, 1, 1])
        self.camera_splitter = camera_splitter

        camera_group = QGroupBox("Live cameras")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.addWidget(camera_splitter)

        self.arm_panes = {
            "left": ArmPane("Left arm"),
            "right": ArmPane("Right arm"),
        }
        arm_splitter = QSplitter(Qt.Horizontal)
        arm_splitter.addWidget(self.arm_panes["left"])
        arm_splitter.addWidget(self.arm_panes["right"])
        arm_splitter.setChildrenCollapsible(False)
        arm_group = QGroupBox("Live arm feedback")
        arm_layout = QVBoxLayout(arm_group)
        arm_layout.addWidget(arm_splitter)

        left_controls = QWidget()
        left_layout = QVBoxLayout(left_controls)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.addWidget(self.build_system_group())
        left_layout.addWidget(self.build_rollout_group())
        left_layout.addStretch(1)
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setWidget(left_controls)

        debug_widget = QWidget()
        debug_layout = QVBoxLayout(debug_widget)
        debug_layout.setContentsMargins(4, 4, 4, 4)
        debug_layout.addWidget(self.build_metrics_group())
        debug_layout.addWidget(self.build_action_table_group(), 1)
        debug_layout.addWidget(self.build_log_group(), 1)

        lower = QSplitter(Qt.Horizontal)
        lower.addWidget(left_scroll)
        lower.addWidget(debug_widget)
        lower.setChildrenCollapsible(False)
        lower.setSizes([540, 1100])
        self.lower_splitter = lower

        main = QSplitter(Qt.Vertical)
        main.addWidget(camera_group)
        main.addWidget(arm_group)
        main.addWidget(lower)
        main.setChildrenCollapsible(False)
        main.setSizes([390, 155, 400])
        self.main_splitter = main

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(main)
        self.setCentralWidget(central)

        self.connect_signals()
        self.monitor_timer = QTimer(self)
        self.monitor_timer.timeout.connect(self.refresh_monitor)
        self.monitor_timer.start(200)
        self.system_timer = QTimer(self)
        self.system_timer.timeout.connect(self.refresh_system_health)
        self.system_timer.start(1200)
        QTimer.singleShot(100, self.refresh_system_health)

    def build_system_group(self) -> QGroupBox:
        self.domain_label = QLabel(f"ROS domain: {os.environ.get('ROS_DOMAIN_ID', '88')}")
        self.can_left_label = QLabel("can_left: checking")
        self.can_right_label = QLabel("can_right: checking")
        self.server_label = QLabel("Policy server: checking")
        self.observation_label = QLabel("Observation: waiting")
        self.command_label = QLabel("Command path: monitor-only")
        self.subsystem_label = QLabel("Subsystems: checking")
        for label in (
            self.can_left_label,
            self.can_right_label,
            self.server_label,
            self.observation_label,
            self.command_label,
            self.subsystem_label,
        ):
            label.setWordWrap(True)

        self.start_can_btn = QPushButton("Start CAN — 1 Mbit/s")
        self.start_cameras_btn = QPushButton("Start cameras")
        self.start_arms_btn = QPushButton("Start arms — observe only")
        self.stop_managed_btn = QPushButton("Stop managed subsystems")
        self.query_server_btn = QPushButton("Query policy metadata")
        self.snapshot_btn = QPushButton("Save observation snapshot")

        grid = QGridLayout()
        grid.addWidget(self.start_can_btn, 0, 0)
        grid.addWidget(self.start_cameras_btn, 0, 1)
        grid.addWidget(self.start_arms_btn, 1, 0)
        grid.addWidget(self.stop_managed_btn, 1, 1)
        grid.addWidget(self.query_server_btn, 2, 0)
        grid.addWidget(self.snapshot_btn, 2, 1)

        group = QGroupBox("1. System")
        layout = QVBoxLayout(group)
        layout.addWidget(self.domain_label)
        layout.addWidget(self.subsystem_label)
        layout.addWidget(self.can_left_label)
        layout.addWidget(self.can_right_label)
        layout.addWidget(self.server_label)
        layout.addWidget(self.observation_label)
        layout.addWidget(self.command_label)
        layout.addLayout(grid)
        return group

    def build_rollout_group(self) -> QGroupBox:
        self.prompt_edit = QLineEdit("Fold the shirt.")
        self.host_edit = QLineEdit("127.0.0.1")
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(8000)

        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(1.0, 60.0)
        self.rate_spin.setDecimals(1)
        self.rate_spin.setValue(30.0)
        self.rate_spin.setSuffix(" Hz")

        self.steps_spin = QSpinBox()
        self.steps_spin.setRange(1, 50)
        self.steps_spin.setValue(50)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(5.0, 300.0)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setValue(60.0)
        self.duration_spin.setSuffix(" s")

        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(30)
        self.speed_spin.setSuffix(" %")

        self.joint_step_spin = QDoubleSpinBox()
        self.joint_step_spin.setRange(0.005, 0.20)
        self.joint_step_spin.setDecimals(4)
        self.joint_step_spin.setSingleStep(0.005)
        self.joint_step_spin.setValue(0.08)
        self.joint_step_spin.setSuffix(" rad")

        self.grip_step_spin = QDoubleSpinBox()
        self.grip_step_spin.setRange(0.001, 0.04)
        self.grip_step_spin.setDecimals(4)
        self.grip_step_spin.setSingleStep(0.001)
        self.grip_step_spin.setValue(0.015)
        self.grip_step_spin.setSuffix(" m")

        self.hard_jump_spin = QDoubleSpinBox()
        self.hard_jump_spin.setRange(0.05, 1.0)
        self.hard_jump_spin.setDecimals(3)
        self.hard_jump_spin.setValue(0.35)
        self.hard_jump_spin.setSuffix(" rad")

        self.auto_enable_check = QCheckBox("Enable arms automatically at rollout start")
        self.auto_enable_check.setChecked(True)

        form = QFormLayout()
        form.addRow("Instruction", self.prompt_edit)
        form.addRow("Policy host", self.host_edit)
        form.addRow("Policy port", self.port_spin)
        form.addRow("Publish rate", self.rate_spin)
        form.addRow("Actions / replan", self.steps_spin)
        form.addRow("Task backstop", self.duration_spin)
        form.addRow("Piper speed", self.speed_spin)
        form.addRow("Joint slew cap", self.joint_step_spin)
        form.addRow("Gripper slew cap", self.grip_step_spin)
        form.addRow("Hard raw-jump abort", self.hard_jump_spin)
        form.addRow(self.auto_enable_check)

        self.dry_btn = QPushButton("Dry inference — no motion")
        self.start_btn = QPushButton("START FULL TASK")
        self.start_btn.setObjectName("primaryButton")
        self.stop_btn = QPushButton("STOP / HOLD")
        self.stop_btn.setObjectName("dangerButton")
        self.stop_btn.setEnabled(False)
        self.rollout_badge = QLabel("Idle")
        self.rollout_badge.setAlignment(Qt.AlignCenter)
        self.rollout_badge.setProperty("health", "warn")

        buttons = QGridLayout()
        buttons.addWidget(self.dry_btn, 0, 0)
        buttons.addWidget(self.start_btn, 0, 1)
        buttons.addWidget(self.stop_btn, 1, 0)
        buttons.addWidget(self.rollout_badge, 1, 1)

        group = QGroupBox("2. Inference / motion")
        layout = QVBoxLayout(group)
        layout.addLayout(form)
        layout.addLayout(buttons)
        return group

    def build_metrics_group(self) -> QGroupBox:
        self.metric_labels = {}
        fields = (
            ("cycle", "Replan"),
            ("elapsed", "Elapsed"),
            ("inference_ms", "Inference"),
            ("raw_max_joint_step", "Raw joint step"),
            ("published_max_joint_step", "Published joint step"),
            ("raw_max_gripper_step", "Raw gripper step"),
            ("published_max_gripper_step", "Published gripper step"),
            ("max_publish_lag_ms", "Max publish lag"),
        )
        grid = QGridLayout()
        for index, (key, title) in enumerate(fields):
            title_label = QLabel(title)
            value_label = QLabel("—")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.metric_labels[key] = value_label
            row = index // 4
            col = (index % 4) * 2
            grid.addWidget(title_label, row, col)
            grid.addWidget(value_label, row, col + 1)
        group = QGroupBox("Rollout metrics")
        group.setLayout(grid)
        return group

    def build_action_table_group(self) -> QGroupBox:
        self.action_table = QTableWidget(14, 6)
        self.action_table.setHorizontalHeaderLabels(
            ["Dimension", "Current", "Raw first", "Published first", "Raw Δ", "Published Δ"]
        )
        for row, name in enumerate(DIMENSION_NAMES):
            self.action_table.setItem(row, 0, QTableWidgetItem(name))
        header = self.action_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for col in range(1, 6):
            header.setSectionResizeMode(col, QHeaderView.Stretch)
        self.action_table.verticalHeader().setVisible(False)
        group = QGroupBox("Current state vs first policy target")
        layout = QVBoxLayout(group)
        layout.addWidget(self.action_table)
        return group

    def build_log_group(self) -> QGroupBox:
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setPlaceholderText("Inference and safety events appear here")
        self.clear_log_btn = QPushButton("Clear view")
        self.copy_diag_btn = QPushButton("Copy diagnostics")
        buttons = QHBoxLayout()
        buttons.addWidget(self.clear_log_btn)
        buttons.addWidget(self.copy_diag_btn)
        buttons.addStretch(1)
        group = QGroupBox("Debug log")
        layout = QVBoxLayout(group)
        layout.addWidget(self.log_edit)
        layout.addLayout(buttons)
        return group

    def connect_signals(self) -> None:
        self.start_can_btn.clicked.connect(self.start_can)
        self.start_cameras_btn.clicked.connect(lambda: self.start_subsystem("cameras"))
        self.start_arms_btn.clicked.connect(self.start_arms)
        self.stop_managed_btn.clicked.connect(self.stop_managed_subsystems)
        self.query_server_btn.clicked.connect(self.query_server_metadata)
        self.snapshot_btn.clicked.connect(self.save_snapshot)
        self.dry_btn.clicked.connect(lambda: self.start_runner(dry_run=True))
        self.start_btn.clicked.connect(lambda: self.start_runner(dry_run=False))
        self.stop_btn.clicked.connect(self.stop_rollout)
        self.clear_log_btn.clicked.connect(self.log_edit.clear)
        self.copy_diag_btn.clicked.connect(self.copy_diagnostics)

        self.bridge.log.connect(self.append_log)
        self.bridge.rollout_state.connect(self.on_rollout_state)
        self.bridge.rollout_metrics.connect(self.on_metrics)
        self.bridge.action_debug.connect(self.on_action_debug)
        self.bridge.worker_error.connect(self.on_worker_error)
        self.bridge.server_metadata.connect(self.on_server_metadata)
        self.bridge.snapshot_saved.connect(
            lambda path: self.statusBar().showMessage(f"Snapshot saved: {path}", 7000)
        )

    def append_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_edit.append(f"[{stamp}] {text}")

    def rollout_config(self) -> RolloutConfig:
        return RolloutConfig(
            prompt=self.prompt_edit.text().strip() or "Fold the shirt.",
            host=self.host_edit.text().strip() or "127.0.0.1",
            port=int(self.port_spin.value()),
            rate_hz=float(self.rate_spin.value()),
            exec_steps=int(self.steps_spin.value()),
            max_seconds=float(self.duration_spin.value()),
            max_joint_step=float(self.joint_step_spin.value()),
            max_gripper_step=float(self.grip_step_spin.value()),
            hard_raw_joint_jump=float(self.hard_jump_spin.value()),
            command_speed=int(self.speed_spin.value()),
            auto_enable=bool(self.auto_enable_check.isChecked()),
        )

    def start_runner(self, dry_run: bool) -> None:
        if self.rollout_thread is not None and self.rollout_thread.is_alive():
            return
        cfg = self.rollout_config()
        if not tcp_server_up(cfg.host, cfg.port, timeout=0.25):
            QMessageBox.critical(
                self,
                "Policy server unavailable",
                f"Nothing is listening at {cfg.host}:{cfg.port}.",
            )
            return
        if not self.node.observation_ready(cfg.observation_age_limit):
            QMessageBox.critical(
                self,
                "Observation not ready",
                "All three cameras and both joint streams must be fresh.\n\n"
                + repr(self.node.age_report()),
            )
            return
        if not dry_run:
            answer = QMessageBox.question(
                self,
                "Start full physical rollout",
                "This will publish policy actions to BOTH Piper arms and may enable them.\n\n"
                f"Replan every {cfg.exec_steps} actions at {cfg.rate_hz:.1f} Hz.\n"
                f"Joint slew cap: {cfg.max_joint_step:.4f} rad.\n"
                f"Backstop: {cfg.max_seconds:.1f} s.\n\n"
                "Confirm the workspace is clear and the hardware E-stop is accessible.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return

        self.rollout_stop = threading.Event()
        runner = RolloutRunner(self.node, self.bridge, cfg, self.rollout_stop, dry_run)
        self.rollout_thread = threading.Thread(target=runner.run, daemon=True)
        self.set_controls_running(True, dry_run)
        self.rollout_thread.start()

    def stop_rollout(self) -> None:
        if self.rollout_thread is None or not self.rollout_thread.is_alive():
            return
        self.rollout_stop.set()
        self.stop_btn.setEnabled(False)
        self.append_log("STOP requested — worker will hold at the latest measured pose")
        self.statusBar().showMessage("Stop requested")

    def set_controls_running(self, running: bool, dry_run: bool = False) -> None:
        self.start_btn.setEnabled(not running)
        self.dry_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running and not dry_run)
        for widget in (
            self.start_can_btn,
            self.start_cameras_btn,
            self.start_arms_btn,
            self.stop_managed_btn,
        ):
            widget.setEnabled(not running)

    def on_rollout_state(self, state: str, detail: str) -> None:
        status = "pass" if state == "running" else "warn"
        if state in {"checking", "enabling", "holding", "dry"}:
            status = "warn"
        if state == "idle":
            status = "warn"
            self.set_controls_running(False)
        health(self.rollout_badge, detail, status)
        self.statusBar().showMessage(detail)
        self.append_log(detail)

    def on_metrics(self, metrics: dict[str, object]) -> None:
        self.last_metrics = dict(metrics)
        formatting = {
            "cycle": lambda x: str(int(x)),
            "elapsed": lambda x: f"{float(x):.1f} s",
            "inference_ms": lambda x: f"{float(x):.1f} ms",
            "raw_max_joint_step": lambda x: f"{float(x):.5f} rad",
            "published_max_joint_step": lambda x: f"{float(x):.5f} rad",
            "raw_max_gripper_step": lambda x: f"{float(x):.5f} m",
            "published_max_gripper_step": lambda x: f"{float(x):.5f} m",
            "max_publish_lag_ms": lambda x: f"{float(x):.2f} ms",
        }
        for key, label in self.metric_labels.items():
            if key in metrics:
                label.setText(formatting[key](metrics[key]))

    def on_action_debug(self, debug: dict[str, np.ndarray]) -> None:
        self.last_action_debug = debug
        current = np.asarray(debug["current"], dtype=float)
        raw = np.asarray(debug["raw_first"], dtype=float)
        published = np.asarray(debug["published_first"], dtype=float)
        for row in range(14):
            values = (
                current[row],
                raw[row],
                published[row],
                raw[row] - current[row],
                published[row] - current[row],
            )
            for col, value in enumerate(values, start=1):
                self.action_table.setItem(row, col, QTableWidgetItem(f"{value:+.6f}"))

    def on_worker_error(self, title: str, detail: str) -> None:
        self.append_log(f"{title}: {detail}")
        QMessageBox.critical(self, title, detail)

    def refresh_monitor(self) -> None:
        snap = self.node.monitor_snapshot()
        now = time.monotonic()
        for key, _title, _topic in CAMERAS:
            values = snap["cameras"][key]
            age = float(values["age"])
            stream_rate = float(values["rate"])
            if age < 1.0:
                health(self.camera_panes[key].health, f"{stream_rate:.1f} Hz · age {age:.3f}s", "pass")
            elif age < 3.0:
                health(self.camera_panes[key].health, f"{stream_rate:.1f} Hz · age {age:.2f}s", "warn")
            else:
                health(self.camera_panes[key].health, "No signal", "fail")
            if now - self.last_preview[key] >= 1.0 / PREVIEW_MAX_FPS:
                self.last_preview[key] = now
                image = self.node.preview_qimage(key)
                if image is not None:
                    self.camera_panes[key].preview.set_image(image)

        for side in ("left", "right"):
            values = snap["joints"][side]
            age = float(values["age"])
            stream_rate = float(values["rate"])
            state = values["state"]
            if age < 0.5:
                health(self.arm_panes[side].health, f"{stream_rate:.1f} Hz · age {age:.3f}s", "pass")
            elif age < 1.0:
                health(self.arm_panes[side].health, f"{stream_rate:.1f} Hz · age {age:.3f}s", "warn")
            else:
                health(self.arm_panes[side].health, "No fresh signal", "fail")
            if state is None:
                self.arm_panes[side].values.setText("Waiting for joint feedback")
            else:
                rendered = "  ".join(
                    [f"j{i}: {float(state[i]):+.4f}" for i in range(6)]
                    + [f"grip: {float(state[6]):+.4f} m"]
                )
                self.arm_panes[side].values.setText(rendered)

        cfg = self.rollout_config()
        ready = self.node.observation_ready(cfg.observation_age_limit)
        health(
            self.observation_label,
            "Observation: READY" if ready else "Observation: stale / incomplete",
            "pass" if ready else "fail",
        )
        if self.node.command_publishers:
            counts = self.node.command_subscription_counts()
            ok = counts.get("left", 0) >= 1 and counts.get("right", 0) >= 1
            health(
                self.command_label,
                f"Command path: ARMED · subscribers {counts}",
                "pass" if ok else "fail",
            )
        else:
            health(self.command_label, "Command path: monitor-only / no publishers", "pass")

    def refresh_system_health(self) -> None:
        nodes = self.node.ros_nodes()
        cams = CAMERA_NODES & nodes
        arms = ARM_NODES & nodes
        health(
            self.subsystem_label,
            f"Subsystems: cameras {len(cams)}/3 · arms {len(arms)}/2",
            "pass" if len(cams) == 3 and len(arms) == 2 else "warn",
        )
        self.start_cameras_btn.setEnabled(len(cams) == 0 and not self.runner_active())
        self.start_arms_btn.setEnabled(len(arms) == 0 and not self.runner_active())
        self.start_can_btn.setEnabled(len(arms) == 0 and not self.runner_active())
        self.stop_managed_btn.setEnabled(bool(self.managed) and not self.runner_active())

        for iface, label in (("can_left", self.can_left_label), ("can_right", self.can_right_label)):
            status, detail = can_status(iface)
            health(label, f"{iface}: {detail}", status)

        cfg = self.rollout_config()
        up = tcp_server_up(cfg.host, cfg.port)
        health(
            self.server_label,
            f"Policy server: {'UP' if up else 'DOWN'} · {cfg.host}:{cfg.port}",
            "pass" if up else "fail",
        )

        for name, managed in list(self.managed.items()):
            if managed.process.poll() is not None:
                try:
                    managed.log_handle.close()
                except Exception:
                    pass
                del self.managed[name]

    def runner_active(self) -> bool:
        return self.rollout_thread is not None and self.rollout_thread.is_alive()

    def start_can(self) -> None:
        if self.runner_active():
            return
        if ARM_NODES & self.node.ros_nodes():
            QMessageBox.warning(self, "Arms are running", "Stop arm nodes before reconfiguring CAN.")
            return

        def target() -> None:
            try:
                result = configure_can_interfaces()
            except BaseException as exc:
                self.bridge.worker_error.emit("CAN setup failed", str(exc))
            else:
                self.bridge.log.emit(result)
                QTimer.singleShot(0, self.refresh_system_health)

        threading.Thread(target=target, daemon=True).start()

    def start_arms(self) -> None:
        answer = QMessageBox.question(
            self,
            "Observation-arm startup",
            "Confirm both arms are in the required reset pose, the labeled CAN mapping is unchanged, "
            "and the hardware E-stop is accessible.\n\n"
            "The observation launch itself does not publish commands; this inference GUI creates command "
            "publishers only when a physical rollout starts.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.start_subsystem("arms")

    def start_subsystem(self, name: str) -> None:
        expected = CAMERA_NODES if name == "cameras" else ARM_NODES
        launch_file = CAMERA_LAUNCH if name == "cameras" else ARM_LAUNCH
        present = expected & self.node.ros_nodes()
        if present:
            QMessageBox.warning(
                self,
                "Nodes already present",
                "Refusing duplicate launch:\n\n" + "\n".join(sorted(present)),
            )
            return
        if not launch_file.is_file():
            QMessageBox.critical(self, "Launch file missing", str(launch_file))
            return
        log_path = RUNTIME_ROOT / f"{name}-{datetime.now():%Y%m%d-%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        try:
            process = subprocess.Popen(
                ["ros2", "launch", str(launch_file)],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except Exception:
            log_handle.close()
            raise
        self.managed[name] = ManagedProcess(name, process, log_handle, log_path)
        self.append_log(f"Started {name}; log={log_path}")
        self.refresh_system_health()

    def stop_managed_subsystems(self) -> None:
        for managed in list(self.managed.values()):
            self.stop_process_sync(managed)
            try:
                managed.log_handle.close()
            except Exception:
                pass
        self.managed.clear()
        self.append_log("Stopped managed camera/arm subprocesses")
        self.refresh_system_health()

    @staticmethod
    def stop_process_sync(managed: ManagedProcess) -> None:
        process = managed.process
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
            return
        except Exception:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
            return
        except Exception:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except Exception:
            pass

    def query_server_metadata(self) -> None:
        cfg = self.rollout_config()

        def target() -> None:
            try:
                if not tcp_server_up(cfg.host, cfg.port, timeout=0.5):
                    raise RuntimeError(f"Server down at {cfg.host}:{cfg.port}")
                client = websocket_client_policy.WebsocketClientPolicy(host=cfg.host, port=cfg.port)
                self.bridge.server_metadata.emit(client.get_server_metadata())
            except BaseException as exc:
                self.bridge.worker_error.emit("Policy metadata failed", str(exc))

        threading.Thread(target=target, daemon=True).start()

    def on_server_metadata(self, metadata: object) -> None:
        self.append_log("Policy metadata: " + json.dumps(metadata, sort_keys=True))

    def save_snapshot(self) -> None:
        prompt = self.prompt_edit.text().strip() or "Fold the shirt."

        def target() -> None:
            try:
                root = self.node.save_snapshot(prompt)
            except BaseException as exc:
                self.bridge.worker_error.emit("Snapshot failed", str(exc))
            else:
                self.bridge.snapshot_saved.emit(str(root))
                self.bridge.log.emit(f"Snapshot saved: {root}")

        threading.Thread(target=target, daemon=True).start()

    def copy_diagnostics(self) -> None:
        snap = self.node.monitor_snapshot()
        data = {
            "time": datetime.now().astimezone().isoformat(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "88"),
            "ages": self.node.age_report(),
            "camera_rates_hz": {k: float(v["rate"]) for k, v in snap["cameras"].items()},
            "joint_rates_hz": {k: float(v["rate"]) for k, v in snap["joints"].items()},
            "command_subscribers": self.node.command_subscription_counts()
            if self.node.command_publishers
            else {},
            "last_metrics": self.last_metrics,
        }
        QApplication.clipboard().setText(json.dumps(data, indent=2))
        self.statusBar().showMessage("Diagnostics copied to clipboard", 5000)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.runner_active():
            answer = QMessageBox.question(
                self,
                "Rollout still active",
                "Request STOP/HOLD and close the GUI?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            self.rollout_stop.set()
            deadline = time.monotonic() + 3.0
            while self.runner_active() and time.monotonic() < deadline:
                QApplication.processEvents()
                time.sleep(0.03)

        try:
            self.node.hold_current()
        except Exception:
            pass
        try:
            self.node.release_command_interfaces()
        except Exception:
            pass
        self.stop_managed_subsystems()
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("mainSplitter", self.main_splitter.saveState())
        self.settings.setValue("lowerSplitter", self.lower_splitter.saveState())
        self.settings.setValue("cameraSplitter", self.camera_splitter.saveState())
        event.accept()


def select_font_family() -> str:
    families = set(QFontDatabase().families())
    for candidate in ("Roboto Mono", "RobotoMono", "DejaVu Sans Mono", "Monospace"):
        if candidate in families:
            return candidate
    return "Monospace"


def apply_application_style(app: QApplication) -> None:
    app.setFont(QFont(select_font_family(), 10, QFont.Bold))
    app.setStyleSheet(
        """
        QWidget { font-weight: 700; }
        QGroupBox { margin-top: 0.8em; padding-top: 0.8em; }
        QGroupBox::title { subcontrol-origin: margin; left: 0.7em; padding: 0 0.3em; }
        QLabel[sectionHeading="true"] { font-weight: 700; }
        QLabel[health="pass"] { color: #24704a; }
        QLabel[health="warn"] { color: #9a6710; }
        QLabel[health="fail"] { color: #b23a3a; }
        QPushButton { padding: 0.45em 0.7em; }
        QPushButton#primaryButton { background: #3f7f62; color: white; }
        QPushButton#warningButton { background: #b87824; color: white; }
        QPushButton#dangerButton { background: #a84444; color: white; }
        QPushButton:disabled { color: #8c8c8c; background: #dddddd; }
        QTableWidget { gridline-color: #c7c7c7; }
        """
    )


def main() -> int:
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RobotNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    apply_application_style(app)
    bridge = Bridge()
    window = Window(node, bridge)
    window.show()

    try:
        return int(app.exec_())
    finally:
        try:
            node.hold_current()
        except Exception:
            pass
        try:
            node.release_command_interfaces()
        except Exception:
            pass
        try:
            executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        # SIGINT can already have shut the context down; avoid the RCLError seen
        # in the original terminal rollout by checking before calling shutdown.
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
