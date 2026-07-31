#!/usr/bin/env python3
"""Responsive synchronized data collection, review, and playback GUI."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, JointState

from PyQt5.QtCore import QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QFontDatabase, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
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
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from episode_store import (
    EpisodePaths,
    create_episode,
    git_revision,
    list_episodes,
    move_episode_to_trash,
    paths_for_episode,
    read_json,
    update_episode,
    utc_now_iso,
)
from preflight import ARM_NODES, CAMERA_NODES, REQUIRED_TOPICS, Report, run_preflight
from validation import validate_episode

APP_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = APP_DIR.parent
OUTPUT_ROOT = Path(
    os.environ.get("LAUNDRY_BUTLER_DATA_ROOT", str(APP_DIR / "output"))
).expanduser()
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
MIN_FREE_GB = float(os.environ.get("LAUNDRY_BUTLER_MIN_FREE_GB", "20"))

VIEWER_PREFIX = "/laundry_butler/viewer"
READINESS_VALID_SECONDS = 120.0
PREVIEW_MAX_FPS = 8.0
RATE_WINDOW_SECONDS = 2.5

CAMERAS = (
    ("left", "Left wrist", "/camera_l/color/image_raw", "/camera_l/color/camera_info"),
    ("front", "Front / top", "/camera_f/color/image_raw", "/camera_f/color/camera_info"),
    ("right", "Right wrist", "/camera_r/color/image_raw", "/camera_r/color/camera_info"),
)

ARM_SPECS = (
    ("left", "joints", "/puppet/joint_left"),
    ("left", "pose", "/puppet/end_pose_left"),
    ("left", "status", "/piper_left_ctrl_node/arm_status"),
    ("right", "joints", "/puppet/joint_right"),
    ("right", "pose", "/puppet/end_pose_right"),
    ("right", "status", "/piper_right_ctrl_node/arm_status"),
)

OUTCOME_OPTIONS = (
    ("Not assessed", "not_assessed"),
    ("Success", "success"),
    ("Partial", "partial"),
    ("Failure", "failure"),
)
DISPOSITION_OPTIONS = (
    ("Usable", "usable"),
    ("Needs review", "needs_review"),
    ("Unusable", "unusable"),
)


class Bridge(QObject):
    frame = pyqtSignal(str, str, object)
    worker_done = pyqtSignal(str, object)
    worker_failed = pyqtSignal(str, str)


class MonitorNode(Node):
    """Collect reduced-rate previews and compact arm observations."""

    def __init__(self, bridge: Bridge) -> None:
        super().__init__("laundry_butler_data_collection_gui")
        self.bridge = bridge
        self.lock = threading.Lock()
        self.last_preview: dict[tuple[str, str], float] = {}
        self.camera_arrivals: dict[tuple[str, str], deque[float]] = {}
        self.camera_last: dict[tuple[str, str], float] = {}
        self.arm_arrivals: dict[tuple[str, str, str], deque[float]] = {}
        self.arm_last: dict[tuple[str, str, str], float] = {}
        self.arm_text: dict[tuple[str, str, str], str] = {}
        self.dynamic_subscriptions: dict[str, object] = {}
        self._subscriptions = []

        image_group = ReentrantCallbackGroup()
        info_group = ReentrantCallbackGroup()
        arm_group = ReentrantCallbackGroup()

        for source in ("live", "viewer"):
            for key, _title, image_topic, info_topic in CAMERAS:
                actual_image = self.source_topic(source, image_topic)
                actual_info = self.source_topic(source, info_topic)
                channel = (source, key)
                self.last_preview[channel] = 0.0
                self.camera_arrivals[channel] = deque()
                self.camera_last[channel] = 0.0
                self._subscriptions.append(
                    self.create_subscription(
                        Image,
                        actual_image,
                        self._image_callback(source, key),
                        qos_profile_sensor_data,
                        callback_group=image_group,
                    )
                )
                self._subscriptions.append(
                    self.create_subscription(
                        CameraInfo,
                        actual_info,
                        self._camera_info_callback(source, key),
                        qos_profile_sensor_data,
                        callback_group=info_group,
                    )
                )

            for side, kind, topic in ARM_SPECS:
                if kind != "joints":
                    continue
                actual_topic = self.source_topic(source, topic)
                channel = (source, side, kind)
                self.arm_arrivals[channel] = deque()
                self.arm_last[channel] = 0.0
                self.arm_text[channel] = "Waiting for data"
                self._subscriptions.append(
                    self.create_subscription(
                        JointState,
                        actual_topic,
                        self._joint_callback(source, side),
                        qos_profile_sensor_data,
                        callback_group=arm_group,
                    )
                )

        self.dynamic_timer = self.create_timer(1.0, self.ensure_dynamic_subscriptions)

    @staticmethod
    def source_topic(source: str, topic: str) -> str:
        return topic if source == "live" else f"{VIEWER_PREFIX}{topic}"

    def _image_callback(self, source: str, key: str) -> Callable[[Image], None]:
        def callback(message: Image) -> None:
            now = time.monotonic()
            channel = (source, key)
            with self.lock:
                arrivals = self.camera_arrivals[channel]
                arrivals.append(now)
                self.camera_last[channel] = now
                trim(arrivals, now)
                if now - self.last_preview[channel] < 1.0 / PREVIEW_MAX_FPS:
                    return
                self.last_preview[channel] = now
            try:
                image = to_qimage(message)
            except ValueError:
                return
            self.bridge.frame.emit(source, key, image)

        return callback

    def _camera_info_callback(self, source: str, key: str) -> Callable[[CameraInfo], None]:
        def callback(_message: CameraInfo) -> None:
            # Keep a subscriber on camera_info for graph visibility, but calculate
            # the displayed stream rate from image messages only.
            _ = source, key

        return callback

    def _joint_callback(self, source: str, side: str) -> Callable[[JointState], None]:
        def callback(message: JointState) -> None:
            names = [short_joint_name(name) for name in message.name]
            positions = [round(float(value), 4) for value in message.position]
            if names and len(names) == len(positions):
                text = "  ".join(f"{name}: {value:+.4f}" for name, value in zip(names, positions))
            else:
                text = "Position: " + format_numeric_list(positions)
            self.store_arm_sample(source, side, "joints", text)

        return callback

    def ensure_dynamic_subscriptions(self) -> None:
        try:
            topic_types = dict(self.get_topic_names_and_types())
        except Exception:
            return

        for source in ("live", "viewer"):
            for side, kind, topic in ARM_SPECS:
                if kind == "joints":
                    continue
                actual_topic = self.source_topic(source, topic)
                if actual_topic in self.dynamic_subscriptions:
                    continue
                types = topic_types.get(actual_topic, [])
                if not types:
                    continue
                try:
                    message_type = get_message(types[0])
                    subscription = self.create_subscription(
                        message_type,
                        actual_topic,
                        self._generic_arm_callback(source, side, kind),
                        qos_profile_sensor_data,
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not subscribe to {actual_topic}: {exc}"
                    )
                    continue
                channel = (source, side, kind)
                with self.lock:
                    self.arm_arrivals.setdefault(channel, deque())
                    self.arm_last.setdefault(channel, 0.0)
                    self.arm_text.setdefault(channel, "Waiting for data")
                self.dynamic_subscriptions[actual_topic] = subscription

    def _generic_arm_callback(self, source: str, side: str, kind: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                payload = message_to_ordereddict(message)
                text = summarize_mapping(payload, kind)
            except Exception:
                text = str(message).replace("\n", " ")[:500]
            self.store_arm_sample(source, side, kind, text)

        return callback

    def store_arm_sample(self, source: str, side: str, kind: str, text: str) -> None:
        now = time.monotonic()
        channel = (source, side, kind)
        with self.lock:
            arrivals = self.arm_arrivals.setdefault(channel, deque())
            arrivals.append(now)
            self.arm_last[channel] = now
            self.arm_text[channel] = text
            trim(arrivals, now)

    def clear_source(self, source: str) -> None:
        now = time.monotonic()
        with self.lock:
            for channel in list(self.camera_arrivals):
                if channel[0] != source:
                    continue
                self.camera_arrivals[channel].clear()
                self.camera_last[channel] = 0.0
                self.last_preview[channel] = 0.0
            for channel in list(self.arm_arrivals):
                if channel[0] != source:
                    continue
                self.arm_arrivals[channel].clear()
                self.arm_last[channel] = 0.0
                self.arm_text[channel] = "Waiting for data"

    def snapshot(self) -> dict[str, object]:
        now = time.monotonic()
        cameras: dict[str, dict[str, dict[str, float]]] = {"live": {}, "viewer": {}}
        arms: dict[str, dict[str, dict[str, dict[str, object]]]] = {
            "live": {"left": {}, "right": {}},
            "viewer": {"left": {}, "right": {}},
        }
        with self.lock:
            for (source, key), arrivals in self.camera_arrivals.items():
                trim(arrivals, now)
                cameras[source][key] = {
                    "rate": rate(arrivals),
                    "age": now - self.camera_last[(source, key)]
                    if self.camera_last[(source, key)] > 0.0
                    else float("inf"),
                }
            for (source, side, kind), arrivals in self.arm_arrivals.items():
                trim(arrivals, now)
                arms[source][side][kind] = {
                    "rate": rate(arrivals),
                    "age": now - self.arm_last[(source, side, kind)]
                    if self.arm_last[(source, side, kind)] > 0.0
                    else float("inf"),
                    "text": self.arm_text.get((source, side, kind), "Waiting for data"),
                }
        return {"cameras": cameras, "arms": arms}


def trim(arrivals: deque[float], now: float) -> None:
    cutoff = now - RATE_WINDOW_SECONDS
    while arrivals and arrivals[0] < cutoff:
        arrivals.popleft()


def rate(arrivals: deque[float]) -> float:
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        return 0.0
    return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])


def short_joint_name(name: str) -> str:
    value = name.rsplit("/", 1)[-1]
    value = value.replace("joint", "j").replace("_", "")
    return value or "j"


def format_numeric_list(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:+.4f}" for value in values) + "]"


def flatten_mapping(value: Any, prefix: str = "") -> list[tuple[str, Any]]:
    result: list[tuple[str, Any]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            next_prefix = f"{prefix}.{key}" if prefix else str(key)
            result.extend(flatten_mapping(item, next_prefix))
    elif isinstance(value, (list, tuple)) and len(value) <= 12:
        result.append((prefix, value))
    else:
        result.append((prefix, value))
    return result


def summarize_mapping(payload: Any, kind: str) -> str:
    flattened = flatten_mapping(payload)
    preferred_tokens = (
        ("position", "orientation", "x", "y", "z", "roll", "pitch", "yaw")
        if kind == "pose"
        else ("ctrl", "mode", "status", "error", "code", "enabled", "feedback")
    )
    selected = [
        (key, value)
        for key, value in flattened
        if any(token in key.lower() for token in preferred_tokens)
    ]
    if not selected:
        selected = flattened
    lines = []
    for key, value in selected[:12]:
        if isinstance(value, float):
            rendered = f"{value:.5f}"
        elif isinstance(value, (list, tuple)):
            rendered = json.dumps(value, separators=(",", ":"))
        else:
            rendered = str(value)
        lines.append(f"{key}: {rendered}")
    text = " | ".join(lines)
    return text[:520] if text else "Sample received"


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


class PreviewLabel(QLabel):
    def __init__(self) -> None:
        super().__init__("Waiting for camera")
        self.source: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(170, 115)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#111;color:#aaa;border:1px solid #444")

    def set_image(self, image: QImage) -> None:
        self.source = QPixmap.fromImage(image)
        self._rescale()

    def clear_image(self, message: str) -> None:
        self.source = None
        self.clear()
        self.setText(message)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self) -> None:
        if self.source is None:
            return
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
        self.rate_label = QLabel("No signal")
        self.rate_label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(heading)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.rate_label)

    def update_health(self, stream_rate: float, age: float, source: str) -> None:
        if age > 3.0:
            self.rate_label.setText(f"{source.title()}: no signal")
            self.rate_label.setProperty("health", "fail")
        else:
            self.rate_label.setText(f"{source.title()}: {stream_rate:.1f} Hz")
            self.rate_label.setProperty("health", "pass")
        refresh_style(self.rate_label)


class ArmPane(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        heading = QLabel(title)
        heading.setProperty("sectionHeading", True)
        self.health = QLabel("No signal")
        self.joints = QLabel("Joints: waiting for data")
        self.pose = QLabel("Pose: waiting for data")
        self.status = QLabel("Status: waiting for data")
        for label in (self.joints, self.pose, self.status):
            label.setWordWrap(True)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        header = QHBoxLayout()
        header.addWidget(heading)
        header.addStretch(1)
        header.addWidget(self.health)
        layout.addLayout(header)
        layout.addWidget(self.joints)
        layout.addWidget(self.pose)
        layout.addWidget(self.status)

    def update_observation(self, values: dict[str, dict[str, object]], source: str) -> None:
        joint = values.get("joints", {})
        pose = values.get("pose", {})
        status = values.get("status", {})
        ages = [float(item.get("age", float("inf"))) for item in (joint, pose, status) if item]
        best_age = min(ages) if ages else float("inf")
        if best_age > 3.0:
            self.health.setText(f"{source.title()}: no signal")
            self.health.setProperty("health", "fail")
        else:
            joint_rate = float(joint.get("rate", 0.0)) if joint else 0.0
            self.health.setText(f"{source.title()}: {joint_rate:.1f} Hz")
            self.health.setProperty("health", "pass")
        refresh_style(self.health)
        self.joints.setText("Joints: " + str(joint.get("text", "Waiting for data")))
        self.pose.setText("Pose: " + str(pose.get("text", "Waiting for data")))
        self.status.setText("Status: " + str(status.get("text", "Waiting for data")))


def refresh_style(widget: QWidget) -> None:
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


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
    /usr/sbin/ip link set dev "$iface" up
    echo "Configured $iface"
done
'''

    try:
        result = subprocess.run(
            ["pkexec", "/bin/bash", "-c", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=45.0,
            check=False,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("pkexec is unavailable on this computer.") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("CAN configuration timed out.") from exc

    output = result.stdout.strip()
    if result.returncode != 0:
        raise RuntimeError(
            output or "Authorization was cancelled or CAN configuration failed."
        )

    verified = []
    for iface in ("can_left", "can_right"):
        details = subprocess.run(
            ["/usr/sbin/ip", "-details", "link", "show", "dev", iface],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5.0,
            check=False,
        )
        detail = details.stdout.strip()

        if details.returncode != 0:
            raise RuntimeError(f"Could not verify {iface}:\n{detail}")

        if "state ERROR-ACTIVE" not in detail:
            raise RuntimeError(f"{iface} is not ERROR-ACTIVE:\n{detail}")

        if "bitrate 1000000" not in detail:
            raise RuntimeError(f"{iface} is not at 1 Mbit/s:\n{detail}")

        verified.append(f"{iface}: UP, ERROR-ACTIVE, 1 Mbit/s")

    return "\n".join(verified)


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    log_handle: object
    log_path: Path
    stop_requested_at: float = 0.0


class Window(QMainWindow):
    def __init__(self, node: MonitorNode, bridge: Bridge) -> None:
        super().__init__()
        self.node = node
        self.bridge = bridge
        self.settings = QSettings("LaundryButler", "DataCollection")
        self.managed: dict[str, ManagedProcess] = {}
        self.recorder: Optional[ManagedProcess] = None
        self.playback: Optional[ManagedProcess] = None
        self.current_episode: Optional[EpisodePaths] = None
        self.current_playback_root: Optional[Path] = None
        self.recording_started_monotonic = 0.0
        self.playback_started_monotonic = 0.0
        self.playback_pause_started = 0.0
        self.playback_paused_total = 0.0
        self.playback_duration = 0.0
        self.playback_paused = False
        self.stop_disposition = "usable"
        self.last_preflight: Optional[Report] = None
        self.preflight_valid_until = 0.0
        self.worker_active = False
        self.pending_record_after_readiness = False
        self.active_source = "live"
        self.selected_episode_root: Optional[Path] = None
        self._last_font_size = 0

        self.setWindowTitle("Laundry Butler data collection")
        self.setMinimumSize(1100, 700)
        self.resize(1650, 950)
        saved_geometry = self.settings.value("geometry")
        if saved_geometry is not None:
            self.restoreGeometry(saved_geometry)
        self.setStatusBar(QStatusBar())

        self.camera_panes = {key: CameraPane(title) for key, title, *_ in CAMERAS}
        self.camera_splitter = QSplitter(Qt.Horizontal)
        for key, _title, *_ in CAMERAS:
            self.camera_splitter.addWidget(self.camera_panes[key])
        self.camera_splitter.setChildrenCollapsible(False)
        self.camera_splitter.setSizes([1, 1, 1])

        self.source_label = QLabel("Viewing live signals")
        self.source_label.setProperty("sourceBanner", True)
        camera_container = QWidget()
        camera_layout = QVBoxLayout(camera_container)
        camera_layout.setContentsMargins(8, 4, 8, 4)
        camera_layout.addWidget(self.source_label)
        camera_layout.addWidget(self.camera_splitter, 1)

        self.arm_panes = {
            "left": ArmPane("Left arm observation"),
            "right": ArmPane("Right arm observation"),
        }
        arm_splitter = QSplitter(Qt.Horizontal)
        arm_splitter.addWidget(self.arm_panes["left"])
        arm_splitter.addWidget(self.arm_panes["right"])
        arm_splitter.setChildrenCollapsible(False)
        arm_group = QGroupBox("Arm observation")
        arm_layout = QVBoxLayout(arm_group)
        arm_layout.addWidget(arm_splitter)

        setup_group = self.build_setup_group()
        episode_group = self.build_episode_metadata_group()
        record_group = self.build_record_group()

        workflow_content = QWidget()
        workflow_layout = QVBoxLayout(workflow_content)
        workflow_layout.setContentsMargins(4, 4, 4, 4)
        workflow_layout.addWidget(setup_group)
        workflow_layout.addWidget(episode_group)
        workflow_layout.addWidget(record_group)
        workflow_layout.addStretch(1)
        workflow_scroll = QScrollArea()
        workflow_scroll.setWidgetResizable(True)
        workflow_scroll.setFrameShape(QFrame.NoFrame)
        workflow_scroll.setWidget(workflow_content)

        viewer_group = self.build_episode_viewer_group()
        lower_splitter = QSplitter(Qt.Horizontal)
        lower_splitter.addWidget(workflow_scroll)
        lower_splitter.addWidget(viewer_group)
        lower_splitter.setChildrenCollapsible(False)
        lower_splitter.setSizes([560, 1050])
        self.lower_splitter = lower_splitter

        main_splitter = QSplitter(Qt.Vertical)
        main_splitter.addWidget(camera_container)
        main_splitter.addWidget(arm_group)
        main_splitter.addWidget(lower_splitter)
        main_splitter.setChildrenCollapsible(False)
        main_splitter.setSizes([410, 190, 350])
        self.main_splitter = main_splitter

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(4, 4, 4, 4)
        central_layout.addWidget(main_splitter)
        self.setCentralWidget(central)

        self.connect_signals()
        self.install_timers()

        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        self.refresh_episodes()
        QTimer.singleShot(100, self.refresh_subsystem_state)
        QTimer.singleShot(1200, lambda: self.run_preflight_async(automatic=True))
        QTimer.singleShot(150, self.restore_splitters)
        self.apply_scale()

    def build_setup_group(self) -> QGroupBox:
        self.systems_label = QLabel("Checking subsystems…")
        self.systems_label.setWordWrap(True)
        self.start_can_btn = QPushButton("Start CAN — 1 Mbit/s")
        self.start_cameras_btn = QPushButton("Start cameras")
        self.start_arms_btn = QPushButton("Start arms — observe only")
        self.stop_managed_btn = QPushButton("Stop managed subsystems")
        self.preflight_btn = QPushButton("Check readiness")
        self.readiness_badge = QLabel("Not checked")
        self.readiness_badge.setAlignment(Qt.AlignCenter)
        self.readiness_badge.setProperty("health", "warn")
        self.preflight_result = QTextEdit()
        self.preflight_result.setReadOnly(True)
        self.preflight_result.setMaximumHeight(145)
        self.preflight_result.setPlaceholderText("Readiness details appear here")

        subsystem_buttons = QGridLayout()
        subsystem_buttons.addWidget(self.start_can_btn, 0, 0)
        subsystem_buttons.addWidget(self.start_cameras_btn, 0, 1)
        subsystem_buttons.addWidget(self.start_arms_btn, 1, 0)
        subsystem_buttons.addWidget(self.stop_managed_btn, 1, 1)
        subsystem_buttons.addWidget(self.preflight_btn, 2, 0)
        subsystem_buttons.addWidget(self.readiness_badge, 2, 1)

        group = QGroupBox("1. Setup")
        layout = QVBoxLayout(group)
        layout.addWidget(self.systems_label)
        layout.addLayout(subsystem_buttons)
        layout.addWidget(self.preflight_result)
        return group

    def build_episode_metadata_group(self) -> QGroupBox:
        self.task_edit = QLineEdit("shirt folding")
        self.garment_edit = QLineEdit("t-shirt")
        self.initial_state_combo = QComboBox()
        self.initial_state_combo.addItem("Level 1 — spread shirt", "level_1_spread")
        self.initial_state_combo.addItem("Level 2 — messy shirt", "level_2_messy")
        self.initial_state_combo.addItem("Other", "other")
        self.instruction_edit = QLineEdit("Spread and fold the shirt into a compact final state.")
        self.notes_edit = QTextEdit()
        self.notes_edit.setPlaceholderText("Optional scene, garment, lighting, or demonstration notes")
        self.notes_edit.setMaximumHeight(80)

        form = QFormLayout()
        form.addRow("Task", self.task_edit)
        form.addRow("Garment type", self.garment_edit)
        form.addRow("Initial state", self.initial_state_combo)
        form.addRow("Instruction", self.instruction_edit)
        form.addRow("Notes", self.notes_edit)

        group = QGroupBox("2. Episode")
        group.setLayout(form)
        return group

    def build_record_group(self) -> QGroupBox:
        self.start_episode_btn = QPushButton("Start episode")
        self.start_episode_btn.setObjectName("primaryButton")
        self.start_episode_btn.setEnabled(False)
        self.stop_keep_btn = QPushButton("Stop and keep")
        self.stop_unusable_btn = QPushButton("Stop and mark unusable")
        self.stop_unusable_btn.setObjectName("warningButton")
        self.stop_keep_btn.setEnabled(False)
        self.stop_unusable_btn.setEnabled(False)
        self.recording_label = QLabel("Not recording")
        self.recording_label.setWordWrap(True)

        record_buttons = QHBoxLayout()
        record_buttons.addWidget(self.stop_keep_btn)
        record_buttons.addWidget(self.stop_unusable_btn)

        group = QGroupBox("3. Record")
        layout = QVBoxLayout(group)
        layout.addWidget(self.start_episode_btn)
        layout.addLayout(record_buttons)
        layout.addWidget(self.recording_label)
        return group

    def build_episode_viewer_group(self) -> QGroupBox:
        self.episodes_table = QTableWidget(0, 6)
        self.episodes_table.setHorizontalHeaderLabels(
            ["Created", "Episode", "Task", "Outcome", "Disposition", "Validation"]
        )
        header = self.episodes_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        self.episodes_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.episodes_table.setSelectionMode(QTableWidget.SingleSelection)
        self.episodes_table.setEditTriggers(QTableWidget.NoEditTriggers)

        self.selected_episode_label = QLabel("No episode selected")
        self.selected_episode_label.setWordWrap(True)
        self.outcome_combo = QComboBox()
        for label, value in OUTCOME_OPTIONS:
            self.outcome_combo.addItem(label, value)
        self.disposition_combo = QComboBox()
        for label, value in DISPOSITION_OPTIONS:
            self.disposition_combo.addItem(label, value)
        self.viewer_notes_edit = QTextEdit()
        self.viewer_notes_edit.setPlaceholderText("Episode review notes")
        self.viewer_notes_edit.setMaximumHeight(90)
        self.save_episode_btn = QPushButton("Save properties")
        self.open_folder_btn = QPushButton("Open folder")
        self.delete_episode_btn = QPushButton("Delete episode")
        self.delete_episode_btn.setObjectName("dangerButton")
        self.refresh_episodes_btn = QPushButton("Refresh")

        property_form = QFormLayout()
        property_form.addRow("Outcome", self.outcome_combo)
        property_form.addRow("Disposition", self.disposition_combo)
        property_form.addRow("Notes", self.viewer_notes_edit)
        property_buttons = QHBoxLayout()
        property_buttons.addWidget(self.save_episode_btn)
        property_buttons.addWidget(self.open_folder_btn)
        property_buttons.addWidget(self.delete_episode_btn)
        property_buttons.addStretch(1)
        property_buttons.addWidget(self.refresh_episodes_btn)
        property_box = QGroupBox("Selected episode")
        property_layout = QVBoxLayout(property_box)
        property_layout.addWidget(self.selected_episode_label)
        property_layout.addLayout(property_form)
        property_layout.addLayout(property_buttons)

        self.play_btn = QPushButton("Play episode")
        self.pause_btn = QPushButton("Pause")
        self.stop_playback_btn = QPushButton("Stop playback")
        self.playback_rate_combo = QComboBox()
        for label, value in (("0.5×", 0.5), ("1×", 1.0), ("2×", 2.0)):
            self.playback_rate_combo.addItem(label, value)
        self.playback_rate_combo.setCurrentIndex(1)
        self.playback_label = QLabel("No episode playing")
        playback_buttons = QHBoxLayout()
        playback_buttons.addWidget(self.play_btn)
        playback_buttons.addWidget(self.pause_btn)
        playback_buttons.addWidget(self.stop_playback_btn)
        playback_buttons.addWidget(QLabel("Rate"))
        playback_buttons.addWidget(self.playback_rate_combo)
        playback_buttons.addStretch(1)
        playback_box = QGroupBox("Playback")
        playback_layout = QVBoxLayout(playback_box)
        playback_layout.addLayout(playback_buttons)
        playback_layout.addWidget(self.playback_label)

        editor_container = QWidget()
        editor_layout = QVBoxLayout(editor_container)
        editor_layout.setContentsMargins(0, 0, 0, 0)
        editor_layout.addWidget(property_box)
        editor_layout.addWidget(playback_box)

        viewer_splitter = QSplitter(Qt.Vertical)
        viewer_splitter.addWidget(self.episodes_table)
        viewer_splitter.addWidget(editor_container)
        viewer_splitter.setChildrenCollapsible(False)
        viewer_splitter.setSizes([420, 260])
        self.viewer_splitter = viewer_splitter

        group = QGroupBox("Episodes")
        layout = QVBoxLayout(group)
        layout.addWidget(viewer_splitter)
        self.set_episode_editor_enabled(False)
        self.pause_btn.setEnabled(False)
        self.stop_playback_btn.setEnabled(False)
        return group

    def connect_signals(self) -> None:
        self.bridge.frame.connect(self.on_frame)
        self.bridge.worker_done.connect(self.on_worker_done)
        self.bridge.worker_failed.connect(self.on_worker_failed)
        self.start_can_btn.clicked.connect(self.start_can)
        self.start_cameras_btn.clicked.connect(self.start_cameras)
        self.start_arms_btn.clicked.connect(self.start_arms)
        self.stop_managed_btn.clicked.connect(self.stop_managed_subsystems)
        self.preflight_btn.clicked.connect(lambda: self.run_preflight_async(automatic=False))
        self.start_episode_btn.clicked.connect(self.start_episode)
        self.stop_keep_btn.clicked.connect(lambda: self.request_recorder_stop("usable"))
        self.stop_unusable_btn.clicked.connect(lambda: self.request_recorder_stop("unusable"))
        self.episodes_table.itemSelectionChanged.connect(self.load_selected_episode)
        self.save_episode_btn.clicked.connect(self.save_selected_episode)
        self.open_folder_btn.clicked.connect(self.open_selected_folder)
        self.delete_episode_btn.clicked.connect(self.delete_selected_episode)
        self.refresh_episodes_btn.clicked.connect(self.refresh_episodes)
        self.play_btn.clicked.connect(self.play_selected_episode)
        self.pause_btn.clicked.connect(self.toggle_playback_pause)
        self.stop_playback_btn.clicked.connect(self.stop_playback)

    def install_timers(self) -> None:
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(500)
        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.check_processes)
        self.process_timer.start(250)
        self.node_timer = QTimer(self)
        self.node_timer.timeout.connect(self.refresh_subsystem_state)
        self.node_timer.start(2500)
        self.recording_timer = QTimer(self)
        self.recording_timer.timeout.connect(self.refresh_activity_labels)
        self.recording_timer.start(250)

    def restore_splitters(self) -> None:
        for key, splitter in (
            ("mainSplitter", self.main_splitter),
            ("lowerSplitter", self.lower_splitter),
            ("viewerSplitter", self.viewer_splitter),
            ("cameraSplitter", self.camera_splitter),
        ):
            state = self.settings.value(key)
            if state is not None:
                splitter.restoreState(state)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.apply_scale()

    def apply_scale(self) -> None:
        point_size = max(9, min(13, round(min(self.width() / 155, self.height() / 88))))
        if point_size == self._last_font_size:
            return
        self._last_font_size = point_size
        font = QFont(select_font_family(), point_size, QFont.Bold)
        self.setFont(font)
        self.start_episode_btn.setMinimumHeight(max(44, int(point_size * 4.5)))
        self.episodes_table.verticalHeader().setDefaultSectionSize(max(28, point_size * 3))

    def on_frame(self, source: str, key: str, image: QImage) -> None:
        if source == self.active_source:
            self.camera_panes[key].preview.set_image(image)

    def set_active_source(self, source: str) -> None:
        self.active_source = source
        label = "Viewing episode playback" if source == "viewer" else "Viewing live signals"
        self.source_label.setText(label)
        for pane in self.camera_panes.values():
            pane.preview.clear_image("Waiting for episode" if source == "viewer" else "Waiting for camera")

    def refresh_health(self) -> None:
        snapshot = self.node.snapshot()
        cameras = snapshot["cameras"][self.active_source]
        for key, pane in self.camera_panes.items():
            health = cameras.get(key, {"rate": 0.0, "age": float("inf")})
            pane.update_health(float(health["rate"]), float(health["age"]), self.active_source)
        arms = snapshot["arms"][self.active_source]
        for side, pane in self.arm_panes.items():
            pane.update_observation(arms.get(side, {}), self.active_source)

    def ros_nodes(self) -> set[str]:
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def refresh_subsystem_state(self) -> None:
        nodes = self.ros_nodes()
        camera_count = len(CAMERA_NODES & nodes)
        arm_count = len(ARM_NODES & nodes)
        camera_mode = self.process_mode("cameras", nodes)
        arm_mode = self.process_mode("arms", nodes)
        self.systems_label.setText(
            f"Cameras: {camera_count}/3 {camera_mode}\n"
            f"Arms: {arm_count}/2 {arm_mode}\n"
            f"ROS domain: {os.environ.get('ROS_DOMAIN_ID', '')}"
        )
        idle = self.recorder is None and self.playback is None
        self.start_can_btn.setEnabled(
            idle and not self.worker_active and arm_count == 0
        )
        self.start_cameras_btn.setEnabled(idle and camera_count == 0 and "cameras" not in self.managed)
        self.start_arms_btn.setEnabled(idle and arm_count == 0 and "arms" not in self.managed)
        self.stop_managed_btn.setEnabled(idle and bool(self.managed))
        if camera_count != 3 or arm_count != 2:
            self.invalidate_readiness()

    def process_mode(self, name: str, nodes: set[str]) -> str:
        managed = self.managed.get(name)
        if managed and managed.process.poll() is None:
            return "(started here)"
        expected = CAMERA_NODES if name == "cameras" else ARM_NODES
        return "(external)" if expected <= nodes else ""

    def invalidate_readiness(self) -> None:
        self.preflight_valid_until = 0.0
        self.last_preflight = None
        self.readiness_badge.setText("Not ready")
        self.readiness_badge.setProperty("health", "warn")
        refresh_style(self.readiness_badge)
        if self.recorder is None:
            self.start_episode_btn.setEnabled(False)

    def start_can(self) -> None:
        if self.worker_active:
            return

        if self.recorder is not None or self.playback is not None:
            QMessageBox.warning(
                self,
                "Busy",
                "Stop recording or playback before configuring CAN.",
            )
            return

        present = ARM_NODES & self.ros_nodes()
        if present:
            QMessageBox.warning(
                self,
                "Arms are running",
                "Stop the arm nodes before restarting the CAN interfaces.\n\n"
                + "\n".join(sorted(present)),
            )
            return

        self.worker_active = True
        self.start_can_btn.setEnabled(False)
        self.preflight_btn.setEnabled(False)
        self.start_episode_btn.setEnabled(False)
        self.invalidate_readiness()
        self.preflight_result.setPlainText(
            "Configuring can_left and can_right at 1 Mbit/s…"
        )
        self.statusBar().showMessage("Configuring CAN interfaces")
        self.start_worker("can", configure_can_interfaces)

    def start_cameras(self) -> None:
        self.start_subsystem("cameras", CAMERA_NODES, CAMERA_LAUNCH)

    def start_arms(self) -> None:
        answer = QMessageBox.question(
            self,
            "Observation-only arm startup",
            "Confirm both arms are in the required horizontal/reset pose, both grippers are manually closed, labeled CAN connections are preserved, and the emergency stop is accessible.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.start_subsystem("arms", ARM_NODES, ARM_LAUNCH)

    def start_subsystem(self, name: str, expected_nodes: set[str], launch_file: Path) -> None:
        if self.recorder is not None or self.playback is not None:
            return
        present = expected_nodes & self.ros_nodes()
        if present:
            QMessageBox.warning(self, "Nodes already present", "Refusing a duplicate launch.\n\n" + "\n".join(sorted(present)))
            return
        if not launch_file.is_file():
            QMessageBox.critical(self, "Launch file missing", str(launch_file))
            return
        runtime = OUTPUT_ROOT / "runtime"
        runtime.mkdir(parents=True, exist_ok=True)
        log_path = runtime / f"{name}-{datetime.now():%Y%m%d-%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        try:
            process = subprocess.Popen(
                ["ros2", "launch", str(launch_file)],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            log_handle.close()
            QMessageBox.critical(self, f"Could not start {name}", str(exc))
            return
        self.managed[name] = ManagedProcess(name, process, log_handle, log_path)
        self.invalidate_readiness()
        self.statusBar().showMessage(f"Starting {name}; log: {log_path}")
        self.refresh_subsystem_state()
        QTimer.singleShot(3000, lambda: self.run_preflight_async(automatic=True))

    def stop_managed_subsystems(self) -> None:
        if self.recorder is not None or self.playback is not None:
            QMessageBox.warning(self, "Busy", "Stop recording or playback first.")
            return
        for managed in self.managed.values():
            self.request_process_stop(managed)
        self.invalidate_readiness()

    def run_preflight_async(self, automatic: bool = False) -> None:
        if self.worker_active or self.recorder is not None or self.playback is not None:
            return
        self.worker_active = True
        self.preflight_btn.setEnabled(False)
        self.start_episode_btn.setEnabled(False)
        self.readiness_badge.setText("Checking…")
        self.readiness_badge.setProperty("health", "warn")
        refresh_style(self.readiness_badge)
        if not automatic or not self.preflight_result.toPlainText():
            self.preflight_result.setPlainText("Running read-only readiness checks…")
        self.statusBar().showMessage("Checking readiness")
        self.start_worker("preflight", lambda: run_preflight(OUTPUT_ROOT, MIN_FREE_GB))

    def start_worker(self, name: str, function: Callable[[], object]) -> None:
        def target() -> None:
            try:
                result = function()
            except BaseException as exc:
                self.bridge.worker_failed.emit(name, str(exc))
            else:
                self.bridge.worker_done.emit(name, result)

        threading.Thread(target=target, daemon=True).start()

    def on_worker_done(self, name: str, result: object) -> None:
        self.worker_active = False
        idle = self.recorder is None and self.playback is None
        self.preflight_btn.setEnabled(idle)
        self.start_can_btn.setEnabled(
            idle and not bool(ARM_NODES & self.ros_nodes())
        )

        if name == "can":
            message = str(result)
            self.preflight_result.setPlainText(message)
            self.statusBar().showMessage("CAN interfaces configured")
            QMessageBox.information(self, "CAN ready", message)
            self.refresh_subsystem_state()
            QTimer.singleShot(
                250,
                lambda: self.run_preflight_async(automatic=True),
            )
        elif name == "preflight":
            report = result
            assert isinstance(report, Report)
            self.last_preflight = report
            self.preflight_valid_until = time.monotonic() + READINESS_VALID_SECONDS if report.passed else 0.0
            text = [f"Readiness: {report.status}"]
            for check in report.checks:
                text.append(f"[{check.status}] {check.name}: {check.detail}")
            self.preflight_result.setPlainText("\n".join(text))
            self.readiness_badge.setText("Ready" if report.passed else "Blocked")
            self.readiness_badge.setProperty("health", "pass" if report.passed else "fail")
            refresh_style(self.readiness_badge)
            self.start_episode_btn.setEnabled(report.passed and self.playback is None)
            self.statusBar().showMessage("Ready to record" if report.passed else "Readiness checks blocked recording")
            if self.pending_record_after_readiness:
                self.pending_record_after_readiness = False
                if report.passed:
                    QTimer.singleShot(0, self.start_episode)
        elif name == "validation":
            validation = result
            assert isinstance(validation, dict)
            self.statusBar().showMessage(f"Validation: {validation.get('status', 'unknown')}")
            self.refresh_episodes()
            self.refresh_subsystem_state()
            QTimer.singleShot(250, lambda: self.run_preflight_async(automatic=True))

    def on_worker_failed(self, name: str, error: str) -> None:
        self.worker_active = False
        self.pending_record_after_readiness = False
        idle = self.recorder is None and self.playback is None
        self.preflight_btn.setEnabled(idle)
        self.start_can_btn.setEnabled(
            idle and not bool(ARM_NODES & self.ros_nodes())
        )
        self.start_episode_btn.setEnabled(False)
        QMessageBox.critical(self, f"{name.title()} failed", error)
        self.statusBar().showMessage(f"{name.title()} failed")

    def metadata(self) -> dict[str, object] | None:
        instruction = self.instruction_edit.text().strip()
        task = self.task_edit.text().strip()
        if not task or not instruction:
            QMessageBox.warning(self, "Missing metadata", "Task and instruction are required.")
            return None
        return {
            "task": task,
            "garment_type": self.garment_edit.text().strip() or None,
            "initial_state_level": self.initial_state_combo.currentData(),
            "instruction": instruction,
            "notes": self.notes_edit.toPlainText().strip(),
            "outcome": "not_assessed",
            "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "88")),
            "camera_mapping": {
                "front": "camera_f / CC1WC52009R",
                "left": "camera_l / CC1WC52006V",
                "right": "camera_r / CC1WC52012P",
            },
            "arm_mapping": {"left": "can_left / LARM", "right": "can_right / RARM"},
            "recorded_topics": list(REQUIRED_TOPICS),
            "software_revision": git_revision(REPOSITORY_ROOT),
        }

    def start_episode(self) -> None:
        if self.recorder is not None or self.playback is not None or self.worker_active:
            return
        if self.last_preflight is None or not self.last_preflight.passed or time.monotonic() > self.preflight_valid_until:
            self.pending_record_after_readiness = True
            self.run_preflight_async(automatic=True)
            return
        metadata = self.metadata()
        if metadata is None:
            return
        paths = create_episode(
            OUTPUT_ROOT,
            task=str(metadata["task"]),
            metadata={**metadata, "recording_start": utc_now_iso(), "preflight": self.last_preflight.to_dict()},
        )
        command = ["ros2", "bag", "record", "-s", "mcap", "-o", str(paths.bag), "--topics", *REQUIRED_TOPICS]
        log_handle = paths.recorder_log.open("w", encoding="utf-8", buffering=1)
        try:
            process = subprocess.Popen(
                command,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            log_handle.close()
            update_episode(paths, status="recorder_start_failed", recording_end=utc_now_iso(), error=str(exc))
            QMessageBox.critical(self, "Could not start recorder", str(exc))
            self.refresh_episodes()
            return
        self.current_episode = paths
        self.recorder = ManagedProcess("recorder", process, log_handle, paths.recorder_log)
        self.recording_started_monotonic = time.monotonic()
        self.stop_disposition = "usable"
        self.preflight_valid_until = 0.0
        self.start_episode_btn.setEnabled(False)
        self.stop_keep_btn.setEnabled(True)
        self.stop_unusable_btn.setEnabled(True)
        self.preflight_btn.setEnabled(False)
        self.set_workflow_enabled(False)
        self.statusBar().showMessage(f"Recording {paths.episode_id}")

    def request_recorder_stop(self, disposition: str) -> None:
        if self.recorder is None:
            return
        self.stop_disposition = disposition
        self.request_process_stop(self.recorder)
        self.stop_keep_btn.setEnabled(False)
        self.stop_unusable_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping recorder cleanly…")

    def request_process_stop(self, managed: ManagedProcess) -> None:
        if managed.process.poll() is not None:
            return
        if managed.stop_requested_at <= 0.0:
            managed.stop_requested_at = time.monotonic()
            try:
                os.killpg(managed.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def refresh_activity_labels(self) -> None:
        if self.recorder is None or self.current_episode is None:
            self.recording_label.setText("Not recording")
        else:
            elapsed = time.monotonic() - self.recording_started_monotonic
            self.recording_label.setText(f"Recording: {elapsed:6.1f} s\n{self.current_episode.root}")
        if self.playback is None or self.current_playback_root is None:
            self.playback_label.setText("No episode playing")
        else:
            elapsed = time.monotonic() - self.playback_started_monotonic - self.playback_paused_total
            if self.playback_paused:
                elapsed -= time.monotonic() - self.playback_pause_started
            elapsed = max(0.0, elapsed)
            total = f" / {self.playback_duration:.1f} s" if self.playback_duration > 0 else ""
            state = "Paused" if self.playback_paused else "Playing"
            self.playback_label.setText(f"{state}: {elapsed:.1f} s{total}\n{self.current_playback_root.name}")

    def check_processes(self) -> None:
        self.check_recorder()
        self.check_playback()
        subsystem_changed = False
        for name in list(self.managed):
            managed = self.managed[name]
            code = managed.process.poll()
            if code is None:
                self.escalate_stop(managed)
                continue
            managed.log_handle.close()
            del self.managed[name]
            subsystem_changed = True
            if code not in (0, -signal.SIGINT):
                self.statusBar().showMessage(f"{name} exited with {code}; log: {managed.log_path}")
                self.invalidate_readiness()
        if subsystem_changed:
            self.refresh_subsystem_state()

    def check_recorder(self) -> None:
        if self.recorder is None:
            return
        code = self.recorder.process.poll()
        if code is None:
            self.escalate_stop(self.recorder)
            return
        self.recorder.log_handle.close()
        paths = self.current_episode
        elapsed = max(0.0, time.monotonic() - self.recording_started_monotonic)
        disposition = self.stop_disposition
        self.recorder = None
        self.current_episode = None
        self.recording_started_monotonic = 0.0
        self.stop_keep_btn.setEnabled(False)
        self.stop_unusable_btn.setEnabled(False)
        self.preflight_btn.setEnabled(True)
        self.set_workflow_enabled(True)
        if paths is None:
            return
        status = "recorded" if code in (0, -signal.SIGINT) else "recorder_failed"
        update_episode(
            paths,
            status=status,
            operator_disposition=disposition,
            recording_end=utc_now_iso(),
            duration_seconds=elapsed,
            recorder_exit_code=code,
        )
        self.refresh_episodes(select_path=paths.root)
        self.statusBar().showMessage("Validating recorded episode…")
        self.worker_active = True
        self.preflight_btn.setEnabled(False)
        self.start_worker("validation", lambda: validate_episode(paths.bag, paths.validation_json, paths.recorder_log))

    def check_playback(self) -> None:
        if self.playback is None:
            return
        code = self.playback.process.poll()
        if code is None:
            self.escalate_stop(self.playback)
            return
        self.playback.log_handle.close()
        log_path = self.playback.log_path
        self.playback = None
        self.current_playback_root = None
        self.playback_paused = False
        self.pause_btn.setText("Pause")
        self.pause_btn.setEnabled(False)
        self.stop_playback_btn.setEnabled(False)
        self.play_btn.setEnabled(self.selected_episode_root is not None)
        self.set_active_source("live")
        self.set_workflow_enabled(True)
        if code not in (0, -signal.SIGINT):
            self.statusBar().showMessage(f"Playback exited with {code}; log: {log_path}")
        else:
            self.statusBar().showMessage("Playback finished")
        self.refresh_subsystem_state()

    @staticmethod
    def escalate_stop(managed: ManagedProcess) -> None:
        if managed.stop_requested_at <= 0.0:
            return
        elapsed = time.monotonic() - managed.stop_requested_at
        try:
            if elapsed > 12.0:
                os.killpg(managed.process.pid, signal.SIGKILL)
            elif elapsed > 7.0:
                os.killpg(managed.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def set_workflow_enabled(self, enabled: bool) -> None:
        for widget in (
            self.task_edit,
            self.garment_edit,
            self.initial_state_combo,
            self.instruction_edit,
            self.notes_edit,
            self.start_can_btn,
            self.start_cameras_btn,
            self.start_arms_btn,
        ):
            widget.setEnabled(enabled)
        self.stop_managed_btn.setEnabled(enabled and bool(self.managed))
        self.preflight_btn.setEnabled(enabled and not self.worker_active)
        self.episodes_table.setEnabled(enabled)
        self.set_episode_editor_enabled(enabled and self.selected_episode_root is not None)

    def set_episode_editor_enabled(self, enabled: bool) -> None:
        for widget in (
            self.outcome_combo,
            self.disposition_combo,
            self.viewer_notes_edit,
            self.save_episode_btn,
            self.open_folder_btn,
            self.delete_episode_btn,
            self.play_btn,
            self.playback_rate_combo,
        ):
            widget.setEnabled(enabled)

    def refresh_episodes(self, select_path: Optional[Path] = None) -> None:
        requested = str(select_path.resolve()) if select_path is not None else (
            str(self.selected_episode_root.resolve()) if self.selected_episode_root is not None else None
        )
        episodes = list_episodes(OUTPUT_ROOT)
        self.episodes_table.blockSignals(True)
        self.episodes_table.setRowCount(len(episodes))
        selected_row = -1
        for row, episode in enumerate(episodes):
            values = (
                str(episode.get("created_at", ""))[:19].replace("T", " "),
                str(episode.get("episode_id", "")),
                str(episode.get("task", "")),
                display_option(OUTCOME_OPTIONS, str(episode.get("outcome", "not_assessed"))),
                display_option(DISPOSITION_OPTIONS, str(episode.get("operator_disposition", "needs_review"))),
                str(episode.get("_validation_status", "")),
            )
            path = str(episode.get("_path", ""))
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setData(Qt.UserRole, path)
                self.episodes_table.setItem(row, column, item)
            if requested and path == requested:
                selected_row = row
        self.episodes_table.blockSignals(False)
        if selected_row >= 0:
            self.episodes_table.selectRow(selected_row)
            self.load_selected_episode()
        elif self.episodes_table.rowCount() > 0 and self.selected_episode_root is None:
            self.episodes_table.selectRow(0)
            self.load_selected_episode()
        elif self.episodes_table.rowCount() == 0:
            self.clear_selected_episode()

    def selected_path_from_table(self) -> Optional[Path]:
        row = self.episodes_table.currentRow()
        if row < 0:
            return None
        item = self.episodes_table.item(row, 0)
        path = item.data(Qt.UserRole) if item is not None else None
        return Path(str(path)).expanduser() if path else None

    def load_selected_episode(self) -> None:
        root = self.selected_path_from_table()
        if root is None:
            self.clear_selected_episode()
            return
        try:
            payload = read_json(root / "episode.json")
        except Exception as exc:
            QMessageBox.warning(self, "Could not read episode", str(exc))
            self.clear_selected_episode()
            return
        self.selected_episode_root = root
        self.selected_episode_label.setText(
            f"{payload.get('episode_id', root.name)}\n"
            f"Duration: {float(payload.get('duration_seconds', 0.0)):.1f} s | "
            f"Task: {payload.get('task', '')}"
        )
        set_combo_data(self.outcome_combo, str(payload.get("outcome", "not_assessed")))
        set_combo_data(self.disposition_combo, str(payload.get("operator_disposition", "needs_review")))
        self.viewer_notes_edit.setPlainText(str(payload.get("notes", "")))
        self.set_episode_editor_enabled(self.recorder is None and self.playback is None)

    def clear_selected_episode(self) -> None:
        self.selected_episode_root = None
        self.selected_episode_label.setText("No episode selected")
        self.viewer_notes_edit.clear()
        self.set_episode_editor_enabled(False)

    def save_selected_episode(self) -> None:
        root = self.selected_episode_root
        if root is None:
            return
        try:
            update_episode(
                paths_for_episode(root),
                outcome=self.outcome_combo.currentData(),
                operator_disposition=self.disposition_combo.currentData(),
                notes=self.viewer_notes_edit.toPlainText().strip(),
            )
        except Exception as exc:
            QMessageBox.critical(self, "Could not save episode", str(exc))
            return
        self.statusBar().showMessage("Episode properties saved")
        self.refresh_episodes(select_path=root)

    def delete_selected_episode(self) -> None:
        root = self.selected_episode_root
        if root is None or self.recorder is not None or self.playback is not None:
            return
        answer = QMessageBox.warning(
            self,
            "Delete episode?",
            f"Move this entire episode to {OUTPUT_ROOT / '.trash'}?\n\n{root.name}\n\nThe MCAP and sidecars will be removed from the episode list but not permanently erased.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            destination = move_episode_to_trash(root, OUTPUT_ROOT)
        except Exception as exc:
            QMessageBox.critical(self, "Could not delete episode", str(exc))
            return
        self.clear_selected_episode()
        self.refresh_episodes()
        self.statusBar().showMessage(f"Episode moved to {destination}")

    def open_selected_folder(self) -> None:
        root = self.selected_episode_root
        if root is None:
            return
        try:
            subprocess.Popen(["xdg-open", str(root)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            QMessageBox.warning(self, "Could not open folder", str(exc))

    def play_selected_episode(self) -> None:
        root = self.selected_episode_root
        if root is None or self.recorder is not None or self.playback is not None:
            return
        paths = paths_for_episode(root)
        if not paths.bag.is_dir():
            QMessageBox.warning(self, "Bag missing", str(paths.bag))
            return
        try:
            payload = read_json(paths.episode_json)
        except Exception as exc:
            QMessageBox.warning(self, "Could not read episode", str(exc))
            return
        runtime = OUTPUT_ROOT / "runtime"
        runtime.mkdir(parents=True, exist_ok=True)
        log_path = runtime / f"playback-{datetime.now():%Y%m%d-%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        remaps = [f"{topic}:={VIEWER_PREFIX}{topic}" for topic in REQUIRED_TOPICS]
        command = [
            "ros2",
            "bag",
            "play",
            str(paths.bag),
            "--rate",
            str(float(self.playback_rate_combo.currentData())),
            "--disable-keyboard-controls",
            "--remap",
            *remaps,
        ]
        self.node.clear_source("viewer")
        try:
            process = subprocess.Popen(
                command,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            log_handle.close()
            QMessageBox.critical(self, "Could not start playback", str(exc))
            return
        self.playback = ManagedProcess("playback", process, log_handle, log_path)
        self.current_playback_root = root
        self.playback_started_monotonic = time.monotonic()
        self.playback_pause_started = 0.0
        self.playback_paused_total = 0.0
        self.playback_duration = float(payload.get("duration_seconds", 0.0))
        self.playback_paused = False
        self.set_active_source("viewer")
        self.set_workflow_enabled(False)
        self.play_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        self.stop_playback_btn.setEnabled(True)
        self.statusBar().showMessage(f"Playing {root.name}")

    def toggle_playback_pause(self) -> None:
        if self.playback is None or self.playback.process.poll() is not None:
            return
        try:
            if self.playback_paused:
                os.killpg(self.playback.process.pid, signal.SIGCONT)
                self.playback_paused_total += time.monotonic() - self.playback_pause_started
                self.playback_pause_started = 0.0
                self.playback_paused = False
                self.pause_btn.setText("Pause")
            else:
                os.killpg(self.playback.process.pid, signal.SIGSTOP)
                self.playback_pause_started = time.monotonic()
                self.playback_paused = True
                self.pause_btn.setText("Resume")
        except ProcessLookupError:
            pass

    def stop_playback(self) -> None:
        if self.playback is None:
            return
        if self.playback_paused:
            try:
                os.killpg(self.playback.process.pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
            self.playback_paused = False
        self.request_process_stop(self.playback)
        self.pause_btn.setEnabled(False)
        self.stop_playback_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping playback…")

    @staticmethod
    def stop_process_sync(managed: ManagedProcess) -> None:
        if managed.process.poll() is not None:
            return
        try:
            os.killpg(managed.process.pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        try:
            os.killpg(managed.process.pid, signal.SIGINT)
            managed.process.wait(timeout=7.0)
        except subprocess.TimeoutExpired:
            os.killpg(managed.process.pid, signal.SIGTERM)
            try:
                managed.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(managed.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.recorder is not None:
            answer = QMessageBox.question(
                self,
                "Stop active episode?",
                "The current recording will be stopped, preserved as unusable, and validated before the interface closes.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            managed = self.recorder
            paths = self.current_episode
            elapsed = max(0.0, time.monotonic() - self.recording_started_monotonic)
            self.stop_process_sync(managed)
            managed.log_handle.close()
            code = managed.process.poll()
            if paths is not None:
                update_episode(
                    paths,
                    status="recorded" if code in (0, -signal.SIGINT) else "recorder_failed",
                    operator_disposition="unusable",
                    recording_end=utc_now_iso(),
                    duration_seconds=elapsed,
                    recorder_exit_code=code,
                )
                try:
                    validate_episode(paths.bag, paths.validation_json, paths.recorder_log)
                except Exception as exc:
                    update_episode(paths, validation_error=str(exc))
            self.recorder = None
            self.current_episode = None
        if self.playback is not None:
            self.stop_process_sync(self.playback)
            self.playback.log_handle.close()
            self.playback = None
        for managed in list(self.managed.values()):
            self.stop_process_sync(managed)
            managed.log_handle.close()
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("mainSplitter", self.main_splitter.saveState())
        self.settings.setValue("lowerSplitter", self.lower_splitter.saveState())
        self.settings.setValue("viewerSplitter", self.viewer_splitter.saveState())
        self.settings.setValue("cameraSplitter", self.camera_splitter.saveState())
        event.accept()


def display_option(options: tuple[tuple[str, str], ...], value: str) -> str:
    for label, stored in options:
        if stored == value:
            return label
    return value.replace("_", " ").title()


def set_combo_data(combo: QComboBox, value: str) -> None:
    index = combo.findData(value)
    combo.setCurrentIndex(index if index >= 0 else 0)


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
        QLabel[sourceBanner="true"] { padding: 0.35em 0.6em; background: #e7edf2; }
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
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    bridge = Bridge()
    node = MonitorNode(bridge)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    apply_application_style(app)
    window = Window(node, bridge)
    window.show()
    try:
        return int(app.exec_())
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
