#!/usr/bin/env python3
"""Laundry Butler cleansing-only episode review and playback GUI.

The raw rosbag2/MCAP recording is never modified. Review fields are written to
``episode.json`` and deletion is reversible: the full episode directory is
moved into a sibling ``.trash`` directory.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

try:
    import yaml
except ImportError as exc:  # Report a clear package-level error before Qt starts.
    raise SystemExit(
        "python3-yaml is required. Install it with: sudo apt install python3-yaml"
    ) from exc

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from PyQt5.QtCore import QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QColor, QFont, QFontDatabase, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


DEFAULT_DATA_ROOT = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_DATA_ROOT",
        "/home/laundrybutler/Aloha Shared SSD/dxx_data",
    )
).expanduser()
VIEWER_PREFIX = "/laundry_butler/cleanser"
MIN_DURATION_SECONDS = float(os.environ.get("LAUNDRY_BUTLER_MIN_EPISODE_SECONDS", "20"))
PREVIEW_MAX_FPS = 15.0
RATE_WINDOW_SECONDS = 2.5
CACHE_ROOT = Path.home() / ".cache" / "laundry_butler" / "data_cleansing"

CAMERAS = (
    ("left", "Left wrist", "/camera_l/color/image_raw"),
    ("front", "Front / top", "/camera_f/color/image_raw"),
    ("right", "Right wrist", "/camera_r/color/image_raw"),
)
JOINTS = (
    ("left", "Left joints", "/puppet/joint_left"),
    ("right", "Right joints", "/puppet/joint_right"),
)
REQUIRED_TOPICS = tuple(topic for _key, _title, topic in (*CAMERAS, *JOINTS))
CAMERA_INFO_TOPICS = (
    "/camera_f/color/camera_info",
    "/camera_l/color/camera_info",
    "/camera_r/color/camera_info",
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
FILTER_OPTIONS = (
    ("All episodes", "all"),
    ("Integrity problems", "problems"),
    ("Pass", "pass"),
    (f"Under {MIN_DURATION_SECONDS:g} seconds", "short"),
    ("Missing or empty topics", "topics"),
    ("Needs review", "needs_review"),
    ("Unusable", "unusable"),
    ("Trash", "trash"),
)
EPISODE_STAMP_RE = re.compile(
    r"episode_(?P<date>\d{8})_(?P<time>\d{6})(?:_(?P<millis>\d{3}))?"
)


@dataclass
class EpisodeRecord:
    root: Path
    bag: Path
    metadata_path: Path
    duration_s: Optional[float]
    counts: dict[str, int]
    metadata_error: str = ""
    payload: dict[str, Any] = field(default_factory=dict)
    validation_status: str = "not run"
    in_trash: bool = False
    relative_path: str = ""
    batch: str = ""
    created_display: str = ""
    created_iso: str = ""
    task: str = ""
    outcome: str = "not_assessed"
    disposition: str = "needs_review"
    notes: str = ""
    missing: list[str] = field(default_factory=list)
    empty: list[str] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)

    @property
    def integrity(self) -> str:
        return "PASS" if not self.flags else " + ".join(self.flags)

    @property
    def is_problem(self) -> bool:
        return bool(self.flags)

    @property
    def search_blob(self) -> str:
        values = (
            self.root.name,
            self.relative_path,
            self.batch,
            self.created_display,
            self.task,
            self.outcome,
            self.disposition,
            self.validation_status,
            self.integrity,
            self.notes,
            " ".join(self.missing),
            " ".join(self.empty),
        )
        return " ".join(str(value) for value in values).lower()


@dataclass
class ManagedPlayback:
    process: subprocess.Popen
    log_handle: Any
    log_path: Path
    record: EpisodeRecord
    rate: float
    started_at: float
    paused: bool = False
    pause_started_at: float = 0.0
    paused_total: float = 0.0
    stop_requested_at: float = 0.0


class Bridge(QObject):
    frame = pyqtSignal(str, object)
    records_ready = pyqtSignal(object)
    scan_failed = pyqtSignal(str)


class PlaybackMonitor(Node):
    """Subscribe only to remapped playback topics, never live robot topics."""

    def __init__(self, bridge: Bridge) -> None:
        super().__init__("laundry_butler_data_cleansing_gui")
        self.bridge = bridge
        self.lock = threading.Lock()
        self.last_preview: dict[str, float] = {key: 0.0 for key, _title, _topic in CAMERAS}
        self.camera_arrivals: dict[str, deque[float]] = {
            key: deque() for key, _title, _topic in CAMERAS
        }
        self.camera_last: dict[str, float] = {key: 0.0 for key, _title, _topic in CAMERAS}
        self.joint_arrivals: dict[str, deque[float]] = {
            side: deque() for side, _title, _topic in JOINTS
        }
        self.joint_last: dict[str, float] = {side: 0.0 for side, _title, _topic in JOINTS}
        self.joint_text: dict[str, str] = {
            side: "Waiting for data" for side, _title, _topic in JOINTS
        }
        self._subscriptions: list[Any] = []

        image_group = ReentrantCallbackGroup()
        joint_group = ReentrantCallbackGroup()
        for key, _title, topic in CAMERAS:
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    viewer_topic(topic),
                    self._image_callback(key),
                    qos_profile_sensor_data,
                    callback_group=image_group,
                )
            )
        for side, _title, topic in JOINTS:
            self._subscriptions.append(
                self.create_subscription(
                    JointState,
                    viewer_topic(topic),
                    self._joint_callback(side),
                    qos_profile_sensor_data,
                    callback_group=joint_group,
                )
            )

    def _image_callback(self, key: str) -> Callable[[Image], None]:
        def callback(message: Image) -> None:
            now = time.monotonic()
            with self.lock:
                arrivals = self.camera_arrivals[key]
                arrivals.append(now)
                self.camera_last[key] = now
                trim(arrivals, now)
                if now - self.last_preview[key] < 1.0 / PREVIEW_MAX_FPS:
                    return
                self.last_preview[key] = now
            try:
                image = to_qimage(message)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return
            self.bridge.frame.emit(key, image)

        return callback

    def _joint_callback(self, side: str) -> Callable[[JointState], None]:
        def callback(message: JointState) -> None:
            names = [short_joint_name(name) for name in message.name]
            positions = [float(value) for value in message.position]
            if names and len(names) == len(positions):
                text = "  ".join(
                    f"{name}: {value:+.4f}" for name, value in zip(names, positions)
                )
            else:
                text = "Position: [" + ", ".join(f"{value:+.4f}" for value in positions) + "]"
            now = time.monotonic()
            with self.lock:
                arrivals = self.joint_arrivals[side]
                arrivals.append(now)
                self.joint_last[side] = now
                self.joint_text[side] = text
                trim(arrivals, now)

        return callback

    def clear(self) -> None:
        with self.lock:
            for key in self.camera_arrivals:
                self.camera_arrivals[key].clear()
                self.camera_last[key] = 0.0
                self.last_preview[key] = 0.0
            for side in self.joint_arrivals:
                self.joint_arrivals[side].clear()
                self.joint_last[side] = 0.0
                self.joint_text[side] = "Waiting for data"

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            cameras: dict[str, dict[str, float]] = {}
            for key, arrivals in self.camera_arrivals.items():
                trim(arrivals, now)
                last = self.camera_last[key]
                cameras[key] = {
                    "rate": arrival_rate(arrivals),
                    "age": now - last if last else float("inf"),
                }
            joints: dict[str, dict[str, Any]] = {}
            for side, arrivals in self.joint_arrivals.items():
                trim(arrivals, now)
                last = self.joint_last[side]
                joints[side] = {
                    "rate": arrival_rate(arrivals),
                    "age": now - last if last else float("inf"),
                    "text": self.joint_text[side],
                }
        return {"cameras": cameras, "joints": joints}


def viewer_topic(topic: str) -> str:
    return f"{VIEWER_PREFIX}{topic}"


def trim(arrivals: deque[float], now: float) -> None:
    cutoff = now - RATE_WINDOW_SECONDS
    while arrivals and arrivals[0] < cutoff:
        arrivals.popleft()


def arrival_rate(arrivals: deque[float]) -> float:
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        return 0.0
    return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])


def short_joint_name(name: str) -> str:
    value = name.rsplit("/", 1)[-1]
    value = value.replace("joint", "j").replace("_", "")
    return value or "j"


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
        super().__init__("Select an episode and press Play")
        self.source: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(190, 125)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#111;color:#aaa;border:1px solid #444")

    def set_image(self, image: QImage) -> None:
        self.source = QPixmap.fromImage(image)
        self._rescale()

    def clear_image(self, message: str = "Waiting for playback") -> None:
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
        self.health = QLabel("No playback signal")
        self.health.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(7, 7, 7, 7)
        layout.addWidget(heading)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.health)

    def update_health(self, stream_rate: float, age: float, playing: bool) -> None:
        if not playing:
            text, health = "Stopped", "warn"
        elif age > 3.0:
            text, health = "No playback signal", "fail"
        else:
            text, health = f"Playback: {stream_rate:.1f} Hz", "pass"
        self.health.setText(text)
        self.health.setProperty("health", health)
        refresh_style(self.health)


class JointPane(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        heading = QLabel(title)
        heading.setProperty("sectionHeading", True)
        self.health = QLabel("No playback signal")
        self.values = QLabel("Waiting for data")
        self.values.setWordWrap(True)
        self.values.setTextInteractionFlags(Qt.TextSelectableByMouse)
        header = QHBoxLayout()
        header.addWidget(heading)
        header.addStretch(1)
        header.addWidget(self.health)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.addLayout(header)
        layout.addWidget(self.values)

    def update_observation(self, value: dict[str, Any], playing: bool) -> None:
        age = float(value.get("age", float("inf")))
        if not playing:
            text, health = "Stopped", "warn"
        elif age > 3.0:
            text, health = "No playback signal", "fail"
        else:
            text, health = f"Playback: {float(value.get('rate', 0.0)):.1f} Hz", "pass"
        self.health.setText(text)
        self.health.setProperty("health", health)
        refresh_style(self.health)
        self.values.setText(str(value.get("text", "Waiting for data")))


def refresh_style(widget: QWidget) -> None:
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def _duration_seconds(value: Any) -> Optional[float]:
    if isinstance(value, dict):
        ns = value.get("nanoseconds")
        if isinstance(ns, (int, float)):
            return float(ns) / 1_000_000_000.0
    if isinstance(value, (int, float)):
        return float(value) / 1_000_000_000.0
    return None


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent), text=True
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


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def metadata_episode_root(metadata_path: Path) -> tuple[Path, Path]:
    bag = metadata_path.parent
    root = bag.parent if bag.name == "bag" else bag
    return root, bag


def is_in_trash(path: Path, data_root: Path) -> bool:
    try:
        return ".trash" in path.resolve().relative_to(data_root.resolve()).parts
    except ValueError:
        return ".trash" in path.parts


def created_from_payload_or_name(root: Path, payload: dict[str, Any]) -> tuple[str, str]:
    raw = str(payload.get("created_at", "")).strip()
    if raw:
        display = raw[:19].replace("T", " ")
        return display, raw
    match = EPISODE_STAMP_RE.search(root.name)
    if match:
        value = f"{match.group('date')}{match.group('time')}"
        parsed = datetime.strptime(value, "%Y%m%d%H%M%S")
        iso = parsed.replace(tzinfo=timezone.utc).isoformat(timespec="seconds")
        return parsed.strftime("%Y-%m-%d %H:%M:%S"), iso
    modified = datetime.fromtimestamp(root.stat().st_mtime, tz=timezone.utc)
    return modified.strftime("%Y-%m-%d %H:%M:%S"), modified.isoformat(timespec="seconds")


def task_from_payload_or_name(root: Path, payload: dict[str, Any]) -> str:
    task = str(payload.get("task", "")).strip()
    if task:
        return task
    match = EPISODE_STAMP_RE.search(root.name)
    if not match:
        return ""
    suffix = root.name[match.end() :].strip("_-")
    return suffix.replace("-", " ").replace("_", " ")


def inspect_episode(metadata_path: Path, data_root: Path) -> EpisodeRecord:
    root, bag = metadata_episode_root(metadata_path)
    counts: dict[str, int] = {}
    duration_s: Optional[float] = None
    metadata_error = ""
    try:
        with metadata_path.open("r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle) or {}
        info = document.get("rosbag2_bagfile_information", document)
        duration_s = _duration_seconds(info.get("duration"))
        for item in info.get("topics_with_message_count", []) or []:
            topic_meta = item.get("topic_metadata", {}) or {}
            name = topic_meta.get("name")
            count = item.get("message_count", 0)
            if isinstance(name, str):
                counts[name] = int(count)
    except Exception as exc:
        metadata_error = str(exc)

    payload: dict[str, Any] = {}
    episode_json = root / "episode.json"
    if episode_json.is_file():
        try:
            payload = read_json(episode_json)
        except Exception as exc:
            payload = {"sidecar_error": str(exc)}

    validation_status = "not run"
    validation_json = root / "validation.json"
    if validation_json.is_file():
        try:
            validation_status = str(read_json(validation_json).get("status", "unknown"))
        except Exception:
            validation_status = "unreadable"

    try:
        relative = root.resolve().relative_to(data_root.resolve())
        relative_path = str(relative)
        batch_parts = [part for part in relative.parent.parts if part != ".trash"]
        batch = "/".join(batch_parts) if batch_parts else "."
    except ValueError:
        relative_path = str(root)
        batch = str(root.parent)

    created_display, created_iso = created_from_payload_or_name(root, payload)
    missing = [topic for topic in REQUIRED_TOPICS if topic not in counts]
    empty = [topic for topic in REQUIRED_TOPICS if counts.get(topic) == 0]
    flags: list[str] = []
    if metadata_error:
        flags.append("UNREADABLE")
    if duration_s is None:
        flags.append("NO DURATION")
    elif duration_s < MIN_DURATION_SECONDS:
        flags.append("SHORT")
    if missing:
        flags.append("MISSING")
    if empty:
        flags.append("EMPTY")

    return EpisodeRecord(
        root=root.resolve(),
        bag=bag.resolve(),
        metadata_path=metadata_path.resolve(),
        duration_s=duration_s,
        counts=counts,
        metadata_error=metadata_error,
        payload=payload,
        validation_status=validation_status,
        in_trash=is_in_trash(root, data_root),
        relative_path=relative_path,
        batch=batch,
        created_display=created_display,
        created_iso=created_iso,
        task=task_from_payload_or_name(root, payload),
        outcome=str(payload.get("outcome", "not_assessed")),
        disposition=str(payload.get("operator_disposition", "needs_review")),
        notes=str(payload.get("notes", "")),
        missing=missing,
        empty=empty,
        flags=flags,
    )


def discover_records(data_root: Path) -> list[EpisodeRecord]:
    root = data_root.expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Dataset directory does not exist: {root}")
    records: list[EpisodeRecord] = []
    seen_roots: set[Path] = set()
    for metadata_path in root.rglob("metadata.yaml"):
        episode_root, _bag = metadata_episode_root(metadata_path)
        resolved = episode_root.resolve()
        if resolved in seen_roots:
            continue
        seen_roots.add(resolved)
        records.append(inspect_episode(metadata_path, root))
    records.sort(key=lambda item: (item.created_display, item.relative_path), reverse=True)
    return records


def display_option(options: tuple[tuple[str, str], ...], value: str) -> str:
    for label, stored in options:
        if stored == value:
            return label
    return value.replace("_", " ").title()


def set_combo_data(combo: QComboBox, value: str) -> None:
    index = combo.findData(value)
    combo.setCurrentIndex(index if index >= 0 else 0)


def format_duration(value: Optional[float]) -> str:
    return "unknown" if value is None else f"{value:.3f} s"


def format_clock(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def path_within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


class Window(QMainWindow):
    def __init__(self, node: PlaybackMonitor, bridge: Bridge) -> None:
        super().__init__()
        self.node = node
        self.bridge = bridge
        self.settings = QSettings("LaundryButler", "DataCleansing")
        self.data_root = DEFAULT_DATA_ROOT.resolve() if DEFAULT_DATA_ROOT.exists() else DEFAULT_DATA_ROOT
        self.records: list[EpisodeRecord] = []
        self.records_by_path: dict[str, EpisodeRecord] = {}
        self.filtered_records: list[EpisodeRecord] = []
        self.selected_record: Optional[EpisodeRecord] = None
        self.playback: Optional[ManagedPlayback] = None
        self.scan_active = False
        self._last_font_size = 0

        self.setWindowTitle("Laundry Butler data cleansing")
        self.setMinimumSize(1100, 720)
        self.resize(1650, 950)
        saved_geometry = self.settings.value("geometry")
        if saved_geometry is not None:
            self.restoreGeometry(saved_geometry)
        self.setStatusBar(QStatusBar())

        self.camera_panes = {key: CameraPane(title) for key, title, _topic in CAMERAS}
        self.camera_splitter = QSplitter(Qt.Horizontal)
        for key, _title, _topic in CAMERAS:
            self.camera_splitter.addWidget(self.camera_panes[key])
        self.camera_splitter.setChildrenCollapsible(False)
        self.camera_splitter.setSizes([1, 1, 1])
        camera_group = QGroupBox("Synchronized camera playback")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.addWidget(self.camera_splitter)

        self.joint_panes = {
            side: JointPane(title) for side, title, _topic in JOINTS
        }
        self.joint_splitter = QSplitter(Qt.Horizontal)
        for side, _title, _topic in JOINTS:
            self.joint_splitter.addWidget(self.joint_panes[side])
        self.joint_splitter.setChildrenCollapsible(False)
        joint_group = QGroupBox("Joint playback")
        joint_layout = QVBoxLayout(joint_group)
        joint_layout.addWidget(self.joint_splitter)

        browser = self.build_browser()
        editor = self.build_editor()
        self.lower_splitter = QSplitter(Qt.Horizontal)
        self.lower_splitter.addWidget(browser)
        self.lower_splitter.addWidget(editor)
        self.lower_splitter.setChildrenCollapsible(False)
        self.lower_splitter.setSizes([980, 620])

        self.main_splitter = QSplitter(Qt.Vertical)
        self.main_splitter.addWidget(camera_group)
        self.main_splitter.addWidget(joint_group)
        self.main_splitter.addWidget(self.lower_splitter)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.setSizes([410, 155, 385])

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(4, 4, 4, 4)
        central_layout.addWidget(self.main_splitter)
        self.setCentralWidget(central)

        self.connect_signals()
        self.install_timers()
        QTimer.singleShot(100, self.restore_splitters)
        QTimer.singleShot(150, self.refresh_records)
        self.apply_scale()

    def build_browser(self) -> QWidget:
        root_row = QHBoxLayout()
        self.root_label = QLineEdit(str(self.data_root))
        self.root_label.setReadOnly(True)
        self.choose_root_btn = QPushButton("Choose dataset")
        self.refresh_btn = QPushButton("Refresh")
        root_row.addWidget(QLabel("Dataset"))
        root_row.addWidget(self.root_label, 1)
        root_row.addWidget(self.choose_root_btn)
        root_row.addWidget(self.refresh_btn)

        filter_row = QHBoxLayout()
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Search episode, path, task, outcome, notes, or status")
        self.filter_combo = QComboBox()
        for label, value in FILTER_OPTIONS:
            self.filter_combo.addItem(label, value)
        self.show_trash_check = QCheckBox("Show trash")
        filter_row.addWidget(QLabel("Search"))
        filter_row.addWidget(self.search_edit, 1)
        filter_row.addWidget(self.filter_combo)
        filter_row.addWidget(self.show_trash_check)

        summary_row = QHBoxLayout()
        self.active_count = QLabel("Active: —")
        self.problem_count = QLabel("Problems: —")
        self.trash_count = QLabel("Trash: —")
        self.shown_count = QLabel("Shown: —")
        for label in (self.active_count, self.problem_count, self.trash_count, self.shown_count):
            label.setProperty("summary", True)
            summary_row.addWidget(label)
        summary_row.addStretch(1)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ["Created", "Episode", "Duration", "Integrity", "Outcome", "Disposition", "Batch"]
        )
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSortingEnabled(True)

        group = QGroupBox("Episodes")
        layout = QVBoxLayout(group)
        layout.addLayout(root_row)
        layout.addLayout(filter_row)
        layout.addLayout(summary_row)
        layout.addWidget(self.table, 1)
        return group

    def build_editor(self) -> QWidget:
        self.selected_label = QLabel("No episode selected")
        self.selected_label.setWordWrap(True)
        self.selected_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.integrity_details = QTextEdit()
        self.integrity_details.setReadOnly(True)
        self.integrity_details.setMaximumHeight(170)
        self.integrity_details.setPlaceholderText("Integrity and topic counts")

        self.outcome_combo = QComboBox()
        for label, value in OUTCOME_OPTIONS:
            self.outcome_combo.addItem(label, value)
        self.disposition_combo = QComboBox()
        for label, value in DISPOSITION_OPTIONS:
            self.disposition_combo.addItem(label, value)
        self.notes_edit = QTextEdit()
        self.notes_edit.setPlaceholderText("Episode review notes")
        self.notes_edit.setMaximumHeight(90)

        form = QFormLayout()
        form.addRow("Outcome", self.outcome_combo)
        form.addRow("Disposition", self.disposition_combo)
        form.addRow("Notes", self.notes_edit)

        self.save_btn = QPushButton("Save properties")
        self.open_btn = QPushButton("Open folder")
        self.trash_btn = QPushButton("Move to trash")
        self.trash_btn.setObjectName("dangerButton")
        property_buttons = QHBoxLayout()
        property_buttons.addWidget(self.save_btn)
        property_buttons.addWidget(self.open_btn)
        property_buttons.addWidget(self.trash_btn)

        property_group = QGroupBox("Selected episode")
        property_layout = QVBoxLayout(property_group)
        property_layout.addWidget(self.selected_label)
        property_layout.addWidget(self.integrity_details)
        property_layout.addLayout(form)
        property_layout.addLayout(property_buttons)

        self.play_btn = QPushButton("Play episode")
        self.play_btn.setObjectName("primaryButton")
        self.pause_btn = QPushButton("Pause")
        self.stop_btn = QPushButton("Stop playback")
        self.speed_combo = QComboBox()
        for label, value in (("0.5×", 0.5), ("1×", 1.0), ("2×", 2.0), ("4×", 4.0)):
            self.speed_combo.addItem(label, value)
        self.speed_combo.setCurrentIndex(1)
        controls = QHBoxLayout()
        controls.addWidget(self.play_btn)
        controls.addWidget(self.pause_btn)
        controls.addWidget(self.stop_btn)
        controls.addWidget(QLabel("Rate"))
        controls.addWidget(self.speed_combo)
        controls.addStretch(1)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.playback_label = QLabel("No episode playing")
        self.playback_label.setWordWrap(True)

        playback_group = QGroupBox("Playback")
        playback_layout = QVBoxLayout(playback_group)
        playback_layout.addLayout(controls)
        playback_layout.addWidget(self.progress)
        playback_layout.addWidget(self.playback_label)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(property_group)
        layout.addWidget(playback_group)
        layout.addStretch(1)
        self.set_editor_enabled(False)
        self.pause_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        return container

    def connect_signals(self) -> None:
        self.bridge.frame.connect(self.on_frame)
        self.bridge.records_ready.connect(self.on_records_ready)
        self.bridge.scan_failed.connect(self.on_scan_failed)
        self.choose_root_btn.clicked.connect(self.choose_data_root)
        self.refresh_btn.clicked.connect(self.refresh_records)
        self.search_edit.textChanged.connect(self.apply_filters)
        self.filter_combo.currentIndexChanged.connect(self.on_filter_changed)
        self.show_trash_check.toggled.connect(self.apply_filters)
        self.table.itemSelectionChanged.connect(self.load_selected_record)
        self.table.itemDoubleClicked.connect(lambda _item: self.play_selected())
        self.save_btn.clicked.connect(self.save_selected)
        self.open_btn.clicked.connect(self.open_selected_folder)
        self.trash_btn.clicked.connect(self.toggle_trash_selected)
        self.play_btn.clicked.connect(self.play_selected)
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.stop_btn.clicked.connect(self.stop_playback)

    def on_frame(self, camera_key: str, image: object) -> None:
        """Display a camera frame delivered from the ROS executor thread."""
        pane = self.camera_panes.get(camera_key)
        if pane is None or not isinstance(image, QImage):
            return
        pane.preview.set_image(image)

    def install_timers(self) -> None:
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(400)
        self.playback_timer = QTimer(self)
        self.playback_timer.timeout.connect(self.refresh_playback)
        self.playback_timer.start(200)

    def restore_splitters(self) -> None:
        for key, splitter in (
            ("mainSplitter", self.main_splitter),
            ("lowerSplitter", self.lower_splitter),
            ("cameraSplitter", self.camera_splitter),
            ("jointSplitter", self.joint_splitter),
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
        self.setFont(QFont(select_font_family(), point_size, QFont.Bold))
        self.table.verticalHeader().setDefaultSectionSize(max(28, point_size * 3))

    def choose_data_root(self) -> None:
        chosen = QFileDialog.getExistingDirectory(
            self, "Choose dataset root", str(self.data_root)
        )
        if not chosen:
            return
        self.stop_playback()
        self.data_root = Path(chosen).expanduser().resolve()
        self.root_label.setText(str(self.data_root))
        self.settings.setValue("lastDataRoot", str(self.data_root))
        self.refresh_records()

    def refresh_records(self) -> None:
        if self.scan_active:
            return
        root = self.data_root
        self.scan_active = True
        self.refresh_btn.setEnabled(False)
        self.choose_root_btn.setEnabled(False)
        self.statusBar().showMessage(f"Scanning {root}…")

        def worker() -> None:
            try:
                records = discover_records(root)
            except Exception as exc:
                self.bridge.scan_failed.emit(str(exc))
                return
            self.bridge.records_ready.emit(records)

        threading.Thread(target=worker, daemon=True).start()

    def on_records_ready(self, records: object) -> None:
        selected_path = str(self.selected_record.root) if self.selected_record else None
        self.scan_active = False
        self.refresh_btn.setEnabled(True)
        self.choose_root_btn.setEnabled(self.playback is None)
        self.records = list(records)
        self.records_by_path = {str(record.root): record for record in self.records}
        self.apply_filters(select_path=selected_path)
        active = sum(not record.in_trash for record in self.records)
        problems = sum(record.is_problem and not record.in_trash for record in self.records)
        trash = sum(record.in_trash for record in self.records)
        self.active_count.setText(f"Active: {active}")
        self.problem_count.setText(f"Problems: {problems}")
        self.trash_count.setText(f"Trash: {trash}")
        self.statusBar().showMessage(
            f"Scanned {len(self.records)} episodes: {problems} active integrity problems"
        )

    def on_scan_failed(self, message: str) -> None:
        self.scan_active = False
        self.refresh_btn.setEnabled(True)
        self.choose_root_btn.setEnabled(True)
        QMessageBox.critical(self, "Dataset scan failed", message)
        self.statusBar().showMessage("Dataset scan failed")

    def on_filter_changed(self) -> None:
        if self.filter_combo.currentData() == "trash" and not self.show_trash_check.isChecked():
            self.show_trash_check.setChecked(True)
            return
        self.apply_filters()

    def record_matches_filter(self, record: EpisodeRecord, selected_filter: str) -> bool:
        if record.in_trash and not self.show_trash_check.isChecked():
            return False
        if selected_filter == "all":
            return True
        if selected_filter == "problems":
            return record.is_problem and not record.in_trash
        if selected_filter == "pass":
            return not record.is_problem and not record.in_trash
        if selected_filter == "short":
            return "SHORT" in record.flags
        if selected_filter == "topics":
            return bool(record.missing or record.empty)
        if selected_filter == "needs_review":
            return record.disposition == "needs_review"
        if selected_filter == "unusable":
            return record.disposition == "unusable"
        if selected_filter == "trash":
            return record.in_trash
        return True

    def apply_filters(self, _unused: object = None, select_path: Optional[str] = None) -> None:
        search = self.search_edit.text().strip().lower()
        selected_filter = str(self.filter_combo.currentData())
        self.filtered_records = [
            record
            for record in self.records
            if self.record_matches_filter(record, selected_filter)
            and (not search or search in record.search_blob)
        ]
        if select_path is None and self.selected_record is not None:
            select_path = str(self.selected_record.root)
        self.populate_table(select_path)
        self.shown_count.setText(f"Shown: {len(self.filtered_records)}")

    def populate_table(self, select_path: Optional[str]) -> None:
        self.table.blockSignals(True)
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(self.filtered_records))
        selected_row = -1
        for row, record in enumerate(self.filtered_records):
            values = (
                record.created_display,
                record.root.name,
                format_duration(record.duration_s),
                ("TRASH · " if record.in_trash else "") + record.integrity,
                display_option(OUTCOME_OPTIONS, record.outcome),
                display_option(DISPOSITION_OPTIONS, record.disposition),
                record.batch,
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setData(Qt.UserRole, str(record.root))
                if record.in_trash:
                    item.setForeground(QColor("#777777"))
                elif record.is_problem:
                    item.setForeground(QColor("#a33b32"))
                self.table.setItem(row, column, item)
            if select_path and str(record.root) == select_path:
                selected_row = row
        self.table.setSortingEnabled(True)
        self.table.blockSignals(False)
        if select_path:
            selected_row = -1
            for row in range(self.table.rowCount()):
                item = self.table.item(row, 0)
                if item is not None and str(item.data(Qt.UserRole)) == select_path:
                    selected_row = row
                    break
        if selected_row >= 0:
            self.table.selectRow(selected_row)
            self.load_selected_record()
        elif self.table.rowCount() > 0 and self.selected_record is None:
            self.table.selectRow(0)
            self.load_selected_record()
        elif self.table.rowCount() == 0:
            self.clear_selection()

    def selected_path_from_table(self) -> Optional[str]:
        row = self.table.currentRow()
        if row < 0:
            return None
        item = self.table.item(row, 0)
        path = item.data(Qt.UserRole) if item is not None else None
        return str(path) if path else None

    def load_selected_record(self) -> None:
        path = self.selected_path_from_table()
        record = self.records_by_path.get(path or "")
        if record is None:
            self.clear_selection()
            return
        self.selected_record = record
        duration = format_duration(record.duration_s)
        trash_text = " · in trash" if record.in_trash else ""
        self.selected_label.setText(
            f"{record.root.name}{trash_text}\n"
            f"Duration: {duration} · Task: {record.task or '—'}\n"
            f"{record.relative_path}"
        )
        set_combo_data(self.outcome_combo, record.outcome)
        set_combo_data(self.disposition_combo, record.disposition)
        self.notes_edit.setPlainText(record.notes)
        self.integrity_details.setPlainText(self.integrity_text(record))
        self.trash_btn.setText("Restore from trash" if record.in_trash else "Move to trash")
        self.trash_btn.setObjectName("warningButton" if record.in_trash else "dangerButton")
        refresh_style(self.trash_btn)
        self.set_editor_enabled(self.playback is None)

    def integrity_text(self, record: EpisodeRecord) -> str:
        lines = [
            f"Integrity: {record.integrity}",
            f"Validation sidecar: {record.validation_status}",
            f"Bag: {record.bag}",
        ]
        if record.duration_s is not None and record.duration_s < MIN_DURATION_SECONDS:
            lines.append(
                f"Duration warning: {record.duration_s:.3f} s is under {MIN_DURATION_SECONDS:g} s"
            )
        if record.missing:
            lines.append("Missing: " + ", ".join(record.missing))
        if record.empty:
            lines.append("Zero messages: " + ", ".join(record.empty))
        if record.metadata_error:
            lines.append("Metadata error: " + record.metadata_error)
        lines.append("")
        lines.append("Required topic counts:")
        for topic in REQUIRED_TOPICS:
            count = record.counts.get(topic)
            lines.append(f"  {topic}: {'missing' if count is None else count}")
        lines.append("Camera-info counts:")
        for topic in CAMERA_INFO_TOPICS:
            count = record.counts.get(topic)
            lines.append(f"  {topic}: {'missing' if count is None else count}")
        if not (record.root / "episode.json").is_file():
            lines.append("\nepisode.json is absent; Save properties will create it without modifying the bag.")
        return "\n".join(lines)

    def clear_selection(self) -> None:
        self.selected_record = None
        self.selected_label.setText("No episode selected")
        self.integrity_details.clear()
        self.notes_edit.clear()
        self.set_editor_enabled(False)

    def set_editor_enabled(self, enabled: bool) -> None:
        has_record = self.selected_record is not None
        effective = enabled and has_record
        for widget in (
            self.outcome_combo,
            self.disposition_combo,
            self.notes_edit,
            self.save_btn,
            self.open_btn,
            self.trash_btn,
            self.play_btn,
            self.speed_combo,
        ):
            widget.setEnabled(effective)

    def save_selected(self) -> None:
        record = self.selected_record
        if record is None or self.playback is not None:
            return
        disposition = str(self.disposition_combo.currentData())
        if disposition == "usable" and record.is_problem:
            answer = QMessageBox.warning(
                self,
                "Integrity problem",
                f"This episode is marked {record.integrity}. Save it as usable anyway?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        path = record.root / "episode.json"
        try:
            payload = read_json(path) if path.is_file() else {}
            payload.update(
                {
                    "schema_version": str(payload.get("schema_version", "1.1")),
                    "episode_id": str(payload.get("episode_id", record.root.name)),
                    "created_at": str(payload.get("created_at", record.created_iso)),
                    "task": str(payload.get("task", record.task)),
                    "duration_seconds": (
                        record.duration_s
                        if record.duration_s is not None
                        else payload.get("duration_seconds", 0.0)
                    ),
                    "source_mcap_immutable": True,
                    "outcome": str(self.outcome_combo.currentData()),
                    "operator_disposition": disposition,
                    "notes": self.notes_edit.toPlainText().strip(),
                    "last_edited_at": utc_now_iso(),
                }
            )
            write_json_atomic(path, payload)
        except Exception as exc:
            QMessageBox.critical(self, "Could not save properties", str(exc))
            return
        self.statusBar().showMessage("Episode properties saved")
        self.refresh_records()

    def open_selected_folder(self) -> None:
        record = self.selected_record
        if record is None:
            return
        try:
            subprocess.Popen(
                ["xdg-open", str(record.root)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            QMessageBox.warning(self, "Could not open folder", str(exc))

    def toggle_trash_selected(self) -> None:
        record = self.selected_record
        if record is None or self.playback is not None:
            return
        if record.in_trash:
            self.restore_selected(record)
        else:
            self.move_selected_to_trash(record)

    def verify_episode_for_move(self, record: EpisodeRecord) -> None:
        if not path_within(record.root, self.data_root):
            raise ValueError("Refusing to move an episode outside the selected dataset root")
        if not record.metadata_path.is_file():
            raise ValueError("Refusing to move an episode whose metadata.yaml is missing")
        if record.root == self.data_root.resolve():
            raise ValueError("Refusing to move the dataset root")

    def move_selected_to_trash(self, record: EpisodeRecord) -> None:
        answer = QMessageBox.warning(
            self,
            "Move episode to trash?",
            f"Move the full episode directory into:\n\n{record.root.parent / '.trash'}\n\n"
            f"{record.root.name}\n\nThe MCAP is not permanently deleted.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self.verify_episode_for_move(record)
            trash_root = record.root.parent / ".trash"
            trash_root.mkdir(parents=True, exist_ok=True)
            destination = trash_root / record.root.name
            if destination.exists():
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                destination = trash_root / f"{record.root.name}_{stamp}"
            shutil.move(str(record.root), str(destination))
        except Exception as exc:
            QMessageBox.critical(self, "Could not move episode", str(exc))
            return
        self.selected_record = None
        self.statusBar().showMessage(f"Moved to {destination}")
        self.refresh_records()

    def restore_selected(self, record: EpisodeRecord) -> None:
        trash_parent = record.root.parent
        if trash_parent.name != ".trash":
            QMessageBox.critical(
                self,
                "Could not restore episode",
                "This GUI only restores episodes directly inside a sibling .trash directory.",
            )
            return
        destination = trash_parent.parent / record.root.name
        answer = QMessageBox.question(
            self,
            "Restore episode?",
            f"Restore to:\n\n{destination}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self.verify_episode_for_move(record)
            if destination.exists():
                raise FileExistsError(f"Restore destination already exists: {destination}")
            shutil.move(str(record.root), str(destination))
        except Exception as exc:
            QMessageBox.critical(self, "Could not restore episode", str(exc))
            return
        self.selected_record = None
        self.statusBar().showMessage(f"Restored to {destination}")
        self.refresh_records()

    def playback_command(self, record: EpisodeRecord, rate: float) -> list[str]:
        topics = sorted(topic for topic in record.counts if topic.startswith("/"))
        if not topics:
            topics = list(REQUIRED_TOPICS) + list(CAMERA_INFO_TOPICS)
        remaps = [f"{topic}:={viewer_topic(topic)}" for topic in topics]
        return [
            "ros2",
            "bag",
            "play",
            str(record.bag),
            "--rate",
            str(rate),
            "--disable-keyboard-controls",
            "--remap",
            *remaps,
        ]

    def play_selected(self) -> None:
        record = self.selected_record
        if record is None or self.playback is not None:
            return
        if not record.bag.is_dir():
            QMessageBox.warning(self, "Bag missing", str(record.bag))
            return
        CACHE_ROOT.mkdir(parents=True, exist_ok=True)
        log_path = CACHE_ROOT / f"playback-{datetime.now():%Y%m%d-%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8", buffering=1)
        rate = float(self.speed_combo.currentData())
        command = self.playback_command(record, rate)
        self.node.clear()
        for pane in self.camera_panes.values():
            pane.preview.clear_image("Waiting for episode frames")
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
        self.playback = ManagedPlayback(
            process=process,
            log_handle=log_handle,
            log_path=log_path,
            record=record,
            rate=rate,
            started_at=time.monotonic(),
        )
        self.set_editor_enabled(False)
        self.table.setEnabled(False)
        self.search_edit.setEnabled(False)
        self.filter_combo.setEnabled(False)
        self.show_trash_check.setEnabled(False)
        self.choose_root_btn.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        self.stop_btn.setEnabled(True)
        self.progress.setValue(0)
        self.statusBar().showMessage(f"Playing {record.root.name}")

    def toggle_pause(self) -> None:
        playback = self.playback
        if playback is None or playback.process.poll() is not None:
            return
        try:
            if playback.paused:
                os.killpg(playback.process.pid, signal.SIGCONT)
                playback.paused_total += time.monotonic() - playback.pause_started_at
                playback.pause_started_at = 0.0
                playback.paused = False
                self.pause_btn.setText("Pause")
                self.statusBar().showMessage("Playback resumed")
            else:
                os.killpg(playback.process.pid, signal.SIGSTOP)
                playback.pause_started_at = time.monotonic()
                playback.paused = True
                self.pause_btn.setText("Resume")
                self.statusBar().showMessage("Playback paused")
        except ProcessLookupError:
            pass

    def stop_playback(self) -> None:
        playback = self.playback
        if playback is None:
            return
        if playback.paused:
            try:
                os.killpg(playback.process.pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
            playback.paused = False
        if playback.process.poll() is None:
            try:
                os.killpg(playback.process.pid, signal.SIGINT)
                playback.stop_requested_at = time.monotonic()
            except ProcessLookupError:
                pass
        self.pause_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping playback…")

    def refresh_playback(self) -> None:
        playback = self.playback
        if playback is None:
            return
        code = playback.process.poll()
        if code is not None:
            self.finish_playback(code)
            return
        if playback.stop_requested_at:
            elapsed_since_stop = time.monotonic() - playback.stop_requested_at
            try:
                if elapsed_since_stop > 10.0:
                    os.killpg(playback.process.pid, signal.SIGKILL)
                elif elapsed_since_stop > 6.0:
                    os.killpg(playback.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        now = playback.pause_started_at if playback.paused else time.monotonic()
        wall_elapsed = max(0.0, now - playback.started_at - playback.paused_total)
        episode_elapsed = wall_elapsed * playback.rate
        duration = playback.record.duration_s or 0.0
        if duration > 0.0:
            self.progress.setValue(min(1000, int(1000.0 * episode_elapsed / duration)))
        else:
            self.progress.setValue(0)
        paused_text = " · paused" if playback.paused else ""
        self.playback_label.setText(
            f"{playback.record.root.name}\n"
            f"{format_clock(episode_elapsed)} / {format_clock(duration)} at {playback.rate:g}×{paused_text}"
        )

    def finish_playback(self, code: int) -> None:
        playback = self.playback
        if playback is None:
            return
        try:
            playback.log_handle.close()
        except Exception:
            pass
        log_path = playback.log_path
        self.playback = None
        self.node.clear()
        self.pause_btn.setText("Pause")
        self.pause_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.progress.setValue(1000 if code == 0 else self.progress.value())
        self.playback_label.setText("Playback finished" if code == 0 else f"Playback exited with {code}")
        self.table.setEnabled(True)
        self.search_edit.setEnabled(True)
        self.filter_combo.setEnabled(True)
        self.show_trash_check.setEnabled(True)
        self.choose_root_btn.setEnabled(True)
        self.refresh_btn.setEnabled(True)
        self.set_editor_enabled(True)
        if code in (0, -signal.SIGINT):
            self.statusBar().showMessage("Playback finished")
        else:
            self.statusBar().showMessage(f"Playback exited with {code}; log: {log_path}")

    def refresh_health(self) -> None:
        snapshot = self.node.snapshot()
        playing = self.playback is not None and self.playback.process.poll() is None
        for key, pane in self.camera_panes.items():
            value = snapshot["cameras"].get(key, {})
            pane.update_health(
                float(value.get("rate", 0.0)),
                float(value.get("age", float("inf"))),
                playing,
            )
        for side, pane in self.joint_panes.items():
            pane.update_observation(snapshot["joints"].get(side, {}), playing)

    @staticmethod
    def stop_process_sync(playback: ManagedPlayback) -> None:
        if playback.process.poll() is not None:
            return
        try:
            os.killpg(playback.process.pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        try:
            os.killpg(playback.process.pid, signal.SIGINT)
            playback.process.wait(timeout=7.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(playback.process.pid, signal.SIGTERM)
                playback.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(playback.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.playback is not None:
            self.stop_process_sync(self.playback)
            try:
                self.playback.log_handle.close()
            except Exception:
                pass
            self.playback = None
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("mainSplitter", self.main_splitter.saveState())
        self.settings.setValue("lowerSplitter", self.lower_splitter.saveState())
        self.settings.setValue("cameraSplitter", self.camera_splitter.saveState())
        self.settings.setValue("jointSplitter", self.joint_splitter.saveState())
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
        QLabel[summary="true"] { padding: 0.3em 0.55em; background: #e7edf2; }
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
    rclpy.init()
    bridge = Bridge()
    node = PlaybackMonitor(bridge)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    apply_application_style(app)
    window = Window(node, bridge)
    saved_root = window.settings.value("lastDataRoot")
    if saved_root and Path(str(saved_root)).expanduser().is_dir():
        window.data_root = Path(str(saved_root)).expanduser().resolve()
        window.root_label.setText(str(window.data_root))
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
