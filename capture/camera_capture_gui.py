#!/usr/bin/env python3
"""Laundry Butler three-camera preview, snapshot, and MCAP capture GUI."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parent
OUTPUT_ROOT = Path(
    os.environ.get("LAUNDRY_BUTLER_CAPTURE_ROOT", str(APP_DIR / "output"))
).expanduser()
CAMERA_LAUNCH_FILE = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_CAMERA_LAUNCH",
        str(REPO_ROOT / "launch" / "multi_camera_rgb.launch.py"),
    )
).expanduser()

PREVIEW_MAX_FPS = 10.0
RATE_WINDOW_SECONDS = 2.5
EXPECTED_CAMERA_NODES = {
    "/camera_f/camera_f",
    "/camera_l/camera_l",
    "/camera_r/camera_r",
}


@dataclass(frozen=True)
class CameraSpec:
    key: str
    title: str
    image_topic: str
    info_topic: str


CAMERAS = (
    CameraSpec(
        "camera_l",
        "Left",
        "/camera_l/color/image_raw",
        "/camera_l/color/camera_info",
    ),
    CameraSpec(
        "camera_f",
        "Front",
        "/camera_f/color/image_raw",
        "/camera_f/color/camera_info",
    ),
    CameraSpec(
        "camera_r",
        "Right",
        "/camera_r/color/image_raw",
        "/camera_r/color/camera_info",
    ),
)


class Bridge(QObject):
    frame = pyqtSignal(str, object)
    error = pyqtSignal(str, str)


class PreviewNode(Node):
    def __init__(self, bridge: Bridge) -> None:
        super().__init__("laundry_butler_camera_capture_gui")
        self.bridge = bridge
        self.lock = threading.Lock()

        self.stream_arrivals = {spec.key: deque() for spec in CAMERAS}
        self.view_arrivals = {spec.key: deque() for spec in CAMERAS}
        self.last_stream = {spec.key: 0.0 for spec in CAMERAS}
        self.last_preview = {spec.key: 0.0 for spec in CAMERAS}
        self.last_error = {spec.key: "" for spec in CAMERAS}

        image_group = ReentrantCallbackGroup()
        info_group = ReentrantCallbackGroup()
        self._subscriptions = []

        for spec in CAMERAS:
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    spec.image_topic,
                    self._image_callback(spec.key),
                    qos_profile_sensor_data,
                    callback_group=image_group,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    CameraInfo,
                    spec.info_topic,
                    self._info_callback(spec.key),
                    qos_profile_sensor_data,
                    callback_group=info_group,
                )
            )

    def _image_callback(self, key: str):
        def callback(message: Image) -> None:
            now = time.monotonic()

            with self.lock:
                if now - self.last_preview[key] < 1.0 / PREVIEW_MAX_FPS:
                    return
                self.last_preview[key] = now

            try:
                image = to_qimage(message)
            except ValueError as exc:
                error = str(exc)
                with self.lock:
                    if self.last_error[key] == error:
                        return
                    self.last_error[key] = error
                self.bridge.error.emit(key, error)
                return

            emitted_at = time.monotonic()
            with self.lock:
                arrivals = self.view_arrivals[key]
                arrivals.append(emitted_at)
                trim_arrivals(arrivals, emitted_at)

            self.bridge.frame.emit(key, image)

        return callback

    def _info_callback(self, key: str):
        def callback(_message: CameraInfo) -> None:
            now = time.monotonic()
            with self.lock:
                arrivals = self.stream_arrivals[key]
                arrivals.append(now)
                self.last_stream[key] = now
                trim_arrivals(arrivals, now)

        return callback

    def rates(self) -> dict[str, tuple[float, float, float]]:
        now = time.monotonic()
        result: dict[str, tuple[float, float, float]] = {}

        with self.lock:
            for key in self.stream_arrivals:
                stream_arrivals = self.stream_arrivals[key]
                view_arrivals = self.view_arrivals[key]
                trim_arrivals(stream_arrivals, now)
                trim_arrivals(view_arrivals, now)

                stream_rate = calculate_rate(stream_arrivals)
                view_rate = calculate_rate(view_arrivals)
                age = (
                    now - self.last_stream[key]
                    if self.last_stream[key] > 0.0
                    else float("inf")
                )
                result[key] = (stream_rate, view_rate, age)

        return result


def trim_arrivals(arrivals: deque[float], now: float) -> None:
    cutoff = now - RATE_WINDOW_SECONDS
    while arrivals and arrivals[0] < cutoff:
        arrivals.popleft()


def calculate_rate(arrivals: deque[float]) -> float:
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
        raise ValueError(
            f"Could not display {message.width}x{message.height} "
            f"{message.encoding}"
        )
    return image.copy()


class ImageLabel(QLabel):
    def __init__(self) -> None:
        super().__init__("Waiting for camera")
        self.source: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(
            "background:#111;color:#aaa;border:1px solid #444"
        )

    def set_image(self, image: QImage) -> None:
        self.source = QPixmap.fromImage(image)
        self._scale()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._scale()

    def _scale(self) -> None:
        if self.source is None:
            return
        self.setPixmap(
            self.source.scaled(
                self.size(),
                Qt.KeepAspectRatio,
                Qt.FastTransformation,
            )
        )


class CameraPane(QFrame):
    def __init__(self, spec: CameraSpec) -> None:
        super().__init__()
        self.spec = spec
        self.latest: Optional[QImage] = None
        self.setFrameShape(QFrame.StyledPanel)

        title = QLabel(spec.title)
        title.setStyleSheet("font-size:17px;font-weight:600")
        self.checkbox = QCheckBox("Record")
        self.checkbox.setChecked(True)

        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.checkbox)

        self.image = ImageLabel()
        self.rate = QLabel("No signal")
        self.rate.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addLayout(header)
        layout.addWidget(self.image, 1)
        layout.addWidget(self.rate)

    def update_image(self, image: QImage) -> None:
        self.latest = image
        self.image.set_image(image)

    def update_rate(
        self,
        stream_rate: float,
        view_rate: float,
        age: float,
    ) -> None:
        if age > 3.0:
            self.rate.setText("No signal")
            self.rate.setStyleSheet("color:#b44")
            return

        self.rate.setText(
            f"Stream {stream_rate:.1f} fps | View {view_rate:.1f} fps"
        )
        self.rate.setStyleSheet("color:#275")


class Window(QMainWindow):
    def __init__(self, node: PreviewNode, bridge: Bridge) -> None:
        super().__init__()
        self.node = node

        self.recorder_proc: Optional[subprocess.Popen] = None
        self.recorder_log = None
        self.session_dir: Optional[Path] = None
        self.recording_keys: tuple[str, ...] = ()
        self.restart_after_stop = False
        self.recorder_stop_requested_at = 0.0

        self.camera_proc: Optional[subprocess.Popen] = None
        self.camera_log = None
        self.camera_log_path: Optional[Path] = None
        self.camera_stop_requested_at = 0.0
        self.detected_camera_nodes: set[str] = set()

        self.setWindowTitle("Laundry Butler Camera Capture")
        self.resize(1500, 700)
        self.panes = {spec.key: CameraPane(spec) for spec in CAMERAS}

        previews = QHBoxLayout()
        for spec in CAMERAS:
            previews.addWidget(self.panes[spec.key], 1)

        self.start_cameras_btn = QPushButton("Start Cameras")
        self.stop_cameras_btn = QPushButton("Stop Cameras")
        self.start_recording_btn = QPushButton("Start MCAP")
        self.stop_recording_btn = QPushButton("Stop MCAP")
        self.snapshot_btn = QPushButton("Take Snapshot")

        self.stop_cameras_btn.setEnabled(False)
        self.stop_recording_btn.setEnabled(False)

        self.start_cameras_btn.clicked.connect(self.start_cameras)
        self.stop_cameras_btn.clicked.connect(self.stop_cameras)
        self.start_recording_btn.clicked.connect(self.start_recording)
        self.stop_recording_btn.clicked.connect(
            lambda: self.request_recorder_stop(False)
        )
        self.snapshot_btn.clicked.connect(self.snapshot)

        controls = QHBoxLayout()
        controls.addWidget(self.start_cameras_btn)
        controls.addWidget(self.stop_cameras_btn)
        controls.addSpacing(20)
        controls.addWidget(self.start_recording_btn)
        controls.addWidget(self.stop_recording_btn)
        controls.addWidget(self.snapshot_btn)
        controls.addStretch(1)

        self.camera_status_label = QLabel("Cameras: checking…")
        self.output_label = QLabel(f"Output: {OUTPUT_ROOT}")
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        body = QVBoxLayout()
        body.addLayout(previews, 1)
        body.addLayout(controls)
        body.addWidget(self.camera_status_label)
        body.addWidget(self.output_label)

        central = QWidget()
        central.setLayout(body)
        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar())

        bridge.frame.connect(
            lambda key, image: self.panes[key].update_image(image)
        )
        bridge.error.connect(
            lambda key, error: self.statusBar().showMessage(
                f"{key}: {error}", 5000
            )
        )

        self.rate_timer = QTimer(self)
        self.rate_timer.timeout.connect(self.refresh_rates)
        self.rate_timer.start(2000)

        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.check_processes)
        self.process_timer.start(250)

        self.node_timer = QTimer(self)
        self.node_timer.timeout.connect(self.refresh_camera_node_status)
        self.node_timer.start(3000)

        self.segment_timer = QTimer(self)
        self.segment_timer.setSingleShot(True)
        self.segment_timer.timeout.connect(
            lambda: self.request_recorder_stop(True)
        )

        QTimer.singleShot(100, self.refresh_camera_node_status)

    def selected(self) -> tuple[str, ...]:
        return tuple(
            spec.key
            for spec in CAMERAS
            if self.panes[spec.key].checkbox.isChecked()
        )

    def refresh_rates(self) -> None:
        live = False
        for key, (stream_rate, view_rate, age) in self.node.rates().items():
            self.panes[key].update_rate(stream_rate, view_rate, age)
            live = live or age <= 3.0

        if self.recorder_proc is None:
            self.statusBar().showMessage(
                "Camera topics active" if live else "Waiting for camera topics"
            )

    def camera_nodes(self) -> set[str]:
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

    def refresh_camera_node_status(self) -> None:
        self.detected_camera_nodes = self.camera_nodes() & EXPECTED_CAMERA_NODES
        count = len(self.detected_camera_nodes)
        managed = self.camera_proc is not None and self.camera_proc.poll() is None

        if count == len(EXPECTED_CAMERA_NODES):
            suffix = " (started here)" if managed else " (external)"
            self.camera_status_label.setText("Cameras: running" + suffix)
            self.start_cameras_btn.setEnabled(False)
            self.stop_cameras_btn.setEnabled(managed)
        elif count == 0:
            self.camera_status_label.setText("Cameras: stopped")
            self.start_cameras_btn.setEnabled(not managed)
            self.stop_cameras_btn.setEnabled(managed)
        else:
            self.camera_status_label.setText(
                f"Cameras: partial ({count}/3); resolve before starting"
            )
            self.start_cameras_btn.setEnabled(False)
            self.stop_cameras_btn.setEnabled(managed)

    def start_cameras(self) -> None:
        if self.camera_proc is not None and self.camera_proc.poll() is None:
            return

        existing = self.camera_nodes() & EXPECTED_CAMERA_NODES
        if existing:
            QMessageBox.warning(
                self,
                "Camera nodes already present",
                "One or more camera nodes are already running. Stop the "
                "existing launch first to avoid duplicate nodes.\n\n"
                + "\n".join(sorted(existing)),
            )
            self.refresh_camera_node_status()
            return

        if not CAMERA_LAUNCH_FILE.is_file():
            QMessageBox.critical(
                self,
                "Launch file missing",
                f"Camera launch file not found:\n{CAMERA_LAUNCH_FILE}",
            )
            return

        runtime_dir = OUTPUT_ROOT / "runtime"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        self.camera_log_path = runtime_dir / (
            f"camera-launch-{datetime.now():%Y%m%d-%H%M%S}.log"
        )
        self.camera_log = self.camera_log_path.open(
            "w", encoding="utf-8", buffering=1
        )

        command = ["ros2", "launch", str(CAMERA_LAUNCH_FILE)]
        try:
            self.camera_proc = subprocess.Popen(
                command,
                stdout=self.camera_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.camera_log.close()
            self.camera_log = None
            self.camera_log_path = None
            QMessageBox.critical(
                self,
                "Could not start cameras",
                str(exc),
            )
            return

        self.camera_stop_requested_at = 0.0
        self.start_cameras_btn.setEnabled(False)
        self.stop_cameras_btn.setEnabled(True)
        self.camera_status_label.setText("Cameras: starting…")
        self.statusBar().showMessage("Starting camera nodes")

    def stop_cameras(self) -> None:
        if self.recorder_proc is not None:
            QMessageBox.warning(
                self,
                "Recording active",
                "Stop the MCAP recording before stopping camera nodes.",
            )
            return

        if self.camera_proc is None or self.camera_proc.poll() is not None:
            QMessageBox.information(
                self,
                "Cameras not managed here",
                "The visible camera nodes were not started by this interface. "
                "Stop them from their launch terminal.",
            )
            self.refresh_camera_node_status()
            return

        self.camera_stop_requested_at = time.monotonic()
        try:
            os.killpg(self.camera_proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        self.stop_cameras_btn.setEnabled(False)
        self.camera_status_label.setText("Cameras: stopping…")
        self.statusBar().showMessage("Stopping camera nodes")

    def start_recording(self) -> None:
        if self.recorder_proc is not None:
            return

        keys = self.selected()
        if not keys:
            QMessageBox.warning(
                self,
                "No cameras selected",
                "Select at least one camera.",
            )
            return

        missing = EXPECTED_CAMERA_NODES - self.camera_nodes()
        if missing:
            QMessageBox.warning(
                self,
                "Camera nodes missing",
                "Start all camera nodes before recording. Missing:\n\n"
                + "\n".join(sorted(missing)),
            )
            self.refresh_camera_node_status()
            return

        self._start_recorder(keys)

    def _start_recorder(self, keys: tuple[str, ...]) -> None:
        now = datetime.now()
        start, end = bucket(now)
        base = (
            OUTPUT_ROOT
            / start.strftime("%Y%m%d")
            / f"{start:%H%M}-{end:%H%M}"
        )
        session = base / f"recording-{now:%H%M%S}"
        bag = session / "bag"
        session.mkdir(parents=True, exist_ok=False)

        topics: list[str] = []
        for spec in CAMERAS:
            if spec.key in keys:
                topics.extend([spec.image_topic, spec.info_topic])

        command = [
            "ros2",
            "bag",
            "record",
            "-s",
            "mcap",
            "-o",
            str(bag),
            "--topics",
            *topics,
        ]

        self.recorder_log = (session / "rosbag.log").open(
            "w", encoding="utf-8", buffering=1
        )
        self.recorder_proc = subprocess.Popen(
            command,
            stdout=self.recorder_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

        self.session_dir = session
        self.recording_keys = keys
        self.start_recording_btn.setEnabled(False)
        self.stop_recording_btn.setEnabled(True)
        self.start_cameras_btn.setEnabled(False)
        self.stop_cameras_btn.setEnabled(False)
        for pane in self.panes.values():
            pane.checkbox.setEnabled(False)

        self.output_label.setText(f"Recording: {session}")
        self.statusBar().showMessage("Recording " + ", ".join(keys))
        self.segment_timer.start(
            max(1000, int((end - now).total_seconds() * 1000))
        )

    def request_recorder_stop(self, restart: bool) -> None:
        if self.recorder_proc is None:
            return

        self.restart_after_stop = restart
        self.recorder_stop_requested_at = time.monotonic()
        self.segment_timer.stop()
        try:
            os.killpg(self.recorder_proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

        self.stop_recording_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping recorder…")

    def check_processes(self) -> None:
        self.check_recorder_process()
        self.check_camera_process()

    def check_recorder_process(self) -> None:
        if self.recorder_proc is None:
            return

        code = self.recorder_proc.poll()
        if code is not None:
            restart = self.restart_after_stop
            keys = self.recording_keys
            saved = self.session_dir

            if self.recorder_log is not None:
                self.recorder_log.close()

            self.recorder_proc = None
            self.recorder_log = None
            self.restart_after_stop = False
            self.recorder_stop_requested_at = 0.0
            self.start_recording_btn.setEnabled(True)
            self.stop_recording_btn.setEnabled(False)
            for pane in self.panes.values():
                pane.checkbox.setEnabled(True)
            self.output_label.setText(f"Output: {OUTPUT_ROOT}")

            if restart and keys:
                self._start_recorder(keys)
            else:
                self.statusBar().showMessage(f"Recording saved: {saved}")
                self.refresh_camera_node_status()
            return

        self.escalate_stop(
            self.recorder_proc,
            self.recorder_stop_requested_at,
        )

    def check_camera_process(self) -> None:
        if self.camera_proc is None:
            return

        code = self.camera_proc.poll()
        if code is not None:
            log_path = self.camera_log_path
            if self.camera_log is not None:
                self.camera_log.close()

            self.camera_proc = None
            self.camera_log = None
            self.camera_log_path = None
            self.camera_stop_requested_at = 0.0
            self.refresh_camera_node_status()

            if code not in (0, -signal.SIGINT):
                self.statusBar().showMessage(
                    f"Camera launch exited with code {code}; log: {log_path}"
                )
            else:
                self.statusBar().showMessage("Camera nodes stopped")
            return

        self.escalate_stop(
            self.camera_proc,
            self.camera_stop_requested_at,
        )

    @staticmethod
    def escalate_stop(
        process: subprocess.Popen,
        requested_at: float,
    ) -> None:
        if requested_at <= 0.0:
            return

        elapsed = time.monotonic() - requested_at
        try:
            if elapsed > 10.0:
                os.killpg(process.pid, signal.SIGKILL)
            elif elapsed > 5.0:
                os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def snapshot(self) -> None:
        keys = self.selected()
        if not keys:
            QMessageBox.warning(
                self,
                "No cameras selected",
                "Select at least one camera.",
            )
            return

        now = datetime.now()
        if self.recorder_proc is not None and self.session_dir is not None:
            output = self.session_dir / "snapshots"
        else:
            start, end = bucket(now)
            output = (
                OUTPUT_ROOT
                / start.strftime("%Y%m%d")
                / f"{start:%H%M}-{end:%H%M}"
                / "snapshots"
            )

        output.mkdir(parents=True, exist_ok=True)
        stamp = now.strftime("%Y%m%d-%H%M%S-%f")[:-3]
        missing: list[str] = []
        saved = 0

        for key in keys:
            image = self.panes[key].latest
            if image is None:
                missing.append(key)
                continue
            if image.save(str(output / f"{stamp}_{key}.png"), "PNG"):
                saved += 1
            else:
                missing.append(key)

        self.statusBar().showMessage(
            f"Saved {saved} snapshot(s) to {output}", 5000
        )
        if missing:
            QMessageBox.warning(
                self,
                "Snapshot incomplete",
                "No image for: " + ", ".join(missing),
            )

    @staticmethod
    def stop_process_sync(process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.recorder_proc is not None:
            answer = QMessageBox.question(
                self,
                "Stop recording?",
                "A recording is active. Stop it and close?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return

        if self.recorder_proc is not None:
            self.stop_process_sync(self.recorder_proc)
        if self.camera_proc is not None:
            self.stop_process_sync(self.camera_proc)

        if self.recorder_log is not None:
            self.recorder_log.close()
        if self.camera_log is not None:
            self.camera_log.close()

        event.accept()


def bucket(now: datetime) -> tuple[datetime, datetime]:
    start = now.replace(
        minute=0 if now.minute < 30 else 30,
        second=0,
        microsecond=0,
    )
    return start, start + timedelta(minutes=30)


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    bridge = Bridge()
    node = PreviewNode(bridge)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    window = Window(node, bridge)
    window.show()

    try:
        return int(app.exec_())
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
