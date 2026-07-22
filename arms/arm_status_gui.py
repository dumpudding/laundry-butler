#!/usr/bin/env python3
"""Observation-only Piper arm status and MCAP recording interface."""

from __future__ import annotations

import os
import re
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
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperStatusMsg
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


APP_DIR = Path(__file__).resolve().parent

OUTPUT_ROOT = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_ARM_ROOT",
        str(APP_DIR / "output"),
    )
).expanduser()

ARM_LAUNCH_FILE = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_ARM_LAUNCH",
        str(APP_DIR / "dual_arm_observe.launch.py"),
    )
).expanduser()

RATE_WINDOW_SECONDS = 2.5
STALE_AFTER_SECONDS = 1.0

EXPECTED_ARM_NODES = {
    "/piper_left_ctrl_node",
    "/piper_right_ctrl_node",
}

CONTROL_TOPICS = {
    "/laundry_butler/observation_only/left/pos_cmd",
    "/laundry_butler/observation_only/left/joint_ctrl_single",
    "/laundry_butler/observation_only/left/enable_flag",
    "/laundry_butler/observation_only/right/pos_cmd",
    "/laundry_butler/observation_only/right/joint_ctrl_single",
    "/laundry_butler/observation_only/right/enable_flag",
}

ISOLATED_ENABLE_SERVICES = {
    "/laundry_butler/observation_only/left/enable_srv",
    "/laundry_butler/observation_only/right/enable_srv",
}

DEFAULT_ENABLE_SERVICES = {
    "/piper_left_ctrl_node/enable_srv",
    "/piper_right_ctrl_node/enable_srv",
}


@dataclass(frozen=True)
class ArmSpec:
    key: str
    title: str
    can_interface: str
    joint_topic: str
    pose_topic: str
    status_topic: str


ARMS = (
    ArmSpec(
        key="left",
        title="Left Arm",
        can_interface="can_left",
        joint_topic="/puppet/joint_left",
        pose_topic="/puppet/end_pose_left",
        status_topic="/piper_left_ctrl_node/arm_status",
    ),
    ArmSpec(
        key="right",
        title="Right Arm",
        can_interface="can_right",
        joint_topic="/puppet/joint_right",
        pose_topic="/puppet/end_pose_right",
        status_topic="/piper_right_ctrl_node/arm_status",
    ),
)


def trim_arrivals(arrivals: deque[float], now: float) -> None:
    cutoff = now - RATE_WINDOW_SECONDS
    while arrivals and arrivals[0] < cutoff:
        arrivals.popleft()


def calculate_rate(arrivals: deque[float]) -> float:
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        return 0.0
    return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])


def full_node_name(name: str, namespace: str) -> str:
    if namespace == "/":
        return f"/{name}"
    return f"{namespace.rstrip('/')}/{name}"


def half_hour_bucket(now: datetime) -> tuple[datetime, datetime]:
    start_minute = 0 if now.minute < 30 else 30
    start = now.replace(minute=start_minute, second=0, microsecond=0)
    return start, start + timedelta(minutes=30)


def session_directory(now: datetime) -> Path:
    start, end = half_hour_bucket(now)
    bucket = f"{start:%H%M}-{end:%H%M}"
    return (
        OUTPUT_ROOT
        / f"{now:%Y%m%d}"
        / bucket
        / f"recording-{now:%H%M%S}"
    )


def seconds_until_next_bucket(now: datetime) -> int:
    _start, end = half_hour_bucket(now)
    return max(1, int((end - now).total_seconds()))


def signal_process_group(
    process: Optional[subprocess.Popen],
    sig: signal.Signals,
) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        return


def list_ros_nodes() -> set[str]:
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

    return {
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    }


def current_topic_names() -> set[str]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()

    return {
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    }


def parse_can_status(interface: str) -> dict[str, object]:
    try:
        result = subprocess.run(
            [
                "ip",
                "-details",
                "-statistics",
                "link",
                "show",
                "dev",
                interface,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "error": str(exc)}

    if result.returncode != 0:
        return {
            "available": False,
            "error": result.stderr.strip() or "Interface not found",
        }

    text = result.stdout
    first_line = text.splitlines()[0] if text.splitlines() else ""

    link_state_match = re.search(r"\bstate\s+(\S+)", first_line)
    can_state_match = re.search(r"\bcan state\s+(\S+)", text)
    bitrate_match = re.search(r"\bbitrate\s+(\d+)", text)

    rx_match = re.search(
        r"RX:\s+bytes\s+packets\s+errors\s+dropped\s+missed\s+mcast\s*\n"
        r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        text,
    )
    tx_match = re.search(
        r"TX:\s+bytes\s+packets\s+errors\s+dropped\s+carrier\s+collsns\s*\n"
        r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        text,
    )

    return {
        "available": True,
        "link_state": link_state_match.group(1) if link_state_match else "?",
        "can_state": can_state_match.group(1) if can_state_match else "?",
        "bitrate": int(bitrate_match.group(1)) if bitrate_match else 0,
        "rx_packets": int(rx_match.group(2)) if rx_match else 0,
        "rx_errors": int(rx_match.group(3)) if rx_match else 0,
        "rx_dropped": int(rx_match.group(4)) if rx_match else 0,
        "tx_packets": int(tx_match.group(2)) if tx_match else 0,
        "tx_errors": int(tx_match.group(3)) if tx_match else 0,
        "tx_dropped": int(tx_match.group(4)) if tx_match else 0,
    }


class ArmMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("laundry_butler_arm_status_gui")

        self.lock = threading.Lock()
        self.arrivals = {spec.key: deque() for spec in ARMS}
        self.last_joint = {spec.key: 0.0 for spec in ARMS}
        self.joints: dict[str, Optional[dict[str, object]]] = {
            spec.key: None for spec in ARMS
        }
        self.poses: dict[str, Optional[dict[str, object]]] = {
            spec.key: None for spec in ARMS
        }
        self.statuses: dict[str, Optional[dict[str, int]]] = {
            spec.key: None for spec in ARMS
        }

        self._subscriptions = []

        for spec in ARMS:
            self._subscriptions.extend(
                [
                    self.create_subscription(
                        JointState,
                        spec.joint_topic,
                        self._joint_callback(spec.key),
                        qos_profile_sensor_data,
                    ),
                    self.create_subscription(
                        PoseStamped,
                        spec.pose_topic,
                        self._pose_callback(spec.key),
                        qos_profile_sensor_data,
                    ),
                    self.create_subscription(
                        PiperStatusMsg,
                        spec.status_topic,
                        self._status_callback(spec.key),
                        qos_profile_sensor_data,
                    ),
                ]
            )

    def _joint_callback(self, key: str):
        def callback(message: JointState) -> None:
            now = time.monotonic()

            positions = tuple(float(value) for value in message.position)
            efforts = tuple(float(value) for value in message.effort)

            with self.lock:
                arrivals = self.arrivals[key]
                arrivals.append(now)
                trim_arrivals(arrivals, now)
                self.last_joint[key] = now
                self.joints[key] = {
                    "names": tuple(message.name),
                    "positions": positions,
                    "efforts": efforts,
                }

        return callback

    def _pose_callback(self, key: str):
        def callback(message: PoseStamped) -> None:
            pose = message.pose

            with self.lock:
                self.poses[key] = {
                    "position": (
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                    ),
                    "orientation": (
                        float(pose.orientation.x),
                        float(pose.orientation.y),
                        float(pose.orientation.z),
                        float(pose.orientation.w),
                    ),
                }

        return callback

    def _status_callback(self, key: str):
        def callback(message: PiperStatusMsg) -> None:
            with self.lock:
                self.statuses[key] = {
                    "ctrl_mode": int(message.ctrl_mode),
                    "arm_status": int(message.arm_status),
                    "mode_feedback": int(message.mode_feedback),
                    "teach_status": int(message.teach_status),
                    "motion_status": int(message.motion_status),
                    "trajectory_num": int(message.trajectory_num),
                    "err_code": int(message.err_code),
                }

        return callback

    def snapshots(self) -> dict[str, dict[str, object]]:
        now = time.monotonic()
        result: dict[str, dict[str, object]] = {}

        with self.lock:
            for spec in ARMS:
                arrivals = self.arrivals[spec.key]
                trim_arrivals(arrivals, now)

                last = self.last_joint[spec.key]
                result[spec.key] = {
                    "rate": calculate_rate(arrivals),
                    "age": now - last if last > 0.0 else float("inf"),
                    "joint": self.joints[spec.key],
                    "pose": self.poses[spec.key],
                    "status": self.statuses[spec.key],
                }

        return result

    def isolation_state(self) -> tuple[bool, str]:
        nodes = {
            full_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }

        missing_nodes = EXPECTED_ARM_NODES - nodes
        if missing_nodes:
            return False, "Arm nodes are not both running"

        topic_failures = []

        for topic in sorted(CONTROL_TOPICS):
            publishers = len(self.get_publishers_info_by_topic(topic))
            subscriptions = len(self.get_subscriptions_info_by_topic(topic))

            if publishers != 0 or subscriptions < 1:
                topic_failures.append(
                    f"{topic}: pub={publishers}, sub={subscriptions}"
                )

        services = {
            name for name, _types in self.get_service_names_and_types()
        }

        missing_services = ISOLATED_ENABLE_SERVICES - services
        exposed_services = DEFAULT_ENABLE_SERVICES & services

        if topic_failures:
            return False, "; ".join(topic_failures)

        if missing_services:
            return False, "Missing isolated enable services"

        if exposed_services:
            return False, "Default enable service is exposed"

        return True, "6 control topics isolated; 2 enable services remapped"


class ArmPane(QFrame):
    def __init__(self, spec: ArmSpec) -> None:
        super().__init__()
        self.spec = spec
        self.setFrameShape(QFrame.StyledPanel)

        title = QLabel(spec.title)
        title.setStyleSheet("font-size:18px;font-weight:600")

        self.record_checkbox = QCheckBox("Record")
        self.record_checkbox.setChecked(True)

        title_row = QHBoxLayout()
        title_row.addWidget(title)
        title_row.addStretch(1)
        title_row.addWidget(self.record_checkbox)

        self.signal_label = QLabel("GUI sample: no signal")
        self.status_label = QLabel("Status: unavailable")
        self.joint_label = QLabel("Joint feedback unavailable")
        self.gripper_label = QLabel("Gripper feedback unavailable")
        self.pose_label = QLabel("End pose unavailable")

        self.joint_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.gripper_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.pose_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        monospace = "font-family:monospace"
        self.joint_label.setStyleSheet(monospace)
        self.gripper_label.setStyleSheet(monospace)
        self.pose_label.setStyleSheet(monospace)

        layout = QVBoxLayout(self)
        layout.addLayout(title_row)
        layout.addWidget(self.signal_label)
        layout.addWidget(self.status_label)
        layout.addSpacing(8)
        layout.addWidget(self.joint_label)
        layout.addWidget(self.gripper_label)
        layout.addSpacing(8)
        layout.addWidget(self.pose_label)
        layout.addStretch(1)

    def update_snapshot(self, snapshot: dict[str, object]) -> None:
        rate = float(snapshot["rate"])
        age = float(snapshot["age"])

        if age > STALE_AFTER_SECONDS:
            self.signal_label.setText("GUI sample: no signal")
            self.signal_label.setStyleSheet("color:#b44")
        else:
            self.signal_label.setText(
                f"GUI sample: {rate:.1f} Hz | age {age * 1000:.0f} ms"
            )
            self.signal_label.setStyleSheet("color:#275")

        status = snapshot["status"]
        if isinstance(status, dict):
            arm_status = int(status["arm_status"])
            err_code = int(status["err_code"])

            self.status_label.setText(
                "Status: "
                f"arm={arm_status} | error={err_code} | "
                f"ctrl={status['ctrl_mode']} | "
                f"mode={status['mode_feedback']} | "
                f"motion={status['motion_status']} | "
                f"teach={status['teach_status']}"
            )

            if arm_status == 0 and err_code == 0:
                self.status_label.setStyleSheet("color:#275")
            else:
                self.status_label.setStyleSheet("color:#b44")
        else:
            self.status_label.setText("Status: unavailable")
            self.status_label.setStyleSheet("color:#b44")

        joint = snapshot["joint"]
        if isinstance(joint, dict):
            positions = tuple(joint["positions"])
            efforts = tuple(joint["efforts"])

            if len(positions) >= 6:
                rows = []

                for index, radians in enumerate(positions[:6], start=1):
                    degrees = radians * 180.0 / 3.141592653589793
                    rows.append(
                        f"J{index}: {radians:+.5f} rad "
                        f"({degrees:+7.2f} deg)"
                    )

                self.joint_label.setText("\n".join(rows))
            else:
                self.joint_label.setText(
                    f"Unexpected position length: {len(positions)}"
                )

            if len(positions) >= 7:
                raw_metres = float(positions[6])
                display_mm = max(0.0, raw_metres) * 1000.0
                effort = float(efforts[6]) if len(efforts) >= 7 else 0.0

                self.gripper_label.setText(
                    f"Gripper raw: {raw_metres:+.6f} m | "
                    f"display: {display_mm:.2f} mm | "
                    f"effort feedback: {effort:+.3f}"
                )
            else:
                self.gripper_label.setText("Gripper field unavailable")
        else:
            self.joint_label.setText("Joint feedback unavailable")
            self.gripper_label.setText("Gripper feedback unavailable")

        pose = snapshot["pose"]
        if isinstance(pose, dict):
            px, py, pz = pose["position"]
            qx, qy, qz, qw = pose["orientation"]

            self.pose_label.setText(
                "Position [m]\n"
                f"  x {px:+.6f}  y {py:+.6f}  z {pz:+.6f}\n"
                "Quaternion [x y z w]\n"
                f"  {qx:+.6f}  {qy:+.6f}  {qz:+.6f}  {qw:+.6f}"
            )
        else:
            self.pose_label.setText("End pose unavailable")


class Window(QMainWindow):
    def __init__(self, node: ArmMonitorNode) -> None:
        super().__init__()

        self.node = node

        self.arm_proc: Optional[subprocess.Popen] = None
        self.arm_log = None
        self.arm_log_path: Optional[Path] = None
        self.arm_stop_requested_at = 0.0

        self.recorder_proc: Optional[subprocess.Popen] = None
        self.recorder_log = None
        self.session_dir: Optional[Path] = None
        self.recording_sides: tuple[str, ...] = ()
        self.recorder_stop_requested_at = 0.0
        self.restart_after_stop = False

        self.previous_rx_dropped: dict[str, int] = {}

        self.setWindowTitle("Laundry Butler Arm Observation")
        self.resize(1250, 700)

        self.panes = {
            spec.key: ArmPane(spec)
            for spec in ARMS
        }

        pane_row = QHBoxLayout()
        for spec in ARMS:
            pane_row.addWidget(self.panes[spec.key], 1)

        self.start_arms_btn = QPushButton("Start Arm Observation")
        self.stop_arms_btn = QPushButton("Stop Arm Observation")
        self.start_recording_btn = QPushButton("Start Arm MCAP")
        self.stop_recording_btn = QPushButton("Stop MCAP")

        self.stop_arms_btn.setEnabled(False)
        self.stop_recording_btn.setEnabled(False)

        self.start_arms_btn.clicked.connect(self.start_arms)
        self.stop_arms_btn.clicked.connect(self.stop_arms)
        self.start_recording_btn.clicked.connect(self.start_recording)
        self.stop_recording_btn.clicked.connect(
            lambda: self.request_recorder_stop(False)
        )

        controls = QHBoxLayout()
        controls.addWidget(self.start_arms_btn)
        controls.addWidget(self.stop_arms_btn)
        controls.addSpacing(20)
        controls.addWidget(self.start_recording_btn)
        controls.addWidget(self.stop_recording_btn)
        controls.addStretch(1)

        self.node_status_label = QLabel("Arm nodes: checking…")
        self.isolation_label = QLabel("Command isolation: checking…")
        self.can_left_label = QLabel("can_left: checking…")
        self.can_right_label = QLabel("can_right: checking…")
        self.output_label = QLabel(f"Output: {OUTPUT_ROOT}")
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        diagnostics = QGridLayout()
        diagnostics.addWidget(self.node_status_label, 0, 0, 1, 2)
        diagnostics.addWidget(self.isolation_label, 1, 0, 1, 2)
        diagnostics.addWidget(self.can_left_label, 2, 0)
        diagnostics.addWidget(self.can_right_label, 2, 1)
        diagnostics.addWidget(self.output_label, 3, 0, 1, 2)

        body = QVBoxLayout()
        body.addLayout(pane_row, 1)
        body.addLayout(controls)
        body.addLayout(diagnostics)

        central = QWidget()
        central.setLayout(body)
        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar())

        self.data_timer = QTimer(self)
        self.data_timer.timeout.connect(self.refresh_arm_data)
        self.data_timer.start(500)

        self.diagnostic_timer = QTimer(self)
        self.diagnostic_timer.timeout.connect(self.refresh_diagnostics)
        self.diagnostic_timer.start(2000)

        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.check_processes)
        self.process_timer.start(250)

        self.segment_timer = QTimer(self)
        self.segment_timer.setSingleShot(True)
        self.segment_timer.timeout.connect(
            lambda: self.request_recorder_stop(True)
        )

        QTimer.singleShot(100, self.refresh_diagnostics)

    def selected_sides(self) -> tuple[str, ...]:
        return tuple(
            spec.key
            for spec in ARMS
            if self.panes[spec.key].record_checkbox.isChecked()
        )

    def selected_topics(self) -> list[str]:
        sides = set(self.selected_sides())
        topics = []

        for spec in ARMS:
            if spec.key not in sides:
                continue

            topics.extend(
                [
                    spec.joint_topic,
                    spec.pose_topic,
                    spec.status_topic,
                ]
            )

        return topics

    def refresh_arm_data(self) -> None:
        snapshots = self.node.snapshots()
        any_live = False

        for spec in ARMS:
            snapshot = snapshots[spec.key]
            self.panes[spec.key].update_snapshot(snapshot)
            any_live = any_live or float(snapshot["age"]) <= STALE_AFTER_SECONDS

        if self.recorder_proc is None:
            self.statusBar().showMessage(
                "Arm feedback active"
                if any_live
                else "Waiting for arm feedback"
            )

    def refresh_diagnostics(self) -> None:
        detected = list_ros_nodes() & EXPECTED_ARM_NODES
        managed = self.arm_proc is not None and self.arm_proc.poll() is None

        if detected == EXPECTED_ARM_NODES:
            suffix = " (started here)" if managed else " (external)"
            self.node_status_label.setText("Arm nodes: running" + suffix)
            self.start_arms_btn.setEnabled(False)
            self.stop_arms_btn.setEnabled(
                managed and self.recorder_proc is None
            )
        elif not detected:
            self.node_status_label.setText("Arm nodes: stopped")
            self.start_arms_btn.setEnabled(not managed)
            self.stop_arms_btn.setEnabled(False)
        else:
            self.node_status_label.setText(
                f"Arm nodes: partial ({len(detected)}/2); resolve first"
            )
            self.start_arms_btn.setEnabled(False)
            self.stop_arms_btn.setEnabled(
                managed and self.recorder_proc is None
            )

        try:
            isolated, detail = self.node.isolation_state()
        except Exception as exc:  # Graph can briefly change during startup.
            isolated = False
            detail = f"check failed: {exc}"

        self.isolation_label.setText(
            "Command isolation: "
            + ("VERIFIED — " if isolated else "NOT VERIFIED — ")
            + detail
        )
        self.isolation_label.setStyleSheet(
            "color:#275" if isolated else "color:#b44"
        )

        for spec, label in (
            (ARMS[0], self.can_left_label),
            (ARMS[1], self.can_right_label),
        ):
            state = parse_can_status(spec.can_interface)

            if not state.get("available"):
                label.setText(
                    f"{spec.can_interface}: unavailable — "
                    f"{state.get('error', '')}"
                )
                label.setStyleSheet("color:#b44")
                continue

            dropped = int(state["rx_dropped"])
            previous = self.previous_rx_dropped.get(spec.can_interface)
            dropped_delta = 0 if previous is None else dropped - previous
            self.previous_rx_dropped[spec.can_interface] = dropped

            label.setText(
                f"{spec.can_interface}: "
                f"{state['link_state']} / {state['can_state']} | "
                f"{int(state['bitrate']) // 1000} kbit/s | "
                f"RX {state['rx_packets']} | TX {state['tx_packets']} | "
                f"RX drop Δ {dropped_delta} | "
                f"errors RX/TX {state['rx_errors']}/{state['tx_errors']}"
            )

            healthy = (
                state["link_state"] == "UP"
                and state["can_state"] == "ERROR-ACTIVE"
                and state["bitrate"] == 1_000_000
                and state["rx_errors"] == 0
                and state["tx_errors"] == 0
                and dropped_delta == 0
            )
            label.setStyleSheet("color:#275" if healthy else "color:#b44")

    def start_arms(self) -> None:
        if self.arm_proc is not None and self.arm_proc.poll() is None:
            return

        existing = list_ros_nodes() & EXPECTED_ARM_NODES
        if existing:
            QMessageBox.warning(
                self,
                "Arm nodes already present",
                "One or more Piper nodes are already running.\n\n"
                "The interface will not start a duplicate launch.",
            )
            self.refresh_diagnostics()
            return

        if not ARM_LAUNCH_FILE.is_file():
            QMessageBox.critical(
                self,
                "Launch file missing",
                str(ARM_LAUNCH_FILE),
            )
            return

        runtime_dir = OUTPUT_ROOT / "runtime"
        runtime_dir.mkdir(parents=True, exist_ok=True)

        self.arm_log_path = (
            runtime_dir
            / f"arm-launch-{datetime.now():%Y%m%d-%H%M%S}.log"
        )
        self.arm_log = self.arm_log_path.open(
            "w",
            encoding="utf-8",
            buffering=1,
        )

        try:
            self.arm_proc = subprocess.Popen(
                ["ros2", "launch", str(ARM_LAUNCH_FILE)],
                stdout=self.arm_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.arm_log.close()
            self.arm_log = None
            self.arm_proc = None
            QMessageBox.critical(
                self,
                "Could not start arm observation",
                str(exc),
            )
            return

        self.statusBar().showMessage(
            f"Starting arm observation; log: {self.arm_log_path}"
        )
        self.start_arms_btn.setEnabled(False)
        self.stop_arms_btn.setEnabled(True)
        QTimer.singleShot(1500, self.refresh_diagnostics)

    def stop_arms(self) -> None:
        if self.recorder_proc is not None:
            QMessageBox.warning(
                self,
                "Recording is active",
                "Stop the MCAP recording before stopping arm observation.",
            )
            return

        if self.arm_proc is None or self.arm_proc.poll() is not None:
            self.refresh_diagnostics()
            return

        signal_process_group(self.arm_proc, signal.SIGINT)
        self.arm_stop_requested_at = time.monotonic()
        self.stop_arms_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping arm observation…")

    def start_recording(self) -> None:
        if self.recorder_proc is not None:
            return

        selected = self.selected_sides()
        if not selected:
            QMessageBox.warning(
                self,
                "No arms selected",
                "Select at least one arm to record.",
            )
            return

        try:
            isolated, detail = self.node.isolation_state()
        except Exception as exc:
            isolated = False
            detail = str(exc)

        if not isolated:
            QMessageBox.critical(
                self,
                "Command isolation not verified",
                "Recording was not started.\n\n" + detail,
            )
            return

        topics = self.selected_topics()
        available = current_topic_names()
        missing = [topic for topic in topics if topic not in available]

        if missing:
            QMessageBox.warning(
                self,
                "Arm topics unavailable",
                "Recording was not started. Missing:\n\n"
                + "\n".join(missing),
            )
            return

        now = datetime.now()
        self.session_dir = session_directory(now)
        self.session_dir.mkdir(parents=True, exist_ok=False)

        self.recorder_log = (self.session_dir / "rosbag.log").open(
            "w",
            encoding="utf-8",
            buffering=1,
        )

        command = [
            "ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "--output",
            str(self.session_dir / "bag"),
            *topics,
        ]

        try:
            self.recorder_proc = subprocess.Popen(
                command,
                stdout=self.recorder_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            self.recorder_log.close()
            self.recorder_log = None
            self.recorder_proc = None
            QMessageBox.critical(
                self,
                "Could not start MCAP recorder",
                str(exc),
            )
            return

        self.recording_sides = selected
        self.restart_after_stop = False
        self.recorder_stop_requested_at = 0.0

        self.start_recording_btn.setEnabled(False)
        self.stop_recording_btn.setEnabled(True)
        self.stop_arms_btn.setEnabled(False)

        milliseconds = seconds_until_next_bucket(now) * 1000
        self.segment_timer.start(milliseconds)

        self.statusBar().showMessage(
            f"Recording {', '.join(selected)} to {self.session_dir}"
        )

    def request_recorder_stop(self, restart: bool) -> None:
        if self.recorder_proc is None:
            return

        if self.recorder_proc.poll() is not None:
            self.finish_recorder()
            return

        self.restart_after_stop = restart
        self.segment_timer.stop()
        signal_process_group(self.recorder_proc, signal.SIGINT)
        self.recorder_stop_requested_at = time.monotonic()
        self.stop_recording_btn.setEnabled(False)
        self.statusBar().showMessage("Stopping MCAP recorder…")

    def check_processes(self) -> None:
        if self.arm_proc is not None:
            if self.arm_proc.poll() is not None:
                exit_code = self.arm_proc.returncode
                self.finish_arm_process()
                self.statusBar().showMessage(
                    f"Arm observation exited with code {exit_code}",
                    5000,
                )
                self.refresh_diagnostics()
            elif (
                self.arm_stop_requested_at > 0.0
                and time.monotonic() - self.arm_stop_requested_at > 8.0
            ):
                signal_process_group(self.arm_proc, signal.SIGTERM)

        if self.recorder_proc is not None:
            if self.recorder_proc.poll() is not None:
                self.finish_recorder()
            elif self.recorder_stop_requested_at > 0.0:
                elapsed = time.monotonic() - self.recorder_stop_requested_at

                if elapsed > 12.0:
                    signal_process_group(
                        self.recorder_proc,
                        signal.SIGKILL,
                    )
                elif elapsed > 8.0:
                    signal_process_group(
                        self.recorder_proc,
                        signal.SIGTERM,
                    )

    def finish_arm_process(self) -> None:
        if self.arm_log is not None:
            self.arm_log.close()

        self.arm_proc = None
        self.arm_log = None
        self.arm_stop_requested_at = 0.0
        self.stop_arms_btn.setEnabled(False)

    def finish_recorder(self) -> None:
        restart = self.restart_after_stop

        if self.recorder_log is not None:
            self.recorder_log.close()

        completed_dir = self.session_dir

        self.recorder_proc = None
        self.recorder_log = None
        self.session_dir = None
        self.recording_sides = ()
        self.recorder_stop_requested_at = 0.0
        self.restart_after_stop = False

        self.start_recording_btn.setEnabled(True)
        self.stop_recording_btn.setEnabled(False)
        self.refresh_diagnostics()

        if restart:
            QTimer.singleShot(300, self.start_recording)
        elif completed_dir is not None:
            self.statusBar().showMessage(
                f"MCAP saved under {completed_dir}",
                8000,
            )

    def closeEvent(self, event: QCloseEvent) -> None:
        active = (
            self.recorder_proc is not None
            or (
                self.arm_proc is not None
                and self.arm_proc.poll() is None
            )
        )

        if active:
            answer = QMessageBox.question(
                self,
                "Close interface",
                "Stop processes started by this interface and close?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return

        self.segment_timer.stop()

        signal_process_group(self.recorder_proc, signal.SIGINT)
        signal_process_group(self.arm_proc, signal.SIGINT)

        event.accept()


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ArmMonitorNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin,
        name="arm-status-ros",
        daemon=True,
    )
    ros_thread.start()

    application = QApplication(sys.argv)
    window = Window(node)
    window.show()

    exit_code = application.exec_()

    executor.shutdown()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    ros_thread.join(timeout=2.0)
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
