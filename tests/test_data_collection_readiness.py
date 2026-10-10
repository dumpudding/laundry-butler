"""Readiness regressions without ROS participants, device access, or recording."""

from __future__ import annotations

import ast
from contextlib import ExitStack
from pathlib import Path
import sys
import threading
import time
import struct
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "data_collection"))
import preflight
from demonstration_commands import COMMAND_TOPICS, COMMAND_TRACE_TOPICS, CommandAssembler, CommandFrame

GUI_TREE = ast.parse((ROOT / "data_collection/data_collection_gui.py").read_text())
GUI_CONSTANTS = {
    statement.targets[0].id: ast.literal_eval(statement.value)
    for statement in GUI_TREE.body
    if isinstance(statement, ast.Assign)
    and isinstance(statement.targets[0], ast.Name)
    and statement.targets[0].id in {"CAMERAS", "ARM_SPECS"}
}


def gui_method(class_name, method_name):
    """Load actual guard/callback methods without importing ROS or starting Qt."""
    definition = next(
        statement for statement in GUI_TREE.body
        if isinstance(statement, ast.ClassDef) and statement.name == class_name
    )
    method = next(statement for statement in definition.body if getattr(statement, "name", None) == method_name)
    module = ast.Module(
        body=[
            ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
            method,
        ],
        type_ignores=[],
    )
    scope = {
        **GUI_CONSTANTS,
        "time": time,
        "check_received_topic_liveness": preflight.check_received_topic_liveness,
        "Check": preflight.Check,
        "COMMAND_TOPICS": COMMAND_TOPICS,
        "COMMAND_TRACE_TOPICS": COMMAND_TRACE_TOPICS,
    }
    exec(compile(ast.fix_missing_locations(module), "data_collection_gui.py", "exec"), scope)
    return scope[method_name]


class ReceivedTopicLivenessTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.fresh = {topic: self.now - 0.1 for topic in preflight.LIGHTWEIGHT_LIVENESS_TOPICS}

    def test_all_seven_fresh_topics_pass(self):
        check = preflight.check_received_topic_liveness(self.fresh, now=self.now)
        self.assertEqual(check.status, "pass")

    def test_each_missing_or_stale_topic_blocks(self):
        for topic in preflight.LIGHTWEIGHT_LIVENESS_TOPICS:
            for received_at in (None, 0.0, self.now - 3.1):
                with self.subTest(topic=topic, received_at=received_at):
                    samples = dict(self.fresh)
                    if received_at is None:
                        del samples[topic]
                    else:
                        samples[topic] = received_at
                    check = preflight.check_received_topic_liveness(samples, now=self.now)
                    self.assertEqual(check.status, "fail")
                    self.assertIn(topic, check.detail)

    def test_freshness_boundary_and_invalid_timestamps(self):
        topic = preflight.LIGHTWEIGHT_LIVENESS_TOPICS[0]
        for received_at, expected in (
            (self.now - 3.0, "pass"),
            (self.now - 3.001, "fail"),
            (self.now + 1.0, "fail"),
            (float("nan"), "fail"),
            (float("inf"), "fail"),
        ):
            with self.subTest(received_at=received_at):
                samples = {**self.fresh, topic: received_at}
                self.assertEqual(
                    preflight.check_received_topic_liveness(samples, now=self.now).status,
                    expected,
                )

    def test_camera_info_callback_tracks_live_receipt(self):
        node = SimpleNamespace(lock=threading.Lock(), camera_info_last={})
        callback = gui_method("MonitorNode", "_camera_info_callback")(node, "live", "front")
        with mock.patch.object(time, "monotonic", return_value=self.now):
            callback(None)
        self.assertEqual(node.camera_info_last[("live", "front")], self.now)

    def test_playback_samples_cannot_satisfy_live_readiness(self):
        node = SimpleNamespace(lock=threading.Lock(), camera_info_last={}, arm_last={})
        for key, _title, _image, _info in GUI_CONSTANTS["CAMERAS"]:
            node.camera_info_last[("viewer", key)] = self.now
        for side, kind, _topic in GUI_CONSTANTS["ARM_SPECS"]:
            node.arm_last[("viewer", side, kind)] = self.now
        with mock.patch.object(time, "monotonic", return_value=self.now):
            check = gui_method("MonitorNode", "check_live_topic_liveness")(node)
        self.assertEqual(check.status, "fail")
        for topic in preflight.LIGHTWEIGHT_LIVENESS_TOPICS:
            self.assertIn(topic, check.detail)

    def test_live_receipt_passes_using_monitor_samples(self):
        node = SimpleNamespace(lock=threading.Lock(), camera_info_last={}, arm_last={})
        for key, _title, _image, _info in GUI_CONSTANTS["CAMERAS"]:
            node.camera_info_last[("live", key)] = self.now - 0.1
        for side, kind, _topic in GUI_CONSTANTS["ARM_SPECS"]:
            node.arm_last[("live", side, kind)] = self.now - 0.1
        with mock.patch.object(time, "monotonic", return_value=self.now):
            check = gui_method("MonitorNode", "check_live_topic_liveness")(node)
        self.assertEqual(check.status, "pass")


class ReadinessIntegrationTest(unittest.TestCase):
    def test_only_both_real_sources_pass_command_readiness(self):
        node = SimpleNamespace(
            lock=threading.Lock(),
            command_assemblers={side: CommandAssembler() for side in ("left", "right")},
            command_errors={"left": "", "right": ""},
            count_publishers=lambda topic: 1,
            arm_last={("live", "left", "joints"): 100, ("viewer", "right", "commands"): 100},
        )
        check = gui_method("MonitorNode", "check_live_commands")
        with mock.patch.object(time, "monotonic", return_value=100):
            self.assertEqual(check(node).status, "fail")
            for side in ("left", "right"):
                assembler = node.command_assemblers[side]
                for can_id in (0x159, 0x155, 0x156, 0x157):
                    data = struct.pack(">iHBB", 0, 0, 1, 0) if can_id == 0x159 else b"\0" * 8
                    assembler.feed(CommandFrame(can_id, data, 100_000_000_000 + can_id), 100)
                self.assertEqual(check(node).status, "pass" if side == "right" else "fail")
            node.count_publishers = lambda topic: 2
            self.assertEqual(check(node).status, "fail", "Duplicate captures must block")

    def mock_other_checks(self, stack):
        for name in (
            "check_dependencies", "check_nodes", "check_topics", "check_can",
            "check_command_isolation", "check_storage",
            "check_command_liveness",
        ):
            stack.enter_context(mock.patch.object(preflight, name, return_value=preflight.Check(name, "pass", "ok")))

    def test_gui_check_does_not_launch_temporary_subscriptions(self):
        for status in ("pass", "fail"):
            with self.subTest(status=status), ExitStack() as stack:
                self.mock_other_checks(stack)
                stack.enter_context(mock.patch.object(preflight, "check_topic_liveness", side_effect=AssertionError("CLI should not run")))
                callback = mock.Mock(return_value=preflight.Check("Topic liveness", status, "live samples"))
                report = preflight.run_preflight(Path("/unused"), topic_liveness_check=callback)
                callback.assert_called_once_with()
                self.assertEqual(len(report.checks), 9)
                self.assertEqual(report.status, status)

    def test_standalone_check_keeps_cli_fallback(self):
        with ExitStack() as stack:
            self.mock_other_checks(stack)
            cli = stack.enter_context(mock.patch.object(preflight, "check_topic_liveness", return_value=preflight.Check("Topic liveness", "pass", "CLI")))
            self.assertTrue(preflight.run_preflight(Path("/unused")).passed)
            cli.assert_called_once_with()

    def test_start_rechecks_live_data_even_with_valid_cached_readiness(self):
        start_episode = gui_method("Window", "start_episode")
        for status in ("pass", "fail"):
            with self.subTest(status=status):
                window = SimpleNamespace(
                    recorder=None,
                    playback=None,
                    worker_active=False,
                    last_preflight=preflight.Report("pass", ()),
                    preflight_valid_until=time.monotonic() + 120.0,
                    pending_record_after_readiness=False,
                    node=SimpleNamespace(
                        check_live_topic_liveness=mock.Mock(return_value=preflight.Check("Topic liveness", status, "samples")),
                        check_live_commands=mock.Mock(return_value=preflight.Check("Demonstration commands", "pass", "actual frames")),
                    ),
                    metadata=mock.Mock(return_value=None),
                    run_preflight_async=mock.Mock(),
                )
                start_episode(window)
                window.node.check_live_topic_liveness.assert_called_once_with()
                if status == "fail":
                    self.assertTrue(window.pending_record_after_readiness)
                    window.run_preflight_async.assert_called_once_with(automatic=True)
                    window.metadata.assert_not_called()
                else:
                    window.run_preflight_async.assert_not_called()
                    window.metadata.assert_called_once_with()

    def test_missing_commands_block_even_with_fresh_observations(self):
        window = SimpleNamespace(
            recorder=None, playback=None, worker_active=False,
            last_preflight=preflight.Report("pass", ()),
            preflight_valid_until=time.monotonic() + 120,
            pending_record_after_readiness=False,
            node=SimpleNamespace(
                check_live_topic_liveness=lambda: preflight.Check("observations", "pass", "fresh"),
                check_live_commands=lambda: preflight.Check("commands", "fail", "no real targets"),
            ),
            metadata=mock.Mock(), run_preflight_async=mock.Mock(),
        )
        gui_method("Window", "start_episode")(window)
        window.metadata.assert_not_called()
        window.run_preflight_async.assert_called_once_with(automatic=True)

    def test_command_loss_stops_and_marks_unusable(self):
        window = SimpleNamespace(
            recorder=SimpleNamespace(stop_requested_at=0), current_episode=None,
            node=SimpleNamespace(check_live_commands=lambda: preflight.Check("commands", "fail", "lost right targets")),
            command_capture_error=None, request_recorder_stop=mock.Mock(),
            preflight_result=mock.Mock(), statusBar=mock.Mock(return_value=mock.Mock()),
        )
        gui_method("Window", "check_recording_commands")(window)
        self.assertEqual(window.command_capture_error, "lost right targets")
        window.request_recorder_stop.assert_called_once_with("unusable")


if __name__ == "__main__":
    unittest.main()
