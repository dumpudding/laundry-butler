"""Real MCAP round trips for independent actions and legacy recordings.

Source ROS 2 Jazzy and the Piper workspace to run these tests. They write
synthetic messages only; they create no ROS nodes or physical-device sockets.
"""

from pathlib import Path
import json
import struct
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "data_collection"))
from demonstration_commands import COMMAND_TOPICS, COMMAND_TRACE_TOPICS, JOINT_NAMES, CommandAssembler, CommandFrame, command_metadata
from episode_store import paths_for_episode, update_episode, write_json_atomic
from preflight import CAMERA_TOPICS, OBSERVATION_TOPICS, REQUIRED_TOPICS
from validation import validate_episode

try:
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from piper_msgs.msg import PiperStatusMsg
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import String
except ImportError:
    ROS_AVAILABLE = False
else:
    ROS_AVAILABLE = True


@unittest.skipUnless(ROS_AVAILABLE, "Source ROS and Piper workspace for MCAP round trips")
class DemonstrationRecordingTest(unittest.TestCase):
    def make_episode(self, root, *, legacy=False, missing=False, corrupt=False, gap=False, cached=False, boundary_missing=False, interior_missing=False):
        metadata = {"schema_version": "1.1" if legacy else "1.2", "recorded_topics": list(OBSERVATION_TOPICS if legacy else REQUIRED_TOPICS)}
        if not legacy:
            metadata["demonstration_commands"] = command_metadata()
        write_json_atomic(root / "episode.json", metadata)
        log = root / "recorder.log"
        log.write_text("Synthetic test: no devices or live ROS graph used\n")
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(root / "bag"), storage_id="mcap",
                    storage_config_uri=str(ROOT / "config/recording/mcap_zstd.yaml")), rosbag2_py.ConverterOptions("", ""))
        types = {}
        for topic in REQUIRED_TOPICS:
            if topic in CAMERA_TOPICS:
                msg_type, name = (Image, "sensor_msgs/msg/Image") if topic.endswith("image_raw") else (CameraInfo, "sensor_msgs/msg/CameraInfo")
            elif topic in COMMAND_TRACE_TOPICS:
                msg_type, name = String, "std_msgs/msg/String"
            elif topic in COMMAND_TOPICS or "/joint_" in topic:
                msg_type, name = JointState, "sensor_msgs/msg/JointState"
            elif "/end_pose_" in topic:
                msg_type, name = PoseStamped, "geometry_msgs/msg/PoseStamped"
            else:
                msg_type, name = PiperStatusMsg, "piper_msgs/msg/PiperStatusMsg"
            types[topic] = msg_type
            if (legacy or missing) and topic not in OBSERVATION_TOPICS:
                continue
            writer.create_topic(rosbag2_py.TopicMetadata(id=len(types), name=topic, type=name, serialization_format="cdr"))
        base = time.time_ns()
        assemblers = {side: CommandAssembler() for side in ("left", "right")}
        for assembler in assemblers.values():
            assembler.feed(CommandFrame(0x159, struct.pack(">iHBB", 70000, 1500, 1, 0), base), 1.0)
        original = {}
        # Cameras at 30 Hz, observations at 200 Hz, commands at 100 Hz.
        for tick in range(1200):
            stamp = base + round(tick * 1e9 / 600)
            for topic in OBSERVATION_TOPICS:
                if tick % (20 if topic in CAMERA_TOPICS else 3):
                    continue
                message = types[topic]()
                if isinstance(message, JointState):
                    message.name = list(JOINT_NAMES)
                    message.position = [.05] * 6 + [.02]  # Deliberately differs from commands.
                writer.write(topic, serialize_message(message), stamp)
            if legacy or missing or tick % 6 or (gap and 300 <= tick < 720):
                continue
            for index, side in enumerate(("left", "right")):
                assembler = assemblers[side]
                for offset, can_id in enumerate((0x155, 0x156, 0x157)):
                    sample = assembler.feed(CommandFrame(can_id, struct.pack(">ii", 90000, -90000), stamp + offset * 1000), 1 + tick / 600)
                if sample is None:  # Gap invalidates gripper context: send a real new target.
                    sample = assembler.feed(CommandFrame(0x159, struct.pack(">iHBB", 70000, 1500, 1, 0), stamp + 3000), 1 + tick / 600)
                if cached and side in original:
                    sample = original[side]
                original.setdefault(side, sample)
                trace = String(data=json.dumps(sample.trace(f"can_{side}")))
                message = JointState()
                message.name = list(JOINT_NAMES)
                message.header.frame_id = f"piper_control_can_{side}"
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample.timestamp_ns, 1_000_000_000)
                message.position = list(sample.positions)
                message.effort = list(sample.efforts)
                if corrupt and tick == 12:
                    message.position[0] = .05
                if not (side == "left" and ((boundary_missing and tick == 0) or (interior_missing and tick == 600))):
                    writer.write(COMMAND_TRACE_TOPICS[index], serialize_message(trace), stamp + 10000)
                writer.write(COMMAND_TOPICS[index], serialize_message(message), stamp + 11000)
        del writer
        return validate_episode(root / "bag", root / "validation.json", log)

    def test_real_command_values_survive_compressed_mcap(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name))
        self.assertEqual(result["status"], "pass", result["checks"])
        self.assertTrue(result["has_demonstration_commands"])
        self.assertEqual(len(result["topics"]), 16)

    def test_legacy_observation_episode_remains_readable_with_warning(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), legacy=True)
        self.assertEqual(result["status"], "warn", result["checks"])
        self.assertFalse(result["has_demonstration_commands"])
        self.assertIn("Legacy observation-only", str(result["checks"]))

    def test_missing_new_actions_fail(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), missing=True)
        self.assertEqual(result["status"], "fail")
        self.assertFalse(result["has_demonstration_commands"])

    def test_review_edit_does_not_turn_legacy_bag_into_command_recording(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            self.make_episode(root, legacy=True)
            update_episode(paths_for_episode(root), notes="Reviewed old observation recording")
            result = validate_episode(root / "bag", root / "validation.json", root / "recorder.log")
        self.assertEqual(result["status"], "warn")
        self.assertFalse(result["has_demonstration_commands"])

    def test_state_substitution_does_not_pass_provenance(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), corrupt=True)
        check = next(item for item in result["checks"] if item["name"] == "Command provenance")
        self.assertEqual(check["status"], "fail")
        self.assertIn("differ from CAN", check["detail"])

    def test_command_dropout_fails_even_with_nonzero_topics(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), gap=True)
        self.assertEqual(result["status"], "fail")
        self.assertIn("command gap", str(result["checks"]))

    def test_cached_sources_cannot_become_fresh_actions(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), cached=True)
        self.assertEqual(result["status"], "fail")
        self.assertIn("timestamps repeated", str(result["checks"]))

    def test_unpaired_start_sample_warns_and_is_excluded_from_verified_interval(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), boundary_missing=True)
        self.assertEqual(result["status"], "warn", result["checks"])
        self.assertTrue(result["has_demonstration_commands"])
        self.assertIn("unpaired boundary", str(result["checks"]))
        first, last = result["command_verified_interval_ns"]
        self.assertGreater(first, result["topics"][COMMAND_TOPICS[0]]["first_timestamp_ns"])
        self.assertGreater(last, first)

    def test_unpaired_interior_command_fails(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.make_episode(Path(name), interior_missing=True)
        self.assertEqual(result["status"], "fail")
        self.assertIn("unmatched command/source sample within recording", str(result["checks"]))


if __name__ == "__main__":
    unittest.main()
