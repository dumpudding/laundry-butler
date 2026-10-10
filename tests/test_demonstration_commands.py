"""Piper control-frame regressions, independent of ROS and physical devices."""

from pathlib import Path
import math
import socket
import struct
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "data_collection"))
from demonstration_commands import (
    CAN_FRAME, COMMAND_MAX_AGE_SECONDS, CONTROL_IDS, SO_TIMESTAMPNS,
    CommandAssembler, CommandFrame, PassiveCANReader, decode_frame, sample_from_trace,
)


def frame(can_id, values=(0, 0), stamp=1_000_000_000):
    data = struct.pack(">iHBB", values[0], values[1], 1, 0) if can_id == 0x159 else struct.pack(">ii", *values)
    return CommandFrame(can_id, data, stamp)


def cycle(assembler, stamp=1_000_000_000, now=1.0, values=(0, 0)):
    result = None
    for offset, can_id in enumerate(CONTROL_IDS[:3]):
        result = assembler.feed(frame(can_id, values, stamp + offset * 1000), now)
    return result


class CommandAssemblerTest(unittest.TestCase):
    def test_big_endian_signed_angles_and_exact_units(self):
        values = decode_frame(frame(0x155, (90000, -180000)))
        self.assertAlmostEqual(values[0], math.pi / 2)
        self.assertAlmostEqual(values[1], -math.pi)
        self.assertEqual(decode_frame(frame(0x159, (70000, 1500))), (.07, 1.5))

    def test_feedback_and_missing_initial_gripper_cannot_create_action(self):
        assembler = CommandAssembler()
        self.assertIsNone(assembler.feed(frame(0x2A5, (200, 300)), 1))
        self.assertIsNone(cycle(assembler))
        self.assertIsNotNone(assembler.problem(1))
        self.assertIsNone(assembler.last_sample)

    def test_real_zero_targets_valid_and_provenance_round_trip(self):
        assembler = CommandAssembler()
        assembler.feed(frame(0x159), 1)
        sample = cycle(assembler)
        self.assertEqual(sample.positions, (0.0,) * 7)
        self.assertEqual(sample, sample_from_trace(sample.trace("can_left")))
        self.assertIsNone(assembler.problem(1))

    def test_no_periodic_republish_or_partial_joint_cycle(self):
        assembler = CommandAssembler()
        assembler.feed(frame(0x159), 1)
        original = cycle(assembler)
        self.assertIsNone(assembler.feed(frame(0x155, (10000, 10000), 1_010_000_000), 1.01))
        self.assertEqual(assembler.last_sample, original)
        self.assertIsNotNone(assembler.problem(1.501))

    def test_gripper_is_held_until_new_real_command(self):
        assembler = CommandAssembler()
        assembler.feed(frame(0x159, (70000, 1500)), 1)
        cycle(assembler)
        for index in range(1, 11):
            sample = cycle(assembler, 1_000_000_000 + index * 100_000_000, 1 + index * .1)
            self.assertEqual(sample.positions[-1], .07)
            self.assertEqual(sample.frames[-1].timestamp_ns, 1_000_000_000)
        new = assembler.feed(frame(0x159, (10000, 2000), 2_010_000_000), 2.01)
        self.assertEqual(new.positions[-1], .01)
        self.assertEqual(new.efforts[-1], 2.0)

    def test_gap_or_reconnect_requires_new_gripper_context(self):
        for reset in (False, True):
            with self.subTest(reset=reset):
                assembler = CommandAssembler()
                assembler.feed(frame(0x159), 1)
                cycle(assembler)
                if reset:
                    assembler.reset()
                self.assertIsNone(cycle(assembler, 2_000_000_000, 2))
                self.assertIsNotNone(assembler.problem(2))
                self.assertIsNotNone(assembler.feed(frame(0x159, stamp=2_000_004_000), 2))

    def test_incoherent_groups_and_zeroing_are_rejected(self):
        assembler = CommandAssembler()
        assembler.feed(frame(0x159), 1)
        assembler.feed(frame(0x155), 1)
        assembler.feed(frame(0x156), 1)
        self.assertIsNone(assembler.feed(frame(0x157, stamp=1_200_000_000), 1.2))
        zeroing = CommandFrame(0x159, struct.pack(">iHBB", 0, 0, 1, 0xAE), 1_201_000_000)
        self.assertIsNone(assembler.feed(zeroing, 1.201))
        self.assertIsNotNone(assembler.problem(1.201))

    def test_trace_rejects_wrong_ids_and_timestamp(self):
        assembler = CommandAssembler()
        assembler.feed(frame(0x159), 1)
        sample = cycle(assembler)
        payload = sample.trace("can_right")
        payload["timestamp_ns"] += 1
        with self.assertRaises(ValueError):
            sample_from_trace(payload)
        payload = sample.trace("can_right")
        payload["frames"][0]["can_id"] = 0x2A5
        with self.assertRaises(ValueError):
            sample_from_trace(payload)


class PassiveReaderTest(unittest.TestCase):
    def test_socket_receive_only_and_kernel_timestamp(self):
        received = []
        errors = []
        reader = PassiveCANReader("can_test", lambda item, now: received.append(item), errors.append)
        raw = CAN_FRAME.pack(0x155, 8, struct.pack(">ii", 90000, -90000))
        fake_socket = mock.MagicMock()
        fake_socket.__enter__.return_value = fake_socket
        def receive(*args):
            reader.stop_event.set()
            return raw, [(socket.SOL_SOCKET, SO_TIMESTAMPNS, struct.pack("@ll", 10, 0))], 0, ("can_test",)
        fake_socket.recvmsg.side_effect = receive
        with mock.patch("demonstration_commands.socket.socket", return_value=fake_socket), mock.patch("demonstration_commands.time.time_ns", return_value=10_001_000_000):
            reader._run()
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].timestamp_ns, 10_000_000_000)
        fake_socket.send.assert_not_called()
        fake_socket.sendall.assert_not_called()
        fake_socket.sendto.assert_not_called()
        self.assertFalse(errors)

    def test_delayed_receive_queue_does_not_refresh_old_commands(self):
        received = []
        errors = []
        reader = PassiveCANReader("can_test", lambda *args: received.append(args), errors.append)
        fake_socket = mock.MagicMock()
        fake_socket.__enter__.return_value = fake_socket
        def receive(*args):
            reader.stop_event.set()
            return CAN_FRAME.pack(0x155, 8, b"\0" * 8), [(socket.SOL_SOCKET, SO_TIMESTAMPNS, struct.pack("@ll", 10, 0))], 0, ()
        fake_socket.recvmsg.side_effect = receive
        with mock.patch("demonstration_commands.socket.socket", return_value=fake_socket), mock.patch("demonstration_commands.time.time_ns", return_value=11_000_000_000):
            reader._run()
        self.assertFalse(received)
        self.assertIn("Delayed", errors[0])


if __name__ == "__main__":
    unittest.main()
