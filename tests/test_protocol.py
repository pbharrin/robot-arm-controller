import math
import unittest

from arm_controller.protocol import AXES, Controller, DemoTransport, coordinates, jog_command


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Device(DemoTransport):
    def __init__(self, clock):
        super().__init__(clock)
        self.sent = []
        self.silent = False

    def write(self, data):
        self.sent.append(data)
        if self.silent:
            return len(data)
        return super().write(data)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.device = Device(self.clock)
        self.c = Controller(self.device, clock=self.clock)

    def advance(self, seconds=.3):
        self.clock.now += seconds
        self.c.tick()

    def ready(self):
        self.advance(2.1)
        for _ in range(4):
            self.advance()
        self.assertTrue(self.c.idle)

    def arm(self):
        self.ready()
        self.c.set_zero()
        self.c.arm()

    def test_handshake_never_moves_or_writes_settings(self):
        self.ready()
        self.assertFalse(self.c.can_move)
        self.assertEqual([x for x in self.device.sent if x != b"?"], [b"$$\n", b"$I\n"])

    def test_movement_requires_zero_and_enable(self):
        self.ready()
        with self.assertRaises(ValueError):
            self.c.arm()
        self.c.set_zero()
        with self.assertRaises(ValueError):
            self.c.move("X", 2, 60)
        self.c.arm()
        self.assertTrue(self.c.can_move)

    def test_move_completion_not_just_ack(self):
        self.arm()
        self.c.move("A", 2, 60)
        self.advance()
        self.assertIsNone(self.c.pending)
        self.assertTrue(self.c.moving)
        with self.assertRaises(ValueError):
            self.c.move("B", 5, 60)
        for _ in range(10):
            self.advance()
        self.assertFalse(self.c.moving)
        self.assertTrue(self.c.can_move)
        self.assertAlmostEqual(self.c.position[3], 2)

    def test_stale_idle_before_ack_is_not_completion(self):
        self.arm()
        self.c.move("X", 30, 60)
        self.c.receive("<Idle|MPos:0,0,0,0,0,0>")
        self.c.receive("ok")
        self.c.receive("<Idle|MPos:0,0,0,0,0,0>")
        self.assertTrue(self.c.moving)

    def test_stop_cancels_and_requires_reenable(self):
        self.arm()
        self.c.move("C", 30, 60)
        self.advance()
        self.c.stop()
        for _ in range(4):
            self.advance()
        self.assertTrue(self.c.idle)
        self.assertFalse(self.c.can_move)
        self.assertIn(b"\x85!", self.device.sent)
        self.assertLess(self.c.position[5], 30)

    def test_alarm_clears_zero_and_stops(self):
        self.arm()
        self.c.receive("ALARM:1")
        self.assertFalse(self.c.can_move)
        self.assertIsNone(self.c.origin)
        self.assertEqual(self.device.sent[-1], b"\x85!")

    def test_reset_does_not_rearm(self):
        self.arm()
        self.c.receive("Grbl 1.1f ['$' for help]")
        self.assertEqual(self.c.phase, "fault")
        self.assertIsNone(self.c.origin)

    def test_missing_status_stops_motion(self):
        self.arm()
        self.device.output.clear()
        self.device.silent = True
        self.advance(2.1)
        self.assertFalse(self.c.can_move)
        self.assertIn("Position reports stopped", self.c.error)

    def test_ack_timeout_never_retries_motion(self):
        self.arm()
        self.device.silent = True
        self.c.move("X", 10, 60)
        self.advance(5.1)
        self.assertEqual(sum(x.startswith(b"$J=") for x in self.device.sent), 1)
        self.assertIn("acknowledgement timed out", self.c.error)

    def test_inch_reports_rejected(self):
        self.c.phase = "settings"
        self.c.pending = "settings"
        self.c.receive("$13=1")
        self.c.receive("$10=1")
        self.c.receive("ok")
        self.assertIn("$13=0", self.c.error)

    def test_work_position_reports_rejected(self):
        self.c.phase = "settings"
        self.c.pending = "settings"
        self.c.receive("$13=0")
        self.c.receive("$10=0")
        self.c.receive("ok")
        self.assertIn("$10", self.c.error)

    def test_wrong_axis_firmware_rejected(self):
        self.c.phase = "identity"
        self.c.pending = "identity"
        self.c.receive("[AXS:3:XYZ]")
        self.c.receive("ok")
        self.assertIn("Expected Arctos", self.c.error)

    def test_fragmented_serial_lines(self):
        self.device.output = bytearray(b"<Idle|MPos:1,2,")
        self.c.tick()
        self.assertIsNone(self.c.position)
        self.device.output.extend(b"3,4,5,6>\r\n")
        self.c.tick()
        self.assertEqual(self.c.position, (1, 2, 3, 4, 5, 6))

    def test_serial_unplug_and_partial_write_fault(self):
        self.arm()
        self.device.read = lambda size: (_ for _ in ()).throw(OSError("USB unplugged"))
        self.c.tick()
        self.assertIn("USB unplugged", self.c.error)
        self.assertFalse(self.c.armed)

    def test_partial_motion_write_fault(self):
        self.arm()
        self.device.write = lambda data: len(data) - 1
        self.c.move("X", 1, 60)
        self.assertIn("Incomplete serial write", self.c.error)

    def test_close_requests_stop_and_clears_reference(self):
        self.arm()
        self.c.close()
        self.assertEqual(self.device.sent[-1], b"\x85!")
        self.assertTrue(self.device.closed)
        self.assertIsNone(self.c.origin)

    def test_zero_is_local_and_uses_nonzero_machine_position(self):
        self.ready()
        self.c.receive("<Idle|MPos:10,20,30,40,50,60>")
        count = len(self.device.sent)
        self.c.set_zero()
        self.assertEqual(len(self.device.sent), count)
        self.c.arm()
        self.c.move("B", -2, 60)
        self.assertEqual(self.device.sent[-1], b"$J=G21 G90 G53 B48.000 F60.0\n")


class FormattingTests(unittest.TestCase):
    def test_all_six_axes_and_boundaries(self):
        for axis in AXES:
            for angle in (-180, 0, 180):
                self.assertEqual(jog_command(axis, angle, 0, 60),
                                 f"$J=G21 G90 G53 {axis}{angle:.3f} F60.0\n".encode())

    def test_invalid_targets_and_injection(self):
        for axis, angle, origin, feed in [("XX", 0, 0, 60), ("X\n", 0, 0, 60),
                ("X", 181, 0, 60), ("X", math.nan, 0, 60), ("X", 0, math.inf, 60),
                ("X", 0, 0, 0), ("X", 0, 0, 601), ("X", 0, 0, math.nan)]:
            with self.assertRaises(ValueError):
                jog_command(axis, angle, origin, feed)

    def test_invalid_position(self):
        for value in ("1,2,3", "0,0,nan,0,0,0", "0,0,0,inf,0,0"):
            with self.assertRaises(ValueError):
                coordinates(value)


if __name__ == "__main__":
    unittest.main()
