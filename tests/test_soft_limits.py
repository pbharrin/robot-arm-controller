import unittest
from arm_controller.protocol import Controller
from test_protocol import Clock, Device


class SoftLimitTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.device = Device(self.clock)
        self.c = Controller(self.device, clock=self.clock)
        self.advance(2.1)
        for _ in range(5):
            self.advance()
        self.c.set_zero()
        self.c.arm()

    def advance(self, seconds=.3):
        self.clock.now += seconds
        self.c.tick()

    def test_confirmed_setting_is_verified_then_requires_zero_and_enable(self):
        self.c.set_soft_limits(False)
        self.assertFalse(self.c.armed)
        self.assertIsNone(self.c.origin)
        self.assertEqual(self.device.sent[-1], b"$20=0\n")
        self.assertFalse(self.c.can_move)
        for _ in range(4):
            self.advance()
        self.assertEqual(self.c.settings["$20"], "0")
        self.assertTrue(self.c.idle)
        self.assertFalse(self.c.can_move)
        self.c.set_zero()
        self.c.arm()
        self.c.move("X", .1, 60)
        self.assertEqual(self.device.sent[-1], b"$J=G21 G90 G53 X0.100 F60.0\n")

    def test_reenable_and_disconnect_never_changes_setting_implicitly(self):
        self.c.set_soft_limits(False)
        for _ in range(4):
            self.advance()
        self.c.set_soft_limits(True)
        for _ in range(4):
            self.advance()
        self.assertEqual(self.c.settings["$20"], "1")
        writes = [x for x in self.device.sent if x.startswith(b"$20=")]
        self.c.close()
        self.assertEqual(writes, [b"$20=0\n", b"$20=1\n"])
        self.assertEqual(writes, [x for x in self.device.sent if x.startswith(b"$20=")])

    def test_rejected_or_unverified_setting_stays_faulted(self):
        for outcome in ("error:10", "missing", "mismatch", "timeout"):
            with self.subTest(outcome=outcome):
                self.setUp()
                self.c.set_soft_limits(False)
                self.device.output.clear()
                if outcome == "error:10":
                    self.c.receive(outcome)
                elif outcome == "timeout":
                    self.device.silent = True
                    self.advance(6)
                else:
                    self.c.receive("ok")
                    if outcome == "mismatch":
                        self.c.receive("$20=1")
                    self.c.receive("ok")
                self.assertTrue(self.c.error)
                self.assertFalse(self.c.can_move)

    def test_busy_and_stale_changes_are_rejected(self):
        self.c.move("X", 10, 60)
        with self.assertRaises(ValueError):
            self.c.set_soft_limits(False)
        self.setUp()
        self.clock.now += 3
        with self.assertRaises(ValueError):
            self.c.set_soft_limits(False)

    def test_homing_prerequisite_for_reenabling(self):
        self.c.settings["$22"] = "0"
        with self.assertRaises(ValueError):
            self.c.set_soft_limits(True)

    def test_setting_can_change_while_alarm_locked_without_unlocking(self):
        self.c.receive("<Alarm|MPos:0,0,0,0,0,0>")
        self.c.set_soft_limits(False)
        self.assertEqual(self.device.sent[-1], b"$20=0\n")
        self.assertNotIn(b"$X\n", self.device.sent)
