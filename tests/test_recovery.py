import unittest

from arm_controller.protocol import Controller
from test_protocol import Clock, Device


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.device = Device(self.clock)
        self.c = Controller(self.device, clock=self.clock)

    def locked(self):
        # Replay the important ordering from the actual Arduino log: Alarm arrives
        # between the $$ acknowledgement and the $I identification response.
        self.c.phase = "settings"
        self.c.pending = "settings"
        for line in ("$10=1", "$13=0", "$20=1", "$21=0", "$22=1", "ok",
                     "<Alarm|MPos:0,0,0,0,0,0>", "[VER:1.1t.20210510:]",
                     "[AXS:6:XYZABC]", "[OPT:VNMGBH,35,255,48]", "ok"):
            self.c.receive(line)
        self.device.output.clear()

    def test_actual_startup_order_finishes_handshake(self):
        self.locked()
        self.assertEqual(self.c.phase, "ready")
        self.assertFalse(self.c.error)
        self.assertTrue(self.c.can_recover)
        self.assertFalse(self.c.can_move)
        self.assertEqual(self.device.sent, [b"$I\n"])
        for _ in range(20):
            self.clock.now += .25
            self.c.receive("<Alarm|MPos:0,0,0,0,0,0>")
        self.assertFalse(self.c.error)
        self.assertNotIn(b"\x85!", self.device.sent)

    def test_unlock_is_explicit_and_waits_for_post_ack_idle(self):
        self.locked()
        self.c.recover("unlock")
        self.assertEqual(self.device.sent[-1], b"$X\n")
        self.assertFalse(self.c.can_recover)
        self.c.receive("ok")
        self.c.receive("<Idle|MPos:0,0,0,0,0,0>")
        self.assertFalse(self.c.idle)
        self.clock.now += .3
        self.c.last_poll = self.clock()
        self.c.receive("<Idle|MPos:0,0,0,0,0,0>")
        self.assertTrue(self.c.idle)
        self.assertIsNone(self.c.origin)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.settings["$20"], "1")
        self.assertEqual(self.c.settings["$21"], "0")

    def test_homing_tolerates_silent_loop_and_completes(self):
        self.locked()
        self.c.recover("home")
        self.assertEqual(self.device.sent[-1], b"$H\n")
        self.device.silent = True
        self.clock.now += 30
        self.c.tick()
        self.assertFalse(self.c.error)
        self.assertEqual(self.c.pending, "home")
        self.c.receive("ok")
        self.assertFalse(self.c.idle)
        self.clock.now += .3
        self.c.last_poll = self.clock()
        self.c.receive("<Idle|MPos:-5,-5,-5,0,0,0>")
        self.assertTrue(self.c.idle)
        self.assertIsNone(self.c.origin)
        self.assertFalse(self.c.armed)

    def test_homing_stop_uses_reset(self):
        self.locked()
        self.c.recover("home")
        self.c.stop()
        self.assertEqual(self.device.sent[-1], b"\x18")
        self.assertEqual(self.c.phase, "fault")
        self.assertIsNone(self.c.pending)

    def test_homing_timeout_uses_reset_and_keeps_original_error(self):
        self.locked()
        self.c.recover("home")
        self.device.silent = True
        self.clock.now += 181
        self.c.tick()
        self.assertIn(b"\x18", self.device.sent)
        error = self.c.error
        self.assertIn("timed out", error)
        self.c.receive("Grbl 1.1t ['$' for help]")
        self.assertEqual(self.c.error, error)

    def test_homing_failure_is_still_fatal(self):
        self.locked()
        self.c.recover("home")
        self.c.receive("ALARM:9")
        self.assertEqual(self.c.phase, "fault")
        self.assertFalse(self.c.can_recover)
        self.assertIn("ALARM:9", self.c.error)

    def test_unlock_error_and_timeout_never_enable_motion(self):
        for response in ("error:9", "timeout"):
            with self.subTest(response=response):
                self.setUp()
                self.locked()
                self.c.recover("unlock")
                if response == "timeout":
                    self.device.silent = True
                    self.clock.now += 6
                    self.c.tick()
                else:
                    self.c.receive(response)
                self.assertFalse(self.c.can_move)
                self.assertIsNone(self.c.origin)
                self.assertTrue(self.c.error)

    def test_recovery_rejects_unverified_stale_disabled_and_busy(self):
        with self.assertRaises(ValueError):
            self.c.recover("unlock")
        self.locked()
        self.clock.now += 3
        with self.assertRaises(ValueError):
            self.c.recover("unlock")
        self.c.receive("<Alarm|MPos:0,0,0,0,0,0>")
        self.c.settings["$22"] = "0"
        with self.assertRaises(ValueError):
            self.c.recover("home")
        self.c.recover("unlock")
        with self.assertRaises(ValueError):
            self.c.recover("unlock")
