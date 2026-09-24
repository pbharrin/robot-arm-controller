import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from unittest.mock import patch
from PySide6.QtWidgets import QMessageBox
from PySide6.QtWidgets import QApplication
from arm_controller.app import Window
from arm_controller.protocol import Controller, DemoTransport


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_demo_slider_release_and_stop(self):
        window = Window()
        try:
            now = [0.0]
            clock = lambda: now[0]
            device = DemoTransport(clock)
            window.controller = c = Controller(device, clock=clock)
            for _ in range(12):
                now[0] += .3
                window.tick()
            self.assertFalse(window.rows[0].slider.isEnabled())
            c.set_zero()
            c.arm()
            window.render()
            self.assertTrue(window.rows[0].slider.isEnabled())
            window.rows[0].slider.setValue(100)
            self.assertFalse(c.moving, "Changing the slider should not stream commands")
            self.assertEqual(window.rows[0].value.value(), 10)
            window.rows[0].slider.sliderReleased.emit()
            self.assertTrue(c.moving)
            self.assertFalse(window.rows[1].slider.isEnabled())
            window.stop_button.click()
            self.assertFalse(c.armed)
            self.assertEqual(device.target, device.position)
        finally:
            window.close()

    def test_locked_connection_recovery_confirmation(self):
        window = Window()
        try:
            device = DemoTransport()
            window.controller = c = Controller(device)
            c.phase = "ready"
            c.settings["$22"] = "1"
            c.receive("<Alarm|MPos:0,0,0,0,0,0>")
            window.render()
            self.assertTrue(window.unlock_button.isEnabled())
            self.assertTrue(window.home_button.isEnabled())
            self.assertFalse(window.rows[0].slider.isEnabled())
            with patch.object(QMessageBox, "question", return_value=QMessageBox.StandardButton.Cancel):
                window.unlock_button.click()
            self.assertIsNone(c.pending)
            with patch.object(QMessageBox, "question", return_value=QMessageBox.StandardButton.Yes):
                window.unlock_button.click()
            self.assertEqual(c.pending, "unlock")
            self.assertFalse(window.home_button.isEnabled())
        finally:
            window.close()

    def test_soft_limit_toggle_requires_confirmation_and_disarms(self):
        window = Window()
        try:
            window.controller = c = Controller(DemoTransport())
            c.phase = "ready"
            c.settings.update({"$20": "1", "$22": "1"})
            c.receive("<Idle|MPos:0,0,0,0,0,0>")
            c.set_zero()
            c.arm()
            window.render()
            with patch.object(QMessageBox, "question", return_value=QMessageBox.StandardButton.Cancel):
                window.limits_button.click()
            self.assertIsNone(c.pending)
            self.assertTrue(c.armed)
            with patch.object(QMessageBox, "question", return_value=QMessageBox.StandardButton.Yes):
                window.limits_button.click()
            self.assertEqual(c.pending, "soft_limit_write")
            self.assertFalse(c.armed)
            self.assertFalse(window.rows[0].slider.isEnabled())
            self.assertIn("verifying", window.limits_status.text())
        finally:
            window.close()
