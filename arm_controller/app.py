"""Qt desktop interface; serial reads are nonblocking and polled every 20 ms."""
import sys
from datetime import datetime

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider,
    QVBoxLayout, QWidget, QScrollArea, QSizePolicy,
)

from arm_controller.protocol import AXES, Controller, DemoTransport


class JointRow(QFrame):
    def __init__(self, axis, send):
        super().__init__()
        self.setObjectName("joint")
        self.axis = axis
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setVerticalSpacing(0)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        title = QLabel(axis)
        title.setObjectName("axis")
        title.setFixedWidth(38)
        self.actual = QLabel("Reported: —")
        self.actual.setMinimumWidth(155)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-1800, 1800)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(100)
        self.slider.setTracking(True)
        self.slider.setAccessibleName(f"Joint {axis} target angle")
        self.value = QDoubleSpinBox()
        self.value.setRange(-180, 180)
        self.value.setDecimals(1)
        self.value.setSingleStep(.1)
        self.value.setSuffix(" °")
        self.value.setFixedWidth(115)
        self.value.setKeyboardTracking(False)
        self.value.setAccessibleName(f"Joint {axis} target degrees")
        self.go = QPushButton("Move")
        self.go.setFixedWidth(65)
        layout.addWidget(title, 0, 0, 2, 1)
        layout.addWidget(self.slider, 0, 1)
        layout.addWidget(self.value, 0, 2)
        layout.addWidget(self.go, 0, 3)
        limits = QHBoxLayout()
        limits.addWidget(QLabel("−180°"))
        limits.addStretch()
        limits.addWidget(self.actual)
        limits.addStretch()
        limits.addWidget(QLabel("+180°"))
        layout.addLayout(limits, 1, 1, 1, 3)
        self.slider.valueChanged.connect(lambda v: self.value.setValue(v / 10))
        self.value.valueChanged.connect(lambda v: self.slider.setValue(round(v * 10)))
        # Mouse/touch drag sends only on release. Keyboard/track clicks use Move.
        self.slider.sliderReleased.connect(lambda: send(axis, self.value.value()))
        self.go.clicked.connect(lambda: send(axis, self.value.value()))

    def set_controls_enabled(self, enabled):
        for control in (self.slider, self.value, self.go):
            control.setEnabled(enabled)


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.controller = None
        self.setWindowTitle("Robot Arm Controller")
        self.resize(860, 880)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)
        title = QLabel("Robot Arm Controller")
        title.setObjectName("title")
        layout.addWidget(title)
        layout.addWidget(QLabel("ARCTOS  /  SIX JOINTS  /  USB SERIAL"))
        connection = QHBoxLayout()
        self.ports = QComboBox()
        self.ports.setMinimumWidth(300)
        self.ports.setAccessibleName("USB serial device")
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.baud = QComboBox()
        self.baud.addItems(["115200", "230400", "57600", "9600"])
        self.baud.setAccessibleName("Serial baud rate")
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        for widget in (self.ports, self.refresh_button, QLabel("Baud"), self.baud, self.connect_button):
            connection.addWidget(widget)
        layout.addLayout(connection)
        self.status = QLabel("Disconnected")
        self.status.setObjectName("status")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        recovery = QHBoxLayout()
        self.home_button = QPushButton("Home configured axes…")
        self.home_button.clicked.connect(lambda: self.recover("home"))
        self.unlock_button = QPushButton("Unlock without homing…")
        self.unlock_button.clicked.connect(lambda: self.recover("unlock"))
        recovery.addWidget(self.home_button)
        recovery.addWidget(self.unlock_button)
        recovery.addStretch()
        layout.addLayout(recovery)
        tools = QHBoxLayout()
        self.zero = QPushButton("Use current pose as zero")
        self.zero.clicked.connect(self.set_zero)
        self.enable = QPushButton("Enable motion")
        self.enable.clicked.connect(self.toggle_arm)
        self.feed = QDoubleSpinBox()
        self.feed.setRange(1, 600)
        self.feed.setValue(60)
        self.feed.setDecimals(0)
        self.feed.setSuffix(" °/min")
        tools.addWidget(self.zero)
        tools.addWidget(self.enable)
        tools.addStretch()
        tools.addWidget(QLabel("Speed"))
        tools.addWidget(self.feed)
        layout.addLayout(tools)
        layout.addWidget(QLabel("Drag and release to move. For typed angles or keyboard changes, click Move."))
        self.rows = []
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setMinimumHeight(240)
        joints = QWidget()
        joint_layout = QVBoxLayout(joints)
        joint_layout.setContentsMargins(0, 0, 0, 0)
        joint_layout.setSpacing(8)
        for axis in AXES:
            row = JointRow(axis, self.move)
            self.rows.append(row)
            joint_layout.addWidget(row)
        joint_layout.addStretch()
        scroll.setWidget(joints)
        layout.addWidget(scroll, 1)
        stop_row = QHBoxLayout()
        self.stop_button = QPushButton("STOP MOTION  ·  Esc")
        self.stop_button.setObjectName("stop")
        self.stop_button.setMinimumHeight(42)
        self.stop_button.clicked.connect(self.stop)
        stop_row.addWidget(self.stop_button)
        layout.addLayout(stop_row)
        note = QLabel("Stop cancels a jog and requests feed hold. Use a physical emergency stop for power cutoff.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(500)
        self.console.setMaximumHeight(70)
        self.console.setPlaceholderText("Connection messages and command responses")
        layout.addWidget(self.console)
        self.escape = QShortcut(QKeySequence("Escape"), self)
        self.escape.activated.connect(self.stop)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)
        self.refresh_ports()
        self.render()

    def log(self, message):
        self.console.appendPlainText(f"{datetime.now():%H:%M:%S}  {message}")

    def refresh_ports(self):
        previous = self.ports.currentData()
        self.ports.clear()
        try:
            for port in sorted(list_ports.comports(), key=lambda p: p.device):
                if sys.platform == "darwin" and port.device.startswith("/dev/tty."):
                    continue
                self.ports.addItem(f"{port.description} — {port.device}", port.device)
        except (OSError, serial.SerialException) as exc:
            self.log(f"Could not list serial ports: {exc}")
        self.ports.addItem("Demo arm — no hardware", "demo")
        index = self.ports.findData(previous)
        if index >= 0:
            self.ports.setCurrentIndex(index)

    def toggle_connection(self):
        if self.controller:
            self.disconnect()
            return
        transport = None
        try:
            device = self.ports.currentData()
            if device == "demo":
                transport = DemoTransport()
            else:
                transport = serial.Serial(
                    port=None, baudrate=int(self.baud.currentText()), timeout=0,
                    write_timeout=.05, exclusive=True,
                )
                transport.dtr = False
                transport.rts = False
                transport.port = device
                transport.open()
            self.controller = Controller(transport, self.log)
            self.log(f"Connected to {device}; waiting for startup.")
        except (OSError, ValueError, serial.SerialException) as exc:
            if transport:
                transport.close()
            self.log(str(exc))
            QMessageBox.warning(self, "Connection failed", str(exc))
        self.render()

    def disconnect(self):
        if self.controller:
            try:
                self.controller.close()
            except (OSError, ValueError) as exc:
                self.log(str(exc))
            self.controller = None
        self.log("Disconnected. Session zero cleared.")
        self.render()

    def set_zero(self):
        if not self.controller or not self.controller.idle:
            return
        answer = QMessageBox.question(
            self, "Set session zero",
            "Use the current pose as 0° for all six joints? This does not move the arm.\n\n"
            "The firmware must already be calibrated so one coordinate unit equals one joint degree "
            "($100–$105, including gearing). Confirm your sensor/limit configuration before moving.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Yes:
            try:
                self.controller.set_zero()
                for row in self.rows:
                    row.value.setValue(0)
            except ValueError as exc:
                self.log(str(exc))
        self.render()

    def recover(self, kind):
        c = self.controller
        if not c or not c.can_recover:
            return
        if kind == "home":
            title = "Home configured axes"
            message = (
                "This sends $H and moves the arm toward its homing sensors.\n\n"
                "Verify sensor wiring, homing directions, and clear travel first. The Arctos v2 "
                "default cycle homes Z, then X/Y; it does not establish A/B/C home positions. "
                "The axes moved depend on your firmware build.\n\n"
                "Stop/Esc will reset the controller to abort homing. Start homing?"
            )
        else:
            title = "Unlock without homing"
            message = (
                "This sends $X to clear the alarm lock without moving or homing the arm. "
                "It does not establish valid machine coordinates.\n\n"
                "Soft limits remain enabled if configured and may reject moves from an unhomed "
                "position. Session zero does not replace machine homing. Use this only for "
                "controlled commissioning with the physical position understood.\n\n"
                "Motion stays disabled until you set session zero and enable it. Unlock?"
            )
        answer = QMessageBox.question(self, title, message,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel)
        if answer == QMessageBox.StandardButton.Yes and self.controller is c:
            try:
                c.recover(kind)
            except ValueError as exc:
                self.log(str(exc))
        self.render()

    def toggle_arm(self):
        if self.controller:
            if self.controller.armed:
                self.stop()
            else:
                try:
                    self.controller.arm()
                except ValueError as exc:
                    self.log(str(exc))
        self.render()

    def move(self, axis, angle):
        if self.controller:
            try:
                self.controller.move(axis, angle, self.feed.value())
            except ValueError as exc:
                self.log(str(exc))
        self.render()

    def stop(self):
        if self.controller:
            self.controller.stop()
        self.render()

    def tick(self):
        if self.controller:
            self.controller.tick()
        self.render()

    def render(self):
        c = self.controller
        connected = c is not None
        self.connect_button.setText("Disconnect" if connected else "Connect")
        for widget in (self.ports, self.baud, self.refresh_button):
            widget.setEnabled(not connected)
        self.zero.setEnabled(bool(c and c.idle))
        self.home_button.setEnabled(bool(c and c.can_recover and c.settings.get("$22") == "1"))
        self.unlock_button.setEnabled(bool(c and c.can_recover and c.state == "Alarm"))
        self.enable.setEnabled(bool(c and (c.armed or (c.idle and c.origin is not None))))
        self.enable.setText("Disable motion" if c and c.armed else "Enable motion")
        self.stop_button.setEnabled(connected)
        self.feed.setEnabled(bool(c and c.idle))
        if not c:
            text = "Disconnected — choose a USB device or try the demo arm."
        elif c.error:
            text = c.error
        elif c.phase != "ready":
            text = "Connecting — verifying firmware and position reporting…"
        elif c.operation == "home":
            text = "Homing configured axes — Stop/Esc resets the controller to abort."
        elif c.operation == "unlock":
            text = "Unlock requested — waiting for a fresh Idle report."
        elif c.state == "Alarm":
            text = "Connected — controller locked. Home configured axes or explicitly unlock without homing."
        elif c.origin is None:
            text = f"{c.state} — use the current pose as zero to begin."
        elif c.moving:
            text = "Moving — waiting for the arm to finish. Stop is always available."
        else:
            text = f"{c.state} — " + ("motion enabled" if c.armed else "motion disabled")
        if c and isinstance(c.transport, DemoTransport):
            text = "DEMO  |  " + text
        self.status.setText(text)
        for i, row in enumerate(self.rows):
            row.set_controls_enabled(bool(c and c.can_move))
            if c and c.position is not None and c.origin is not None:
                row.actual.setText(f"Reported: {c.position[i] - c.origin[i]:+.1f}°")
            else:
                row.actual.setText("Reported: —")

    def closeEvent(self, event):
        self.timer.stop()
        self.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Arm Controller")
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QWidget { background: #f3f5f7; color: #192c3d; font-size: 13px; }
        QLabel#title { font-size: 27px; font-weight: 700; }
        QLabel#axis { font-size: 24px; font-weight: 700; color: #087f83; }
        QLabel#status { background: #dfeef0; border-radius: 6px; padding: 10px; }
        QFrame#joint { background: white; border: 1px solid #dce3e8; border-radius: 8px; }
        QFrame#joint QLabel { background: transparent; }
        QPushButton { padding: 7px 10px; background: white; border: 1px solid #b7c4cd; border-radius: 5px; }
        QPushButton:hover { background: #e4f2f1; }
        QPushButton:disabled { color: #929da5; background: #edf0f2; }
        QPushButton#stop { background: #b83838; color: white; font-weight: 700; }
        QComboBox, QDoubleSpinBox { padding: 5px; background: white; }
        QSlider::groove:horizontal { height: 5px; background: #dce3e8; border-radius: 2px; }
        QSlider::sub-page:horizontal { background: #17898b; }
        QSlider::handle:horizontal { background: #087f83; width: 16px; margin: -6px 0; border-radius: 8px; }
        QSlider::handle:horizontal:disabled { background: #a6b7bc; }
        QPlainTextEdit { background: #e8edf1; border: none; border-radius: 5px; font-family: monospace; }
    """)
    window = Window()
    window.show()
    return app.exec()
