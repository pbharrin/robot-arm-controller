"""Transport-independent GRBL session. No motion is queued or retried."""
import math
import time

AXES = "XYZABC"


def coordinates(value):
    result = tuple(float(x) for x in value.split(","))
    if len(result) != 6 or not all(math.isfinite(x) for x in result):
        raise ValueError("Expected six finite joint coordinates")
    return result


def jog_command(axis, angle, origin, feed):
    if axis not in tuple(AXES) or not math.isfinite(angle) or not -180 <= angle <= 180:
        raise ValueError("Joint target must be between -180 and +180 degrees")
    if not math.isfinite(origin) or not math.isfinite(feed) or not 1 <= feed <= 600:
        raise ValueError("Invalid zero position or feed rate (1–600 degrees/minute)")
    # G21 prevents inch conversion on X/Y/Z; numeric units must be calibrated degrees.
    # G53 bypasses G54/G92 offsets; zero is a local session reference, not EEPROM.
    return f"$J=G21 G90 G53 {axis}{origin + angle:.3f} F{feed:.1f}\n".encode("ascii")


class Controller:
    def __init__(self, transport, log=lambda message: None, clock=time.monotonic):
        self.transport, self.log, self.clock = transport, log, clock
        self.buffer = bytearray()
        self.phase = "boot"
        self.state = "Connecting"
        self.error = ""
        self.settings = {}
        self.axes_verified = False
        self.position = None
        self.origin = None
        self.armed = False
        self.pending = None
        self.moving = False
        self.last_status = -math.inf
        self.last_poll = -math.inf
        self.deadline = clock() + 2.0  # Arduino USB opening may reset the board.
        self.ack_time = None

    @property
    def idle(self):
        return (self.phase == "ready" and not self.error and self.state == "Idle"
                and not self.pending and not self.moving and self.position is not None
                and self.clock() - self.last_status < 2)

    @property
    def can_move(self):
        return self.idle and self.armed and self.origin is not None

    def write(self, data):
        if self.transport.write(data) != len(data):
            raise OSError("Incomplete serial write; reconnect before continuing")
        if data != b"?":
            self.log("> " + (data.decode("ascii", errors="backslashreplace").strip()))

    def command(self, data, kind):
        if self.pending:
            raise ValueError("A command is still awaiting acknowledgement")
        self.pending = kind
        self.deadline = self.clock() + 5
        self.write(data)

    def fail(self, message):
        if not self.error:
            try:
                self.write(b"\x85!")  # Cancel jog, or hold any unexpected non-jog motion.
            except (OSError, ValueError):
                pass
            self.log(message)
        self.error = message
        self.armed = False
        self.origin = None
        self.phase = "fault"

    def tick(self):
        if self.phase == "closed":
            return
        try:
            chunk = self.transport.read(4096)
            self.buffer.extend(chunk)
            if len(self.buffer) > 32768:
                raise ValueError("Serial input exceeded the receive buffer")
            while b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                self.receive(line.decode("ascii", errors="replace").strip())
            now = self.clock()
            if self.phase == "boot" and now >= self.deadline:
                self.phase = "settings"
                self.command(b"$$\n", "settings")
            elif self.pending and now >= self.deadline:
                self.fail("GRBL acknowledgement timed out. Reconnect; command was not retried.")
            if self.phase != "boot" and now - self.last_poll >= .25:
                self.write(b"?")
                self.last_poll = now
            if self.phase == "ready" and now - self.last_status >= 2:
                self.fail("Position reports stopped. Motion disabled; reconnect.")
        except (OSError, ValueError) as exc:
            self.fail(str(exc))

    def receive(self, line):
        if not line:
            return
        if not line.startswith("<"):
            self.log("< " + line)
        if line.startswith("Grbl") and self.phase != "boot":
            self.fail("Controller restarted. Reconnect and establish zero again.")
            return
        if line.startswith(("error:", "ALARM:")):
            self.fail(f"Controller reported {line}. Resolve the cause and reconnect.")
            return
        if self.error:
            return
        if line.startswith("$") and "=" in line:
            key, value = line.split("=", 1)
            self.settings[key] = value.split()[0]
        elif line.startswith("[AXS:"):
            self.axes_verified = line == "[AXS:6:XYZABC]"
        elif line.startswith("<") and line.endswith(">"):
            fields = line[1:-1].split("|")
            self.state = fields[0]
            if self.state.split(":")[0] in ("Alarm", "Door", "Hold", "Sleep", "Check"):
                self.fail(f"Controller is in {self.state}. Resolve the cause and reconnect.")
                return
            values = dict(field.split(":", 1) for field in fields[1:] if ":" in field)
            if "MPos" in values:
                self.position = coordinates(values["MPos"])
                self.last_status = self.clock()
            # Only accept completion after a post-ack poll can have been sent.
            if (self.moving and not self.pending and self.ack_time is not None
                    and self.last_poll > self.ack_time and self.state == "Idle"):
                self.moving = False
        elif line == "ok":
            kind, self.pending = self.pending, None
            if kind == "settings":
                if self.settings.get("$13") != "0":
                    self.fail("Requires $13=0 (non-inch reports). Set it in your GRBL setup tool and reconnect.")
                elif not int(self.settings.get("$10", "0")) & 1:
                    self.fail("Requires machine position reports ($10 bit 0 = 1). Configure and reconnect.")
                else:
                    self.phase = "identity"
                    self.command(b"$I\n", "identity")
            elif kind == "identity":
                if not self.axes_verified:
                    self.fail("Expected Arctos axis report [AXS:6:XYZABC]. Check firmware.")
                else:
                    self.phase = "ready"
                    # Allow first fresh position report two seconds to arrive.
                    self.last_status = self.clock()
            elif kind == "jog":
                self.ack_time = self.clock()

    def set_zero(self):
        if not self.idle:
            raise ValueError("Wait for a fresh Idle position report before setting zero")
        self.origin = tuple(self.position)
        self.armed = False
        self.log("Current pose captured as session zero; no command sent.")

    def arm(self):
        if not self.idle or self.origin is None:
            raise ValueError("Set zero while the controller is idle first")
        self.armed = True

    def move(self, axis, angle, feed):
        if not self.can_move:
            raise ValueError("Motion is disabled or another move is in progress")
        data = jog_command(axis, angle, self.origin[AXES.index(axis)], feed)
        self.moving = True
        self.ack_time = None
        try:
            self.command(data, "jog")
        except (OSError, ValueError) as exc:
            self.fail(str(exc))

    def stop(self):
        self.armed = False
        try:
            self.write(b"\x85!")
        except (OSError, ValueError) as exc:
            self.fail(str(exc))
        # Keep pending acknowledgement/completion barrier: never race a cancelled jog.
        self.log("Stop requested. Re-enable motion only after Idle.")

    def close(self):
        try:
            self.stop()
        finally:
            self.transport.close()
            self.phase = "closed"
            self.origin = None


class DemoTransport:
    """In-memory six-axis device for exploring the app without USB hardware."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.updated = clock()
        self.position = [0.0] * 6
        self.target = list(self.position)
        self.feed = 60.0
        self.output = bytearray(b"Grbl 1.1 demo\n")
        self.closed = False

    def advance(self):
        now = self.clock()
        step = (now - self.updated) * self.feed / 60
        self.updated = now
        for i in range(6):
            delta = self.target[i] - self.position[i]
            self.position[i] += max(-step, min(step, delta))

    def write(self, data):
        self.advance()
        if data == b"?":
            state = "Idle" if self.position == self.target else "Jog"
            pos = ",".join(f"{x:.3f}" for x in self.position)
            self.output.extend(f"<{state}|MPos:{pos}>\n".encode())
        elif data == b"$$\n":
            self.output.extend(b"$10=1\n$13=0\nok\n")
        elif data == b"$I\n":
            self.output.extend(b"[VER:1.1:DEMO]\n[AXS:6:XYZABC]\nok\n")
        elif data.startswith(b"$J="):
            for word in data.decode().split():
                if word[0] in AXES:
                    self.target[AXES.index(word[0])] = float(word[1:])
                elif word[0] == "F":
                    self.feed = float(word[1:])
            self.output.extend(b"ok\n")
        elif b"\x85" in data or b"!" in data:
            self.target = list(self.position)
        return len(data)

    def read(self, size):
        result = bytes(self.output[:size])
        del self.output[:size]
        return result

    def close(self):
        self.closed = True
