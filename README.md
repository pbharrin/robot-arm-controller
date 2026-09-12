# Robot Arm Controller

A simple macOS desktop app for an Arctos six-axis Arduino robot arm. Built with
Python, Qt (PySide6), and pySerial, with an Apple Silicon `.app` build for an M2 Mac.

## Get the app

If you received this as a source ZIP: extract it, open Terminal in the extracted
`robot-arm-controller` folder, and run `bash scripts/build_macos.sh` with an ARM64
Python 3.12 installed. The build produces the `.app` in `dist/`. The GitHub download
steps below apply once this source has been uploaded and its workflow succeeds.

1. Open [GitHub Actions](https://github.com/pbharrin/robot-arm-controller/actions/workflows/macos.yml).
2. Open the latest successful **Build Apple Silicon app** run.
3. Download **Robot-Arm-Controller-macOS-arm64** under Artifacts (sign in to GitHub).
4. Extract the artifact, then extract the enclosed ZIP. Move **Robot Arm Controller.app** to Applications.
5. Open it. This personal build is ad-hoc signed, not Apple Developer signed or
   notarized. If macOS blocks the first launch, use **System Settings → Privacy &
   Security → Open Anyway** after checking that it is the build from this repository.

The app bundles Python and its dependencies. The packaged build targets Apple
Silicon and macOS 14 or later. For an older OS, build locally with a compatible
Python/Qt toolchain; the current pinned Qt version requires macOS 12 or later.

## Use it

1. Connect the Arduino by USB and close Arduino Serial Monitor or any other sender.
2. Select its serial device from the dropdown and leave baud at **115200**.
   macOS uses names such as `/dev/cu.usbmodem…` or `/dev/cu.usbserial…`, rather than COM numbers.
   Use **Refresh** after plugging in a device.
3. Click **Connect**. The app waits for startup, reads settings, checks the six-axis
   firmware identity, and polls machine position. It does not move on connection.
4. If it says **controller locked**, use the recovery controls described below.
   At your chosen reference pose, click **Use current pose as zero** and confirm
   the calibration requirement. This captures a local reference; it does not move,
   home, or change the Arduino's coordinate offsets.
5. Click **Enable motion**. Drag an X, Y, Z, A, B, or C slider and release to move
   that joint. All targets span **−180° to +180°** relative to the captured pose,
   with 0.1° increments. For typed targets or slider keyboard/track changes, click **Move**.
6. Only one move is allowed at a time. The readout shows the controller-reported
   angle; the slider remains your requested target. A GRBL `ok` means accepted,
   so controls remain locked until the controller subsequently reports `Idle`.
7. **Stop Motion** or **Esc** cancels jogging and requests feed hold. It disables
   further motion until you explicitly re-enable it. After cancelling, the slider
   can differ from the reported position; pressing Move requests that target again.

Speed defaults to **60°/minute (1°/second)** and is adjustable from 1–600°/minute.
Start with a small move when commissioning. The **Demo arm — no hardware** option
lets you try the complete connection/zero/slider/stop flow without USB.

## Firmware and units

This app targets [Arctos-grbl-v0.1 release v2](https://github.com/Arctos-Robotics/Arctos-grbl-v0.1/releases/tag/v2),
whose source reports six axes in `XYZABC` order and supports GRBL 1.1 jogging.
It expects the `$I` response `[AXS:6:XYZABC]`.

**One firmware coordinate unit must equal one joint degree for every axis.** The
firmware's generic X/Y/Z defaults are not proof of this calibration. Verify
`$100`–`$105` as steps per output-joint degree, including microstepping and gearing:

```
steps per degree = motor full steps per revolution × microsteps × gear ratio / 360
```

Mechanical coupling compensation remains the firmware's responsibility (for
example, Arctos COREBC). This app sends joint coordinates, not individual motor
pulses or Cartesian/inverse-kinematics commands. Match the firmware release to
your arm's mechanical version.

The app reads but never changes EEPROM settings. It requires:

- `$13=0`: non-inch position reporting.
- `$10` bit 0 enabled: machine position (`MPos`) in status reports. For example,
  `$10=1` enables this; preserve other bits needed by your setup.
- Six finite machine-position values in every accepted position report.

If a setting is incompatible, the app explains what to configure using your GRBL
setup tool before reconnecting. It never automatically unlocks or homes the arm.

### Startup alarm lock and recovery

With homing enabled (`$22=1`), a normal startup can announce
`[MSG:'$H'|'$X' to unlock]` and report `<Alarm|MPos:…>`. This is not a USB failure.
The app completes the `$$` / `$I` handshake while locked and keeps sliders disabled.

- **Home configured axes…** sends `$H` only after confirmation. It moves the arm;
  verify sensor wiring, directions, and clear travel first. The linked v2 source
  defaults to Z followed by X/Y, with A/B/C excluded from the default cycle.
  Your flashed firmware determines the actual sequence. No automatic all-six-axis
  homing or configuration change is attempted.
- **Unlock without homing…** sends `$X` only after confirmation. This does not
  move or home the arm and does not establish valid machine coordinates. It is
  intended for controlled commissioning, not a substitute for homing. Soft limits
  remain in force and may reject moves from an unhomed position.
- Both actions clear session zero and require a new post-acknowledgement Idle
  report, then **Use current pose as zero** and **Enable motion** before jogging.
- Homing can take up to three minutes before acknowledgement. This firmware does
  not service status queries during its homing loop, so the normal two-second
  report watchdog is suspended only while `$H` is pending. The homing timeout
  triggers a reset. **Stop/Esc during homing sends Ctrl-X (reset)**, because feed
  hold and jog cancellation do not abort this firmware's homing routine. Reconnect
  after aborting. Outside homing, Stop retains jog-cancel/feed-hold behavior.
- Explicit `ALARM:n` errors (including failed homing), command errors, and transport
  failures still disable motion and require resolving the cause and reconnecting.

If your settings show `$20=1`, soft limits are enabled. `$21=0` means hard limits
are disabled, even if Hall sensors are physically installed. The app does not
change either setting or assume the sensors already provide hard-limit protection.

Example: with captured X machine coordinate 12°, requesting slider X = +5° sends:

```gcode
$J=G21 G90 G53 X17.000 F60.0
```

`G53` addresses machine coordinates, bypassing G54/G92 offsets. `G21` prevents
inch conversion of X/Y/Z; their numeric units are interpreted as degrees through
the required calibration. The jog command does not alter normal parser modes.
Each line ends with LF. Status requests use realtime `?`; stopping sends realtime
jog-cancel byte `0x85` followed by feed hold `!`.

## Limits and recovery

- The ±180° slider range is a requested target range around session zero, not a
  model of collisions or each joint's physical travel.
- Hall sensors must be wired and configured as effective limits in the firmware.
  Their presence alone does not establish hard-limit protection. The app preserves
  firmware limits and never disables them. Alarm unlock is an explicit confirmed action.
- A software stop is not a physical emergency stop or motor power cutoff. USB
  loss can prevent it reaching the board. Use the arm's physical stop for that case.
- Controller reset, alarm, communication error, invalid position, or stale reports
  disables motion and clears session zero. Resolve the cause, reconnect, and
  establish zero again. No motion command is retried or replayed after reconnect.
- Reported positions are GRBL's step-based estimate, not encoder feedback. Lost
  steps or manual movement can invalidate them.

## Run from source

Use an ARM64 Python 3.12 installation on your Mac:

```bash
git clone https://github.com/pbharrin/robot-arm-controller.git
cd robot-arm-controller
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

To build a self-contained application locally:

```bash
bash scripts/build_macos.sh
```

The output is `dist/Robot Arm Controller.app` and a ZIP in `dist/`. GitHub Actions
performs the same build and a packaged launch check on an ARM64 macOS runner.
No Apple developer certificate is embedded.

## Development and validation

```bash
python -m unittest discover -s tests -v
```

Tests cover startup-lock handshake ordering, confirmed recovery, homing stop/timeouts,
all six command letters, bounds, non-finite values, session zero,
firmware/settings checks, serial fragmentation, acknowledgement/completion ordering,
stop and reset behavior, missing reports, command timeouts, unplug/partial-write
failures, and a Qt demo slider interaction. The simulated device does not validate
motor directions, steps-per-degree calibration, sensor wiring, physical limits,
or actual USB behavior on your particular Arduino. Hardware commissioning remains
necessary before normal use.

Protocol references:
[configuration](https://github.com/Arctos-Robotics/Arctos-grbl-v0.1/blob/v2/grbl/config.h),
[jog parser](https://github.com/Arctos-Robotics/Arctos-grbl-v0.1/blob/v2/grbl/gcode.c),
[status reporting](https://github.com/Arctos-Robotics/Arctos-grbl-v0.1/blob/v2/grbl/report.c).
