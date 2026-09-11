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
4. At your chosen reference pose, click **Use current pose as zero** and confirm
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
setup tool before reconnecting. It does not automatically unlock alarms or home
the arm: the release's default homing configuration does not home all six axes.

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
  firmware limits and never sends alarm unlock or disables them.
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

Tests cover all six command letters, bounds, non-finite values, session zero,
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
