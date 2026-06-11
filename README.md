# Pixhawk IMU Demo — Classroom Safety Monitor

Real-time artificial-horizon (PFD) display with a crash-warning overlay, driven by MAVLink ATTITUDE messages read directly from a Pixhawk over USB. Display only — it never sends commands to the autopilot.

Works on Windows, macOS, and Linux.

## Hardware

- Pixhawk (e.g. 6c) running ArduPilot, connected to the computer via USB
- No motors, propellers, or servos — display only

## Software requirements

- Python 3
- `pip install -r requirements.txt` (pymavlink + pyserial)
- `tkinter` — included with standard Python on Windows and macOS; on Linux: `sudo apt install python3-tk`

The alert beep uses whatever the OS provides (`winsound` on Windows, `afplay` on macOS, `paplay` on Linux, with Tk's bell as a fallback) — no extra install needed.

## Run

1. Connect the Pixhawk via USB.
2. ```
   python3 imu_demo.py
   ```

The serial port is autodetected from the USB device description. If detection picks the wrong port (or fails), set `MAV_PORT` at the top of `imu_demo.py` to your port, e.g. `'COM5'` on Windows or `'/dev/ttyACM0'` on Linux.

> **Note:** the script opens the serial port directly, so close Mission Planner / QGroundControl (or anything else holding the port) before running it. If telemetry stops, the display greys out and shows **NO DATA** after one second.

## Thresholds

Edit the constants at the top of `imu_demo.py` to adjust when the warning triggers:

| Constant          | Default | Meaning                                        |
|-------------------|---------|------------------------------------------------|
| `PITCH_MIN_DEG`   | -30     | Nose-down limit (degrees)                      |
| `PITCH_MAX_DEG`   | 35      | Nose-up limit (degrees)                        |
| `ROLL_MAX_ABS_DEG`| 45      | Maximum roll in either direction (degrees)     |

## What the display shows

- **SAFE** (green banner): roll and pitch are within all thresholds.
- **CRASH WARNING** (red banner): at least one threshold is exceeded.
- An audible beep plays once each time the warning state is first triggered; it does not repeat until the aircraft returns to safe and exceeds a threshold again.
- **NO DATA** (grey status bar): no ATTITUDE message received for over a second.

## Tests

```
python3 -m unittest discover tests
```

No extra packages needed — the tests use Python's built-in `unittest`.

## Repository notes

Telemetry logs (`*.tlog`, `*.tlog.raw`) and parameter dumps (`*.parm`) are git-ignored — they can grow to tens of megabytes per session.

## License

[MIT](LICENSE)
