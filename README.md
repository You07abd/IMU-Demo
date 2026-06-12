# IMU Flight Monitor

A small cross-platform dashboard that shows a real-time artificial horizon (PFD) with a crash-warning overlay, plus optional GPS and lidar panels. It reads from **any MAVLink flight controller** (ArduPilot or PX4 — Pixhawk, Cube, Kakute, SpeedyBee, …), from your **laptop's own IMU** if it has one, or from a built-in **simulation** so it works with no hardware at all.

Display only — it never sends commands that change autopilot state.

Works on Windows, macOS, and Linux.

## Install

```
pip install imu-flight-monitor
imu-demo
```

Or run from source:

```
pip install -r requirements.txt
python3 imu_demo.py
```

Pick **Demo: Simulated** from the Source dropdown and press **Connect**. You'll get a gentle flight with a crash warning every ~30 s, a GPS ground track, and a lidar map painting a virtual room.

On Linux you may first need `sudo apt install python3-tk`; tkinter ships with standard Python on Windows and macOS.

## Sources

| Source | What you get |
|---|---|
| **Serial port** (flight controller) | Attitude from `ATTITUDE`; GPS panel from `GPS_RAW_INT`; lidar map from `DISTANCE_SENSOR` / `OBSTACLE_DISTANCE`. Panels for sensors your FC doesn't have simply grey out — the horizon works with nothing but the IMU. |
| **Demo: Laptop IMU** | Appears only on machines with a readable built-in IMU (many 2-in-1s/convertibles). Tilt the laptop and the horizon follows. Yaw shows `—`; GPS/lidar stay greyed. On Windows this needs `pip install winsdk`. |
| **Demo: Simulated** | Always available. Full fake flight: attitude, GPS circuit, lidar room scan. |

## Connecting a flight controller

1. Plug the FC in over USB (no motors/props needed — display only).
2. Open the Source dropdown — ports refresh each time you open it. A likely flight controller is pre-selected when one is found.
3. Press **Connect**. The link indicator turns green and shows the attitude message rate.

Any board that speaks MAVLink over serial works. The app asks the FC for the message streams it displays (`MAV_CMD_SET_MESSAGE_INTERVAL`, with the legacy stream request as fallback).

The Source box also accepts a typed MAVLink connection string — e.g. `udpin:0.0.0.0:14551` to read a MAVProxy UDP output while QGC or MAVProxy keeps the serial port.

## The display

- **PFD** (left): artificial horizon with pitch ladder, roll arc, and heading tape. Goes grey with **NO DATA** if attitude stops arriving for 1 s.
- **ATTITUDE / GPS / LIDAR MAP cards** (right): each greys out independently after 2 s without data, or shows *not available* if the source can never provide it.
- **GPS card**: fix type, satellites, position, altitude, speed, and an offline north-up track plot (first fix = origin). Click the card to pop it out into a resizable window.
- **LIDAR MAP card**: accumulates rangefinder/proximity returns into a north-up point map, oriented by the vehicle's yaw — keep the vehicle still and rotate it, and the outline of the room paints itself. **Clear** resets the map; click the card to pop it out.
- **Status bar**: green **SAFE** / red **CRASH WARNING** with the live angles. One beep plays each time a warning begins.

## Warning thresholds

Edit the constants at the top of `safety.py`:

| Constant           | Default | Meaning                                    |
|--------------------|---------|--------------------------------------------|
| `PITCH_MIN_DEG`    | -30     | Nose-down limit (degrees)                  |
| `PITCH_MAX_DEG`    | 35      | Nose-up limit (degrees)                    |
| `ROLL_MAX_ABS_DEG` | 45      | Maximum roll in either direction (degrees) |

## Troubleshooting

- **"port busy or no permission"** — close Mission Planner / QGroundControl (or anything else holding the port). On Linux, add yourself to the serial group: `sudo usermod -aG dialout $USER` (log out and back in).
- **Wrong/stale port selected** — re-open the dropdown to refresh; on Linux, stable `/dev/serial/by-id/...` paths are listed when available, so the right board is easy to spot.
- **"no MAVLink heartbeat"** — wrong port, wrong baud device, or the FC is still booting; wait a few seconds and reconnect.
- **No "Demo: Laptop IMU" entry** — your machine has no readable IMU (most regular laptops and desktops don't). Use the simulator instead.

## Code layout

| File | Purpose |
|---|---|
| `imu_demo.py` | Entry point: window, toolbar, poll loop |
| `pfd.py` | Artificial-horizon canvas |
| `panels.py` | Attitude / GPS / lidar cards |
| `safety.py` | Warning thresholds + beep |
| `sources.py` | Telemetry model, MAVLink source, simulator |
| `laptop_imu.py` | Laptop IMU probes (Linux iio, Windows sensors) |

## Publishing a new version

1. Bump `version` in `pyproject.toml`.
2. Build and upload:
   ```
   pip install build twine
   python3 -m build
   twine upload dist/*
   ```
   `twine` will ask for your PyPI credentials the first time (free account at pypi.org).

## Tests

```
python3 -m unittest discover tests
```

No extra packages needed — the tests use Python's built-in `unittest`.

## Repository notes

Telemetry logs (`*.tlog`, `*.tlog.raw`) and parameter dumps (`*.parm`) are git-ignored — they can grow to tens of megabytes per session.

## License

[MIT](LICENSE)
