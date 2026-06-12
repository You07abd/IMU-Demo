"""Laptop built-in IMU as an attitude source (tilt the laptop, the horizon moves).

Linux: industrial-IO accelerometers under /sys/bus/iio/devices (common in 2-in-1s).
Windows: inclinometer via the optional `winsdk` package.
macOS: no public sensor API — probe always fails, the option simply never appears.
"""
import glob
import math
import os
import sys

from sources import ATTITUDE, Source, Telemetry

IIO_ROOT = '/sys/bus/iio/devices'
SMOOTH = 0.35      # EMA factor; accelerometers are jittery


def accel_to_roll_pitch(ax, ay, az):
    """Gravity vector in device frame (x right, y up-screen, z out of keyboard)
    -> (roll_deg, pitch_deg). Zero vector reads as level."""
    if ax == 0.0 and ay == 0.0 and az == 0.0:
        return 0.0, 0.0
    roll = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
    pitch = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
    return roll, pitch


class IioAccelSource(Source):
    name = 'Demo: Laptop IMU'
    capabilities = frozenset({ATTITUDE})

    def __init__(self, device_dir):
        self.device_dir = device_dir
        self._smoothed = None

    def _read(self, filename, default=None):
        try:
            with open(os.path.join(self.device_dir, filename)) as f:
                return float(f.read().strip())
        except (OSError, ValueError):
            return default

    def poll(self, telemetry: Telemetry, now=None):
        scale = self._read('in_accel_scale', default=1.0)
        axes = [self._read(f'in_accel_{a}_raw') for a in 'xyz']
        if any(v is None for v in axes):
            return                      # transient sysfs hiccup; keep last values
        ax, ay, az = (v * scale for v in axes)
        roll, pitch = accel_to_roll_pitch(ax, ay, az)
        if self._smoothed is None:
            self._smoothed = (roll, pitch)
        else:
            pr, pp = self._smoothed
            self._smoothed = (pr + SMOOTH * (roll - pr), pp + SMOOTH * (pitch - pp))
        telemetry.roll, telemetry.pitch = self._smoothed
        telemetry.yaw = 0.0
        telemetry.yaw_valid = False
        telemetry.touch(ATTITUDE, now=now)


class WindowsInclinometerSource(Source):
    name = 'Demo: Laptop IMU'
    capabilities = frozenset({ATTITUDE})

    def __init__(self, inclinometer):
        self.inclinometer = inclinometer

    def poll(self, telemetry: Telemetry, now=None):
        r = self.inclinometer.get_current_reading()
        if r is None:
            return
        telemetry.roll = r.roll_degrees
        telemetry.pitch = r.pitch_degrees
        telemetry.yaw = ((r.yaw_degrees + 180.0) % 360.0) - 180.0
        telemetry.yaw_valid = True
        telemetry.touch(ATTITUDE, now=now)


def _probe_linux(root, dry_run):
    for dev in sorted(glob.glob(os.path.join(root, 'iio:device*'))):
        if all(os.path.exists(os.path.join(dev, f'in_accel_{a}_raw')) for a in 'xyz'):
            return True if dry_run else IioAccelSource(dev)
    return None


def _probe_windows(dry_run):
    try:
        from winsdk.windows.devices.sensors import Inclinometer
        inc = Inclinometer.get_default()
    except Exception:
        return None
    if inc is None:
        return None
    return True if dry_run else WindowsInclinometerSource(inc)


def probe_laptop_imu(dry_run=False, root=IIO_ROOT):
    """Return a ready laptop-IMU source (or True for dry_run); None if no IMU."""
    if sys.platform == 'win32':
        return _probe_windows(dry_run)
    return _probe_linux(root, dry_run)
