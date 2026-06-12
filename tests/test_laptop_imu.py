import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sources import ATTITUDE, GPS, LIDAR, Telemetry
from laptop_imu import accel_to_roll_pitch, probe_laptop_imu


def make_fake_iio(root, raw=(0, 0, 256), scale=0.0383):
    dev = os.path.join(root, 'iio:device0')
    os.makedirs(dev)
    for axis, value in zip('xyz', raw):
        with open(os.path.join(dev, f'in_accel_{axis}_raw'), 'w') as f:
            f.write(f'{value}\n')
    with open(os.path.join(dev, 'in_accel_scale'), 'w') as f:
        f.write(f'{scale}\n')
    return dev


class TestAccelMath(unittest.TestCase):
    def test_flat_is_level(self):
        roll, pitch = accel_to_roll_pitch(0.0, 0.0, 9.8)
        self.assertAlmostEqual(roll, 0.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)

    def test_x_tilt_is_roll(self):
        g = 9.8
        roll, pitch = accel_to_roll_pitch(g * math.sin(math.radians(30)), 0.0,
                                          g * math.cos(math.radians(30)))
        self.assertAlmostEqual(roll, 30.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)

    def test_y_tilt_is_pitch(self):
        g = 9.8
        roll, pitch = accel_to_roll_pitch(0.0, g * math.sin(math.radians(20)),
                                          g * math.cos(math.radians(20)))
        self.assertAlmostEqual(roll, 0.0, places=4)
        self.assertAlmostEqual(pitch, 20.0, places=4)

    def test_zero_vector_is_level(self):
        self.assertEqual(accel_to_roll_pitch(0.0, 0.0, 0.0), (0.0, 0.0))


class TestProbeAndSource(unittest.TestCase):
    def test_probe_empty_dir_returns_none(self):
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(probe_laptop_imu(root=root))
            self.assertIsNone(probe_laptop_imu(root=root, dry_run=True))

    def test_probe_missing_root_returns_none(self):
        self.assertIsNone(probe_laptop_imu(root='/nonexistent/path'))

    def test_probe_dry_run_true_when_accel_present(self):
        with tempfile.TemporaryDirectory() as root:
            make_fake_iio(root)
            self.assertTrue(probe_laptop_imu(root=root, dry_run=True))

    def test_source_reads_attitude_only(self):
        with tempfile.TemporaryDirectory() as root:
            # 30 deg x-tilt: raw such that raw*scale = g*sin(30), g*cos(30)
            scale = 0.01
            gx = 9.8 * math.sin(math.radians(30)) / scale
            gz = 9.8 * math.cos(math.radians(30)) / scale
            make_fake_iio(root, raw=(int(round(gx)), 0, int(round(gz))), scale=scale)
            src = probe_laptop_imu(root=root)
            self.assertIsNotNone(src)
            self.assertEqual(src.capabilities, {ATTITUDE})
            src.start()
            t = Telemetry()
            src.poll(t, now=100.0)
            self.assertAlmostEqual(t.roll, 30.0, places=1)
            self.assertAlmostEqual(t.pitch, 0.0, places=1)
            self.assertFalse(t.yaw_valid)
            self.assertFalse(t.is_stale(ATTITUDE, now=100.1))
            self.assertTrue(t.is_stale(GPS, now=100.1))
            self.assertTrue(t.is_stale(LIDAR, now=100.1))


if __name__ == '__main__':
    unittest.main()
