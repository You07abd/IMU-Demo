import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sources import ATTITUDE, GPS, LIDAR, Telemetry, MavlinkSource


class Fake:
    def __init__(self, msg_type, **fields):
        self._type = msg_type
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


class TestMavlinkHandle(unittest.TestCase):
    def setUp(self):
        self.src = MavlinkSource(port=None)
        self.t = Telemetry()

    def test_attitude_radians_to_degrees(self):
        msg = Fake('ATTITUDE', roll=math.radians(10.0), pitch=math.radians(-5.0),
                   yaw=math.radians(90.0))
        self.src._handle(msg, self.t, now=100.0)
        self.assertAlmostEqual(self.t.roll, 10.0, places=4)
        self.assertAlmostEqual(self.t.pitch, -5.0, places=4)
        self.assertAlmostEqual(self.t.yaw, 90.0, places=4)
        self.assertTrue(self.t.yaw_valid)
        self.assertFalse(self.t.is_stale(ATTITUDE, now=100.1))

    def test_gps_raw_int_scaling_and_fix(self):
        msg = Fake('GPS_RAW_INT', fix_type=3, lat=301234567, lon=311234567,
                   alt=12345, vel=250, satellites_visible=9)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.fix_type, '3D')
        self.assertAlmostEqual(self.t.lat, 30.1234567, places=6)
        self.assertAlmostEqual(self.t.lon, 31.1234567, places=6)
        self.assertAlmostEqual(self.t.alt_m, 12.345, places=3)
        self.assertAlmostEqual(self.t.speed_ms, 2.5, places=3)
        self.assertEqual(self.t.sats, 9)
        self.assertFalse(self.t.is_stale(GPS, now=100.1))

    def test_gps_no_fix(self):
        msg = Fake('GPS_RAW_INT', fix_type=1, lat=0, lon=0, alt=0,
                   vel=65535, satellites_visible=0)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.fix_type, 'NO FIX')
        self.assertEqual(self.t.speed_ms, 0.0)  # vel=65535 means unknown

    def test_distance_sensor_yaw_orientations(self):
        # orientation 0..7 = yaw 0,45,...,315 deg; distance in cm
        msg = Fake('DISTANCE_SENSOR', orientation=2, current_distance=150)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.lidar_returns, [(90.0, 1.5)])
        self.assertFalse(self.t.is_stale(LIDAR, now=100.1))

    def test_distance_sensor_non_yaw_orientation_ignored(self):
        msg = Fake('DISTANCE_SENSOR', orientation=25, current_distance=150)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.lidar_returns, [])
        self.assertTrue(self.t.is_stale(LIDAR, now=100.1))

    def test_obstacle_distance_sweep(self):
        distances = [65535] * 72
        distances[0] = 200    # bearing angle_offset + 0*increment
        distances[10] = 300   # bearing angle_offset + 10*increment
        msg = Fake('OBSTACLE_DISTANCE', distances=distances, increment=5,
                   angle_offset=-10.0, min_distance=20, max_distance=1000)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.lidar_returns,
                         [((-10.0) % 360.0, 2.0), ((-10.0 + 50.0) % 360.0, 3.0)])

    def test_obstacle_distance_out_of_range_dropped(self):
        distances = [65535] * 72
        distances[0] = 1001   # beyond max_distance -> no obstacle
        distances[1] = 10     # below min_distance -> invalid
        msg = Fake('OBSTACLE_DISTANCE', distances=distances, increment=5,
                   angle_offset=0.0, min_distance=20, max_distance=1000)
        self.src._handle(msg, self.t, now=100.0)
        self.assertEqual(self.t.lidar_returns, [])

    def test_unknown_message_ignored(self):
        self.src._handle(Fake('HEARTBEAT', custom_mode=0), self.t, now=100.0)
        self.assertTrue(self.t.is_stale(ATTITUDE, now=100.1))


if __name__ == '__main__':
    unittest.main()
