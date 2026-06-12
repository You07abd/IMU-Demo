import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from safety import is_unsafe
from sources import (
    ATTITUDE, GPS, LIDAR, STALE_AFTER_S,
    Telemetry, SimSource, lidar_to_xy, latlon_to_xy,
)


class TestTelemetryStaleness(unittest.TestCase):
    def test_never_updated_is_stale(self):
        t = Telemetry()
        self.assertTrue(t.is_stale(ATTITUDE, now=100.0))

    def test_fresh_section_not_stale(self):
        t = Telemetry()
        t.touch(ATTITUDE, now=100.0)
        self.assertFalse(t.is_stale(ATTITUDE, now=100.5))

    def test_old_section_is_stale(self):
        t = Telemetry()
        t.touch(GPS, now=100.0)
        self.assertTrue(t.is_stale(GPS, now=100.0 + STALE_AFTER_S + 0.1))

    def test_sections_age_independently(self):
        t = Telemetry()
        t.touch(ATTITUDE, now=100.0)
        t.touch(LIDAR, now=103.0)
        self.assertTrue(t.is_stale(ATTITUDE, now=103.5))
        self.assertFalse(t.is_stale(LIDAR, now=103.5))


class TestTransforms(unittest.TestCase):
    def test_lidar_straight_ahead_facing_north(self):
        x, y = lidar_to_xy(bearing_deg=0, distance_m=2.0, yaw_deg=0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 2.0, places=6)

    def test_lidar_straight_ahead_facing_east(self):
        x, y = lidar_to_xy(bearing_deg=0, distance_m=2.0, yaw_deg=90)
        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_lidar_right_beam_facing_north(self):
        x, y = lidar_to_xy(bearing_deg=90, distance_m=3.0, yaw_deg=0)
        self.assertAlmostEqual(x, 3.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_latlon_origin_is_zero(self):
        x, y = latlon_to_xy(30.0, 31.0, lat0=30.0, lon0=31.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_latlon_one_degree_north_is_about_111km(self):
        x, y = latlon_to_xy(31.0, 31.0, lat0=30.0, lon0=31.0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 111320.0, delta=200.0)

    def test_latlon_east_scales_with_cos_lat(self):
        x, y = latlon_to_xy(60.0, 1.0, lat0=60.0, lon0=0.0)
        self.assertAlmostEqual(x, 111320.0 * math.cos(math.radians(60.0)), delta=200.0)
        self.assertAlmostEqual(y, 0.0, places=3)


class TestSimSource(unittest.TestCase):
    def _run(self, seed, seconds, hz=20):
        src = SimSource(seed=seed)
        src.start()
        t = Telemetry()
        frames = []
        for i in range(int(seconds * hz)):
            now = 1000.0 + i / hz
            src.poll(t, now=now)
            frames.append((t.roll, t.pitch, t.yaw, t.lat, t.lon,
                           tuple(t.lidar_returns)))
        return t, frames

    def test_capabilities_cover_everything(self):
        self.assertEqual(SimSource(seed=0).capabilities, {ATTITUDE, GPS, LIDAR})

    def test_attitude_is_bounded(self):
        _, frames = self._run(seed=1, seconds=60)
        for roll, pitch, yaw, *_ in frames:
            self.assertLessEqual(abs(roll), 90.0)
            self.assertLessEqual(abs(pitch), 90.0)
            self.assertTrue(-180.0 <= yaw < 180.0)

    def test_mostly_safe_but_eventually_unsafe(self):
        _, frames = self._run(seed=2, seconds=60)
        flags = [is_unsafe(roll, pitch) for roll, pitch, *_ in frames]
        self.assertTrue(any(flags), 'sim never triggered the crash warning in 60 s')
        self.assertLess(sum(flags) / len(flags), 0.5, 'sim is unsafe most of the time')

    def test_deterministic_with_seed(self):
        _, a = self._run(seed=7, seconds=5)
        _, b = self._run(seed=7, seconds=5)
        self.assertEqual(a, b)

    def test_gps_walks_from_fixed_origin(self):
        t, frames = self._run(seed=3, seconds=30)
        self.assertEqual(t.fix_type, '3D')
        self.assertEqual(t.sats, 12)
        lats = [f[3] for f in frames]
        lons = [f[4] for f in frames]
        self.assertGreater(max(lats) - min(lats), 0.0)
        x, y = latlon_to_xy(max(lats), max(lons), lat0=min(lats), lon0=min(lons))
        self.assertLess(max(abs(x), abs(y)), 200.0, 'track wandered unrealistically far')

    def test_lidar_returns_fit_the_room(self):
        t, frames = self._run(seed=4, seconds=10)
        all_returns = [r for f in frames for r in f[5]]
        self.assertGreater(len(all_returns), 100)
        for bearing, dist in all_returns:
            self.assertTrue(0.0 <= bearing < 360.0)
            # 6 m x 4 m room: walls at 2-3 m, corners at ~3.6 m (+ noise margin)
            self.assertGreater(dist, 1.5)
            self.assertLess(dist, 4.0)

    def test_sections_touched_on_poll(self):
        src = SimSource(seed=5)
        src.start()
        t = Telemetry()
        src.poll(t, now=1000.0)
        for section in (ATTITUDE, GPS, LIDAR):
            self.assertFalse(t.is_stale(section, now=1000.1))


if __name__ == '__main__':
    unittest.main()
