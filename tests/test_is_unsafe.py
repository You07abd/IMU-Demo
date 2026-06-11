import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from imu_demo import is_unsafe


class TestIsUnsafe(unittest.TestCase):
    def test_safe(self):
        self.assertFalse(is_unsafe(0, 0))
        self.assertFalse(is_unsafe(44.9, 34.9))
        self.assertFalse(is_unsafe(-44.9, -29.9))

    def test_roll_exceeded(self):
        self.assertTrue(is_unsafe(45.1, 0))
        self.assertTrue(is_unsafe(-45.1, 0))

    def test_pitch_exceeded(self):
        self.assertTrue(is_unsafe(0, 35.1))
        self.assertTrue(is_unsafe(0, -30.1))


if __name__ == '__main__':
    unittest.main()
