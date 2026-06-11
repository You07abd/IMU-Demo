import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Patch mavutil before importing so the module-level connection doesn't fail
from unittest.mock import MagicMock, patch
import importlib

with patch('pymavlink.mavutil.mavlink_connection', return_value=MagicMock()):
    import imu_demo

def test_safe():
    assert not imu_demo.is_unsafe(0, 0)
    assert not imu_demo.is_unsafe(44.9, 34.9)
    assert not imu_demo.is_unsafe(-44.9, -29.9)

def test_roll_exceeded():
    assert imu_demo.is_unsafe(45.1, 0)
    assert imu_demo.is_unsafe(-45.1, 0)

def test_pitch_exceeded():
    assert imu_demo.is_unsafe(0, 35.1)
    assert imu_demo.is_unsafe(0, -30.1)
