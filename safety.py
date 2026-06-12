"""Crash-warning thresholds and alert beep."""
import subprocess
import sys

PITCH_MIN_DEG    = -30
PITCH_MAX_DEG    =  35
ROLL_MAX_ABS_DEG =  45


def is_unsafe(roll: float, pitch: float) -> bool:
    return abs(roll) > ROLL_MAX_ABS_DEG or pitch < PITCH_MIN_DEG or pitch > PITCH_MAX_DEG


def beep(fallback=None) -> None:
    """Play the OS alert sound; call `fallback` (e.g. root.bell) if that fails."""
    try:
        if sys.platform == 'win32':
            import winsound
            winsound.MessageBeep(winsound.MB_ICONHAND)
        elif sys.platform == 'darwin':
            subprocess.Popen(
                ['afplay', '/System/Library/Sounds/Sosumi.aiff'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        else:
            subprocess.Popen(
                ['paplay', '/usr/share/sounds/freedesktop/stereo/bell.oga'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
    except Exception:
        if fallback is not None:
            fallback()
