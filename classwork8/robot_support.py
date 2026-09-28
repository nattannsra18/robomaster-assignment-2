"""Small RoboMaster state helpers used by the Round 1 mission.

These helpers were extracted from the older all-in-one maze mission so this
repository does not need to carry unrelated pickup, drop, and exit code.
"""

from __future__ import annotations

import math
import statistics
import threading
import time
from collections import deque
from typing import Optional, Tuple


TOF_FILTER_SIZE = 3
TOF_STALE_SEC = 0.40
POSE_WAIT_SEC = 1.0


def normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


class PoseTracker:
    """Thread-safe position and attitude cache for SDK callbacks."""

    def __init__(self):
        self._lock = threading.Lock()
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.z: Optional[float] = None
        self.yaw: Optional[float] = None
        self.pitch: Optional[float] = None
        self.roll: Optional[float] = None

    def position_callback(self, data) -> None:
        try:
            if data is None or len(data) < 3:
                return
            x, y, z = data[:3]
            with self._lock:
                self.x = float(x)
                self.y = float(y)
                self.z = float(z)
        except Exception as exc:
            print("Position callback error:", exc)

    def attitude_callback(self, data) -> None:
        try:
            if data is None or len(data) < 3:
                return
            yaw, pitch, roll = data[:3]
            with self._lock:
                self.yaw = normalize_angle_deg(yaw)
                self.pitch = float(pitch)
                self.roll = float(roll)
        except Exception as exc:
            print("Attitude callback error:", exc)

    def get_xy(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.x, self.y

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw

    def has_position(self) -> bool:
        x, y = self.get_xy()
        return x is not None and y is not None


class SensorManager:
    """Filtered front ToF cache used by the ToF-only mission."""

    def __init__(self, _sensor_adapter=None):
        self.tof_buf = deque(maxlen=TOF_FILTER_SIZE)
        self.front_cm: Optional[float] = None
        self.tof_last_update: Optional[float] = None

    def tof_callback(self, data) -> None:
        try:
            if not data or data[0] is None:
                return
            millimetres = float(data[0])
            if not 20.0 <= millimetres <= 4000.0:
                return
            self.tof_buf.append(millimetres / 10.0)
            self.front_cm = statistics.median(self.tof_buf)
            self.tof_last_update = time.monotonic()
        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self) -> Optional[float]:
        if self.front_cm is None or self.tof_last_update is None:
            return None
        if time.monotonic() - self.tof_last_update > TOF_STALE_SEC:
            return None
        return self.front_cm

    def reset_filters(self) -> None:
        self.tof_buf.clear()
        self.front_cm = None
        self.tof_last_update = None


class HeadingManager:
    """Minimal heading state retained for the Round 1 runtime contract."""

    def __init__(self):
        self.base_yaw: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.heading_index = 0

    def initialize(self, yaw: Optional[float]) -> bool:
        if yaw is None or not math.isfinite(float(yaw)):
            return False
        self.base_yaw = normalize_angle_deg(yaw)
        self.target_yaw = self.base_yaw
        self.heading_index = 0
        return True


def wait_for_position(pose: PoseTracker) -> Tuple[float, float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        if pose.has_position():
            x, y = pose.get_xy()
            return float(x), float(y)
        time.sleep(0.05)
    print("WARNING: odometry not ready; using (0,0)")
    return 0.0, 0.0


def wait_for_yaw(pose: PoseTracker) -> Optional[float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        yaw = pose.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)
    print("WARNING: attitude yaw not ready")
    return None
