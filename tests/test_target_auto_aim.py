import inspect
import time
from types import SimpleNamespace
import unittest

import numpy as np

from classwork8.config import Classwork8Config
from classwork8.target_aim import TargetAutoAim, aim_error_ratio
from final_round1_tof_camera_01 import _prepare_optional_media_codec

_prepare_optional_media_codec()

from classwork8 import tof_camera_round1_v05 as v05
from classwork8.tof_camera_round1_v05 import GimbalTracker


class FakeCamera:
    def __init__(self):
        self.calls = 0
        self.frame = np.zeros((360, 640, 3), dtype=np.uint8)

    def latest_with_timestamp(self, max_age_sec=0.6):
        self.calls += 1
        return self.frame.copy(), time.monotonic() + self.calls * 0.001


class ServoDetector:
    def __init__(self, tracker, target_yaw=4.0, target_pitch=-3.0, visible=True):
        self.tracker = tracker
        self.target_yaw = target_yaw
        self.target_pitch = target_pitch
        self.visible = visible

    def detection(self):
        pitch, yaw = self.tracker.get_angles()
        return SimpleNamespace(
            color="blue",
            shape="circle",
            centroid=(
                int(round(320 + (self.target_yaw - yaw) * 10.0)),
                int(round(180 - (self.target_pitch - pitch) * 10.0)),
            ),
        )

    def detect(self, frame):
        return ([self.detection()] if self.visible else []), frame.copy()


class IntermittentServoDetector(ServoDetector):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        if self.calls in (2, 4, 7):
            return [], frame.copy()
        return super().detect(frame)


class FakeGimbal:
    def __init__(self, tracker):
        self.tracker = tracker
        self.commands = []

    def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
        pitch_speed = float(pitch_speed)
        yaw_speed = float(yaw_speed)
        self.commands.append((pitch_speed, yaw_speed))
        with self.tracker._lock:
            self.tracker.pitch += pitch_speed * 0.04
            self.tracker.yaw += yaw_speed * 0.04
            self.tracker._last_update = time.monotonic()
        return True


class TargetAutoAimTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.target_auto_aim_timeout_sec = 0.8
        self.config.target_auto_aim_pulse_sec = 0.001
        self.config.target_auto_aim_settle_sec = 0.0
        self.config.target_auto_aim_stable_frames = 3
        self.tracker = GimbalTracker()
        self.tracker.pitch = 0.0
        self.tracker.yaw = 0.0
        self.tracker._last_update = time.monotonic()
        self.camera = FakeCamera()
        self.gimbal = FakeGimbal(self.tracker)
        self.detector = ServoDetector(self.tracker)

    def run_aim(self):
        return TargetAutoAim(self.config).aim(
            gimbal=self.gimbal,
            tracker=self.tracker,
            camera_service=self.camera,
            detector=self.detector,
            initial_detection=self.detector.detection(),
        )

    def test_converges_on_fresh_frames_with_one_axis_commands(self):
        result = self.run_aim()
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.reason, "AIM_SETTLED")
        self.assertGreaterEqual(result.fresh_frames, 3)
        self.assertTrue(all(
            pitch == 0.0 or yaw == 0.0
            for pitch, yaw in self.gimbal.commands
        ))
        self.assertTrue(any(yaw > 0.0 for _pitch, yaw in self.gimbal.commands))
        self.assertTrue(any(pitch < 0.0 for pitch, _yaw in self.gimbal.commands))

    def test_wrong_yaw_sign_is_stopped_as_diverging(self):
        self.config.target_auto_aim_yaw_drive_sign = -1.0
        result = self.run_aim()
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "AIM_DIVERGING")

    def test_lost_target_fails_without_nonzero_motion(self):
        self.detector.visible = False
        self.config.target_auto_aim_max_lost_frames = 1
        result = self.run_aim()
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "AIM_TARGET_LOST")
        self.assertFalse(any(
            pitch != 0.0 or yaw != 0.0
            for pitch, yaw in self.gimbal.commands
        ))

    def test_intermittent_detection_still_converges(self):
        self.config.target_auto_aim_max_lost_frames = 5
        self.config.target_auto_aim_stable_frames = 2
        self.detector = IntermittentServoDetector(self.tracker)
        result = self.run_aim()
        self.assertTrue(result.success, result.reason)

    def test_visible_target_beyond_legacy_12_degree_limit_converges(self):
        self.detector = ServoDetector(self.tracker, target_yaw=20.0)
        result = self.run_aim()
        self.assertTrue(result.success, result.reason)

    def test_default_yaw_limit_covers_horizontal_camera_view(self):
        self.assertGreaterEqual(
            self.config.target_auto_aim_max_yaw_delta_deg,
            self.config.target_camera_horizontal_fov_deg / 2.0,
        )

    def test_stale_gimbal_feedback_fails_without_nonzero_motion(self):
        self.tracker._last_update = time.monotonic() - 2.0
        result = self.run_aim()
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "AIM_GIMBAL_FEEDBACK_STALE")
        self.assertFalse(any(
            pitch != 0.0 or yaw != 0.0
            for pitch, yaw in self.gimbal.commands
        ))

    def test_calibration_offset_moves_desired_centroid(self):
        error = aim_error_ratio((384, 162), (640, 360), 0.10, -0.05)
        self.assertAlmostEqual(error[0], 0.0)
        self.assertAlmostEqual(error[1], 0.0)

    def test_runtime_requires_successful_auto_aim_before_fire(self):
        source = inspect.getsource(v05._scan_four_directions)
        self.assertLess(
            source.index("target_auto_aim.aim("),
            source.index("target_mission.fire("),
        )
        self.assertIn("aim_confirmed=True", source)
        self.assertIn("if decision.should_fire:", source)

    def test_stationary_mode_exits_before_planner_and_translation(self):
        source = inspect.getsource(v05.run)
        stationary = source.index("if config.stationary_target_test:")
        planner = source.index("plan = _plan_frontier_move(")
        self.assertLess(stationary, planner)
        block = source[stationary:planner]
        self.assertIn("stop_chassis(chassis)", block)
        self.assertIn('finish_reason = "STATIONARY_TARGET_TEST_COMPLETE"', block)
        self.assertIn("break", block)


if __name__ == "__main__":
    unittest.main()
