"""Offline Stable V1 preflight, Gimbal hold and ToF braking regressions."""

import inspect
import sys
import time
import types
import unittest
from unittest.mock import patch

if "libmedia_codec" not in sys.modules:
    media = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media.H264Decoder = H264Decoder
    media.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media

from classwork8.config import Classwork8Config
from classwork8.movement_policy_v05 import (
    cell_pose_within_tolerance,
    preflight_has_clearance,
    preflight_required_cm,
    tof_braking_speed_mps,
)
from classwork8 import tof_camera_round1_v05 as mission
from final_round1_tof_camera_01 import _defaults


class StableMovementPolicyTests(unittest.TestCase):
    def test_assignment_default_uses_two_centimetre_tolerance(self):
        config = Classwork8Config()
        _defaults(config)
        self.assertEqual(config.step_tolerance_m, 0.02)
        self.assertTrue(config.moving_gimbal_check_enabled)
        config.validate()

    def test_preflight_reserves_travel_stop_and_margin(self):
        required = preflight_required_cm(0.60, 0.02, 18.0, 2.0)
        self.assertAlmostEqual(required, 78.0)
        self.assertTrue(preflight_has_clearance(78.0, required))
        self.assertFalse(preflight_has_clearance(77.9, required))
        self.assertFalse(preflight_has_clearance(None, required))

    def test_tof_braking_is_monotonic_and_hard_stop_is_zero(self):
        samples = [
            tof_braking_speed_mps(cm, 0.30, 35.0, 18.0, 0.06)
            for cm in (40.0, 35.0, 30.0, 26.5, 20.0, 18.0)
        ]
        self.assertEqual(samples[0], 0.30)
        self.assertEqual(samples[1], 0.30)
        self.assertAlmostEqual(samples[3], 0.18)
        self.assertEqual(samples[-1], 0.0)
        self.assertTrue(all(a >= b for a, b in zip(samples, samples[1:])))

    def test_cell_arrival_requires_longitudinal_and_lateral_odometry(self):
        self.assertTrue(cell_pose_within_tolerance(0.02, 0.03, 0.02, 0.035))
        self.assertFalse(cell_pose_within_tolerance(-0.03, 0.0, 0.02, 0.035))
        self.assertFalse(cell_pose_within_tolerance(0.0, 0.04, 0.02, 0.035))

    def test_arrival_is_checked_before_hard_stop_and_midcell_never_commits(self):
        source = inspect.getsource(mission._drive_one_cell)
        arrival = source.index("if cell_pose_within_tolerance(")
        live_guard = source.index("safety_reason, observed_cm = _moving_feedback_state(")
        hard_failure = source.index('return False, safety_reason, moved')
        self.assertLess(arrival, live_guard)
        self.assertLess(live_guard, hard_failure)
        self.assertIn("logical cell NOT committed", source)
        self.assertNotIn("auto-reverse", source.lower().split("while true:", 1)[1])

    def test_preflight_veto_is_before_any_drive_and_replans_from_same_cell(self):
        drive = inspect.getsource(mission._drive_one_cell)
        run = inspect.getsource(mission.run)
        self.assertLess(
            drive.index("aimed_age = gimbal_tracker.angle_age_sec()"),
            drive.index("preflight_samples = _collect_fresh_tof_samples("),
        )
        self.assertIn('preflight_reason = "PREFLIGHT_GIMBAL_STALE"', drive)
        self.assertIn("for preflight_attempt in range(2):", drive)
        self.assertIn("statistics.median(preflight_samples)", drive)
        self.assertIn("front_block_confirm_samples", drive)
        self.assertLess(
            drive.index('return False, "PREFLIGHT_BLOCKED", 0.0'),
            drive.index("chassis.drive_speed("),
        )
        self.assertIn('if reason == "PREFLIGHT_BLOCKED" and moved <= 1e-6:', run)
        preflight_branch = run.split(
            'if reason == "PREFLIGHT_BLOCKED"', 1
        )[1].split("finish_reason =", 1)[0]
        self.assertNotIn('_set_edge_state(', preflight_branch)
        self.assertIn('blocked_edges.add(', preflight_branch)
        self.assertIn('continue', preflight_branch)
        self.assertIn('finish_reason = "PREFLIGHT_NO_REACHABLE_ROUTE"', run)
        self.assertIn('excluded_frontiers = _frontier_options(', run)


class MovingFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.sensors = mission.ToFOnlySensorManager()
        self.tracker = mission.GimbalTracker()
        self.sensors.tof_callback([1000.0])
        self.tracker.callback([0.0, 0.0, 0.0, 0.0])

    def test_toggle_bypasses_only_angle_check_not_tof_or_hard_stop(self):
        self.tracker.callback([12.0, 40.0, 0.0, 0.0])
        reason, _ = mission._moving_feedback_state(
            self.config, self.sensors, self.tracker, 0
        )
        self.assertEqual(reason, "MOVING_GIMBAL_UNALIGNED")

        self.config.moving_gimbal_check_enabled = False
        reason, _ = mission._moving_feedback_state(
            self.config, self.sensors, self.tracker, 0
        )
        self.assertIsNone(reason)

        self.sensors.reset_filters()
        self.sensors.tof_callback([170.0])
        reason, distance = mission._moving_feedback_state(
            self.config, self.sensors, self.tracker, 0
        )
        self.assertEqual(reason, "MOVING_HARD_STOP")
        self.assertEqual(distance, 17.0)

        self.sensors.tof_last_update = time.monotonic() - 1.0
        reason, _ = mission._moving_feedback_state(
            self.config, self.sensors, self.tracker, 0
        )
        self.assertEqual(reason, "MOVING_TOF_STALE")

    def test_recovery_requires_three_distinct_tof_and_gimbal_updates(self):
        self.config.moving_feedback_recovery_samples = 3
        updates = {"count": 0}

        def feed_new_sample(_seconds, _stop_event):
            updates["count"] += 1
            self.sensors.tof_callback([1000.0 + updates["count"]])
            self.tracker.callback([0.0, 0.0, 0.0, 0.0])
            return True

        with patch.object(
            mission, "_sleep_interruptible", side_effect=feed_new_sample
        ):
            reason, distance = mission._recover_moving_feedback(
                self.config, self.sensors, self.tracker, 0, None
            )
        self.assertIsNone(reason)
        self.assertIsNotNone(distance)
        self.assertEqual(updates["count"], 3)

    def test_recovery_timeout_reports_error_without_motion(self):
        self.config.moving_feedback_recovery_timeout_sec = 0.02
        reason, _ = mission._recover_moving_feedback(
            self.config, self.sensors, self.tracker, 0, None
        )
        self.assertEqual(reason, "MOVING_FEEDBACK_TIMEOUT")


class TransientRecoveryTests(unittest.TestCase):
    def test_scan_aim_retries_once_then_succeeds(self):
        with patch.object(
            mission, "_point_gimbal", side_effect=(False, True)
        ) as point, patch.object(
            mission, "_sleep_interruptible", return_value=True
        ):
            ok, retries = mission._aim_scan_direction_with_retry(
                object(), object(), object(), 0, Classwork8Config(), None
            )
        self.assertTrue(ok)
        self.assertEqual(retries, 1)
        self.assertEqual(point.call_count, 2)

    def test_scan_aim_stops_after_bounded_retry(self):
        with patch.object(
            mission, "_point_gimbal", return_value=False
        ) as point, patch.object(
            mission, "_sleep_interruptible", return_value=True
        ):
            ok, retries = mission._aim_scan_direction_with_retry(
                object(), object(), object(), 0, Classwork8Config(), None
            )
        self.assertFalse(ok)
        self.assertEqual(retries, 1)
        self.assertEqual(point.call_count, 2)

    def test_failed_scan_records_unknown_and_continues_other_directions(self):
        source = inspect.getsource(mission._scan_four_directions)
        failure = source.split("if not gimbal_ok:", 1)[1].split(
            '_heading_snapshot(\n            "POST_GIMBAL_', 1
        )[0]
        self.assertIn('"GIMBAL_SCAN_UNKNOWN"', failure)
        self.assertIn('"UNKNOWN"', failure)
        self.assertIn("continue", failure)

    def test_preflight_counts_three_distinct_tof_callbacks(self):
        sensors = mission.ToFOnlySensorManager()
        sensors.tof_callback([700.0])
        pending = [800.0, 900.0]

        def feed(_seconds, _stop_event):
            sensors.tof_callback([pending.pop(0)])
            return True

        with patch.object(mission, "_sleep_interruptible", side_effect=feed):
            samples = mission._collect_fresh_tof_samples(
                sensors, 3, 1.0, None
            )
        self.assertEqual(len(samples), 3)
        self.assertEqual(pending, [])

    def test_camera_detector_exception_becomes_skipped_survey(self):
        class BrokenDetector:
            def verify_latest(self, *_args, **_kwargs):
                raise RuntimeError("camera frame decode failed")

        class Recorder:
            def __init__(self):
                self.events = []

            def event(self, *args, **kwargs):
                self.events.append((args, kwargs))

        recorder = Recorder()
        verified, debug = mission._verify_targets_or_empty(
            BrokenDetector(), object(), time.monotonic(), recorder, (1, 2), 3
        )
        self.assertEqual(verified, [])
        self.assertIsNone(debug)
        self.assertEqual(recorder.events[0][0][1], "TARGET_SURVEY_FAILED")

    def test_hard_faults_remain_fatal(self):
        drive = inspect.getsource(mission._drive_one_cell)
        feedback = inspect.getsource(mission._moving_feedback_state)
        stop = inspect.getsource(mission.stop_chassis)
        self.assertIn("MOVING_HARD_STOP", feedback)
        self.assertIn('return False, safety_reason, moved', drive)
        self.assertIn("ODOMETRY_LOST", drive)
        self.assertIn("MOVING_YAW_LIMIT", drive)
        self.assertIn("V05_WHEEL_STOP_NOT_ACKNOWLEDGED", stop)

    def test_moving_feedback_timeout_reaims_only_once(self):
        drive = inspect.getsource(mission._drive_one_cell)
        self.assertIn("moving_reaim_used = False", drive)
        self.assertIn('safety_reason == "MOVING_FEEDBACK_TIMEOUT"', drive)
        self.assertIn("and not moving_reaim_used", drive)
        self.assertIn("moving_reaim_used = True", drive)
        self.assertIn("MOVE_REPREFLIGHT_PASS", drive)


if __name__ == "__main__":
    unittest.main()
