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
    hard_stop_near_target_is_arrival,
    odometry_endpoint_speed_mps,
    preflight_has_clearance,
    preflight_required_cm,
    tof_braking_speed_mps,
    wall_arrival_reached,
)
from classwork8 import tof_camera_round1_v05 as mission
from final_round1_tof_camera_01 import _defaults


class StableMovementPolicyTests(unittest.TestCase):
    def test_assignment_default_uses_two_centimetre_tolerance(self):
        config = Classwork8Config()
        _defaults(config)
        self.assertEqual(config.step_tolerance_m, 0.02)
        self.assertFalse(config.moving_gimbal_check_enabled)
        self.assertEqual(config.target_quick_gate_frames, 2)
        self.assertEqual(config.target_sample_frames, 4)
        self.assertEqual(config.target_verify_frames, 3)
        self.assertFalse(config.target_survey_open_directions)
        self.assertEqual(config.movement_preflight_margin_cm, 0.0)
        self.assertEqual(config.movement_wall_arrival_cm, 20.0)
        self.assertEqual(config.movement_wall_arrival_min_progress_ratio, 0.75)
        self.assertEqual(config.cell_center_tolerance_m, 0.060)
        self.assertEqual(config.target_fire_mode, "selected")
        self.assertEqual(config.moving_gimbal_bad_samples, 3)
        self.assertEqual(config.moving_feedback_recovery_timeout_sec, 2.50)
        config.validate()

    def test_wall_arrival_requires_range_and_minimum_odometry_progress(self):
        self.assertFalse(
            wall_arrival_reached(None, 20.0, 0.50, 0.60, 0.75, 0.03, 0.06)
        )
        self.assertFalse(
            wall_arrival_reached(20.1, 20.0, 0.50, 0.60, 0.75, 0.03, 0.06)
        )
        self.assertFalse(
            wall_arrival_reached(19.9, 20.0, 0.44, 0.60, 0.75, 0.03, 0.06)
        )
        self.assertTrue(
            wall_arrival_reached(20.0, 20.0, 0.45, 0.60, 0.75, 0.06, 0.06)
        )
        self.assertFalse(
            wall_arrival_reached(19.9, 20.0, 0.50, 0.60, 0.75, 0.081, 0.06)
        )

    def test_scan_budget_reserves_time_before_optional_camera_work(self):
        self.assertTrue(
            mission._scan_budget_allows_optional_work(100.0, 106.9, 8.0)
        )
        self.assertFalse(
            mission._scan_budget_allows_optional_work(100.0, 107.1, 8.0)
        )

    def test_mission_clock_warns_then_enters_non_stopping_urgency(self):
        self.assertEqual(
            mission._mission_clock_state(419.9, 420.0, 525.0), "RUNNING"
        )
        self.assertEqual(
            mission._mission_clock_state(420.0, 420.0, 525.0), "WARNING"
        )
        self.assertEqual(
            mission._mission_clock_state(525.0, 420.0, 525.0),
            "SOFT_DEADLINE",
        )
        self.assertEqual(
            mission._mission_clock_state(601.0, 420.0, 525.0),
            "SOFT_DEADLINE",
        )

    def test_preflight_reserves_travel_stop_and_margin(self):
        required = preflight_required_cm(0.60, 0.02, 18.0, 0.0)
        self.assertAlmostEqual(required, 76.0)
        self.assertTrue(preflight_has_clearance(76.0, required))
        self.assertFalse(preflight_has_clearance(75.9, required))
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

    def test_odometry_endpoint_braking_tapers_before_cell_target(self):
        samples = [
            odometry_endpoint_speed_mps(
                remaining, 0.02, 0.30, 0.18, 0.06
            )
            for remaining in (0.30, 0.18, 0.10, 0.04, 0.02)
        ]
        self.assertEqual(samples[0], 0.30)
        self.assertEqual(samples[1], 0.30)
        self.assertAlmostEqual(samples[2], 0.18)
        self.assertEqual(samples[-1], 0.0)
        self.assertTrue(all(a >= b for a, b in zip(samples, samples[1:])))

    def test_cell_arrival_requires_longitudinal_and_lateral_odometry(self):
        self.assertTrue(cell_pose_within_tolerance(0.02, 0.03, 0.02, 0.035))
        self.assertFalse(cell_pose_within_tolerance(-0.03, 0.0, 0.02, 0.035))
        self.assertFalse(cell_pose_within_tolerance(0.0, 0.04, 0.02, 0.035))

    def test_near_target_hard_stop_can_commit_only_within_lateral_tolerance(self):
        self.assertTrue(
            hard_stop_near_target_is_arrival(0.546, 0.60, 0.015, 0.82, 0.06)
        )
        self.assertFalse(
            hard_stop_near_target_is_arrival(0.48, 0.60, 0.015, 0.82, 0.06)
        )
        self.assertFalse(
            hard_stop_near_target_is_arrival(0.546, 0.60, 0.07, 0.82, 0.06)
        )

    def test_odometry_arrival_precedes_configured_wall_arrival(self):
        source = inspect.getsource(mission._drive_one_cell)
        arrival = source.index("if cell_pose_within_tolerance(")
        wall_arrival = source.index("if wall_arrival_reached(")
        live_guard = source.index("safety_reason, observed_cm = _moving_feedback_state(")
        hard_failure = source.index('return False, safety_reason, moved')
        self.assertLess(arrival, wall_arrival)
        self.assertLess(wall_arrival, live_guard)
        self.assertLess(arrival, live_guard)
        self.assertLess(live_guard, hard_failure)
        self.assertIn('return True, "CELL_COMPLETE_WALL_ARRIVAL", moved', source)
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
        self.assertIn('"PREFLIGHT_EXCLUSIONS_CLEARED"', run)
        self.assertIn('blocked_edges.clear()', run)
        self.assertIn('excluded_frontiers = _frontier_options(', run)
        self.assertIn('"MOVE_TRANSIENT_RETRY"', run)

    def test_heading_timeout_can_continue_only_inside_live_yaw_limit(self):
        run = inspect.getsource(mission.run)
        self.assertIn('align_reason == "HEADING_ALIGN_TIMEOUT"', run)
        self.assertIn('<= V05_MOVING_YAW_ABORT_DEG', run)
        self.assertIn('align_reason = "HEADING_ALIGN_RELAXED"', run)


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
        base_stamp = time.monotonic()

        def feed_new_sample(_seconds, _stop_event):
            updates["count"] += 1
            self.sensors.tof_callback([1000.0 + updates["count"]])
            self.tracker.callback([0.0, 0.0, 0.0, 0.0])
            # Keep this deterministic on Windows, where consecutive mocked
            # callbacks may receive timestamps less than 1 microsecond apart.
            stamp = base_stamp + updates["count"] * 0.01
            self.sensors.tof_last_update = stamp
            with self.tracker._lock:
                self.tracker._last_update = stamp
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
        self.assertIn('"GIMBAL_SCAN_UNKNOWN"', source)
        self.assertIn(
            '_set_edge_state(edge_states, current_cell, direction, "UNKNOWN")',
            source,
        )
        for reason in (
            "horizontal pitch restore failed",
            "fresh ToF missing after pitch restore",
            "pitch changed during ToF sampling/retry",
            "camera survey pitch restore failed",
        ):
            with self.subTest(reason=reason):
                self.assertIn("mark_scan_unknown", source)
                self.assertIn(reason, source)

    def test_round1_assignment_auto_aim_uses_tof_parallax_offsets(self):
        source = inspect.getsource(mission._scan_four_directions)
        calibration = source.index("aim_offset_x, aim_offset_y = calibrated_aim_offsets(")
        aim = source.index("aim_result = target_auto_aim.aim(", calibration)
        assignment_path = source[calibration:aim + 600]
        self.assertIn("float(distance_cm) / 100.0", assignment_path)
        self.assertIn("aim_offset_x_ratio=aim_offset_x", assignment_path)
        self.assertIn("aim_offset_y_ratio=aim_offset_y", assignment_path)

    def test_preflight_counts_three_distinct_tof_callbacks(self):
        sensors = mission.ToFOnlySensorManager()
        sensors.tof_callback([700.0])
        pending = [800.0, 900.0]
        base_stamp = time.monotonic()
        sensors.tof_last_update = base_stamp

        def feed(_seconds, _stop_event):
            if not pending:
                return False
            sensors.tof_callback([pending.pop(0)])
            sensors.tof_last_update = base_stamp + (3 - len(pending)) * 0.01
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

    def test_p2_keeps_fatal_guards_while_adding_endpoint_brake_and_clock(self):
        drive = inspect.getsource(mission._drive_one_cell)
        run = inspect.getsource(mission.run)
        self.assertIn("odometry_endpoint_speed_mps(", drive)
        self.assertIn("min(tof_brake_speed, endpoint_brake_speed)", drive)
        self.assertIn('"MISSION_SOFT_DEADLINE"', run)
        self.assertNotIn('finish_reason = "MISSION_HARD_DEADLINE"', run)
        self.assertIn('_mission_clock_state(', run)
        self.assertIn('"MISSION_TIME_WARNING"', run)
        for fatal in (
            "MOVING_HARD_STOP",
            "ODOMETRY_LOST",
            "MOVING_YAW_LIMIT",
        ):
            self.assertIn(fatal, drive + inspect.getsource(
                mission._moving_feedback_state
            ))


if __name__ == "__main__":
    unittest.main()
