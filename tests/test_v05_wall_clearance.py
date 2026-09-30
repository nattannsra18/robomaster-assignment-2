"""Offline V05 four-side clearance planning, GUI and real-command safety."""
import inspect
import threading
import time
import unittest
from unittest.mock import patch

from classwork8.config import Classwork8Config
from classwork8.wall_clearance_v05 import (
    body_clearance_cm,
    choose_clearance_plan,
    clearance_target,
)
from classwork8 import tof_camera_round1_v05 as v05
from pathlib import Path


def enabled_config():
    config = Classwork8Config()
    config.wall_clearance_enabled = True
    # Match the assignment entrypoint defaults used on the real robot.
    config.odom_scale_x = config.odom_scale_y = 1.0
    # Default body gaps plus 5 cm sensor recess give a 15 cm raw ToF target.
    config.wall_clearance_front_cm = 10.0
    config.wall_clearance_right_cm = 10.0
    config.wall_clearance_back_cm = 10.0
    config.wall_clearance_left_cm = 10.0
    return config


class WallClearancePlannerTests(unittest.TestCase):
    def test_all_four_wall_directions_move_opposite(self):
        cfg = enabled_config()
        for wall in range(4):
            with self.subTest(wall=wall):
                readings = {0: 100.0, 1: 100.0, 2: 100.0, 3: 100.0}
                readings[wall] = 10.0
                plan = choose_clearance_plan(readings, cfg)
                self.assertIsNotNone(plan)
                self.assertEqual(plan.wall_direction, wall)
                self.assertEqual(plan.away_direction, (wall + 2) % 4)
                self.assertAlmostEqual(plan.shift_cm, 4.0)

    def test_opposite_wall_budget_and_unsatisfiable_narrow_pair(self):
        cfg = enabled_config()
        # RIGHT too close, LEFT has only 3.5 cm room above its own target.
        readings = {0: 100.0, 1: 10.0, 2: 100.0, 3: 19.0}
        plan = choose_clearance_plan(readings, cfg)
        self.assertEqual(plan.wall_direction, 1)
        self.assertAlmostEqual(plan.shift_cm, 3.5)
        readings[3] = 14.0
        self.assertIsNone(choose_clearance_plan(readings, cfg))

    def test_missing_stale_or_nonwall_readings_do_not_drive(self):
        cfg = enabled_config()
        self.assertIsNone(choose_clearance_plan({0: 12.0, 2: None}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 12.0, 2: float("nan")}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 19.0, 2: 100.0}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 56.0, 2: 100.0}, cfg))

    def test_target_field_validation(self):
        cfg = enabled_config()
        cfg.validate()
        cfg.wall_clearance_front_cm = 100.0
        with self.assertRaisesRegex(ValueError, "wall_clearance_front_cm"):
            cfg.validate()
        cfg.wall_clearance_front_cm = 15.0
        cfg.wall_clearance_max_step_cm = 10.0
        with self.assertRaisesRegex(ValueError, "wall_clearance_max_step_cm"):
            cfg.validate()

    def test_first_gui_tab_lists_all_four_independent_settings(self):
        source = (Path(__file__).resolve().parents[1] / "classwork8" /
                  "config_gui_v05.py").read_text(encoding="utf-8")
        first = source.split('"Mission Settings": [', 1)[1].split('"Motion": [', 1)[0]
        self.assertLess(first.index("wall_clearance_enabled"),
                        first.index("travel_speed_mps"))
        for side in ("front", "right", "back", "left"):
            self.assertIn("wall_clearance_{}_cm".format(side), first)
        self.assertIn('"wall_clearance_enabled": False', source)
        self.assertIn('"wall_clearance_camera_dwell_sec": 0.70', source)

    def test_each_scan_direction_adjusts_before_next_and_only_remeasures_itself(self):
        source = inspect.getsource(v05._scan_four_directions)
        self.assertIn("safety_ranges[direction] = distance_cm", source)
        self.assertIn("_maintain_wall_clearance_checkpoint(", source)
        self.assertIn("if adjusted:", source)
        self.assertIn("safety_ranges.clear()", source)
        self.assertIn("distance_cm = _sample_tof(", source)
        self.assertNotIn("rescanned = _scan_four_directions(", source)
        self.assertLess(
            source.index("_maintain_wall_clearance_checkpoint("),
            source.index("\n        ranges[direction] = distance_cm")
        )
        run_source = inspect.getsource(v05.run)
        self.assertNotIn("_maintain_wall_clearance_checkpoint(", run_source)
        self.assertIn("cache_valid = _should_reuse_scan(", run_source)
        self.assertIn("[SCAN_BUDGET] cell=", source)
        self.assertIn("scanned={}", source)
        self.assertIn("reused={}", source)
        self.assertNotIn("Rescanning current cell", run_source)
        self.assertEqual(source.count("_aim_scan_direction_with_retry("), 1)
        self.assertNotIn("[CLEARANCE_PROBE]", inspect.getsource(
            v05._maintain_wall_clearance_checkpoint
        ))
        self.assertIn("[CLEARANCE_WARN]", source)
        self.assertIn('if failure == "USER_STOP":', source)
        self.assertNotIn(
            'print("[CLEARANCE_FAIL] {} during {} scan."', source
        )
        self.assertIn("not maintenance_only", source)
        self.assertIn("WALL_MAINTENANCE_PASS", source)
        self.assertIn("maintenance_only=maintenance_only", run_source)
        self.assertIn("revisit_topology_complete", run_source)
        self.assertIn(
            "revisit reserved for wall maintenance; camera not repeated",
            run_source,
        )
        self.assertIn('"CLEARANCE_CHECK"', source)
        self.assertIn('"WALL_APPROACH_RECOVERY_STARTED"', source)

    def test_pitch_restore_never_adds_a_yaw_scan(self):
        source = inspect.getsource(v05._scan_four_directions)
        self.assertEqual(source.count("_aim_scan_direction_with_retry("), 1)
        self.assertIn("restore_ok = _set_camera_observation_pitch(", source)
        self.assertIn("clamp_camera_limits=False", source)
        self.assertIn("_allow_endpoint_retry=False", inspect.getsource(
            v05._aim_scan_direction_with_retry
        ))
        self.assertNotIn("[CLEARANCE_PROBE]", source)
        controller = inspect.getsource(v05._maintain_wall_clearance_checkpoint)
        self.assertIn("_point_gimbal(", controller.split('"""', 2)[-1])
        self.assertNotIn("gimbal.drive_speed(", controller)
        cfg = Classwork8Config()
        self.assertEqual(cfg.gimbal_yaw_speed_dps, 255.0)
        self.assertEqual(cfg.gimbal_pitch_max_speed_dps, 57.0)
        self.assertLess(cfg.gimbal_settle_sec, 0.2)

    def test_current_side_camera_hold_occurs_after_retreat_before_next_yaw(self):
        source = inspect.getsource(v05._scan_four_directions)
        self.assertLess(
            source.index("_maintain_wall_clearance_checkpoint("),
            source.index("[CLEARANCE_CAMERA_HOLD]")
        )
        self.assertLess(
            source.index("[CLEARANCE_CAMERA_HOLD]"),
            source.index("verified_targets, target_debug = _verify_targets_or_empty(")
        )
        self.assertIn("survey_frame_epoch", source)
        self.assertIn("verified_retreat_direction=verified_retreat_direction", source)
        self.assertIn("[CLEARANCE_UNVERIFIED]", inspect.getsource(
            v05._maintain_wall_clearance_checkpoint
        ))
        self.assertIn("_point_gimbal(", inspect.getsource(
            v05._maintain_wall_clearance_checkpoint
        ).split('"""', 2)[-1])

    def test_defaults_are_body_clearance_with_directional_tof_recess(self):
        defaults = Classwork8Config()
        self.assertFalse(defaults.wall_clearance_enabled)
        self.assertEqual(defaults.wall_clearance_camera_dwell_sec, 0.70)
        for side in ("front", "right", "back", "left"):
            self.assertEqual(
                getattr(defaults, "wall_clearance_{}_cm".format(side)), 10.0
            )
        self.assertEqual(clearance_target(defaults, 0), 15.0)
        self.assertEqual(clearance_target(defaults, 1), 15.0)
        self.assertEqual(clearance_target(defaults, 2), 15.0)
        self.assertEqual(clearance_target(defaults, 3), 15.0)
        self.assertEqual(body_clearance_cm(defaults, 0, 15.0), 10.0)


class WallClearanceMotionTests(unittest.TestCase):
    @staticmethod
    def _checkpoint_rig(initial_cm, direction, *, response_sign=1.0):
        class Pose:
            x = 0.0
            y = 0.0

            def get_xy(self):
                return self.x, self.y

            def get_yaw(self):
                return 0.0

            def attitude_age_sec(self):
                return 0.01

        class Sensors:
            def __init__(self, pose):
                self.pose = pose
                self.tick = 0

            def reset_filters(self):
                pass

            @property
            def tof_last_update(self):
                self.tick += 1
                return time.monotonic() + self.tick * 1e-5

            def get_front_cm(self):
                if direction in (0, 2):
                    travel = abs(self.pose.x)
                else:
                    travel = abs(self.pose.y)
                return initial_cm + response_sign * travel * 100.0

        class Tracker:
            def get_angles(self):
                return 0.0, (0.0, 90.0, 180.0, -90.0)[direction]

        class Chassis:
            def __init__(self, pose):
                self.pose = pose
                self.moves = 0

            def stop(self):
                pass

            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                return True

            def drive_speed(self, x, y, z, timeout):
                self.moves += 1
                self.pose.x += x * 0.40
                self.pose.y += y * 0.40

        pose = Pose()
        sensors = Sensors(pose)
        return pose, sensors, Tracker(), Chassis(pose)

    def test_three_below_minimum_samples_trigger_emergency_retreat(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(2.1, 2)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {2: 2.1}, 2, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertTrue(telemetry["emergency_near"])
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)

    def test_one_below_minimum_sample_does_not_move(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(2.1, 2)
        values = iter((2.1, 3.5, 3.5))
        sensors.get_front_cm = lambda: next(values, 3.5)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {2: 2.1}, 2, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertFalse(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "LOW_RANGE_TRANSIENT")
        self.assertEqual(chassis.moves, 0)

    def test_guards_off_prioritizes_current_back_after_front_was_scanned(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(4.2, 2)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {0: 19.9, 2: 4.2}, 2, 0.0, 0.0, 0.0,
            threading.Event(), wall_confirmed=True,
            opposite_wall_confirmed=True,
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertEqual(telemetry["mode"], "AWAY")
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)
        self.assertIsNone(telemetry["limit_m"])
        self.assertGreater(chassis.moves, 0)

    def test_guards_off_does_not_wait_for_unmeasured_opposite_wall(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(4.2, 3)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {3: 4.2}, 3, 0.0, 0.0, 0.0,
            threading.Event(), wall_confirmed=True,
            opposite_wall_confirmed=True,
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertNotEqual(telemetry["result"], "WAITING_FOR_OPPOSITE_WALL")

    def test_stable_fresh_range_rebases_instead_of_skipping_wall(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(
            28.5, 3, response_sign=-1.0
        )
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {3: 38.1}, 3, 0.0, 0.0, 0.0,
            threading.Event(), wall_confirmed=True,
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertTrue(telemetry["range_rebased"])
        self.assertEqual(
            telemetry["result"], "WALL_APPROACH_RECOVERY_COMPLETE"
        )
        self.assertLessEqual(telemetry["after_cm"], 15.5)

    def test_heading_guard_stops_aligns_and_resumes_same_clearance(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(8.3, 1)
        original_get_yaw = pose.get_yaw
        yaw_reads = {"count": 0}

        def yaw_with_one_excursion():
            yaw_reads["count"] += 1
            if yaw_reads["count"] in (2, 3):
                return 3.0
            return original_get_yaw()

        pose.get_yaw = yaw_with_one_excursion
        with patch.object(
            v05, "_align_chassis_after_scan", return_value=(True, "ALIGNED")
        ) as align:
            moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
                chassis, object(), pose, sensors, tracker, cfg,
                {1: 8.3}, 1, 0.0, 0.0, 0.0, threading.Event(),
            )

        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)
        align.assert_called_once()

    def test_gimbal_reaim_keeps_retrying_then_resumes_same_clearance(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, _tracker, chassis = self._checkpoint_rig(8.3, 1)

        class Tracker:
            reads = 0
            aligned = True

            def get_angles(self):
                self.reads += 1
                if self.reads == 2:
                    self.aligned = False
                return (0.0, 90.0 if self.aligned else 40.0)

        tracker = Tracker()
        aim_calls = {"count": 0}

        def point_then_recover(*_args, **_kwargs):
            aim_calls["count"] += 1
            if aim_calls["count"] >= 2:
                tracker.aligned = True
                return True
            return False

        with patch.object(v05, "_point_gimbal", side_effect=point_then_recover):
            moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
                chassis, object(), pose, sensors, tracker, cfg,
                {1: 8.3}, 1, 0.0, 0.0, 0.0, threading.Event(),
            )

        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)
        self.assertEqual(aim_calls["count"], 2)

    def test_second_wall_scan_corrects_close_opposite_when_pair_is_feasible(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(
            30.0, 2, response_sign=-1.0
        )
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {0: 2.1, 2: 30.0}, 2, 0.0, 0.0, 0.0,
            threading.Event(), wall_confirmed=True,
            opposite_wall_confirmed=True,
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(
            telemetry["result"], "WALL_APPROACH_RECOVERY_COMPLETE"
        )
        self.assertEqual(telemetry["mode"], "APPROACH")
        self.assertLess(telemetry["after_cm"], 30.0)

    def test_first_of_two_confirmed_walls_waits_for_same_pose_opposite(self):
        cfg = enabled_config()
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            None, None, None, None, None, cfg,
            {0: 4.2}, 0, 0.0, 0.0, 0.0, threading.Event(),
            wall_confirmed=True, opposite_wall_confirmed=True,
        )
        self.assertFalse(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "WAITING_FOR_OPPOSITE_WALL")

    def test_confirmed_far_wall_is_approached_but_open_ray_is_not(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        # Regression for the field reading at cell (4, -2): 28.5 cm is a
        # confirmed wall and must no longer sit inside the maintenance band.
        pose, sensors, tracker, chassis = self._checkpoint_rig(
            28.5, 0, response_sign=-1.0
        )
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {0: 28.5}, 0, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(
            telemetry["result"], "WALL_APPROACH_RECOVERY_COMPLETE"
        )
        self.assertLessEqual(telemetry["after_cm"], 15.5)

        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            None, None, None, None, None, cfg,
            {0: 60.0}, 0, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertFalse(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "OPEN_DIRECTION")

    def test_topology_wall_approach_exhausts_at_forty_centimetres(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        pose, sensors, tracker, chassis = self._checkpoint_rig(
            80.0, 0, response_sign=-1.0
        )
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            chassis, object(), pose, sensors, tracker, cfg,
            {0: 80.0}, 0, 0.0, 0.0, 0.0, threading.Event(),
            wall_confirmed=True,
        )
        self.assertTrue(moved)
        self.assertEqual(reason, "WALL_APPROACH_RECOVERY_EXHAUSTED")
        self.assertEqual(
            telemetry["result"], "WALL_APPROACH_RECOVERY_EXHAUSTED"
        )
        self.assertGreaterEqual(telemetry["shifted_m"], 0.40)

    def test_clearance_continues_past_legacy_total_until_target_reached(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        cfg.odom_scale_x = cfg.odom_scale_y = 1.0

        class Pose:
            x = 0.0

            def get_xy(self):
                return self.x, 0.0

            def get_yaw(self):
                return 0.0

            def attitude_age_sec(self):
                return 0.01

        class Sensors:
            def __init__(self, pose):
                self.pose = pose

            def reset_filters(self):
                pass

            @property
            def tof_last_update(self):
                return time.monotonic()

            def get_front_cm(self):
                # Only 0.5 cm of range improvement per 1 cm translation.
                # Reaching 15 cm therefore needs 14 cm, beyond the legacy
                # 12 cm total cap and far beyond the old 7 cm deficit cap.
                return 8.0 + abs(self.pose.x) * 50.0

        class Tracker:
            def get_angles(self):
                return 0.0, 0.0

        class Chassis:
            def __init__(self, pose):
                self.pose = pose

            def stop(self):
                pass

            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                return True

            def drive_speed(self, x, y, z, timeout):
                self.pose.x += x * 0.40

        pose = Pose()
        sensors = Sensors(pose)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            Chassis(pose), object(), pose, sensors, Tracker(), cfg,
            {0: 8.0}, 0, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)
        self.assertGreater(telemetry["shifted_m"], 0.12)
        self.assertIsNone(telemetry["limit_m"])
        self.assertNotIn(
            "LIMIT_REACHED",
            inspect.getsource(v05._maintain_wall_clearance_checkpoint),
        )

    def test_one_bad_tof_frame_does_not_cancel_clearance_motion(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        cfg.odom_scale_x = cfg.odom_scale_y = 1.0

        class Pose:
            y = 0.0

            def get_xy(self):
                return 0.0, self.y

            def get_yaw(self):
                return 0.0

            def attitude_age_sec(self):
                return 0.01

        class Sensors:
            def __init__(self, pose):
                self.pose = pose
                self.sample = 0

            def reset_filters(self):
                pass

            @property
            def tof_last_update(self):
                self.sample += 1
                return time.monotonic()

            def get_front_cm(self):
                # First movement sample jumps down like the reported foam-wall
                # run; following samples agree with the odometry movement.
                if self.sample == 2:
                    return 11.0
                return 14.0 + abs(self.pose.y) * 100.0

        class Tracker:
            def get_angles(self):
                return 0.0, -90.0

        class Chassis:
            def __init__(self, pose):
                self.pose = pose

            def stop(self):
                pass

            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                return True

            def drive_speed(self, x, y, z, timeout):
                self.pose.y += y * 0.10

        pose = Pose()
        sensors = Sensors(pose)
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            Chassis(pose), object(), pose, sensors, Tracker(), cfg,
            {3: 14.0}, 3, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertTrue(moved, reason)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        self.assertEqual(telemetry["before_cm"], 14.0)
        self.assertGreaterEqual(telemetry["after_cm"], 14.5)
        self.assertGreater(telemetry["shifted_m"], 0.0)
        self.assertIsNone(telemetry["limit_m"])
        self.assertGreater(pose.y, 0.0)

    def test_right_wall_triggers_only_short_left_translation_with_z_zero(self):
        cfg = enabled_config()
        cfg.unsafe_disable_motion_guards = True
        cfg.odom_scale_x = 1.0
        cfg.odom_scale_y = 1.0
        cfg.wall_clearance_speed_mps = 0.035
        class Pose:
            y = 0.0
            def get_xy(self):
                return 0.0, self.y
            def get_yaw(self):
                return 0.0
            def attitude_age_sec(self):
                return 0.01
        class Chassis:
            def __init__(self, pose, sensors):
                self.pose = pose
                self.sensors = sensors
                self.commands = []
                self.stop_count = 0
            def stop(self):
                self.stop_count += 1
            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                self.commands.append(("stop", w1, w2, w3, w4))
                return True
            def drive_speed(self, x, y, z, timeout):
                self.commands.append(("move", x, y, z))
                self.pose.y += y * 0.10
                self.sensors.distance += abs(y * 0.10) * 100.0
                return None
        class Sensors:
            distance = 14.0
            def reset_filters(self):
                pass
            @property
            def tof_last_update(self):
                return time.monotonic()
            def get_front_cm(self):
                return self.distance
        class Tracker:
            def get_angles(self):
                return 0.0, 90.0
        pose, sensors = Pose(), Sensors()
        chassis = Chassis(pose, sensors)
        ranges = {1: 14.0}
        with patch.object(v05, "_point_gimbal", return_value=True):
            moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
                chassis, object(), pose, sensors, Tracker(), cfg, ranges,
                1, 0.0, 0.0, 0.0, threading.Event(),
            )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        motion = [c for c in chassis.commands if c[0] == "move"]
        self.assertGreater(len(motion), 1)
        self.assertTrue(all(x == z == 0.0 and y < 0.0
                            for _, x, y, z in motion))
        self.assertLessEqual(abs(pose.y), 0.052)
        self.assertEqual(chassis.commands[-1], ("stop", 0, 0, 0, 0))

    def test_missing_opposite_defers_without_any_yaw_or_chassis_command(self):
        cfg = enabled_config()
        with patch.object(v05, "_point_gimbal") as yaw:
            moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
                None, object(), None, None, None, cfg,
                {3: 12.0}, 3, 0.0, 0.0, 0.0, threading.Event(),
            )
        self.assertFalse(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "RETREAT_ROUTE_UNVERIFIED")
        yaw.assert_not_called()

    def test_only_current_side_is_corrected_not_an_earlier_side(self):
        cfg = enabled_config()
        # A previously scanned close LEFT must NOT trigger a late move while
        # the Gimbal is now pointing RIGHT. This is the user's key ordering.
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            None, None, None, None, None, cfg,
            {3: 10.0, 1: 22.0}, 1, 0.0, 0.0, 0.0, threading.Event(),
        )
        self.assertFalse(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "WITHIN_MAINTENANCE_BAND")

    def test_front_close_retreats_now_along_just_traversed_back_route(self):
        cfg = enabled_config()
        cfg.odom_scale_x = cfg.odom_scale_y = 1.0
        class Pose:
            x = 0.0
            def get_xy(self):
                return self.x, 0.0
            def get_yaw(self):
                return 0.0
            def attitude_age_sec(self):
                return 0.01
        class Sensors:
            pose = None
            def reset_filters(self):
                pass
            @property
            def tof_last_update(self):
                return time.monotonic()
            def get_front_cm(self):
                return 8.0 - self.pose.x * 100.0
        class Tracker:
            def get_angles(self):
                return 0.0, 0.0
        class Chassis:
            def __init__(self, pose):
                self.pose = pose
                self.commands = []
                self.stops = 0
            def stop(self):
                pass
            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                assert (w1, w2, w3, w4) == (0, 0, 0, 0)
                self.stops += 1
                self.commands.append(("stop", w1, w2, w3, w4))
                return True
            def drive_speed(self, x, y, z, timeout):
                self.commands.append(("move", x, y, z))
                self.pose.x += x * 0.10
                return None
        pose = Pose()
        sensors = Sensors()
        sensors.pose = pose
        chassis = Chassis(pose)
        with patch.object(v05, "_point_gimbal") as yaw:
            moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
                chassis, object(), pose, sensors, Tracker(), cfg,
                {0: 8.0}, 0, 0.0, 0.0, 0.0, threading.Event(),
                verified_retreat_direction=2,
            )
        yaw.assert_not_called()
        self.assertTrue(moved)
        self.assertIsNone(reason)
        self.assertEqual(telemetry["result"], "TARGET_REACHED")
        motion = [c for c in chassis.commands if c[0] == "move"]
        self.assertGreater(len(motion), 1)
        self.assertTrue(all(x < 0.0 and y == z == 0.0
                            for _, x, y, z in motion))
        self.assertGreaterEqual(sensors.get_front_cm(), 14.5)
        self.assertLessEqual(abs(pose.x), 0.092)
        self.assertGreaterEqual(chassis.stops, 2)
        self.assertEqual(chassis.commands[-1], ("stop", 0, 0, 0, 0))

    def test_feature_disabled_does_not_command_chassis(self):
        cfg = Classwork8Config()
        moved, reason, telemetry = v05._maintain_wall_clearance_checkpoint(
            None, None, None, None, None, cfg,
            {0: 12.0, 2: 100.0}, 0, 0.0, 0.0, 0.0, None,
        )
        self.assertEqual((moved, reason, telemetry), (False, None, None))

    def test_clearance_shift_is_persistent_not_retraced(self):
        source = inspect.getsource(v05._scan_four_directions)
        self.assertIn("CLEARANCE_PERSISTED", source)
        self.assertNotIn("_return_to_scan_origin(", source)


if __name__ == "__main__":
    unittest.main()
