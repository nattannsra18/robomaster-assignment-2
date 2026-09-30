"""Offline regression for bounded V05 heading control and one-cell test limits."""
import inspect
import sys
import types
import unittest
from types import SimpleNamespace

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
from classwork8 import tof_camera_round1_v05 as mission
from round1_assignment import _apply_cli_overrides, main


class MovingHeadingGuardTests(unittest.TestCase):
    def test_large_heading_errors_stop_before_next_drive(self):
        config = Classwork8Config()
        config.heading_hold_enabled = True
        config.yaw_isolation_mode = False
        limit = mission.V05_MOVING_YAW_ABORT_DEG
        self.assertEqual(limit, 4.0)
        for err in (-41.0, -(limit + 0.01), limit + 0.01, 41.0):
            with self.subTest(err=err):
                self.assertTrue(
                    mission._moving_heading_over_limit(config, -67.5, -67.5 - err)
                )
        for err in (-limit, -0.35, 0.0, 0.35, limit):
            with self.subTest(err=err):
                self.assertFalse(
                    mission._moving_heading_over_limit(config, -67.5, -67.5 - err)
                )

    def test_hard_yaw_limit_cannot_be_disabled_by_diagnostic_flags(self):
        config = Classwork8Config()
        self.assertFalse(mission._moving_heading_over_limit(config, 0.0, None))
        config.yaw_isolation_mode = True
        self.assertTrue(mission._moving_heading_over_limit(config, 0.0, 4.01))
        config.heading_hold_enabled = False
        self.assertTrue(mission._moving_heading_over_limit(config, 0.0, -4.01))

    def test_guards_off_source_still_stops_on_yaw_and_attitude_loss(self):
        source = inspect.getsource(mission._drive_one_cell)
        yaw_guard = source.split(
            "if _moving_heading_over_limit(config, start_yaw_deg, yaw):", 1
        )[1].split("# Validate observation geometry", 1)[0]
        self.assertNotIn("if not guards_disabled", yaw_guard)
        self.assertIn('return False, "MOVING_YAW_LIMIT", moved', yaw_guard)
        self.assertIn('if yaw is None:', source)
        self.assertNotIn(
            "if not guards_disabled and config.heading_hold_enabled and yaw is None",
            source,
        )

    def test_stop_guard_precedes_remaining_and_drive_command(self):
        source = inspect.getsource(mission._drive_one_cell)
        self.assertLess(
            source.index("if _moving_heading_over_limit("),
            source.index("at_odometry_endpoint = cell_pose_within_tolerance("),
        )
        self.assertLess(
            source.index("if _moving_heading_over_limit("),
            source.index("chassis.drive_speed("),
        )
        self.assertIn('return False, "MOVING_YAW_LIMIT", moved', source)
        self.assertIn("stop_chassis(chassis)", source)


class HardDiagnosticLimitsTests(unittest.TestCase):
    def test_cli_constraints_override_gui_values(self):
        config = Classwork8Config()
        config.max_moves = 500
        config.heading_max_z_dps = 18.0
        config.heading_align_max_z_dps = 10.0
        config.travel_speed_mps = 0.30
        config.target_detection_enabled = True
        args = SimpleNamespace(
            max_moves=1, max_yaw_correction=5.0,
            travel_speed=0.10, no_camera=True, yaw_isolation=False,
        )
        _apply_cli_overrides(config, args)
        self.assertEqual(config.max_moves, 1)
        self.assertEqual(config.heading_max_z_dps, 5.0)
        self.assertEqual(config.heading_align_max_z_dps, 5.0)
        self.assertEqual(config.travel_speed_mps, 0.10)
        self.assertFalse(config.target_detection_enabled)
        self.assertTrue(config.heading_hold_enabled)
        config.validate()

    def test_main_reapplies_cli_bounds_after_gui(self):
        source = inspect.getsource(main)
        self.assertEqual(source.count("_apply_cli_overrides(config, args)"), 2)
        self.assertIn('"--max-moves"', source)
        self.assertIn('"--max-yaw-correction"', source)
        self.assertIn("[DIAG_LIMITS]", source)


if __name__ == "__main__":
    unittest.main()
