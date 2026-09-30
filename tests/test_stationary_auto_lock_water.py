import inspect
import unittest

from stationary_auto_lock_water_test import auto_lock_args
from classwork8.target_aim import vertical_parallax_aim_offset_ratio
from classwork8 import tof_camera_round1_v05 as mission


class StationaryAutoLockWaterTests(unittest.TestCase):
    def test_launcher_forces_front_only_auto_lock_mode(self):
        self.assertEqual(
            auto_lock_args([]),
            ["--stationary-auto-lock-test"],
        )

    def test_five_cm_vertical_parallax_matches_expected_image_offsets(self):
        near = vertical_parallax_aim_offset_ratio(
            0.05, 0.50, 120.0, (640, 360)
        )
        far = vertical_parallax_aim_offset_ratio(
            0.05, 1.00, 120.0, (640, 360)
        )
        self.assertAlmostEqual(near, 0.0513, places=3)
        self.assertAlmostEqual(far, 0.0257, places=3)

    def test_auto_lock_branch_runs_before_navigation_scan_loop(self):
        source = inspect.getsource(mission.run)
        lock = source.index("if config.stationary_auto_lock_test:")
        navigation = source.index("while (", lock)
        self.assertLess(lock, navigation)
        self.assertIn("not config.stationary_auto_lock_test", source[navigation:])
        self.assertIn('config.target_fire_type = "water"', inspect.getsource(
            __import__("round1_assignment")._apply_cli_overrides
        ))


if __name__ == "__main__":
    unittest.main()
