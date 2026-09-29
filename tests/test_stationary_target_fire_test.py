import unittest

from stationary_target_fire_test import stationary_args
from classwork8 import tof_camera_round1_v05 as mission


class StationaryTargetFireLauncherTests(unittest.TestCase):
    def test_forces_stationary_mode_without_removing_operator_options(self):
        self.assertEqual(
            stationary_args(["--fire-type", "ir"]),
            ["--fire-type", "ir", "--stationary-target-test"],
        )

    def test_does_not_duplicate_stationary_flag(self):
        self.assertEqual(
            stationary_args(["--stationary-target-test"]),
            ["--stationary-target-test"],
        )

    def test_stationary_mode_disables_clearance_translation(self):
        import inspect

        source = inspect.getsource(mission._scan_four_directions)
        self.assertIn(
            "config.wall_clearance_enabled and not config.stationary_target_test",
            source,
        )


if __name__ == "__main__":
    unittest.main()
