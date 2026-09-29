"""Hardware-free checks for the keyboard FPS controller."""

import unittest

from keyboard_fps_drive import drive_command, impact_point_px, parse_args


class KeyboardFpsDriveTests(unittest.TestCase):
    def test_body_frame_keyboard_mapping(self):
        self.assertEqual(drive_command({"w", "d", "q"}, 0.25, 30.0), (0.25, 0.25, 30.0))
        self.assertEqual(drive_command({"s", "a", "e"}, 0.25, 30.0), (-0.25, -0.25, -30.0))
        self.assertEqual(drive_command({"w", "s", "q", "e"}, 0.25, 30.0), (0.0, 0.0, 0.0))

    def test_five_centimeter_parallax_moves_close_impact_point_down(self):
        close = impact_point_px((640, 360), 20.0)
        far = impact_point_px((640, 360), 100.0)
        self.assertEqual(close[0], 320)
        self.assertEqual(far[0], 320)
        self.assertGreater(close[1], far[1])
        self.assertGreater(far[1], 180)

    def test_cli_defaults_are_bounded(self):
        args = parse_args([])
        self.assertEqual(args.connection, "ap")
        self.assertEqual(args.drive_speed, 0.25)
        self.assertEqual(args.camera_above_cm, 5.0)


if __name__ == "__main__":
    unittest.main()
