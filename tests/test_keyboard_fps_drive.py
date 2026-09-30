"""Hardware-free checks for the keyboard FPS controller."""

import inspect
import unittest

from keyboard_fps_drive import (
    KeyboardFpsController,
    adjusted_drive_speed,
    drive_command,
    impact_point_px,
    mouse_look_command,
    parse_args,
)


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

    def test_fps_mouse_turns_chassis_and_pitches_gimbal(self):
        yaw, pitch = mouse_look_command(20, -10, 0.5, 8.0, 20.0)
        self.assertEqual(yaw, -8.0)  # Mouse right turns chassis right.
        self.assertEqual(pitch, 5.0)  # Mouse up pitches camera up.

    def test_speed_buttons_are_bounded(self):
        self.assertEqual(adjusted_drive_speed(0.25, 0.05), 0.30)
        self.assertEqual(adjusted_drive_speed(0.50, 0.05), 0.50)
        self.assertEqual(adjusted_drive_speed(0.05, -0.05), 0.05)

    def test_camera_yaw_stays_native_chassis_lead(self):
        start = inspect.getsource(KeyboardFpsController.start)
        tick = inspect.getsource(KeyboardFpsController._tick)
        self.assertIn("robot.CHASSIS_LEAD", start)
        self.assertIn("yaw_speed=0.0", tick)

    def test_cli_defaults_are_bounded(self):
        args = parse_args([])
        self.assertEqual(args.connection, "ap")
        self.assertEqual(args.drive_speed, 0.25)
        self.assertEqual(args.camera_above_cm, 5.0)
        self.assertEqual(args.mouse_sensitivity, 0.45)


if __name__ == "__main__":
    unittest.main()
