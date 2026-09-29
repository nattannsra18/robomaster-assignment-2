"""Keyboard FPS controller for supervised RoboMaster play.

Controls
--------
W/A/S/D  chassis translation       Click view: capture FPS mouse
-/+      drive speed               Hold SPACE  full-auto water fire
TAB      release mouse              ESC  emergency stop and exit

The green reticle is the estimated water-impact point.  It uses the live ToF
range plus the measured 5 cm camera-above-muzzle offset used by the assignment.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import types
from typing import Optional, Set, Tuple


def _prepare_optional_media_codec() -> None:
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

    codec = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    codec.H264Decoder = H264Decoder
    codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = codec


_prepare_optional_media_codec()

import cv2
from PIL import Image, ImageTk
import tkinter as tk
from robomaster import blaster, robot

from classwork8.camera_service import CameraService
from classwork8.target_aim import vertical_parallax_aim_offset_ratio
from classwork8.tof_camera_round1_v05 import stop_chassis


SDK_FULL_AUTO_BATCH = 5
SDK_BURST_SETTLE_SEC = 2.0


def drive_command(
    keys: Set[str], drive_speed: float, turn_speed: float
) -> Tuple[float, float, float]:
    """Translate pressed keys into RoboMaster body-frame velocity."""
    x = float(drive_speed) * (int("w" in keys) - int("s" in keys))
    y = float(drive_speed) * (int("d" in keys) - int("a" in keys))
    z = float(turn_speed) * (int("q" in keys) - int("e" in keys))
    return x, y, z


def mouse_look_command(
    dx: float,
    dy: float,
    sensitivity: float,
    max_turn_speed: float,
    max_pitch_speed: float,
) -> Tuple[float, float]:
    """Return (chassis yaw speed, Gimbal pitch speed) for FPS mouse motion."""
    yaw = max(
        -float(max_turn_speed),
        min(float(max_turn_speed), -float(dx) * float(sensitivity)),
    )
    pitch = max(
        -float(max_pitch_speed),
        min(float(max_pitch_speed), -float(dy) * float(sensitivity)),
    )
    return yaw, pitch


def adjusted_drive_speed(current: float, delta: float) -> float:
    return round(max(0.05, min(0.50, float(current) + float(delta))), 2)


def impact_point_px(
    frame_size: Tuple[int, int],
    distance_cm: float,
    camera_above_m: float = 0.05,
    horizontal_fov_deg: float = 120.0,
    offset_x_ratio: float = 0.0,
    offset_y_ratio: float = 0.0,
) -> Tuple[int, int]:
    """Estimated image pixel hit by the water blaster at this range."""
    width, height = frame_size
    parallax_y = vertical_parallax_aim_offset_ratio(
        camera_above_m,
        max(0.10, float(distance_cm) / 100.0),
        horizontal_fov_deg,
        frame_size,
    )
    x_ratio = max(-0.25, min(0.25, float(offset_x_ratio)))
    y_ratio = max(-0.25, min(0.25, float(offset_y_ratio) + parallax_y))
    return (
        int(round(width * (0.5 + x_ratio))),
        int(round(height * (0.5 + y_ratio))),
    )


class KeyboardFpsController:
    def __init__(self, ep_robot, args) -> None:
        self.robot = ep_robot
        self.args = args
        self.chassis = ep_robot.chassis
        self.gimbal = ep_robot.gimbal
        self.sensor = ep_robot.sensor
        self.blaster = ep_robot.blaster
        self.camera = CameraService(ep_robot, resolution="360p")

        self.keys: Set[str] = set()
        self.last_drive = (0.0, 0.0, 0.0)
        self.last_gimbal = (0.0, 0.0)
        self.last_drive_sent = 0.0
        self.last_gimbal_sent = 0.0
        self.drive_speed = float(args.drive_speed)
        self.mouse_yaw = 0.0
        self.mouse_pitch = 0.0
        self.mouse_at = 0.0
        self.mouse_captured = False
        self.tof_cm: Optional[float] = None
        self.tof_at = 0.0
        self.closing = False
        self.fire_held = threading.Event()
        self.fire_thread: Optional[threading.Thread] = None
        self.fire_batches = 0
        self.hud_message = "READY"

        self.root = tk.Tk()
        self.root.title("RoboMaster Keyboard FPS - WATER FULL AUTO")
        self.root.configure(bg="#101318")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)

        self.video = tk.Label(self.root, bg="black", cursor="crosshair")
        self.video.pack(padx=8, pady=(8, 4))
        self.video.bind("<Motion>", self._mouse_move)
        self.video.bind("<Button-1>", self._capture_mouse)
        tk.Label(
            self.root,
            text=(
                "Click view: mouse look/turn | TAB: release mouse | -/+ speed | "
                "Hold SPACE: WATER FULL AUTO | ESC: STOP + EXIT"
            ),
            fg="#e8edf2",
            bg="#101318",
            font=("Consolas", 11, "bold"),
        ).pack(padx=8, pady=(2, 4))
        self.status = tk.Label(
            self.root,
            text="Connecting...",
            fg="#71ff9d",
            bg="#101318",
            font=("Consolas", 10),
        )
        self.status.pack(padx=8, pady=(0, 4))
        speed_row = tk.Frame(self.root, bg="#101318")
        speed_row.pack(pady=(0, 8))
        tk.Button(
            speed_row,
            text="- SPEED",
            width=10,
            command=lambda: self._change_speed(-0.05),
        ).pack(side="left", padx=4)
        self.speed_label = tk.Label(
            speed_row,
            text="0.25 m/s",
            width=12,
            fg="#71ff9d",
            bg="#101318",
            font=("Consolas", 11, "bold"),
        )
        self.speed_label.pack(side="left", padx=4)
        tk.Button(
            speed_row,
            text="+ SPEED",
            width=10,
            command=lambda: self._change_speed(+0.05),
        ).pack(side="left", padx=4)
        self._refresh_speed_label()

    def start(self) -> None:
        # Chassis-lead is the SDK's native "Gimbal follows chassis" mode.
        # Mouse X can therefore turn the robot without accumulating a Gimbal
        # yaw offset away from the front of the vehicle.
        if self.robot.set_robot_mode(mode=robot.CHASSIS_LEAD) is not True:
            raise RuntimeError("CHASSIS_LEAD_MODE_FAILED")
        if self.sensor.sub_distance(freq=20, callback=self._tof_callback) is not True:
            raise RuntimeError("TOF_SUBSCRIPTION_FAILED")
        if not self.camera.start():
            raise RuntimeError("CAMERA_START_FAILED")
        stop_chassis(self.chassis)
        self.gimbal.recenter(pitch_speed=100.0, yaw_speed=100.0)
        self.root.after(20, self._tick)
        self.root.after(33, self._draw)
        self.root.after(150, self.root.focus_force)
        self.root.mainloop()

    def _tof_callback(self, values) -> None:
        try:
            value = float(values[0]) / 10.0
        except (IndexError, TypeError, ValueError):
            return
        if 1.0 <= value <= 1000.0:
            self.tof_cm = value
            self.tof_at = time.monotonic()

    @staticmethod
    def _key_name(event) -> str:
        return str(event.keysym).lower()

    def _key_press(self, event) -> None:
        key = self._key_name(event)
        if key == "escape":
            self.close()
            return
        if key == "tab":
            if self.mouse_captured:
                self._release_mouse()
            else:
                self._capture_mouse()
            return
        if key in ("minus", "underscore"):
            if key not in self.keys:
                self.keys.add(key)
                self._change_speed(-0.05)
            return
        if key in ("plus", "equal"):
            if key not in self.keys:
                self.keys.add(key)
                self._change_speed(+0.05)
            return
        self.keys.add(key)
        if key == "space" and not self.fire_held.is_set():
            self.fire_held.set()
            self.fire_thread = threading.Thread(
                target=self._full_auto_loop,
                name="water-full-auto",
                daemon=True,
            )
            self.fire_thread.start()

    def _key_release(self, event) -> None:
        key = self._key_name(event)
        self.keys.discard(key)
        if key == "space":
            self.fire_held.clear()

    def _center_mouse(self) -> None:
        if (
            self.closing
            or not self.mouse_captured
            or not self.video.winfo_exists()
        ):
            return
        width = self.video.winfo_width()
        height = self.video.winfo_height()
        if width > 2 and height > 2:
            self.video.event_generate(
                "<Motion>",
                warp=True,
                x=width // 2,
                y=height // 2,
            )

    def _capture_mouse(self, _event=None) -> None:
        if self.closing:
            return
        self.mouse_captured = True
        self.video.configure(cursor="none")
        self.root.focus_force()
        self._center_mouse()
        self.hud_message = "MOUSE CAPTURED - TAB TO RELEASE"

    def _release_mouse(self) -> None:
        self.mouse_captured = False
        self.mouse_yaw = 0.0
        self.mouse_pitch = 0.0
        self.mouse_at = 0.0
        self.video.configure(cursor="crosshair")
        self.hud_message = "MOUSE RELEASED - CLICK VIEW TO CAPTURE"

    def _mouse_move(self, event) -> None:
        if self.closing or not self.mouse_captured:
            return
        center_x = self.video.winfo_width() // 2
        center_y = self.video.winfo_height() // 2
        dx = int(event.x) - center_x
        dy = int(event.y) - center_y
        # The warp-to-center event itself must never command the robot.
        if abs(dx) <= 1 and abs(dy) <= 1:
            return
        self.mouse_yaw, self.mouse_pitch = mouse_look_command(
            dx,
            dy,
            self.args.mouse_sensitivity,
            self.args.turn_speed,
            self.args.gimbal_speed,
        )
        self.mouse_at = time.monotonic()
        self._center_mouse()

    def _change_speed(self, delta: float) -> None:
        self.drive_speed = adjusted_drive_speed(self.drive_speed, delta)
        self.hud_message = "DRIVE SPEED {:.2f} m/s".format(self.drive_speed)
        self._refresh_speed_label()

    def _refresh_speed_label(self) -> None:
        self.speed_label.configure(text="{:.2f} m/s".format(self.drive_speed))

    def _full_auto_loop(self) -> None:
        while self.fire_held.is_set() and not self.closing:
            self.hud_message = "FIRING WATER x{}...".format(SDK_FULL_AUTO_BATCH)
            ack = self.blaster.fire(
                fire_type=blaster.WATER_FIRE,
                times=SDK_FULL_AUTO_BATCH,
            )
            if ack is not True:
                self.hud_message = "FIRE COMMAND FAILED"
                self.fire_held.clear()
                return
            self.fire_batches += 1
            self.hud_message = "FIRING - batch {} ACK".format(self.fire_batches)
            deadline = time.monotonic() + SDK_BURST_SETTLE_SEC
            while (
                self.fire_held.is_set()
                and not self.closing
                and time.monotonic() < deadline
            ):
                time.sleep(0.03)
        if not self.closing:
            self.hud_message = "READY - release completes current 5-shot burst"

    def _tick(self) -> None:
        if self.closing:
            return
        now = time.monotonic()
        keyboard_command = drive_command(
            self.keys,
            self.drive_speed,
            self.args.turn_speed,
        )
        mouse_active = now - self.mouse_at <= 0.10
        mouse_yaw = self.mouse_yaw if mouse_active else 0.0
        arrow_yaw = float(self.args.turn_speed) * (
            int("left" in self.keys) - int("right" in self.keys)
        )
        yaw = max(
            -float(self.args.turn_speed),
            min(float(self.args.turn_speed), keyboard_command[2] + arrow_yaw + mouse_yaw),
        )
        command = (keyboard_command[0], keyboard_command[1], yaw)
        moving = command != (0.0, 0.0, 0.0)
        if moving and (command != self.last_drive or now - self.last_drive_sent >= 0.10):
            self.chassis.drive_speed(
                x=command[0], y=command[1], z=command[2], timeout=0.25
            )
            self.last_drive_sent = now
        elif not moving and self.last_drive != (0.0, 0.0, 0.0):
            try:
                stop_chassis(self.chassis)
            except Exception as exc:
                self.hud_message = "STOP ERROR: {}".format(exc)
                print("[STOP_ERROR] {}".format(exc), flush=True)
                self.root.after(0, self.close)
                return
        self.last_drive = command

        keyboard_pitch = float(self.args.gimbal_speed) * (
            int("up" in self.keys) - int("down" in self.keys)
        )
        pitch = max(
            -float(self.args.gimbal_speed),
            min(
                float(self.args.gimbal_speed),
                keyboard_pitch + (self.mouse_pitch if mouse_active else 0.0),
            ),
        )
        # Never command Gimbal yaw here: CHASSIS_LEAD keeps it aligned with
        # the vehicle front while mouse X rotates the chassis itself.
        gimbal_command = (pitch, 0.0)
        if (
            gimbal_command != self.last_gimbal
            or (gimbal_command != (0.0, 0.0) and now - self.last_gimbal_sent >= 0.10)
        ):
            self.gimbal.drive_speed(pitch_speed=pitch, yaw_speed=0.0)
            self.last_gimbal_sent = now
            self.last_gimbal = gimbal_command

        self.root.after(20, self._tick)

    def _draw(self) -> None:
        if self.closing:
            return
        frame = self.camera.latest(max_age_sec=0.5)
        if frame is not None:
            height, width = frame.shape[:2]
            tof_fresh = (
                self.tof_cm is not None
                and time.monotonic() - self.tof_at <= 0.60
            )
            distance_cm = (
                float(self.tof_cm)
                if tof_fresh
                else float(self.args.fallback_distance_cm)
            )
            aim_x, aim_y = impact_point_px(
                (width, height),
                distance_cm,
                self.args.camera_above_cm / 100.0,
                self.args.horizontal_fov,
                self.args.aim_offset_x,
                self.args.aim_offset_y,
            )
            color = (80, 255, 80) if tof_fresh else (0, 210, 255)
            cv2.circle(frame, (aim_x, aim_y), 13, color, 2, cv2.LINE_AA)
            cv2.line(frame, (aim_x - 23, aim_y), (aim_x + 23, aim_y), color, 2)
            cv2.line(frame, (aim_x, aim_y - 23), (aim_x, aim_y + 23), color, 2)
            cv2.circle(frame, (aim_x, aim_y), 2, color, -1)
            cv2.putText(
                frame,
                "WATER IMPACT  {:.1f}cm {}".format(
                    distance_cm, "LIVE ToF" if tof_fresh else "FALLBACK"
                ),
                (12, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                self.hud_message,
                (12, height - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(Image.fromarray(rgb))
            self.video.configure(image=photo)
            self.video.image = photo
            self.status.configure(
                text="speed={:.2f}  drive=({:+.2f},{:+.2f},{:+.1f})  pitch={:+.0f}  {}".format(
                    self.drive_speed,
                    self.last_drive[0],
                    self.last_drive[1],
                    self.last_drive[2],
                    self.last_gimbal[0],
                    self.hud_message,
                )
            )
        self.root.after(33, self._draw)

    def close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self.fire_held.clear()
        try:
            stop_chassis(self.chassis)
        except Exception as exc:
            print("[STOP_ERROR] {}".format(exc), flush=True)
        try:
            self.gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
        except Exception:
            pass
        if self.fire_thread is not None:
            self.fire_thread.join(timeout=2.5)
        self.camera.stop()
        try:
            self.sensor.unsub_distance()
        except Exception:
            pass
        try:
            self.robot.close()
        finally:
            self.root.destroy()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Supervised keyboard FPS drive with WATER full-auto"
    )
    parser.add_argument("--connection", choices=("ap", "sta"), default="ap")
    parser.add_argument("--drive-speed", type=float, default=0.25, metavar="MPS")
    parser.add_argument("--turn-speed", type=float, default=30.0, metavar="DPS")
    parser.add_argument("--gimbal-speed", type=float, default=60.0, metavar="DPS")
    parser.add_argument("--mouse-sensitivity", type=float, default=0.45)
    parser.add_argument("--fallback-distance-cm", type=float, default=30.0)
    parser.add_argument("--camera-above-cm", type=float, default=5.0)
    parser.add_argument("--horizontal-fov", type=float, default=120.0)
    parser.add_argument("--aim-offset-x", type=float, default=0.0)
    parser.add_argument("--aim-offset-y", type=float, default=0.0)
    args = parser.parse_args(argv)
    if not 0.01 <= args.drive_speed <= 0.50:
        parser.error("--drive-speed must be between 0.01 and 0.50 m/s")
    if not 1.0 <= args.turn_speed <= 120.0:
        parser.error("--turn-speed must be between 1 and 120 deg/s")
    if not 1.0 <= args.gimbal_speed <= 180.0:
        parser.error("--gimbal-speed must be between 1 and 180 deg/s")
    if not 0.05 <= args.mouse_sensitivity <= 3.0:
        parser.error("--mouse-sensitivity must be between 0.05 and 3.0")
    if not 10.0 <= args.fallback_distance_cm <= 500.0:
        parser.error("--fallback-distance-cm must be between 10 and 500")
    if not 0.0 <= args.camera_above_cm <= 25.0:
        parser.error("--camera-above-cm must be between 0 and 25")
    if not 1.0 < args.horizontal_fov < 179.0:
        parser.error("--horizontal-fov must be between 1 and 179")
    for name in ("aim_offset_x", "aim_offset_y"):
        if not -0.25 <= getattr(args, name) <= 0.25:
            parser.error("--{} must be between -0.25 and 0.25".format(name.replace("_", "-")))
    return args


def main(argv=None) -> None:
    args = parse_args(argv)
    ep_robot = robot.Robot()
    connected = False
    controller = None
    try:
        print("Connecting to RoboMaster (mode={})...".format(args.connection), flush=True)
        connected = ep_robot.initialize(conn_type=args.connection) is True
        if not connected:
            raise RuntimeError("RoboMaster connection failed")
        controller = KeyboardFpsController(ep_robot, args)
        controller.start()
    finally:
        if controller is not None:
            controller.close()
        elif connected:
            try:
                stop_chassis(ep_robot.chassis)
            except Exception as exc:
                print("[STOP_ERROR] {}".format(exc), flush=True)
            try:
                ep_robot.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
