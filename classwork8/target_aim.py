"""Bounded image-feedback Gimbal aiming while the chassis is stopped."""

from __future__ import annotations

from dataclasses import dataclass
import copy
import math
import time
from typing import Optional, Tuple


@dataclass(frozen=True)
class AimResult:
    success: bool
    reason: str
    detection: Optional[object]
    frame_size_px: Tuple[int, int]
    debug_frame: Optional[object]
    fresh_frames: int
    final_pitch_deg: Optional[float]
    final_yaw_deg: Optional[float]


def aim_error_ratio(
    centroid_px: Tuple[int, int],
    frame_size_px: Tuple[int, int],
    offset_x_ratio: float,
    offset_y_ratio: float,
) -> Tuple[float, float]:
    """Return target displacement from the calibrated impact point."""
    width, height = frame_size_px
    if width <= 0 or height <= 0:
        raise ValueError("frame size must be positive")
    desired_x = float(width) * (0.5 + float(offset_x_ratio))
    desired_y = float(height) * (0.5 + float(offset_y_ratio))
    return (
        (float(centroid_px[0]) - desired_x) / float(width),
        (float(centroid_px[1]) - desired_y) / float(height),
    )


def vertical_parallax_aim_offset_ratio(
    camera_above_blaster_m: float,
    target_distance_m: float,
    horizontal_fov_deg: float,
    frame_size_px: Tuple[int, int],
) -> float:
    """Return the vertical image offset for a camera above the muzzle."""
    width, height = frame_size_px
    if width <= 0 or height <= 0 or target_distance_m <= 0.0:
        raise ValueError("frame size and target distance must be positive")
    half_hfov = math.radians(float(horizontal_fov_deg)) / 2.0
    vertical_tangent = math.tan(half_hfov) * float(height) / float(width)
    if vertical_tangent <= 0.0:
        raise ValueError("horizontal FOV must be between 0 and 180 degrees")
    return (
        float(camera_above_blaster_m) / float(target_distance_m)
    ) / (2.0 * vertical_tangent)


def calibrated_aim_offsets(
    config,
    target_distance_m: float,
    frame_size_px: Tuple[int, int],
) -> Tuple[float, float]:
    """Return the camera centroid that aligns the offset blaster muzzle."""
    parallax_y = vertical_parallax_aim_offset_ratio(
        config.target_camera_above_blaster_m,
        max(0.10, float(target_distance_m)),
        config.target_camera_horizontal_fov_deg,
        frame_size_px,
    )
    return (
        float(config.target_aim_offset_x_ratio),
        max(-0.25, min(
            0.25,
            float(config.target_aim_offset_y_ratio) + parallax_y,
        )),
    )


class TargetAutoAim:
    """Visual servo with fresh-frame, feedback, travel and timeout guards."""

    def __init__(self, config) -> None:
        self.config = config

    def aim(
        self,
        *,
        gimbal,
        tracker,
        camera_service,
        detector,
        initial_detection,
        stop_event=None,
        aim_offset_x_ratio=None,
        aim_offset_y_ratio=None,
        timeout_sec=None,
        speed_scale=1.0,
    ) -> AimResult:
        started = time.monotonic()
        timeout_sec = (
            float(self.config.target_auto_aim_timeout_sec)
            if timeout_sec is None else float(timeout_sec)
        )
        # Saved profiles from the old unlimited-wait implementation may still
        # contain zero. Never let a frozen camera hold the whole mission open.
        timeout_sec = 6.0 if timeout_sec <= 0.0 else timeout_sec
        deadline = started + timeout_sec
        frame_stall_deadline = started + min(timeout_sec, 0.30)
        offset_x = (
            float(self.config.target_aim_offset_x_ratio)
            if aim_offset_x_ratio is None else float(aim_offset_x_ratio)
        )
        offset_y = (
            float(self.config.target_aim_offset_y_ratio)
            if aim_offset_y_ratio is None else float(aim_offset_y_ratio)
        )
        speed_scale = max(0.1, min(1.0, float(speed_scale)))
        initial_pitch, initial_yaw = tracker.get_angles()
        if initial_pitch is None or initial_yaw is None:
            return self._result(False, "AIM_GIMBAL_FEEDBACK_MISSING", None, (0, 0), None, 0, tracker)

        spec = (
            str(initial_detection.color).lower(),
            str(initial_detection.shape).lower(),
        )
        last_centroid = initial_detection.centroid
        last_frame_timestamp = started
        last_debug = None
        last_detection = None
        frame_size = (0, 0)
        fresh_frames = 0
        lost_frames = 0
        stable_frames = 0
        best_error = None
        worsening = 0
        last_progress_log = 0.0
        color_track_frames = 0

        print(
            "[AUTO_AIM] START target={}:{} timeout={:.1f}s "
            "impact_offset=({:+.3f},{:+.3f})".format(
                spec[0], spec[1], timeout_sec, offset_x, offset_y
            ),
            flush=True,
        )

        try:
            while time.monotonic() < deadline:
                if stop_event is not None and stop_event.is_set():
                    return self._result(False, "USER_STOP", last_detection, frame_size, last_debug, fresh_frames, tracker)
                feedback_age = tracker.angle_age_sec()
                if (
                    feedback_age is None
                    or feedback_age
                    > float(self.config.target_auto_aim_feedback_max_age_sec)
                ):
                    return self._result(False, "AIM_GIMBAL_FEEDBACK_STALE", last_detection, frame_size, last_debug, fresh_frames, tracker)

                sample = camera_service.latest_with_timestamp(
                    max_age_sec=float(self.config.target_max_frame_age_sec)
                )
                if sample is None:
                    if time.monotonic() >= frame_stall_deadline:
                        return self._result(False, "AIM_CAMERA_FRAME_STALE", last_detection, frame_size, last_debug, fresh_frames, tracker)
                    time.sleep(0.01)
                    continue
                frame, captured_at = sample
                if float(captured_at) <= float(last_frame_timestamp):
                    if time.monotonic() >= frame_stall_deadline:
                        return self._result(False, "AIM_CAMERA_FRAME_STALE", last_detection, frame_size, last_debug, fresh_frames, tracker)
                    time.sleep(0.01)
                    continue
                last_frame_timestamp = float(captured_at)
                frame_stall_deadline = time.monotonic() + min(timeout_sec, 0.30)
                fresh_frames += 1
                frame_size = int(frame.shape[1]), int(frame.shape[0])
                detections, last_debug = detector.detect(frame)

                candidates = [
                    item for item in detections
                    if (
                        str(item.color).lower(), str(item.shape).lower()
                    ) == spec
                ]
                continuity_limit = float(
                    self.config.target_auto_aim_max_jump_px
                )
                if not candidates:
                    continuity_limit = min(45.0, continuity_limit)
                    candidates = [
                        item for item in detections
                        if str(item.color).lower() == spec[0]
                    ]
                if candidates:
                    selected = min(
                        candidates,
                        key=lambda item: math.hypot(
                            float(item.centroid[0]) - float(last_centroid[0]),
                            float(item.centroid[1]) - float(last_centroid[1]),
                        ),
                    )
                    jump = math.hypot(
                        float(selected.centroid[0]) - float(last_centroid[0]),
                        float(selected.centroid[1]) - float(last_centroid[1]),
                    )
                    if jump > continuity_limit:
                        candidates = []

                if (
                    not candidates
                    and hasattr(detector, "track_color_centroid")
                ):
                    tracked_centroid = detector.track_color_centroid(
                        frame,
                        spec[0],
                        last_centroid,
                        min(
                            45.0,
                            float(self.config.target_auto_aim_max_jump_px),
                        ),
                    )
                    if tracked_centroid is not None:
                        selected = copy.copy(
                            last_detection
                            if last_detection is not None
                            else initial_detection
                        )
                        selected.centroid = tracked_centroid
                        candidates = [selected]
                        color_track_frames += 1
                        if color_track_frames == 1:
                            print(
                                "[AUTO_AIM] exact shape temporarily lost; "
                                "tracking verified {} target by color + "
                                "centroid continuity.".format(spec[0]),
                                flush=True,
                            )

                if not candidates:
                    lost_frames += 1
                    stable_frames = 0
                    best_error = None
                    if lost_frames > int(self.config.target_auto_aim_max_lost_frames):
                        return self._result(False, "AIM_TARGET_LOST", last_detection, frame_size, last_debug, fresh_frames, tracker)
                    continue

                lost_frames = 0
                last_detection = selected
                last_centroid = selected.centroid
                error_x, error_y = aim_error_ratio(
                    selected.centroid,
                    frame_size,
                    offset_x,
                    offset_y,
                )
                tolerance = float(self.config.target_aim_tolerance_ratio)
                error = max(abs(error_x), abs(error_y))
                now = time.monotonic()
                if now - last_progress_log >= 0.50:
                    pitch_now, yaw_now = tracker.get_angles()
                    print(
                        "[AUTO_AIM] frame={} error=({:+.3f},{:+.3f}) "
                        "stable={}/{} pitch={} yaw={}".format(
                            fresh_frames, error_x, error_y, stable_frames,
                            self.config.target_auto_aim_stable_frames,
                            "---" if pitch_now is None else "{:+.1f}".format(pitch_now),
                            "---" if yaw_now is None else "{:+.1f}".format(yaw_now),
                        ),
                        flush=True,
                    )
                    last_progress_log = now

                if abs(error_x) <= tolerance and abs(error_y) <= tolerance:
                    gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                    stable_frames += 1
                    best_error = error if best_error is None else min(best_error, error)
                    if stable_frames >= int(self.config.target_auto_aim_stable_frames):
                        return self._result(True, "AIM_SETTLED", last_detection, frame_size, last_debug, fresh_frames, tracker)
                    continue

                stable_frames = 0
                if best_error is not None and error > (
                    best_error + float(self.config.target_auto_aim_divergence_ratio)
                ):
                    worsening += 1
                    # Real 360p centroids move a few pixels between frames.
                    # Require a sustained trend before declaring a bad sign.
                    if worsening >= 4:
                        return self._result(False, "AIM_DIVERGING", last_detection, frame_size, last_debug, fresh_frames, tracker)
                else:
                    worsening = 0
                    best_error = error if best_error is None else min(best_error, error)

                pitch, yaw = tracker.get_angles()
                if pitch is None or yaw is None:
                    return self._result(False, "AIM_GIMBAL_FEEDBACK_MISSING", last_detection, frame_size, last_debug, fresh_frames, tracker)

                yaw_speed = 0.0
                pitch_speed = 0.0
                yaw_priority = abs(error_x) / tolerance >= abs(error_y) / tolerance
                if yaw_priority:
                    yaw_speed = self._speed(error_x, speed_scale) * float(
                        self.config.target_auto_aim_yaw_drive_sign
                    )
                    projected = (
                        abs(float(yaw) - float(initial_yaw))
                        + abs(yaw_speed)
                        * float(self.config.target_auto_aim_pulse_sec)
                    )
                    if projected > float(
                        self.config.target_auto_aim_max_yaw_delta_deg
                    ):
                        return self._result(False, "AIM_YAW_LIMIT", last_detection, frame_size, last_debug, fresh_frames, tracker)
                else:
                    # Image +Y is down; Gimbal pitch + is up.
                    pitch_speed = self._speed(-error_y, speed_scale) * float(
                        self.config.target_auto_aim_pitch_drive_sign
                    )
                    projected = (
                        abs(float(pitch) - float(initial_pitch))
                        + abs(pitch_speed)
                        * float(self.config.target_auto_aim_pulse_sec)
                    )
                    if projected > float(
                        self.config.target_auto_aim_max_pitch_delta_deg
                    ):
                        return self._result(False, "AIM_PITCH_LIMIT", last_detection, frame_size, last_debug, fresh_frames, tracker)

                command_ok = gimbal.drive_speed(
                    pitch_speed=pitch_speed,
                    yaw_speed=yaw_speed,
                )
                if command_ok is False:
                    return self._result(False, "AIM_COMMAND_FAILED", last_detection, frame_size, last_debug, fresh_frames, tracker)
                time.sleep(float(self.config.target_auto_aim_pulse_sec))
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                last_frame_timestamp = max(last_frame_timestamp, time.monotonic())
                time.sleep(float(self.config.target_auto_aim_settle_sec))

            return self._result(False, "AIM_TIMEOUT", last_detection, frame_size, last_debug, fresh_frames, tracker)
        finally:
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def _speed(self, error_ratio: float, speed_scale: float = 1.0) -> float:
        scale = max(0.1, min(1.0, float(speed_scale)))
        minimum = float(self.config.target_auto_aim_min_speed_dps)
        maximum = max(
            minimum,
            float(self.config.target_auto_aim_max_speed_dps) * scale,
        )
        magnitude = max(
            minimum,
            min(
                maximum,
                abs(float(error_ratio))
                * float(self.config.target_auto_aim_gain_dps_per_ratio)
                * scale,
            ),
        )
        return math.copysign(magnitude, float(error_ratio))

    @staticmethod
    def _result(
        success,
        reason,
        detection,
        frame_size,
        debug_frame,
        fresh_frames,
        tracker,
    ) -> AimResult:
        pitch, yaw = tracker.get_angles()
        return AimResult(
            success=bool(success),
            reason=str(reason),
            detection=detection,
            frame_size_px=frame_size,
            debug_frame=debug_frame,
            fresh_frames=int(fresh_frames),
            final_pitch_deg=None if pitch is None else float(pitch),
            final_yaw_deg=None if yaw is None else float(yaw),
        )
