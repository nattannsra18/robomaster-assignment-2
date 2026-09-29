"""Shared live target overlay + safe runtime camera-survey angle requests.

CameraService alone owns the RoboMaster stream. This worker only reads fresh
frames and annotates them, independently of the mapping GUI publish cadence.
The GUI NEVER sends gimbal commands: its angle is applied by the mission
worker only while the chassis is stationary at a scan checkpoint.
"""

from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2

from .target_detection import TargetDetector


class LiveSurveyBridge:
    def __init__(self, config) -> None:
        self._lock = threading.Lock()
        self._config = config
        self._camera = None
        self._preview = None
        self._raw_frame = None
        self._preview_timestamp = 0.0
        self._capture_timestamp = 0.0
        self._candidate_count = 0
        self._fps = 0.0
        self._pitch_deg = float(config.target_camera_pitch_deg)
        self._status = "Waiting for camera"
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rescan = threading.Event()
        self._manual_fire_callback = None
        self._manual_fire_thread: Optional[threading.Thread] = None
        self._manual_fire_enabled = False
        self._manual_fire_ready = False
        self._manual_fire_busy = False
        self._manual_fire_status = "Manual fire unavailable"

    def set_pitch(self, value: float) -> float:
        value = max(
            float(self._config.target_camera_pitch_min_deg),
            min(float(self._config.target_camera_pitch_max_deg), float(value)),
        )
        with self._lock:
            self._pitch_deg = float(value)
            self._status = "Camera angle queued for next stopped scan"
        return value

    def get_pitch(self) -> float:
        with self._lock:
            return float(self._pitch_deg)

    def set_roi_bottom(self, value: float) -> float:
        """Apply the camera image ROI immediately; never command the robot.

        Both the preview and scan-time detector read the same config, so an
        operator can see whether a low floor sign is entirely inside the ROI.
        """
        with self._lock:
            lower = float(self._config.target_roi_top_ratio) + 0.06
            bounded = min(0.99, max(lower, float(value)))
            self._config.target_roi_bottom_ratio = bounded
            self._status = "Target ROI updated to {:.0f}% of image height".format(
                bounded * 100.0
            )
            return bounded

    def get_roi_bottom(self) -> float:
        with self._lock:
            return float(self._config.target_roi_bottom_ratio)

    @staticmethod
    def aim_point(width: int, height: int, offset_x: float, offset_y: float):
        """Return the calibrated camera pixel used by Auto-Aim/blaster."""
        x = int(round(float(width) * (0.5 + float(offset_x))))
        y = int(round(float(height) * (0.5 + float(offset_y))))
        return (
            max(0, min(int(width) - 1, x)),
            max(0, min(int(height) - 1, y)),
        )

    @classmethod
    def annotate_live_candidates(
        cls,
        frame,
        candidates,
        roi_bottom: float,
        aim_offset_x: float = 0.0,
        aim_offset_y: float = 0.0,
    ):
        """Overlay the current independent detections on each live frame."""
        h, w = frame.shape[:2]
        count = min(9, len(candidates))
        height = 34 + count * 17
        overlay = frame.copy()
        cv2.rectangle(
            overlay, (3, 3), (min(w - 3, 326), min(h - 3, height)),
            (12, 18, 26), -1,
        )
        cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
        cv2.putText(
            frame,
            "LIVE CANDIDATES: {} (not yet verified)".format(len(candidates)),
            (9, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
            (255, 255, 255), 1, cv2.LINE_AA,
        )
        for index, item in enumerate(candidates[:9], 1):
            cv2.putText(
                frame,
                "#{:02d} {:<6} {:<9} {:.2f}".format(
                    index, item.color.upper(), item.shape.upper(), item.confidence
                ),
                (9, 19 + index * 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.41,
                (255, 255, 255), 1, cv2.LINE_AA,
            )
        cv2.putText(
            frame,
            "ROI bottom {:.0f}%".format(roi_bottom * 100.0),
            (max(0, w - 154), 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42,
            (255, 255, 255), 1, cv2.LINE_AA,
        )

        # Draw after detection so the FPS-style reticle never becomes part of
        # the color/shape input. Its centre is the same calibrated impact point
        # used by TargetAutoAim, not necessarily the optical image centre.
        cx, cy = cls.aim_point(w, h, aim_offset_x, aim_offset_y)
        colour = (60, 255, 80)
        outline = (0, 0, 0)
        for thickness, line_colour in ((4, outline), (2, colour)):
            cv2.circle(frame, (cx, cy), 13, line_colour, thickness, cv2.LINE_AA)
            cv2.line(frame, (cx - 30, cy), (cx - 18, cy), line_colour,
                     thickness, cv2.LINE_AA)
            cv2.line(frame, (cx + 18, cy), (cx + 30, cy), line_colour,
                     thickness, cv2.LINE_AA)
            cv2.line(frame, (cx, cy - 30), (cx, cy - 18), line_colour,
                     thickness, cv2.LINE_AA)
            cv2.line(frame, (cx, cy + 18), (cx, cy + 30), line_colour,
                     thickness, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, colour, -1, cv2.LINE_AA)
        return frame

    def configure_manual_fire(self, callback) -> None:
        """Attach the stationary-test-only hardware fire callback."""
        with self._lock:
            self._manual_fire_callback = callback
            self._manual_fire_enabled = callback is not None
            self._manual_fire_ready = False
            self._manual_fire_status = (
                "Manual fire waits for the stationary scan"
                if callback is not None else "Manual fire unavailable"
            )

    def set_manual_fire_ready(self, ready: bool, status: str = "") -> bool:
        with self._lock:
            self._manual_fire_ready = bool(
                ready and self._manual_fire_enabled
                and not self._manual_fire_busy
            )
            if status:
                self._manual_fire_status = str(status)
            return self._manual_fire_ready

    def request_manual_fire(self) -> bool:
        """Start one non-blocking manual shot; reject unavailable/double clicks."""
        with self._lock:
            if (
                not self._manual_fire_enabled
                or not self._manual_fire_ready
                or self._manual_fire_busy
                or self._manual_fire_callback is None
            ):
                self._manual_fire_status = (
                    "Manual fire rejected: stationary-ready status required"
                )
                return False
            callback = self._manual_fire_callback
            self._manual_fire_busy = True
            self._manual_fire_ready = False
            self._manual_fire_status = "MANUAL FIRE: command in progress..."

        def worker():
            try:
                acknowledged = callback() is True
                status = (
                    "MANUAL FIRE: acknowledged"
                    if acknowledged else "MANUAL FIRE: command failed"
                )
            except Exception as exc:
                status = "MANUAL FIRE failed: {}".format(exc)
            with self._lock:
                self._manual_fire_busy = False
                self._manual_fire_ready = bool(
                    self._manual_fire_enabled and not self._stop.is_set()
                )
                self._manual_fire_status = status

        thread = threading.Thread(
            target=worker, name="stationary-manual-fire", daemon=True
        )
        self._manual_fire_thread = thread
        thread.start()
        return True

    def request_rescan(self) -> None:
        self._rescan.set()
        self.set_status("Rescan requested at the current cell")

    def rescan_requested(self) -> bool:
        return self._rescan.is_set()

    def set_skip_visited_scans(self, enabled: bool) -> bool:
        # Only affects the NEXT cell checkpoint; never bypasses live ToF
        # safety checks during movement.
        with self._lock:
            self._config.skip_scanned_visited_cells = bool(enabled)
            self._status = (
                "Visited-cell full scan reuse {}".format(
                    "ON" if enabled else "OFF"
                )
            )
            return bool(enabled)

    def get_skip_visited_scans(self) -> bool:
        with self._lock:
            return bool(self._config.skip_scanned_visited_cells)

    def set_moving_gimbal_check(self, enabled: bool) -> bool:
        """Toggle only the diagnostic in-motion Gimbal angle/age guard."""
        with self._lock:
            self._config.moving_gimbal_check_enabled = bool(enabled)
            self._status = "Moving Gimbal Check {} (diagnostic)".format(
                "ON" if enabled else "OFF"
            )
            return bool(enabled)

    def get_moving_gimbal_check(self) -> bool:
        with self._lock:
            return bool(self._config.moving_gimbal_check_enabled)

    def set_yaw_speed(self, speed_dps: float) -> float:
        bounded = min(
            180.0,
            max(float(self._config.gimbal_min_yaw_speed_dps), float(speed_dps)),
        )
        with self._lock:
            self._config.gimbal_yaw_speed_dps = float(bounded)
            self._status = "Gimbal yaw speed set to {:.0f} deg/s".format(bounded)
        return bounded

    def get_yaw_speed(self) -> float:
        with self._lock:
            return float(self._config.gimbal_yaw_speed_dps)

    def consume_rescan(self) -> bool:
        if self._rescan.is_set():
            self._rescan.clear()
            return True
        return False

    def set_status(self, status: str) -> None:
        with self._lock:
            self._status = str(status)

    def attach_camera(self, camera) -> None:
        if camera is None or not camera.running:
            self.set_status("Camera unavailable")
            return
        self._camera = camera
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._preview_loop,
            name="final-live-target-preview",
            daemon=True,
        )
        self._thread.start()
        self.set_status("Live color/shape preview active")

    def _preview_loop(self) -> None:
        # Separate detector avoids sharing a mutable OpenCV CLAHE object with
        # the mapping thread's temporal verification.
        detector = TargetDetector(self._config)
        previous_capture = None
        previous_processed = None
        period = 1.0 / max(1.0, float(self._config.target_preview_fps))

        while not self._stop.is_set():
            started = time.monotonic()
            camera = self._camera
            if camera is None or not camera.running:
                self._stop.wait(0.10)
                continue

            sample = camera.latest_with_timestamp(
                max_age_sec=float(self._config.target_max_frame_age_sec)
            )
            if sample is not None:
                frame, capture_time = sample
                if previous_capture is None or capture_time > previous_capture:
                    try:
                        candidates, debug = detector.detect(frame)
                        debug = self.annotate_live_candidates(
                            debug,
                            candidates,
                            self.get_roi_bottom(),
                            self._config.target_aim_offset_x_ratio,
                            self._config.target_aim_offset_y_ratio,
                        )
                        now = time.monotonic()
                        fps = (
                            0.0 if previous_processed is None
                            else 1.0 / max(1e-3, now - previous_processed)
                        )
                        previous_processed = now
                        previous_capture = capture_time
                        with self._lock:
                            self._raw_frame = frame.copy()
                            self._preview = debug
                            self._preview_timestamp = now
                            self._capture_timestamp = float(capture_time)
                            self._candidate_count = len(candidates)
                            self._fps = fps
                    except Exception as exc:
                        self.set_status("Preview detector error: {}".format(exc))

            self._stop.wait(max(0.005, period - (time.monotonic() - started)))

    def latest_preview(self):
        """Return one immutable-by-convention snapshot to the Tk main thread."""
        with self._lock:
            age = (
                float("inf")
                if self._preview_timestamp <= 0.0
                else time.monotonic() - self._preview_timestamp
            )
            if age > 1.25 or self._preview is None:
                frame = None
            else:
                frame = self._preview.copy()
            return {
                "frame": frame,
                "timestamp": float(self._preview_timestamp),
                "age_sec": age,
                "candidate_count": int(self._candidate_count),
                "preview_fps": float(self._fps),
                "pitch_deg": float(self._pitch_deg),
                "roi_bottom": float(self._config.target_roi_bottom_ratio),
                "status": self._status,
                "manual_fire_ready": bool(self._manual_fire_ready),
                "manual_fire_busy": bool(self._manual_fire_busy),
                "manual_fire_status": self._manual_fire_status,
            }

    def save_camera_sample(self, output_dir) -> Optional[tuple]:
        """Save a matching raw/annotated frame for real-lighting calibration."""
        with self._lock:
            if self._raw_frame is None or self._preview is None:
                return None
            raw = self._raw_frame.copy()
            debug = self._preview.copy()

        folder = Path(output_dir) / "target_camera_samples"
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        raw_path = folder / "target_raw_{}.png".format(stamp)
        debug_path = folder / "target_debug_{}.png".format(stamp)
        if not cv2.imwrite(str(raw_path), raw):
            raise RuntimeError("Could not save {}".format(raw_path))
        if not cv2.imwrite(str(debug_path), debug):
            raise RuntimeError("Could not save {}".format(debug_path))
        return str(raw_path), str(debug_path)

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            self._manual_fire_enabled = False
            self._manual_fire_ready = False
        thread = self._thread
        if thread is not None and thread.is_alive() and (
            threading.current_thread() is not thread
        ):
            thread.join(timeout=2.0)
        self._thread = None
        manual_thread = self._manual_fire_thread
        if manual_thread is not None and manual_thread.is_alive() and (
            threading.current_thread() is not manual_thread
        ):
            manual_thread.join(timeout=2.0)
        self._manual_fire_thread = None
        with self._lock:
            self._preview = None
            self._raw_frame = None
