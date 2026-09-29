from __future__ import annotations

import math
import statistics
import threading
from collections import deque
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

from robomaster import robot

from .robot_support import (
    HeadingManager,
    PoseTracker,
    SensorManager,
    normalize_angle_deg,
    wait_for_position,
    wait_for_yaw,
)

from .camera_service import CameraService
from .live_survey import LiveSurveyBridge
from .motion_safety_v05 import adjacent_wall_sides
from .movement_policy_v05 import (
    cell_pose_within_tolerance,
    hard_stop_near_target_is_arrival,
    odometry_endpoint_speed_mps,
    preflight_has_clearance,
    preflight_required_cm,
    tof_braking_speed_mps,
    unsafe_hard_stop_is_arrival,
    wall_arrival_reached,
)
from .wall_clearance_v05 import choose_clearance_plan, clearance_target
from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder
from .target_detection import (
    TargetDetector,
    TargetRegistry,
    save_topology,
)
from .target_aim import (
    TargetAutoAim,
    calibrated_aim_offsets,
    vertical_parallax_aim_offset_ratio,
)
from .target_mission import TargetMission, TargetMissionState
from .vision import CorridorVision


def stop_chassis(chassis) -> None:
    """V05 ONLY: stop in individual-wheel mode, never zero chassis speed mode.

    Real-robot isolation tests showed slow physical left rotation after
    chassis.drive_speed(0,0,0) switched SDK status 8 -> 1. In contrast,
    drive_wheels(0,0,0,0) returned True, switched status to 0, and held
    chassis yaw during and after the independent gimbal sweep. Preserve the
    legacy mission stop helper unchanged for other programs.
    """
    if chassis is None:
        return
    # SDK Chassis.drive_speed(..., timeout=...) arms a Timer that later
    # invokes drive_speed(0,0,0). It could otherwise switch us BACK into
    # the drifting chassis speed mode even after a successful wheel stop.
    # SDK Chassis.stop() cancels only that timer (Module.stop is a no-op).
    timer_error = None
    try:
        chassis.stop()
    except Exception as exc:
        timer_error = exc
    # Still issue a physical zero-wheel command if timer cancellation failed.
    ack = chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    if ack is not True:
        # Fail closed rather than silently fall back to the speed-mode
        # zero command that reproduced physical yaw creep on this robot.
        raise RuntimeError(
            "V05_WHEEL_STOP_NOT_ACKNOWLEDGED: drive_wheels(0,0,0,0) "
            "did not confirm the stop; halt the run and inspect the robot"
        )
    if timer_error is not None:
        raise RuntimeError(
            "V05_STOP_TIMER_CANCEL_FAILED: wheel zero was sent, but the "
            "previous drive_speed timer may remain active"
        ) from timer_error


# Logical map directions relative to the chassis heading at mission start.
# Map convention keeps +Y upward/left, while RoboMaster chassis +Y moves right.
DIR_VEC_MAP = {
    0: (1, 0),    # front
    1: (0, -1),   # right
    2: (-1, 0),   # back
    3: (0, 1),    # left
}

# RoboMaster body-frame drive_speed vectors.
DIR_VEC_DRIVE = {
    0: (1.0, 0.0),   # forward
    1: (0.0, 1.0),   # right strafe
    2: (-1.0, 0.0),  # reverse
    3: (0.0, -1.0),  # left strafe
}


def _mission_clock_state(
    elapsed_sec: float,
    warning_sec: float,
    soft_deadline_sec: float,
) -> str:
    if float(elapsed_sec) >= float(soft_deadline_sec):
        return "SOFT_DEADLINE"
    if float(elapsed_sec) >= float(warning_sec):
        return "WARNING"
    return "RUNNING"


def _scan_budget_allows_optional_work(
    started_at: float,
    now: float,
    budget_sec: float,
    reserve_sec: float = 1.0,
) -> bool:
    """Do not start another camera/aim operation near the cell deadline."""
    return float(now) + float(reserve_sec) <= (
        float(started_at) + float(budget_sec)
    )


def _camera_survey_required(
    wall_face: bool,
    scan_budget_available: bool,
    survey_open_directions: bool,
    preview_candidate: bool,
) -> bool:
    """Every wall gets a quick look; budget gates open-corridor work only."""
    return bool(
        wall_face
        or (
            scan_budget_available
            and survey_open_directions
            and preview_candidate
        )
    )

# Positive camera correction means "move right relative to the current travel
# direction". Convert that travel-frame vector into chassis x/y.
DIR_RIGHT_VEC_DRIVE = {
    0: (0.0, 1.0),    # facing front
    1: (-1.0, 0.0),   # facing right
    2: (0.0, -1.0),   # facing back
    3: (1.0, 0.0),    # facing left
}

DIR_NAME = {
    0: "FRONT",
    1: "RIGHT",
    2: "BACK",
    3: "LEFT",
}


class ToFOnlySensorManager(SensorManager):
    """Use the legacy ToF filtering without touching Sensor Adapter hardware."""

    def __init__(self):
        super().__init__(None)

    def read_front_corner_ir(self):
        return None, None


class V05PoseTracker(PoseTracker):
    """Include attitude receipt time; non-None yaw can still be stale."""

    def __init__(self):
        super().__init__()
        self._yaw_received_at = None

    def attitude_callback(self, data):
        try:
            if data is None or len(data) < 3 or not math.isfinite(float(data[0])):
                return
        except (TypeError, ValueError, IndexError):
            return
        super().attitude_callback(data)
        with self._lock:
            self._yaw_received_at = time.monotonic()

    def attitude_age_sec(self) -> Optional[float]:
        with self._lock:
            timestamp = self._yaw_received_at
        return None if timestamp is None else max(0.0, time.monotonic() - timestamp)


class GimbalTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.pitch = None
        self.yaw = None
        self.yaw_ground = None  # Diagnostic only; not a second chassis controller.
        self._last_update = None
        self._angle_history = deque(maxlen=4000)

    def callback(self, data):
        try:
            if data is None or len(data) < 2:
                return
            pitch = float(data[0])
            yaw = float(data[1])
            ground = float(data[3]) if len(data) >= 4 else None
            received_at = time.monotonic()
            with self._lock:
                self.pitch = pitch
                self.yaw = yaw
                self.yaw_ground = ground
                self._last_update = received_at
                self._angle_history.append((received_at, pitch, yaw))
        except Exception:
            return

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw

    def get_pitch(self) -> Optional[float]:
        with self._lock:
            return self.pitch

    def get_angles(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.pitch, self.yaw

    def get_yaws(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.yaw, self.yaw_ground

    def last_update_monotonic(self) -> Optional[float]:
        with self._lock:
            return self._last_update

    def angle_age_sec(self) -> Optional[float]:
        timestamp = self.last_update_monotonic()
        return None if timestamp is None else max(0.0, time.monotonic() - timestamp)

    def pitch_samples_since(self, start_monotonic: float) -> List[float]:
        """Measured pitch during a yaw sweep, including transient excursions."""
        with self._lock:
            return [
                float(pitch)
                for timestamp, pitch, _yaw in self._angle_history
                if timestamp >= float(start_monotonic)
            ]


def _sleep_interruptible(seconds: float, stop_event: Optional[threading.Event]) -> bool:
    deadline = time.monotonic() + max(0.0, float(seconds))
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False
        time.sleep(min(0.03, max(0.0, deadline - time.monotonic())))
    return True


def _map_xy_from_raw(
    raw_x: float,
    raw_y: float,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
) -> Tuple[float, float]:
    """Rotate DJI power-on odometry into the chassis frame at mission start.

    DJI sub_position(cs=1) keeps the coordinate axes from robot power-on.
    The mission may begin after the robot has already been moved/rotated, so
    subtracting only x/y is not enough.  We rotate by the initial chassis yaw
    so map +X is the robot FRONT at mission start and map +Y is robot LEFT.
    """
    dx = float(raw_x) - float(start_x)
    dy = float(raw_y) - float(start_y)

    theta = math.radians(float(start_yaw_deg))
    c = math.cos(theta)
    sn = math.sin(theta)

    # World(power-on frame) -> body frame at mission start.
    body_x = c * dx + sn * dy
    body_y_right = -sn * dx + c * dy

    # Conventional map frame: +X front, +Y left/up.
    # V02: wheel odometry on the real mecanum platform under-reported travel,
    # so convert raw odometry into calibrated physical maze metres here.
    return body_x * float(scale_x), -body_y_right * float(scale_y)


def _relative_xy(
    pose: PoseTracker,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    config: Optional[Classwork8Config] = None,
) -> Tuple[Optional[float], Optional[float]]:
    x, y = pose.get_xy()
    if x is None or y is None:
        return None, None
    scale_x = 1.0 if config is None else float(config.odom_scale_x)
    scale_y = 1.0 if config is None else float(config.odom_scale_y)
    return _map_xy_from_raw(
        float(x),
        float(y),
        start_x,
        start_y,
        start_yaw_deg,
        scale_x,
        scale_y,
    )


def _direction_angle_rad(direction: int) -> float:
    return {
        0: 0.0,
        1: -math.pi / 2.0,
        2: math.pi,
        3: math.pi / 2.0,
    }[int(direction) % 4]


def _neighbor(node: Tuple[int, int], direction: int) -> Tuple[int, int]:
    dx, dy = DIR_VEC_MAP[int(direction) % 4]
    return node[0] + dx, node[1] + dy


def _set_edge_state(
    edge_states: Dict[Tuple[int, int, int], str],
    cell: Tuple[int, int],
    direction: int,
    state: str,
) -> None:
    """Record a WALL/OPEN edge on both adjacent logical cells."""
    direction %= 4
    state = str(state).upper()
    edge_states[(int(cell[0]), int(cell[1]), direction)] = state
    other = _neighbor(cell, direction)
    edge_states[(int(other[0]), int(other[1]), (direction + 2) % 4)] = state


def _direction_to(
    current: Tuple[int, int],
    target: Tuple[int, int],
) -> Optional[int]:
    delta = (target[0] - current[0], target[1] - current[1])
    for direction, vec in DIR_VEC_MAP.items():
        if vec == delta:
            return direction
    return None


def _inside_working_canvas(
    node: Tuple[int, int],
    config: Classwork8Config,
) -> bool:
    x = node[0] * config.cell_size_m
    y = node[1] * config.cell_size_m
    margin = config.cell_size_m / 2.0
    return (
        abs(x) <= config.map_width_m / 2.0 - margin
        and abs(y) <= config.map_height_m / 2.0 - margin
    )


def _update_tof_ray(
    grid: OccupancyGrid,
    config: Classwork8Config,
    rel_x: float,
    rel_y: float,
    direction: int,
    distance_cm: Optional[float],
) -> None:
    if distance_cm is None or distance_cm < config.mapping_min_cm:
        return

    angle = _direction_angle_rad(direction)
    origin_x = rel_x + math.cos(angle) * config.tof_forward_offset_m
    origin_y = rel_y + math.sin(angle) * config.tof_forward_offset_m
    hit = distance_cm < config.tof_max_mapping_cm - 1.0

    grid.update_ray(
        origin_x,
        origin_y,
        angle,
        float(distance_cm) / 100.0,
        max_range_m=config.tof_max_mapping_cm / 100.0,
        hit=hit,
    )


def _set_gimbal_yaw_only(
    gimbal,
    tracker: GimbalTracker,
    target_yaw: float,
    reference_pitch: float,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Tuple[bool, float, float]:
    """Feedback-controlled yaw motion with pitch speed fixed at zero."""
    stable = 0
    max_pitch_error = 0.0
    started = time.monotonic()
    deadline = started + float(config.gimbal_turn_timeout_sec)
    try:
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False, max_pitch_error, started
            pitch, yaw = tracker.get_angles()
            if pitch is None or yaw is None:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                time.sleep(0.03)
                continue
            max_pitch_error = max(
                max_pitch_error, abs(float(pitch) - float(reference_pitch))
            )
            error = float(target_yaw) - float(yaw)
            if abs(error) <= float(config.gimbal_tolerance_deg):
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                stable += 1
                if stable >= int(config.gimbal_stable_samples):
                    return True, max_pitch_error, started
            else:
                stable = 0
                speed = max(
                    float(config.gimbal_min_yaw_speed_dps),
                    min(
                        float(config.gimbal_yaw_speed_dps),
                        abs(error) * float(config.gimbal_yaw_kp),
                    ),
                )
                gimbal.drive_speed(
                    pitch_speed=0.0,
                    yaw_speed=math.copysign(speed, error),
                )
            time.sleep(0.03)
        return False, max_pitch_error, started
    finally:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)


def _gimbal_yaw_waypoints(current_yaw: float, target_yaw: float):
    """Split a mechanical absolute-yaw sweep that crosses more than 180 deg."""
    if abs(float(target_yaw) - float(current_yaw)) > 180.0:
        return (0.0, float(target_yaw))
    return (float(target_yaw),)


def _point_gimbal(
    gimbal,
    sensors: ToFOnlySensorManager,
    tracker: GimbalTracker,
    direction: int,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
    _allow_endpoint_retry: bool = True,
) -> bool:
    """Staged, single-axis mapping scan: level pitch -> yaw only -> level pitch.

    The real stationary hardware test showed up to 24 degrees transient pitch
    error during the old simultaneous pitch/yaw controller even though every
    final endpoint was level.  Never send nonzero pitch and yaw simultaneously.
    Pitch can transiently move DURING the yaw sweep without a ToF reading.
    Report the excursion but only accept a ray after the gimbal has stopped
    and BOTH axes have been brought back to their horizontal scan targets.
    """
    target_yaw = float(config.gimbal_yaw_for_direction(direction))
    target_pitch = float(config.gimbal_scan_pitch_deg)
    def _stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def _stop_axes() -> None:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def _level_pitch(stage: str) -> bool:
        """Pitch moves only while yaw_speed is exactly zero."""
        stable = 0
        pitch_deadline = time.monotonic() + float(config.gimbal_turn_timeout_sec)
        while time.monotonic() < pitch_deadline:
            if _stopped():
                _stop_axes()
                return False

            pitch, yaw = tracker.get_angles()
            if pitch is None or yaw is None:
                _stop_axes()
                time.sleep(0.03)
                continue

            error = target_pitch - float(pitch)
            if abs(error) <= float(config.gimbal_pitch_tolerance_deg):
                _stop_axes()
                stable += 1
                if stable >= int(config.gimbal_stable_samples):
                    return True
            else:
                stable = 0
                speed = max(
                    float(config.gimbal_pitch_min_speed_dps),
                    min(
                        float(config.gimbal_pitch_max_speed_dps),
                        abs(error) * float(config.gimbal_pitch_kp),
                    ),
                )
                gimbal.drive_speed(
                    pitch_speed=(
                        math.copysign(speed, error)
                        * float(config.gimbal_pitch_drive_sign)
                    ),
                    yaw_speed=0.0,
                )
            time.sleep(0.03)

        _stop_axes()
        print(
            "[GIMBAL_FAIL] {} pitch timeout: target {:+.1f} current {}.".format(
                stage,
                target_pitch,
                "---" if tracker.get_pitch() is None
                else "{:+.1f}".format(float(tracker.get_pitch())),
            ),
            flush=True,
        )
        return False

    if _stopped():
        _stop_axes()
        return False

    # Correct any existing tilt before beginning the horizontal sweep.
    if not _level_pitch("PRE_YAW"):
        return False

    # These are mechanical absolute yaw coordinates, so +178 -> -90 is a
    # 268-degree sweep rather than a wrapped 92-degree sweep. Split that path
    # at FRONT so each phase gets its own timeout and pitch re-level step.
    current_yaw = tracker.get_yaw()
    if current_yaw is None:
        return False
    waypoints = _gimbal_yaw_waypoints(current_yaw, target_yaw)
    if len(waypoints) > 1:
        print(
            "[GIMBAL] Long absolute-yaw sweep {:+.1f}->{:+.1f}; "
            "staging through FRONT.".format(float(current_yaw), target_yaw),
            flush=True,
        )

    max_pitch_during_yaw = 0.0
    started_yaw = time.monotonic()
    for waypoint_index, waypoint in enumerate(waypoints):
        yaw_ok, phase_pitch_error, phase_started = _set_gimbal_yaw_only(
            gimbal,
            tracker,
            waypoint,
            target_pitch,
            config,
            stop_event,
        )
        if waypoint_index == 0:
            started_yaw = phase_started
        max_pitch_during_yaw = max(
            max_pitch_during_yaw, phase_pitch_error
        )
        if not yaw_ok:
            print(
                "[GIMBAL_FAIL] Yaw timeout at {} waypoint {:+.1f}. "
                "Measured yaw={}.".format(
                    DIR_NAME[int(direction) % 4], waypoint,
                    tracker.get_yaw(),
                ),
                flush=True,
            )
            return False

        _stop_axes()
        if not _sleep_interruptible(0.06, stop_event):
            return False

        # Physical yaw motion can pull pitch down substantially. Re-level at
        # every waypoint instead of waiting until a long sweep has finished.
        stage = "POST_YAW" if waypoint_index + 1 == len(waypoints) else "MID_YAW"
        if not _level_pitch(stage):
            return False

    _stop_axes()
    if not _sleep_interruptible(config.gimbal_settle_sec, stop_event):
        return False

    final_pitch, final_yaw = tracker.get_angles()
    if (
        final_pitch is None
        or final_yaw is None
        or abs(target_pitch - float(final_pitch))
        > float(config.gimbal_pitch_tolerance_deg)
        or abs(target_yaw - float(final_yaw))
        > float(config.gimbal_tolerance_deg)
    ):
        if _allow_endpoint_retry and not _stopped():
            print("[GIMBAL] Final endpoint out of tolerance; one bounded retry.", flush=True)
            return _point_gimbal(
                gimbal, sensors, tracker, direction, config, stop_event,
                _allow_endpoint_retry=False,
            )
        print(
            "[GIMBAL] Final orientation unstable: target pitch={:+.1f}, "
            "yaw={:+.1f}; actual pitch={}, yaw={}.".format(
                target_pitch,
                target_yaw,
                "---" if final_pitch is None else "{:+.1f}".format(float(final_pitch)),
                "---" if final_yaw is None else "{:+.1f}".format(float(final_yaw)),
            ),
            flush=True,
        )
        return False

    samples = tracker.pitch_samples_since(started_yaw)
    if max_pitch_during_yaw > float(config.gimbal_yaw_pitch_guard_deg):
        print(
            "[GIMBAL] TRANSIENT_PITCH_WARNING {}: peak={:.1f} deg while yaw "
            "was turning. Final pitch/yaw are verified level; no ToF was "
            "sampled during the sweep.".format(
                DIR_NAME[int(direction) % 4], max_pitch_during_yaw
            ),
            flush=True,
        )
    print(
        "[GIMBAL] {} yaw-only sweep finished; peak pitch error={:.1f} deg "
        "({} feedback samples).".format(
            DIR_NAME[int(direction) % 4],
            max_pitch_during_yaw,
            len(samples),
        ),
        flush=True,
    )
    sensors.reset_filters()
    return True


def _aim_scan_direction_with_retry(
    gimbal,
    sensors: ToFOnlySensorManager,
    tracker: GimbalTracker,
    direction: int,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Tuple[bool, int]:
    """Try one scan aim plus one bounded retry while the chassis is stopped."""
    retries_used = 0
    for attempt in range(2):
        if _point_gimbal(
            gimbal,
            sensors,
            tracker,
            direction,
            config,
            stop_event,
            _allow_endpoint_retry=False,
        ):
            return True, attempt
        retries_used = attempt
        if stop_event is not None and stop_event.is_set():
            break
        if attempt == 0 and not _sleep_interruptible(0.10, stop_event):
            break
    return False, retries_used


def _verify_targets_or_empty(
    target_detector: TargetDetector,
    camera_service: CameraService,
    not_before: float,
    recorder: RunRecorder,
    current_cell: Tuple[int, int],
    direction: int,
):
    """A camera/detector failure skips this survey, never navigation."""
    try:
        return target_detector.verify_latest(
            camera_service,
            not_before=not_before,
        )
    except Exception as exc:
        recorder.event(
            time.monotonic(),
            "TARGET_SURVEY_FAILED",
            str(exc),
            logical_node=current_cell,
            direction=DIR_NAME[int(direction) % 4],
        )
        print(
            "[TARGET_SURVEY_FAILED] {}: {}; navigation continues.".format(
                DIR_NAME[int(direction) % 4], exc
            ),
            flush=True,
        )
        return [], None


def _quick_target_candidate_or_false(
    target_detector: TargetDetector,
    camera_service: CameraService,
    not_before: float,
    recorder: RunRecorder,
    current_cell: Tuple[int, int],
    direction: int,
):
    """A quick-gate failure skips this survey, never navigation."""
    try:
        return target_detector.quick_candidate_latest(
            camera_service,
            not_before=not_before,
        )
    except Exception as exc:
        recorder.event(
            time.monotonic(),
            "TARGET_QUICK_GATE_FAILED",
            str(exc),
            logical_node=current_cell,
            direction=DIR_NAME[int(direction) % 4],
        )
        print(
            "[TARGET_QUICK_GATE_FAILED] {}: {}; navigation continues.".format(
                DIR_NAME[int(direction) % 4], exc
            ),
            flush=True,
        )
        return False, None


def _set_camera_observation_pitch(
    gimbal,
    tracker: GimbalTracker,
    config: Classwork8Config,
    target_pitch: float,
    stop_event: Optional[threading.Event],
    *,
    tolerance_deg: Optional[float] = None,
    clamp_camera_limits: bool = True,
) -> bool:
    """Adjust pitch only: never spend another yaw sweep to restore scan pitch."""
    desired = (
        max(
            float(config.target_camera_pitch_min_deg),
            min(float(config.target_camera_pitch_max_deg), float(target_pitch)),
        )
        if clamp_camera_limits else float(target_pitch)
    )
    tolerance = (
        float(config.target_camera_pitch_tolerance_deg)
        if tolerance_deg is None else float(tolerance_deg)
    )
    deadline = time.monotonic() + float(config.target_camera_pitch_timeout_sec)
    stable = 0

    try:
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False

            pitch, yaw = tracker.get_angles()
            if pitch is None or yaw is None:
                time.sleep(0.025)
                continue

            error = desired - float(pitch)
            if abs(error) <= tolerance:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                stable += 1
                if stable >= int(config.gimbal_stable_samples):
                    if not _sleep_interruptible(
                        config.target_camera_settle_sec, stop_event
                    ):
                        return False
                    final = tracker.get_pitch()
                    return (
                        final is not None
                        and abs(desired - float(final))
                        <= tolerance
                    )
            else:
                stable = 0
                speed = max(
                    float(config.gimbal_pitch_min_speed_dps),
                    min(
                        float(config.gimbal_pitch_max_speed_dps),
                        abs(error) * float(config.gimbal_pitch_kp),
                    ),
                )
                gimbal.drive_speed(
                    pitch_speed=(
                        math.copysign(speed, error)
                        * float(config.gimbal_pitch_drive_sign)
                    ),
                    yaw_speed=0.0,
                )
            time.sleep(0.03)

        print(
            "[CAMERA] Observation pitch timeout: target={:+.1f}, measured={}".format(
                desired, tracker.get_pitch()
            ),
            flush=True,
        )
        return False
    finally:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)


def _restore_auto_aim_start_pose(
    gimbal,
    tracker: GimbalTracker,
    config: Classwork8Config,
    start_pitch: Optional[float],
    start_yaw: Optional[float],
    stop_event: Optional[threading.Event],
) -> bool:
    """Return to the last visible target pose before one slower retry."""
    if start_pitch is None or start_yaw is None:
        return False
    yaw_ok, _pitch_error, _started = _set_gimbal_yaw_only(
        gimbal,
        tracker,
        float(start_yaw),
        float(start_pitch),
        config,
        stop_event,
    )
    if not yaw_ok:
        return False
    return _set_camera_observation_pitch(
        gimbal,
        tracker,
        config,
        float(start_pitch),
        stop_event,
        clamp_camera_limits=False,
    )


def _sample_tof(
    sensors: ToFOnlySensorManager,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Optional[float]:
    values: List[float] = []
    deadline = time.monotonic() + 1.5

    while len(values) < config.scan_samples and time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return None
        value = sensors.get_front_cm()
        if value is not None:
            values.append(float(value))
        time.sleep(config.scan_sample_interval_sec)

    if not values:
        return None
    return float(statistics.median(values))



def _wait_for_fresh_tof(
    sensors: ToFOnlySensorManager,
    timeout_sec: float,
    stop_event: Optional[threading.Event],
) -> Optional[float]:
    """Wait for a fresh ToF sample after a gimbal/filter reset."""
    deadline = time.monotonic() + max(0.05, float(timeout_sec))
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return None
        value = sensors.get_front_cm()
        if value is not None:
            return float(value)
        time.sleep(0.02)
    return None


def _collect_fresh_tof_samples(
    sensors: ToFOnlySensorManager,
    sample_count: int,
    timeout_sec: float,
    stop_event: Optional[threading.Event],
) -> List[float]:
    """Collect distinct ToF callbacks; never count one cached value twice."""
    values: List[float] = []
    last_stamp = None
    deadline = time.monotonic() + max(0.05, float(timeout_sec))
    while len(values) < int(sample_count) and time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            break
        stamp = getattr(sensors, "tof_last_update", None)
        value = sensors.get_front_cm()
        if (
            stamp is not None
            and value is not None
            and math.isfinite(float(value))
            and (
                last_stamp is None
                or float(stamp) > float(last_stamp) + 1e-6
            )
        ):
            values.append(float(value))
            last_stamp = float(stamp)
        if len(values) < int(sample_count):
            if not _sleep_interruptible(0.01, stop_event):
                break
    return values


def _fixed_heading_control_v02(
    config: Classwork8Config,
    target_yaw_deg: float,
    current_yaw_deg: Optional[float],
    x_cmd: float,
    y_cmd: float,
    mode: str,
) -> Tuple[float, float, float, str, Optional[float]]:
    """Continuous yaw P steering without translation pause or speed scaling."""
    if not config.heading_hold_enabled or current_yaw_deg is None:
        return x_cmd, y_cmd, 0.0, mode, None
    error = normalize_angle_deg(float(target_yaw_deg) - float(current_yaw_deg))
    if abs(error) <= float(config.heading_deadband_deg):
        return x_cmd, y_cmd, 0.0, mode, error
    max_z = float(config.heading_max_z_dps)
    z_cmd = error * float(config.heading_kp_z) / float(config.heading_drive_sign)
    z_cmd = max(-max_z, min(max_z, z_cmd))
    return x_cmd, y_cmd, z_cmd, mode, error

def _should_reuse_scan(
    current_cell: Tuple[int, int],
    scanned_cells: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    skip_enabled: bool,
    rescan_requested: bool,
) -> bool:
    """Skip only a previously completed 4-way scan with known topology.

    UNKNOWN edges, first visits, or explicit operator rescans require a
    physical scan. This decision does not disable fresh travel-direction ToF.
    """
    return bool(
        skip_enabled
        and current_cell in scanned_cells
        and not rescan_requested
        and all(
            edge_states.get(
                (current_cell[0], current_cell[1], direction)
            ) in ("OPEN", "WALL")
            for direction in range(4)
        )
    )


def _directions_requiring_scan(
    current_cell: Tuple[int, int],
    preferred_order: List[int],
    edge_states: Dict[Tuple[int, int, int], str],
    traversed_edges: Set[Tuple[Tuple[int, int], Tuple[int, int]]],
) -> Tuple[List[int], List[int]]:
    """Scan unknown edges and every wall face; reuse only known open routes."""
    scan: List[int] = []
    reused: List[int] = []
    for direction in preferred_order:
        state = edge_states.get((current_cell[0], current_cell[1], direction))
        if (_canonical_edge(current_cell, direction) in traversed_edges
                or state == "OPEN"):
            reused.append(direction)
        else:
            scan.append(direction)
    return scan, reused


def _scan_four_directions(
    chassis,
    gimbal,
    pose: PoseTracker,
    sensors: ToFOnlySensorManager,
    gimbal_tracker: GimbalTracker,
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
    current_cell: Tuple[int, int],
    moves: int,
    known_cells: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    traversed_edges: Set[Tuple[Tuple[int, int], Tuple[int, int]]],
    camera_service: Optional[CameraService],
    target_detector: Optional[TargetDetector],
    target_registry: TargetRegistry,
    target_mission: TargetMission,
    target_auto_aim: TargetAutoAim,
    blaster_module,
    target_debug_holder: List[object],
    survey_bridge: LiveSurveyBridge,
    pending_wall_surveys: Set[Tuple[Tuple[int, int], int]],
    wall_survey_failures: Dict[Tuple[Tuple[int, int], int], int],
    verified_retreat_direction: Optional[int] = None,
) -> Optional[Tuple[Dict[int, Optional[float]], Set[int]]]:
    scan_started_at = time.monotonic()
    # Sweep in the direction that is closest to the current gimbal endpoint.
    # This avoids a large BACK(+180) -> LEFT(-90) wrap across the +250 deg
    # mechanical limit.  Each sweep segment is then about 90 degrees.
    current_gimbal_yaw = gimbal_tracker.get_yaw()
    if current_gimbal_yaw is not None and float(current_gimbal_yaw) > 45.0:
        preferred_order = [2, 1, 0, 3]  # BACK -> RIGHT -> FRONT -> LEFT
    else:
        preferred_order = [3, 0, 1, 2]  # LEFT -> FRONT -> RIGHT -> BACK
    ranges: Dict[int, Optional[float]] = {}
    # Opposite-side safety ranges only come from the normal four directions.
    # They become invalid after movement; each mapping ray retains its own
    # real ToF sampling pose, so no extra yaw scan is permitted.
    safety_ranges: Dict[int, Optional[float]] = {}
    open_dirs: Set[int] = set()
    gimbal_scan_retries = 0

    def mark_wall_survey_pending(direction: int, reason: str) -> None:
        key = (current_cell, int(direction) % 4)
        failures = wall_survey_failures.get(key, 0) + 1
        wall_survey_failures[key] = failures
        if failures < 2:
            pending_wall_surveys.add(key)
            event = "WALL_SURVEY_PENDING"
            detail = reason
        else:
            pending_wall_surveys.discard(key)
            event = "WALL_SURVEY_EXHAUSTED"
            detail = "{}; bounded retry exhausted".format(reason)
        recorder.event(
            time.monotonic(),
            event,
            detail,
            logical_node=current_cell,
            direction=DIR_NAME[int(direction) % 4],
            attempts=failures,
        )
        print(
            "[{}] {} {} attempt={}/2".format(
                event,
                current_cell,
                DIR_NAME[int(direction) % 4],
                failures,
            ),
            flush=True,
        )

    def mark_wall_survey_complete(direction: int) -> None:
        key = (current_cell, int(direction) % 4)
        pending_wall_surveys.discard(key)
        wall_survey_failures.pop(key, None)

    def mark_scan_unknown(direction: int, reason: str) -> None:
        """Stop, preserve stronger topology, and defer this scan direction."""
        stop_chassis(chassis)
        ranges[direction] = None
        safety_ranges.pop(direction, None)
        edge_key = _canonical_edge(current_cell, direction)
        prior_state = edge_states.get(
            (current_cell[0], current_cell[1], direction)
        )
        if edge_key in traversed_edges or prior_state == "OPEN":
            open_dirs.add(direction)
            _set_edge_state(edge_states, current_cell, direction, "OPEN")
            known_cells.add(_neighbor(current_cell, direction))
        elif prior_state not in ("OPEN", "WALL"):
            _set_edge_state(edge_states, current_cell, direction, "UNKNOWN")
        recorder.event(
            time.monotonic(),
            "GIMBAL_SCAN_UNKNOWN",
            reason,
            logical_node=current_cell,
            direction=DIR_NAME[direction],
        )
        publish_state(
            status="{} scan UNKNOWN; continuing other directions".format(
                DIR_NAME[direction]
            ),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=None,
            moves=moves,
            force=True,
        )
        print(
            "[GIMBAL_SCAN_UNKNOWN] {}: {}; wheels stopped, continuing.".format(
                DIR_NAME[direction], reason
            ),
            flush=True,
        )

    # Reuse known OPEN routes. A WALL known from its neighbouring cell still
    # needs a physical look here because this is the wall's other camera face.
    order, reused_directions = _directions_requiring_scan(
        current_cell,
        preferred_order,
        edge_states,
        traversed_edges,
    )
    known_wall_directions = {
        direction for direction in order
        if edge_states.get((current_cell[0], current_cell[1], direction))
        == "WALL"
    }
    for direction in reused_directions:
        edge_key = _canonical_edge(current_cell, direction)
        state = edge_states.get((current_cell[0], current_cell[1], direction))
        if edge_key in traversed_edges:
            state = "OPEN"
            _set_edge_state(edge_states, current_cell, direction, state)
        if state == "OPEN":
            open_dirs.add(direction)
            known_cells.add(_neighbor(current_cell, direction))

    if reused_directions:
        recorder.event(
            time.monotonic(),
            "SCAN_DIRECTIONS_REUSED",
            "known topology skipped before physical gimbal sweep",
            logical_node=current_cell,
            directions=[DIR_NAME[direction] for direction in reused_directions],
        )
        print(
            "[SCAN_REUSE] cell={} known={} scanning={}".format(
                current_cell,
                ",".join(DIR_NAME[d] for d in reused_directions),
                ",".join(DIR_NAME[d] for d in order) or "none",
            ),
            flush=True,
        )

    # Caller enters acknowledged zero-wheel mode before stationary scan.
    for direction in order:
        if stop_event is not None and stop_event.is_set():
            return None

        _heading_snapshot(
            "PRE_GIMBAL_{}_{}".format(current_cell, DIR_NAME[direction]),
            pose, gimbal_tracker, float(start_yaw_deg),
        )
        print(
            "[SCAN] Pointing Gimbal {} (target {:+.0f} deg)...".format(
                DIR_NAME[direction],
                config.gimbal_yaw_for_direction(direction),
            ),
            flush=True,
        )

        publish_state(
            status="Scanning {}".format(DIR_NAME[direction]),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=sensors.get_front_cm(),
            moves=moves,
            force=True,
        )

        gimbal_ok, retries_used = _aim_scan_direction_with_retry(
            gimbal, sensors, gimbal_tracker, direction, config, stop_event
        )
        gimbal_scan_retries += retries_used
        if not gimbal_ok:
            if stop_event is not None and stop_event.is_set():
                return None
            print(
                "[SCAN] Gimbal FAILED for {} after bounded retry; "
                "recording UNKNOWN. measured_yaw={}".format(
                    DIR_NAME[direction],
                    "---" if gimbal_tracker.get_yaw() is None
                    else "{:+.1f}".format(float(gimbal_tracker.get_yaw())),
                ),
                flush=True,
            )
            mark_scan_unknown(
                direction,
                "scan aim failed after {} bounded retry attempt(s); "
                "revisit required".format(retries_used),
            )
            continue

        _heading_snapshot(
            "POST_GIMBAL_{}_{}".format(current_cell, DIR_NAME[direction]),
            pose, gimbal_tracker, float(start_yaw_deg),
        )
        print(
            "[SCAN] Gimbal {} ready: yaw={:+.1f} pitch={:+.1f} deg.".format(
                DIR_NAME[direction],
                float(gimbal_tracker.get_yaw()),
                float(gimbal_tracker.get_pitch()),
            ),
            flush=True,
        )

        distance_cm = _sample_tof(sensors, config, stop_event)

        # Do not classify an edge using an upward/downward-tilted ToF ray.
        # Check again AFTER the sample interval, not only when yaw settled.
        pitch_after_sample = gimbal_tracker.get_pitch()
        if (
            pitch_after_sample is None
            or abs(
                float(pitch_after_sample) - float(config.gimbal_scan_pitch_deg)
            ) > float(config.gimbal_pitch_tolerance_deg)
        ):
            print(
                "[SCAN] {} pitch drift after ToF sampling (pitch={}); "
                "restore pitch only; no extra yaw sweep.".format(
                    DIR_NAME[direction],
                    "---" if pitch_after_sample is None
                    else "{:+.1f}".format(float(pitch_after_sample)),
                ),
                flush=True,
            )
            if not _set_camera_observation_pitch(
                gimbal, gimbal_tracker, config,
                config.gimbal_scan_pitch_deg, stop_event,
                tolerance_deg=config.gimbal_pitch_tolerance_deg,
                clamp_camera_limits=False,
            ):
                if stop_event is not None and stop_event.is_set():
                    return None
                mark_scan_unknown(direction, "horizontal pitch restore failed")
                continue
            final_pitch, final_yaw = gimbal_tracker.get_angles()
            if (
                final_pitch is None or final_yaw is None
                or abs(
                    float(final_pitch) - float(config.gimbal_scan_pitch_deg)
                ) > float(config.gimbal_pitch_tolerance_deg)
                or abs(_heading_error(
                    config.gimbal_yaw_for_direction(direction), final_yaw
                )) > float(config.gimbal_tolerance_deg)
            ):
                mark_scan_unknown(
                    direction, "Gimbal feedback invalid after pitch restore"
                )
                continue
            sensors.reset_filters()
            distance_cm = _sample_tof(sensors, config, stop_event)
            if distance_cm is None:
                if stop_event is not None and stop_event.is_set():
                    return None
                mark_scan_unknown(direction, "fresh ToF missing after pitch restore")
                continue
            final_pitch = gimbal_tracker.get_pitch()
            if (
                final_pitch is None
                or abs(
                    float(final_pitch) - float(config.gimbal_scan_pitch_deg)
                ) > float(config.gimbal_pitch_tolerance_deg)
            ):
                print(
                    "[SCAN] Unsafe pitch after retry; refusing ToF/map update.",
                    flush=True,
                )
                mark_scan_unknown(
                    direction, "pitch unsafe after restored ToF retry"
                )
                continue

        if distance_cm is None:
            if stop_event is not None and stop_event.is_set():
                return None
            mark_scan_unknown(direction, "fresh horizontal ToF unavailable")
            continue

        # V02: readings between a definite near wall and the normal OPEN
        # threshold are ambiguous.  A foam edge / floor reflection can create
        # one short median even when the branch is physically open.  Re-sample
        # at the same gimbal angle and keep the larger robust median.  If this
        # turns out to be a false-open remains a mapping risk, so movement also
        # requires its independent fresh-ToF preflight and live hard stop.
        if (
            distance_cm is not None
            and float(config.scan_hard_wall_cm) < float(distance_cm)
            < float(config.tof_open_cm)
            and int(config.scan_ambiguous_retries) > 0
        ):
            retry_values = [float(distance_cm)]
            for _ in range(int(config.scan_ambiguous_retries)):
                sensors.reset_filters()
                if not _sleep_interruptible(
                    config.scan_ambiguous_retry_settle_sec,
                    stop_event,
                ):
                    return None
                retry = _sample_tof(sensors, config, stop_event)
                if retry is not None:
                    retry_values.append(float(retry))
            distance_cm = max(retry_values)
            print(
                "[SCAN] {} ambiguous -> retry candidates {} -> {:.1f} cm".format(
                    DIR_NAME[direction],
                    [round(v, 1) for v in retry_values],
                    float(distance_cm),
                ),
                flush=True,
            )

        # One final pitch guard also covers the ambiguous-range retry period.
        # Never commit WALL/OPEN topology from a ray that has tilted away from
        # its horizontal scan plane.
        final_scan_pitch = gimbal_tracker.get_pitch()
        if (
            final_scan_pitch is None
            or abs(
                float(final_scan_pitch) - float(config.gimbal_scan_pitch_deg)
            ) > float(config.gimbal_pitch_tolerance_deg)
        ):
            print(
                "[SCAN] {} pitch changed during sampling/retry; aborting scan "
                "without updating topology.".format(DIR_NAME[direction]),
                flush=True,
            )
            mark_scan_unknown(
                direction, "pitch changed during ToF sampling/retry"
            )
            continue

        print(
            "[SCAN] {} ToF = {} cm".format(
                DIR_NAME[direction],
                "---" if distance_cm is None else "{:.1f}".format(distance_cm),
            ),
            flush=True,
        )
        # Each direction is self-contained: scan -> if TOO CLOSE, retreat
        # NOW -> hold at this SAME yaw for camera sign verification -> next.
        # No retrospective correction of an earlier direction.
        adjusted = False
        clearance_origin_map = _relative_xy(
            pose, start_x, start_y, start_yaw_deg, config
        )
        # A stationary target test must never translate the chassis, even if
        # wall-clearance adjustment remains checked in a saved GUI profile.
        if config.wall_clearance_enabled and not config.stationary_target_test:
            safety_ranges[direction] = distance_cm
            adjusted, failure, clearance_telemetry = (
                _maintain_wall_clearance_checkpoint(
                    chassis, gimbal, pose, sensors, gimbal_tracker, config,
                    safety_ranges, direction, float(start_x), float(start_y),
                    float(start_yaw_deg), stop_event,
                    verified_retreat_direction=verified_retreat_direction,
                )
            )
            if clearance_telemetry is not None:
                telemetry_fields = {
                    key: clearance_telemetry[key]
                    for key in (
                        "before_cm", "after_cm", "shifted_m", "limit_m",
                        "result",
                    )
                }
                recorder.event(
                    clearance_telemetry["started_at"],
                    "CLEARANCE_ADJUST_STARTED",
                    "bounded same-direction wall-clearance movement started",
                    logical_node=current_cell,
                    direction=DIR_NAME[direction],
                    **dict(telemetry_fields, result="STARTED"),
                )
                completed_event = (
                    "CLEARANCE_TARGET_REACHED"
                    if clearance_telemetry["result"] == "TARGET_REACHED"
                    else "CLEARANCE_ADJUST_FINISHED"
                )
                recorder.event(
                    clearance_telemetry["finished_at"],
                    completed_event,
                    "bounded same-direction wall-clearance movement finished",
                    logical_node=current_cell,
                    direction=DIR_NAME[direction],
                    **telemetry_fields,
                )
            if failure is not None:
                print(
                    "[CLEARANCE_WARN] {} during {} scan; wheels stopped, "
                    "keeping this direction non-fatal and continuing the "
                    "mission.".format(failure, DIR_NAME[direction]),
                    flush=True,
                )
                recorder.event(
                    time.monotonic(), "CLEARANCE_SKIPPED",
                    "bounded clearance adjustment stopped: {}".format(failure),
                    logical_node=current_cell,
                    direction=DIR_NAME[direction],
                )
                if failure == "USER_STOP":
                    return None
            if adjusted:
                print(
                    "[CLEARANCE_DURING_SCAN] {} adjusted; using live ToF "
                    "at this SAME direction, no additional yaw sweep.".format(
                        DIR_NAME[direction]
                    ), flush=True,
                )
                # Restart ToF median at the adjusted pose, without restarting
                # the four-way scan. Previous mapped rays were sampled at
                # their original measured poses; invalidate safety cache.
                # Gimbal has stayed at THIS scan direction for the entire
                # corrective move: no second yaw turn or extra direction.
                sensors.reset_filters()
                distance_cm = _sample_tof(sensors, config, stop_event)
                final_pitch, final_yaw = gimbal_tracker.get_angles()
                if (
                    distance_cm is None or final_pitch is None
                    or final_yaw is None
                    or abs(float(final_pitch) - float(config.gimbal_scan_pitch_deg))
                    > float(config.gimbal_pitch_tolerance_deg)
                    or abs(_heading_error(
                        config.gimbal_yaw_for_direction(direction), final_yaw
                    )) > float(config.gimbal_tolerance_deg)
                ):
                    print(
                        "[CLEARANCE_WARN] Fresh same-direction ToF/pitch/yaw "
                        "unavailable after movement; marking this ray UNKNOWN "
                        "and continuing the mission.",
                        flush=True,
                    )
                    recorder.event(
                        time.monotonic(), "CLEARANCE_POSTCHECK_UNKNOWN",
                        "post-adjustment ray unavailable; navigation continues",
                        logical_node=current_cell,
                        direction=DIR_NAME[direction],
                    )
                    distance_cm = None
                    safety_ranges.clear()
                    verified_retreat_direction = None
                    ranges[direction] = None
                    continue
                safety_ranges.clear()
                safety_ranges[direction] = distance_cm
                # After translating in this cell, the just-traversed route
                # is no longer a guaranteed 4 cm corridor for OTHER sides.
                verified_retreat_direction = None
                print(
                    "[CLEARANCE_DURING_SCAN] {} final ToF={:.1f}cm".format(
                        DIR_NAME[direction], distance_cm
                    ), flush=True,
                )
        ranges[direction] = distance_cm

        # Camera targets are physically lower than the horizontal ToF ray.
        # Keep the proven mapping range above, then (only while chassis is
        # stopped) tip the camera down to its independently configured angle.
        # Restore horizontal ToF before any next ray or chassis movement.
        near_wall = (
            distance_cm is not None
            and float(distance_cm) < float(config.tof_open_cm)
        )
        live_preview_status = survey_bridge.latest_preview()
        preview_candidate = bool(
            float(live_preview_status.get("age_sec", float("inf"))) <= 1.25
            and int(live_preview_status.get("candidate_count", 0)) > 0
        )
        scan_budget_available = _scan_budget_allows_optional_work(
            scan_started_at,
            time.monotonic(),
            config.scan_cell_budget_sec,
        )
        known_wall_face = direction in known_wall_directions
        wall_face = near_wall or known_wall_face
        survey_requested = _camera_survey_required(
            wall_face,
            scan_budget_available,
            bool(config.target_survey_open_directions),
            preview_candidate,
        )
        survey_this_direction = (
            camera_service is not None
            and camera_service.running
            and target_detector is not None
            and survey_requested
        )
        wall_survey_completed = False

        if survey_this_direction:
            # Explicit intentional pause: target observation, NOT motion safety.
            recorder.event(
                time.monotonic(), "TARGET_SCAN_PAUSE",
                "stationary camera target observation",
                logical_node=current_cell, direction=DIR_NAME[direction],
            )
            print(
                "[TARGET_SCAN_PAUSE] stationary survey {}".format(
                    DIR_NAME[direction]
                ), flush=True,
            )
            selected_pitch = float(survey_bridge.get_pitch())
            survey_bridge.set_status(
                "Survey {} at pitch {:+.1f} deg (robot stopped)".format(
                    DIR_NAME[direction], selected_pitch
                )
            )
            publish_state(
                status="Camera surveying {} at {:+.1f} deg".format(
                    DIR_NAME[direction], selected_pitch
                ),
                logical_cell=current_cell,
                gimbal_direction=direction,
                tof_cm=distance_cm,
                moves=moves,
                force=True,
            )

            verified_targets = []
            restore_ok = False
            camera_position_ok = _set_camera_observation_pitch(
                gimbal,
                gimbal_tracker,
                config,
                selected_pitch,
                stop_event,
            )
            try:
                if camera_position_ok:
                    # Do not verify cached frames from the previous horizontal
                    # ToF viewpoint: the low sign may only enter the image
                    # after the new camera pitch has settled.
                    survey_frame_epoch = time.monotonic()
                    if adjusted:
                        # The camera is now pitched down AND the chassis has
                        # already stopped after retreat. Keep exactly THIS yaw
                        # direction and let new floor-sign frames accumulate
                        # before verification; do not turn to the next side.
                        print(
                            "[CLEARANCE_CAMERA_HOLD] {} {:.2f}s at new pose; "
                            "camera pitch={:+.1f}deg; awaiting fresh sign frames".format(
                                DIR_NAME[direction],
                                float(config.wall_clearance_camera_dwell_sec),
                                selected_pitch,
                            ), flush=True,
                        )
                        if not _sleep_interruptible(
                            float(config.wall_clearance_camera_dwell_sec),
                            stop_event,
                        ):
                            return None
                    quick_gate_allowed = bool(
                        wall_face
                        or _scan_budget_allows_optional_work(
                            scan_started_at,
                            time.monotonic(),
                            config.scan_cell_budget_sec,
                        )
                    )
                    if quick_gate_allowed:
                        quick_candidate, target_debug = (
                            _quick_target_candidate_or_false(
                                target_detector,
                                camera_service,
                                survey_frame_epoch,
                                recorder,
                                current_cell,
                                direction,
                            )
                        )
                    else:
                        quick_candidate, target_debug = False, None
                        print(
                            "[SCAN_BUDGET] {} camera quick gate skipped; "
                            "cell budget is near its deadline.".format(
                                DIR_NAME[direction]
                            ),
                            flush=True,
                        )
                    target_debug_holder[0] = target_debug
                    recorder.event(
                        time.monotonic(),
                        (
                            "TARGET_QUICK_GATE"
                            if quick_gate_allowed
                            else "TARGET_QUICK_GATE_SKIPPED"
                        ),
                        (
                            "candidate found" if quick_candidate
                            else "no candidate" if quick_gate_allowed
                            else "cell scan budget near deadline"
                        ),
                        logical_node=current_cell,
                        direction=DIR_NAME[direction],
                        frames=int(config.target_quick_gate_frames),
                        candidate=quick_candidate,
                    )
                    full_verify_allowed = bool(
                        wall_face
                        or _scan_budget_allows_optional_work(
                            scan_started_at,
                            time.monotonic(),
                            config.scan_cell_budget_sec,
                        )
                    )
                    if quick_candidate and full_verify_allowed:
                        # Full temporal verification starts after the gate so
                        # it still requires four new distinct camera frames.
                        verified_targets, target_debug = _verify_targets_or_empty(
                            target_detector,
                            camera_service,
                            time.monotonic(),
                            recorder,
                            current_cell,
                            direction,
                        )
                        target_debug_holder[0] = target_debug
                        wall_survey_completed = bool(
                            not wall_face or verified_targets
                        )
                    elif not quick_candidate and quick_gate_allowed:
                        wall_survey_completed = bool(
                            not wall_face or target_debug is not None
                        )
                        print(
                            "[TARGET_QUICK_GATE] {} no candidate in {} fresh "
                            "frame(s); skipping full verification.".format(
                                DIR_NAME[direction],
                                int(config.target_quick_gate_frames),
                            ),
                            flush=True,
                        )
                    elif quick_candidate:
                        recorder.event(
                            time.monotonic(),
                            "TARGET_VERIFY_SKIPPED",
                            "cell scan budget near deadline",
                            logical_node=current_cell,
                            direction=DIR_NAME[direction],
                        )
                        print(
                            "[SCAN_BUDGET] {} full target verification "
                            "skipped near deadline.".format(
                                DIR_NAME[direction]
                            ),
                            flush=True,
                        )

                    measured_pitch = gimbal_tracker.get_pitch()
                    if (
                        measured_pitch is None
                        or abs(float(measured_pitch) - selected_pitch)
                        > float(config.target_camera_pitch_tolerance_deg)
                    ):
                        print(
                            "[TARGET] Discarding observations: camera pitch "
                            "changed while sampling.",
                            flush=True,
                        )
                        verified_targets = []

                    for verified_index, verified in enumerate(verified_targets):
                        saved_target = target_registry.add_verified(
                            verified,
                            current_cell,
                            direction,
                            distance_cm,
                            range_confirmed_wall=near_wall,
                            camera_pitch_deg=selected_pitch,
                        )
                        recorder.event(
                            time.monotonic(),
                            "TARGET_OBSERVATION",
                            "{} {} from {} {}".format(
                                verified.detection.color.upper(),
                                verified.detection.shape.upper(),
                                current_cell,
                                DIR_NAME[direction],
                            ),
                            logical_node=current_cell,
                            direction=DIR_NAME[direction],
                            target_id=saved_target["target_id"],
                            target_color=verified.detection.color,
                            target_shape=verified.detection.shape,
                            target_confidence=round(float(verified.confidence), 4),
                            camera_pitch_deg=selected_pitch,
                            range_confirmed_wall=near_wall,
                            tof_cm=distance_cm,
                            localization_status=saved_target["localization_status"],
                            sighting_cell_hint=saved_target.get("sighting_cell_hint"),
                        )
                        print(
                            "[TARGET] {} {} -> {} conf={:.2f} {} from {} {}".format(
                                verified.detection.color.upper(),
                                verified.detection.shape.upper(),
                                saved_target["target_id"],
                                float(verified.confidence),
                                "NEAR_WALL_ESTIMATE" if near_wall
                                else "SIGHTING_ONLY (distance unconfirmed)",
                                current_cell,
                                DIR_NAME[direction],
                            ),
                            flush=True,
                        )
                        debug_shape = (
                            None if target_debug is None else target_debug.shape
                        )
                        frame_size = (
                            (0, 0)
                            if debug_shape is None
                            else (int(debug_shape[1]), int(debug_shape[0]))
                        )
                        aim_offset_x = float(config.target_aim_offset_x_ratio)
                        aim_offset_y = float(config.target_aim_offset_y_ratio)
                        if (
                            near_wall
                            and distance_cm is not None
                            and frame_size[0] > 0
                            and frame_size[1] > 0
                        ):
                            aim_offset_x, aim_offset_y = calibrated_aim_offsets(
                                config,
                                max(0.10, float(distance_cm) / 100.0),
                                frame_size,
                            )
                            print(
                                "[TARGET_AIM_CAL] {} ToF={:.1f}cm "
                                "camera_above={:.1f}cm impact_offset="
                                "({:+.3f},{:+.3f})".format(
                                    saved_target["target_id"],
                                    float(distance_cm),
                                    float(config.target_camera_above_blaster_m) * 100.0,
                                    aim_offset_x,
                                    aim_offset_y,
                                ),
                                flush=True,
                            )
                        decision = target_mission.assess(
                            saved_target,
                            centroid_px=verified.detection.centroid,
                            frame_size_px=frame_size,
                            tof_cm=distance_cm,
                            range_confirmed=near_wall,
                            aim_confirmed=False,
                        )
                        target_mission.annotate_target(saved_target, decision)
                        aim_result = None
                        aim_budget_sec = (
                            scan_started_at
                            + float(config.scan_cell_budget_sec)
                            - time.monotonic()
                            - 1.0
                        )
                        configured_aim_timeout = float(
                            config.target_auto_aim_timeout_sec
                        )
                        if configured_aim_timeout <= 0.0:
                            configured_aim_timeout = 6.0
                        if wall_face:
                            aim_budget_sec = configured_aim_timeout
                        if (
                            decision.state == TargetMissionState.NEEDS_AIM
                            and aim_budget_sec >= 0.50
                        ):
                            # Target selection and range gates have passed.
                            # Enter visual servo only while wheel-zero is ACKed.
                            stop_chassis(chassis)
                            target_mission.mark_aiming(decision.target_id)
                            recorder.event(
                                time.monotonic(), "TARGET_AIM",
                                "bounded fresh-frame auto-aim started",
                                target_id=decision.target_id,
                                target_spec=decision.spec.key,
                            )
                            survey_bridge.set_status(
                                "Auto-aiming {} (robot stopped)".format(
                                    decision.spec.key
                                )
                            )
                            aim_start_pitch, aim_start_yaw = (
                                gimbal_tracker.get_angles()
                            )
                            aim_result = target_auto_aim.aim(
                                gimbal=gimbal,
                                tracker=gimbal_tracker,
                                camera_service=camera_service,
                                detector=target_detector,
                                initial_detection=verified.detection,
                                stop_event=stop_event,
                                aim_offset_x_ratio=aim_offset_x,
                                aim_offset_y_ratio=aim_offset_y,
                                timeout_sec=min(
                                    configured_aim_timeout,
                                    aim_budget_sec,
                                ),
                            )
                            retryable_aim_reasons = {
                                "AIM_TARGET_LOST",
                                "AIM_DIVERGING",
                                "AIM_TIMEOUT",
                                "AIM_CAMERA_FRAME_STALE",
                                "AIM_GIMBAL_FEEDBACK_STALE",
                            }
                            if (
                                not aim_result.success
                                and aim_result.reason in retryable_aim_reasons
                                and (stop_event is None or not stop_event.is_set())
                                and (
                                    wall_face
                                    or scan_started_at
                                    + float(config.scan_cell_budget_sec)
                                    - time.monotonic() - 1.0 >= 0.50
                                )
                            ):
                                print(
                                    "[TARGET_AIM] {} {} -> restore start pose "
                                    "and retry slowly.".format(
                                        decision.target_id, aim_result.reason
                                    ),
                                    flush=True,
                                )
                                if _restore_auto_aim_start_pose(
                                    gimbal,
                                    gimbal_tracker,
                                    config,
                                    aim_start_pitch,
                                    aim_start_yaw,
                                    stop_event,
                                ):
                                    _sleep_interruptible(0.15, stop_event)
                                    aim_result = target_auto_aim.aim(
                                        gimbal=gimbal,
                                        tracker=gimbal_tracker,
                                        camera_service=camera_service,
                                        detector=target_detector,
                                        initial_detection=verified.detection,
                                        stop_event=stop_event,
                                        aim_offset_x_ratio=aim_offset_x,
                                        aim_offset_y_ratio=aim_offset_y,
                                        speed_scale=0.50,
                                        timeout_sec=(
                                            configured_aim_timeout
                                            if wall_face else min(
                                                configured_aim_timeout,
                                                max(
                                                    0.50,
                                                    scan_started_at
                                                    + float(config.scan_cell_budget_sec)
                                                    - time.monotonic() - 1.0,
                                                ),
                                            )
                                        ),
                                    )
                                else:
                                    print(
                                        "[TARGET_AIM] {} start-pose restore "
                                        "failed; skipping unsafe retry.".format(
                                            decision.target_id
                                        ),
                                        flush=True,
                                    )
                            print(
                                "[TARGET_AIM] {} {} fresh={} pitch={} yaw={}.".format(
                                    decision.target_id,
                                    aim_result.reason,
                                    aim_result.fresh_frames,
                                    "---" if aim_result.final_pitch_deg is None
                                    else "{:+.1f}".format(aim_result.final_pitch_deg),
                                    "---" if aim_result.final_yaw_deg is None
                                    else "{:+.1f}".format(aim_result.final_yaw_deg),
                                ),
                                flush=True,
                            )
                            target_mission.mark_aim_result(
                                decision.target_id, aim_result.success
                            )
                            saved_target["auto_aim_attempted"] = True
                            saved_target["auto_aim_success"] = aim_result.success
                            saved_target["auto_aim_reason"] = aim_result.reason
                            saved_target["auto_aim_fresh_frames"] = (
                                aim_result.fresh_frames
                            )
                            saved_target["auto_aim_final_pitch_deg"] = (
                                aim_result.final_pitch_deg
                            )
                            saved_target["auto_aim_final_yaw_deg"] = (
                                aim_result.final_yaw_deg
                            )
                            if aim_result.detection is not None:
                                saved_target["auto_aim_centroid_px"] = list(
                                    aim_result.detection.centroid
                                )
                            if aim_result.debug_frame is not None:
                                target_debug_holder[0] = aim_result.debug_frame
                            recorder.event(
                                time.monotonic(), "TARGET_AIM",
                                aim_result.reason,
                                target_id=decision.target_id,
                                success=aim_result.success,
                                fresh_frames=aim_result.fresh_frames,
                                final_pitch_deg=aim_result.final_pitch_deg,
                                final_yaw_deg=aim_result.final_yaw_deg,
                            )
                            if (
                                aim_result.success
                                and aim_result.detection is not None
                            ):
                                decision = target_mission.assess(
                                    saved_target,
                                    centroid_px=aim_result.detection.centroid,
                                    frame_size_px=aim_result.frame_size_px,
                                    tof_cm=distance_cm,
                                    range_confirmed=near_wall,
                                    aim_confirmed=True,
                                    aim_offset_x_ratio=aim_offset_x,
                                    aim_offset_y_ratio=aim_offset_y,
                                )
                                target_mission.annotate_target(
                                    saved_target, decision
                                )
                            else:
                                target_mission.annotate_target(
                                    saved_target,
                                    decision,
                                    detail_override=aim_result.reason,
                                )
                        elif decision.state == TargetMissionState.NEEDS_AIM:
                            target_mission.annotate_target(
                                saved_target,
                                decision,
                                detail_override="SCAN_BUDGET_AUTO_AIM_DEFERRED",
                            )
                            recorder.event(
                                time.monotonic(),
                                "TARGET_AIM_SKIPPED",
                                "cell scan budget near deadline",
                                target_id=decision.target_id,
                                target_spec=decision.spec.key,
                            )
                            print(
                                "[SCAN_BUDGET] {} auto-aim deferred; "
                                "navigation continues.".format(
                                    decision.target_id
                                ),
                                flush=True,
                            )
                        fire_ack = False
                        if decision.should_fire:
                            # The chassis has remained in acknowledged wheel-zero
                            # mode throughout this target survey.
                            stop_chassis(chassis)
                            fire_ack = target_mission.fire(
                                decision, blaster_module
                            )
                            target_mission.annotate_target(saved_target, decision)
                        recorder.event(
                            time.monotonic(), "TARGET_MISSION",
                            target_mission.states[decision.target_id].value,
                            target_id=decision.target_id,
                            target_spec=decision.spec.key,
                            selected=saved_target["selected_for_fire"],
                            distance_m=decision.distance_m,
                            fire_acknowledged=fire_ack,
                        )
                        print(
                            "[TARGET_MISSION] {} {} distance={} fire_ack={}".format(
                                decision.target_id,
                                target_mission.states[decision.target_id].value,
                                "---" if decision.distance_m is None
                                else "{:.2f}m".format(decision.distance_m),
                                fire_ack,
                            ),
                            flush=True,
                        )
                        if not near_wall:
                            print(
                                "[TARGET] {} is a distant sighting only. "
                                "Candidate cell {} along {}; do NOT use as "
                                "Round-2 target position until close revalidation.".format(
                                    saved_target["target_id"],
                                    saved_target.get("sighting_cell_hint"),
                                    DIR_NAME[direction],
                                ),
                                flush=True,
                            )
                        if aim_result is not None:
                            # Auto-aim intentionally leaves the Gimbal at the
                            # firing pose. Return yaw without sampling ToF; this
                            # is not an extra map scan direction.
                            yaw_restored, _pitch_excursion, _started = (
                                _set_gimbal_yaw_only(
                                    gimbal,
                                    gimbal_tracker,
                                    config.gimbal_yaw_for_direction(direction),
                                    selected_pitch,
                                    config,
                                    stop_event,
                                )
                            )
                            if not yaw_restored:
                                print(
                                    "[TARGET_AIM] Cannot restore scan yaw; "
                                    "skipping remaining target work for this direction.",
                                    flush=True,
                                )
                                break
                            if not _set_camera_observation_pitch(
                                gimbal,
                                gimbal_tracker,
                                config,
                                (
                                    selected_pitch
                                    if verified_index + 1 < len(verified_targets)
                                    else config.gimbal_scan_pitch_deg
                                ),
                                stop_event,
                                clamp_camera_limits=(
                                    verified_index + 1 < len(verified_targets)
                                ),
                            ):
                                break
                else:
                    print(
                        "[TARGET] Camera pitch not reached at {}; survey skipped.".format(
                            DIR_NAME[direction]
                        ),
                        flush=True,
                    )
            finally:
                # This restore is mandatory. The ToF ray is not safe for
                # navigation or topology if the camera remains angled down.
                # The camera already faces THIS direction. Restore pitch
                # directly without an additional yaw controller/sweep.
                for restore_attempt in range(2):
                    restore_ok = _set_camera_observation_pitch(
                        gimbal, gimbal_tracker, config,
                        config.gimbal_scan_pitch_deg, stop_event,
                        tolerance_deg=config.gimbal_pitch_tolerance_deg,
                        clamp_camera_limits=False,
                    )
                    restored_pitch, restored_yaw = gimbal_tracker.get_angles()
                    restore_ok = bool(
                        restore_ok and restored_pitch is not None
                        and restored_yaw is not None
                        and abs(
                            float(restored_pitch) - config.gimbal_scan_pitch_deg
                        ) <= config.gimbal_pitch_tolerance_deg
                        and abs(_heading_error(
                            config.gimbal_yaw_for_direction(direction),
                            restored_yaw,
                        )) <= config.gimbal_tolerance_deg
                    )
                    if restore_ok:
                        break
                    if restore_attempt == 0:
                        recorder.event(
                            time.monotonic(), "TARGET_GIMBAL_RESTORE_RETRY",
                            "retrying horizontal ToF pose once",
                            logical_node=current_cell,
                            direction=DIR_NAME[direction],
                        )
                if restore_ok:
                    sensors.reset_filters()
                survey_bridge.set_status(
                    "Live preview; ToF horizontal restored"
                    if restore_ok else "ERROR: cannot restore horizontal ToF"
                )

            if not restore_ok:
                if stop_event is not None and stop_event.is_set():
                    return None
                if adjusted and clearance_origin_map[0] is not None:
                    _return_to_scan_origin(
                        chassis, pose, config, clearance_origin_map,
                        start_x, start_y, start_yaw_deg, stop_event,
                    )
                if wall_face:
                    mark_wall_survey_pending(
                        direction, "camera survey or horizontal restore failed"
                    )
                mark_scan_unknown(
                    direction, "camera survey pitch restore failed"
                )
                continue

            if wall_face:
                if wall_survey_completed:
                    mark_wall_survey_complete(direction)
                else:
                    mark_wall_survey_pending(
                        direction,
                        "no fresh quick-gate frame or candidate did not verify",
                    )

            print(
                "[TARGET_SCAN_RESUME] survey completed; continuing scan/navigation.",
                flush=True,
            )
            live_preview_status = survey_bridge.latest_preview()
            recorder.event(
                time.monotonic(),
                "TARGET_SURVEY",
                "{} at {:+.1f} deg: {} verified / {} current candidates, "
                "wall_range_confirmed={}".format(
                    DIR_NAME[direction],
                    selected_pitch,
                    len(verified_targets),
                    live_preview_status["candidate_count"],
                    near_wall,
                ),
                logical_node=current_cell,
                direction=DIR_NAME[direction],
                verified_targets=len(verified_targets),
                live_candidate_count=live_preview_status["candidate_count"],
                camera_pitch_deg=selected_pitch,
                range_confirmed_wall=near_wall,
                tof_cm=distance_cm,
            )
            print(
                "[TARGET_SURVEY] {} pitch={:+.1f} candidates={} verified={} wall={}".format(
                    DIR_NAME[direction],
                    selected_pitch,
                    live_preview_status["candidate_count"],
                    len(verified_targets),
                    near_wall,
                ),
                flush=True,
            )
        elif adjusted:
            # With no camera survey available, still remain stationary for
            # the configured post-adjustment pause before changing direction.
            print(
                "[CLEARANCE_HOLD] {} {:.2f}s, camera survey unavailable; "
                "next direction only after hold".format(
                    DIR_NAME[direction],
                    float(config.wall_clearance_camera_dwell_sec),
                ), flush=True,
            )
            if not _sleep_interruptible(
                float(config.wall_clearance_camera_dwell_sec), stop_event,
            ):
                return None
        elif camera_service is not None and camera_service.running:
            if not scan_budget_available:
                skip_reason = "cell scan budget exhausted"
            elif not near_wall and not preview_candidate:
                skip_reason = "open direction with no fresh preview candidate"
            else:
                skip_reason = "camera survey disabled for this direction"
            recorder.event(
                time.monotonic(),
                "TARGET_SURVEY_SKIPPED",
                "{}: {}".format(DIR_NAME[direction], skip_reason),
                logical_node=current_cell,
                direction=DIR_NAME[direction],
                tof_cm=distance_cm,
                scan_elapsed_sec=round(time.monotonic() - scan_started_at, 3),
                scan_budget_sec=float(config.scan_cell_budget_sec),
                preview_candidate=preview_candidate,
            )

        if adjusted and clearance_origin_map[0] is not None:
            centered, center_reason, residual_m = _return_to_scan_origin(
                chassis, pose, config, clearance_origin_map,
                start_x, start_y, start_yaw_deg, stop_event,
            )
            recorder.event(
                time.monotonic(),
                "CLEARANCE_CENTER_RETURN",
                center_reason,
                logical_node=current_cell,
                direction=DIR_NAME[direction],
                success=centered,
                residual_m=round(float(residual_m), 4),
            )
            print(
                "[CLEARANCE_CENTER_RETURN] {} {} residual={:.3f}m".format(
                    DIR_NAME[direction], center_reason, residual_m
                ),
                flush=True,
            )
            if center_reason == "USER_STOP":
                return None
            if centered:
                sensors.reset_filters()
                centered_cm = _sample_tof(sensors, config, stop_event)
                if centered_cm is not None:
                    distance_cm = centered_cm
                    ranges[direction] = centered_cm
                    safety_ranges.clear()
                    safety_ranges[direction] = centered_cm

        if (
            wall_face
            and not survey_this_direction
            and bool(config.target_detection_enabled)
        ):
            mark_wall_survey_pending(
                direction, "camera survey service unavailable"
            )

        rel_x, rel_y = _relative_xy(
            pose, start_x, start_y, start_yaw_deg, config
        )
        yaw = pose.get_yaw()

        if rel_x is not None and rel_y is not None:
            _update_tof_ray(
                grid,
                config,
                rel_x,
                rel_y,
                direction,
                distance_cm,
            )

        edge_key = _canonical_edge(current_cell, direction)

        if edge_key in traversed_edges:
            # Physical traversal is stronger evidence than a later noisy scan.
            open_dirs.add(direction)
            _set_edge_state(edge_states, current_cell, direction, "OPEN")
            known_cells.add(_neighbor(current_cell, direction))
        elif direction in known_wall_directions:
            # Reciprocal wall topology is stronger than one contradictory ray,
            # but this side was still scanned for clearance and camera signs.
            _set_edge_state(edge_states, current_cell, direction, "WALL")
        elif distance_cm is not None:
            if distance_cm >= config.tof_open_cm:
                open_dirs.add(direction)
                _set_edge_state(edge_states, current_cell, direction, "OPEN")
                known_cells.add(_neighbor(current_cell, direction))
            else:
                _set_edge_state(edge_states, current_cell, direction, "WALL")

        recorder.record_sample(
            time.monotonic(),
            rel_x,
            rel_y,
            yaw,
            direction,
            distance_cm,
            None,
            None,
            None,
            None,
            "GIMBAL_SCAN_{}".format(DIR_NAME[direction]),
        )

        publish_state(
            status="Scanned {}".format(DIR_NAME[direction]),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=distance_cm,
            moves=moves,
            force=True,
        )

    scan_elapsed_sec = time.monotonic() - scan_started_at
    budget_overrun = scan_elapsed_sec > float(config.scan_cell_budget_sec)
    recorder.event(
        time.monotonic(),
        "SCAN_BUDGET",
        "new-cell scan timing",
        logical_node=current_cell,
        elapsed_sec=round(scan_elapsed_sec, 3),
        budget_sec=float(config.scan_cell_budget_sec),
        scanned_directions=[DIR_NAME[d] for d in order],
        reused_directions=[DIR_NAME[d] for d in reused_directions],
        overrun=budget_overrun,
    )
    print(
        "[SCAN_BUDGET] cell={} elapsed={:.2f}s budget={:.2f}s "
        "scanned={} reused={} gimbal_retries={} overrun={}".format(
            current_cell,
            scan_elapsed_sec,
            float(config.scan_cell_budget_sec),
            ",".join(DIR_NAME[d] for d in order) or "none",
            ",".join(DIR_NAME[d] for d in reused_directions) or "none",
            gimbal_scan_retries,
            budget_overrun,
        ), flush=True,
    )
    return ranges, open_dirs



def _heading_error(target: float, actual: float) -> float:
    return normalize_angle_deg(float(target) - float(actual))


def _heading_snapshot(phase: str, pose: PoseTracker, tracker: GimbalTracker,
                      reference: float) -> Optional[float]:
    """Compare chassis attitude with relative/ground gimbal yaw across phases."""
    actual = pose.get_yaw()
    age = pose.attitude_age_sec() if hasattr(pose, "attitude_age_sec") else None
    rel, ground = tracker.get_yaws()
    error = None if actual is None else _heading_error(reference, actual)
    proxy = None if rel is None or ground is None else normalize_angle_deg(ground - rel)
    def fmt(v):
        return "---" if v is None else "{:+.2f}".format(float(v))
    print(
        "[HEADING_TRACE] phase={} chassis={} ref={} diff={} "
        "yaw_age_sec={} gimbal_relative={} gimbal_ground={} "
        "ground_minus_relative={}".format(
            phase, fmt(actual), fmt(reference), fmt(error), fmt(age),
            fmt(rel), fmt(ground), fmt(proxy)
        ), flush=True,
    )
    return actual


def _align_chassis_after_scan(chassis, pose: PoseTracker, config: Classwork8Config,
                              target: float, stop_event: Optional[threading.Event]
                              ) -> Tuple[bool, str]:
    """Single bounded, stationary heading correction before departing a cell."""
    stop_chassis(chassis)
    actual = pose.get_yaw()
    if actual is None:
        return False, "HEADING_FEEDBACK_LOST"
    initial_error = abs(_heading_error(target, actual))
    print("[HEADING_ALIGN] target={:+.2f} actual={:+.2f} diff={:+.2f}".format(
        target, actual, _heading_error(target, actual)), flush=True)
    if config.yaw_isolation_mode or not config.heading_hold_enabled:
        return True, "YAW_ISOLATION" if config.yaw_isolation_mode else "HEADING_HOLD_DISABLED"
    if initial_error <= float(config.heading_align_tolerance_deg):
        return True, "ALREADY_ALIGNED"
    if initial_error > float(config.heading_align_max_error_deg):
        return False, "HEADING_LARGE_DRIFT"
    started = time.monotonic()
    deadline = started + float(config.heading_align_timeout_sec)
    stable = 0
    try:
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False, "USER_STOP"
            actual = pose.get_yaw()
            if actual is None:
                return False, "HEADING_FEEDBACK_LOST"
            error = _heading_error(target, actual)
            if abs(error) <= float(config.heading_align_tolerance_deg):
                stop_chassis(chassis)
                stable += 1
                if stable >= 3:
                    print("[HEADING_ALIGN] OK final={:+.2f} error={:+.2f}".format(
                        actual, error), flush=True)
                    return True, "ALIGNED"
            else:
                stable = 0
                # A wrong z-to-attitude sign must not induce runaway rotation.
                if time.monotonic() - started > 0.55 and abs(error) > initial_error + 1.0:
                    print("[HEADING_FAIL] SIGN_MISMATCH: {:.2f} -> {:.2f}; "
                          "inspect heading_drive_sign.".format(
                        initial_error, abs(error)), flush=True)
                    return False, "HEADING_SIGN_MISMATCH"
                z = max(-float(config.heading_align_max_z_dps),
                        min(float(config.heading_align_max_z_dps),
                            error * float(config.heading_kp_z)
                            / float(config.heading_drive_sign)))
                chassis.drive_speed(x=0.0, y=0.0, z=z,
                                    timeout=config.drive_timeout_sec)
            time.sleep(0.05)
        print("[HEADING_FAIL] ALIGN_TIMEOUT actual={} target={}".format(
            pose.get_yaw(), target), flush=True)
        return False, "HEADING_ALIGN_TIMEOUT"
    finally:
        stop_chassis(chassis)


# A live moving yaw fault must abort before the existing 0.65-second
# divergence probe can send progressively larger turning commands.
# Field run 2026-09-28 reached 5.51 deg then 40.87 deg while moving.
V05_MOVING_YAW_ABORT_DEG = 8.0


def _moving_heading_over_limit(
    config: Classwork8Config,
    target_yaw_deg: float,
    actual_yaw_deg: Optional[float],
) -> bool:
    if (
        not config.heading_hold_enabled
        or config.yaw_isolation_mode
        or actual_yaw_deg is None
    ):
        return False
    return abs(_heading_error(target_yaw_deg, actual_yaw_deg)) > V05_MOVING_YAW_ABORT_DEG


def _moving_gimbal_aligned(
    config: Classwork8Config,
    tracker: GimbalTracker,
    direction: int,
) -> bool:
    """Check only the optional in-motion Gimbal diagnostic."""
    if not config.moving_gimbal_check_enabled:
        return True
    pitch, yaw = tracker.get_angles()
    age = tracker.angle_age_sec()
    return bool(
        pitch is not None
        and yaw is not None
        and age is not None
        and age <= float(config.moving_gimbal_feedback_max_age_sec)
        and math.isfinite(float(pitch))
        and math.isfinite(float(yaw))
        and abs(float(pitch) - float(config.gimbal_scan_pitch_deg))
        <= float(config.moving_gimbal_pitch_tolerance_deg)
        and abs(normalize_angle_deg(
            float(yaw) - float(config.gimbal_yaw_for_direction(direction))
        )) <= float(config.moving_gimbal_yaw_tolerance_deg)
    )


def _moving_feedback_state(
    config: Classwork8Config,
    sensors: ToFOnlySensorManager,
    tracker: GimbalTracker,
    direction: int,
) -> Tuple[Optional[str], Optional[float]]:
    """Return the live hold/stop reason before the next motor command."""
    stamp = getattr(sensors, "tof_last_update", None)
    if (
        stamp is None
        or time.monotonic() - float(stamp)
        > float(config.moving_gimbal_feedback_max_age_sec)
    ):
        return "MOVING_TOF_STALE", None
    distance = sensors.get_front_cm()
    if distance is None or not math.isfinite(float(distance)):
        return "MOVING_TOF_STALE", None
    # Hard stop is conservative and never bypassed by the diagnostic toggle
    # or its debounce, even if the Gimbal angle is currently questionable.
    if float(distance) <= float(config.stop_front_cm):
        return "MOVING_HARD_STOP", float(distance)
    if not _moving_gimbal_aligned(config, tracker, direction):
        return "MOVING_GIMBAL_UNALIGNED", float(distance)
    return None, float(distance)


def _recover_moving_feedback(
    config: Classwork8Config,
    sensors: ToFOnlySensorManager,
    tracker: GimbalTracker,
    direction: int,
    stop_event: Optional[threading.Event],
) -> Tuple[Optional[str], Optional[float]]:
    """While wheel-stopped, require consecutive distinct fresh samples."""
    deadline = time.monotonic() + float(config.moving_feedback_recovery_timeout_sec)
    stable = 0
    last_tof = getattr(sensors, "tof_last_update", None)
    last_angle = tracker.last_update_monotonic()
    new_tof = False
    new_angle = not config.moving_gimbal_check_enabled
    latest = None

    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return "USER_STOP", None
        reason, latest = _moving_feedback_state(
            config, sensors, tracker, direction
        )
        if reason == "MOVING_HARD_STOP":
            return reason, latest
        if reason is None:
            tof_stamp = getattr(sensors, "tof_last_update", None)
            angle_stamp = tracker.last_update_monotonic()
            if tof_stamp is not None and (
                last_tof is None or float(tof_stamp) > float(last_tof) + 1e-6
            ):
                last_tof = float(tof_stamp)
                new_tof = True
            if not config.moving_gimbal_check_enabled:
                new_angle = True
            elif angle_stamp is not None and (
                last_angle is None or float(angle_stamp) > float(last_angle) + 1e-6
            ):
                last_angle = float(angle_stamp)
                new_angle = True
            if new_tof and new_angle:
                stable += 1
                new_tof = False
                new_angle = not config.moving_gimbal_check_enabled
                if stable >= int(config.moving_feedback_recovery_samples):
                    return None, latest
        else:
            stable = 0
            new_tof = False
            new_angle = not config.moving_gimbal_check_enabled
        if not _sleep_interruptible(0.02, stop_event):
            return "USER_STOP", None
    return "MOVING_FEEDBACK_TIMEOUT", latest


def _maintain_wall_clearance_checkpoint(
    chassis, gimbal, pose: PoseTracker, sensors: ToFOnlySensorManager,
    tracker: GimbalTracker, config: Classwork8Config,
    ranges: Dict[int, Optional[float]], direction: int, raw_start_x: float,
    raw_start_y: float, raw_start_yaw: float,
    stop_event: Optional[threading.Event],
    *,
    verified_retreat_direction: Optional[int] = None,
) -> Tuple[bool, Optional[str], Optional[dict]]:
    """Adjust the CURRENT wall immediately, with no other Gimbal yaw aim.

    The only approved space behind a requested movement is either:
      1) the opposite range in this SAME unmoved scan, or
      2) a <= 8 cm retrace of the robot's just-traversed cell edge.
    Otherwise a single forward-facing ToF cannot rule out an obstacle
    behind the robot: skip motion, never blind-drive or scan an extra side.

    Preserve the current Gimbal yaw, continuously observe the current wall,
    cap each translation segment to config.wall_clearance_max_step_cm,
    and stop in verified wheel-zero mode between segments and at exit.
    Caller then holds the SAME angle for a fresh camera-sign observation.
    """
    _ = gimbal  # NEVER command Gimbal yaw inside clearance adjustment.
    if not config.wall_clearance_enabled or config.yaw_isolation_mode:
        return False, None, None
    direction = int(direction) % 4
    opposite = (direction + 2) % 4
    current_cm = ranges.get(direction)
    if current_cm is None or not math.isfinite(float(current_cm)):
        return False, None, None
    desired = clearance_target(config, direction)
    tol = float(config.wall_clearance_deadband_cm)
    if (float(current_cm) >= desired - tol
            or float(current_cm) >= float(config.tof_open_cm)):
        return False, None, None
    if stop_event is not None and stop_event.is_set():
        return False, "USER_STOP", None

    opposite_cm = ranges.get(opposite)
    known_opposite = (
        opposite_cm is not None
        and math.isfinite(float(opposite_cm))
        and float(opposite_cm) >= float(config.mapping_min_cm)
    )
    retrace_verified = (
        verified_retreat_direction is not None
        and opposite == int(verified_retreat_direction) % 4
    )
    unsafe_operator_move = bool(
        config.unsafe_disable_motion_guards
        and not known_opposite
        and not retrace_verified
    )
    if not known_opposite and not retrace_verified and not unsafe_operator_move:
        print(
            "[CLEARANCE_UNVERIFIED] {}={:.1f}cm target={:.1f}cm, no "
            "opposite-wall measurement or just-traversed retreat route; "
            "skip unsafe blind movement; keep this Gimbal direction for "
            "camera survey.".format(DIR_NAME[direction], current_cm, desired),
            flush=True,
        )
        return False, None, None

    yaw = pose.get_yaw()
    age = pose.attitude_age_sec()
    observed_pitch, observed_yaw = tracker.get_angles()
    if (yaw is None or age is None or age > 0.3
            or abs(_heading_error(raw_start_yaw, yaw)) > 2.0
            or observed_pitch is None or observed_yaw is None
            or abs(float(observed_pitch) - config.gimbal_scan_pitch_deg)
            > config.gimbal_pitch_tolerance_deg
            or abs(_heading_error(
                config.gimbal_yaw_for_direction(direction), observed_yaw
            )) > config.gimbal_tolerance_deg):
        print("[CLEARANCE] SKIP: chassis/Gimbal feedback not aligned or fresh.",
              flush=True)
        return False, None, None

    stop_chassis(chassis)
    sensors.reset_filters()
    fresh = _wait_for_fresh_tof(sensors, 1.0, stop_event)
    if (fresh is None or not math.isfinite(float(fresh))
            or abs(float(fresh) - float(current_cm)) > 8.0
            or float(fresh) < float(config.mapping_min_cm)):
        print("[CLEARANCE] SKIP: current-side ToF not fresh/consistent.", flush=True)
        return False, None, None
    deficit_cm = desired - float(fresh)
    if deficit_cm <= tol:
        return False, None, None
    step_cm = float(config.wall_clearance_max_step_cm)
    # At most TWO individually stopped bounded segments on this side.
    # With a measured opposite range, leave its configured minimum plus
    # tolerance and 1 cm uncertainty margin. With recent traversed-edge
    # evidence, retrace no more than 2 * step_cm, not an unknown cell.
    budget_cm = (min(
        float(opposite_cm) - clearance_target(config, opposite) - tol - 1.0,
        2.0 * step_cm,
    ) if known_opposite else 2.0 * step_cm)
    limit_cm = min(deficit_cm, 2.0 * step_cm, budget_cm)
    if limit_cm <= tol:
        print(
            "[CLEARANCE_NARROW_PAIR] {}={:.1f}cm {}={:.1f}cm; "
            "cannot satisfy both configured sensor ranges, so no unsafe "
            "one-sided shift is made.".format(
                DIR_NAME[direction], fresh,
                DIR_NAME[opposite], float(opposite_cm),
            ) if known_opposite else
            "[CLEARANCE] SKIP: opposing wall/route leaves no safe room.",
            flush=True,
        )
        return False, None, None

    xy = pose.get_xy()
    if xy[0] is None or xy[1] is None:
        return False, "CLEARANCE_ODOMETRY_MISSING", None
    initial_map = _map_xy_from_raw(
        float(xy[0]), float(xy[1]), raw_start_x, raw_start_y,
        raw_start_yaw, config.odom_scale_x, config.odom_scale_y,
    )
    unit_map = DIR_VEC_MAP[opposite]
    unit_body = DIR_VEC_DRIVE[opposite]
    speed = float(config.wall_clearance_speed_mps)
    started = time.monotonic()
    deadline = started + limit_cm / 100.0 / speed + 1.5
    segment_origin = 0.0
    last_progress = 0.0
    best_live = float(fresh)
    last_tof_stamp = sensors.tof_last_update
    wrong_range_samples = 0
    sent_motion = False
    latest_cm = float(fresh)

    def outcome(result: str, failure: Optional[str] = None):
        return sent_motion, failure, {
            "started_at": started,
            "finished_at": time.monotonic(),
            "before_cm": round(float(fresh), 3),
            "after_cm": round(float(latest_cm), 3),
            "shifted_m": round(float(last_progress), 4),
            "limit_m": round(float(limit_cm) / 100.0, 4),
            "result": result,
        }
    print(
        "[CLEARANCE_NOW] observed={} range={:.1f}cm target={:.1f}cm "
        "move={} limit={:.1f}cm verified_by={} speed={:.3f} z=0".format(
            DIR_NAME[direction], fresh, desired, DIR_NAME[opposite],
            limit_cm, "SAME_SCAN_OPPOSITE" if known_opposite
            else ("JUST_TRAVERSED_ROUTE" if retrace_verified
                  else "UNSAFE_OPERATOR_SUPERVISED"), speed,
        ), flush=True,
    )
    try:
        while time.monotonic() <= deadline:
            if stop_event is not None and stop_event.is_set():
                return outcome("USER_STOP", "USER_STOP")
            yaw = pose.get_yaw()
            yaw_age = pose.attitude_age_sec()
            pitch, camera_yaw = tracker.get_angles()
            if (yaw is None or yaw_age is None or yaw_age > 0.3
                    or abs(_heading_error(raw_start_yaw, yaw)) > 2.0):
                return outcome(
                    "CLEARANCE_HEADING_GUARD", "CLEARANCE_HEADING_GUARD"
                )
            if (pitch is None or camera_yaw is None
                    or abs(float(pitch) - config.gimbal_scan_pitch_deg)
                    > config.gimbal_pitch_tolerance_deg
                    or abs(_heading_error(
                        config.gimbal_yaw_for_direction(direction), camera_yaw
                    )) > config.gimbal_tolerance_deg):
                return outcome(
                    "CLEARANCE_GIMBAL_MOVED", "CLEARANCE_GIMBAL_MOVED"
                )
            stamp = sensors.tof_last_update
            now = time.monotonic()
            if stamp is None or now - stamp > 0.35:
                return outcome("CLEARANCE_TOF_STALE", "CLEARANCE_TOF_STALE")
            live = sensors.get_front_cm()
            if live is None or not math.isfinite(float(live)):
                return outcome("CLEARANCE_TOF_STALE", "CLEARANCE_TOF_STALE")
            latest_cm = float(live)
            xy = pose.get_xy()
            if xy[0] is None or xy[1] is None:
                return outcome(
                    "CLEARANCE_ODOMETRY_LOST", "CLEARANCE_ODOMETRY_LOST"
                )
            map_xy = _map_xy_from_raw(
                float(xy[0]), float(xy[1]), raw_start_x, raw_start_y,
                raw_start_yaw, config.odom_scale_x, config.odom_scale_y,
            )
            progress = (
                (map_xy[0] - initial_map[0]) * unit_map[0]
                + (map_xy[1] - initial_map[1]) * unit_map[1]
            )
            if progress < -0.005 or progress > limit_cm / 100.0 + 0.012:
                return outcome(
                    "CLEARANCE_ODOMETRY_DIRECTION",
                    "CLEARANCE_ODOMETRY_DIRECTION",
                )
            last_progress = max(0.0, progress)
            # A single ToF frame can jump on foam edges or angled walls.
            # Only stop for range-direction disagreement after three NEW
            # consecutive samples; odometry remains the movement reference.
            if (last_tof_stamp is None
                    or float(stamp) > float(last_tof_stamp) + 1e-6):
                if float(live) < best_live - 2.0:
                    wrong_range_samples += 1
                else:
                    wrong_range_samples = 0
                    best_live = max(best_live, float(live))
                last_tof_stamp = float(stamp)
                if wrong_range_samples >= 3:
                    return outcome(
                        "CLEARANCE_RANGE_DIRECTION",
                        "CLEARANCE_RANGE_DIRECTION",
                    )
            if float(live) >= desired - tol:
                print(
                    "[CLEARANCE] STOP {} live={:.1f}cm shifted={:.3f}m "
                    "target={:.1f}cm".format(
                        DIR_NAME[direction], live, progress, desired,
                    ), flush=True,
                )
                return outcome("TARGET_REACHED")
            if progress >= limit_cm / 100.0:
                print(
                    "[CLEARANCE] STOP {} live={:.1f}cm shifted={:.3f}m "
                    "limit reached before target={:.1f}cm".format(
                        DIR_NAME[direction], live, progress, desired,
                    ), flush=True,
                )
                return outcome("LIMIT_REACHED")
            # The second segment is not a second scan: stop all wheels
            # and re-evaluate the current ToF/yaw, without any Gimbal aim.
            if progress - segment_origin >= step_cm / 100.0:
                stop_chassis(chassis)
                segment_origin = progress
                print(
                    "[CLEARANCE_SEGMENT] {} {:.3f}m / {:.3f}m; same-angle "
                    "live ToF {:.1f}cm".format(
                        DIR_NAME[direction], progress, limit_cm / 100.0, live
                    ), flush=True,
                )
                if not _sleep_interruptible(0.06, stop_event):
                    return outcome("USER_STOP", "USER_STOP")
                continue
            sent_motion = True
            chassis.drive_speed(
                x=unit_body[0] * speed, y=unit_body[1] * speed,
                z=0.0, timeout=config.drive_timeout_sec,
            )
            if not _sleep_interruptible(0.04, stop_event):
                return outcome("USER_STOP", "USER_STOP")
        return outcome(
            "CLEARANCE_MOTION_TIMEOUT", "CLEARANCE_MOTION_TIMEOUT"
        )
    finally:
        stop_chassis(chassis)


def _return_to_scan_origin(
    chassis,
    pose: PoseTracker,
    config: Classwork8Config,
    origin_map: Tuple[Optional[float], Optional[float]],
    raw_start_x: float,
    raw_start_y: float,
    raw_start_yaw: float,
    stop_event: Optional[threading.Event],
) -> Tuple[bool, str, float]:
    """Retrace a bounded clearance shift before scanning or driving onward."""
    if origin_map[0] is None or origin_map[1] is None:
        return False, "CENTER_ORIGIN_MISSING", float("inf")
    speed = float(config.wall_clearance_speed_mps)
    tolerance_m = 0.010
    max_return_m = 2.0 * float(config.wall_clearance_max_step_cm) / 100.0 + 0.02
    started = time.monotonic()
    deadline = started + max_return_m / max(0.01, speed) + 1.5
    residual = float("inf")
    try:
        while time.monotonic() <= deadline:
            if stop_event is not None and stop_event.is_set():
                return False, "USER_STOP", residual
            current_x, current_y = _relative_xy(
                pose, raw_start_x, raw_start_y, raw_start_yaw, config
            )
            if current_x is None or current_y is None:
                return False, "CENTER_ODOMETRY_LOST", residual
            dx = float(origin_map[0]) - float(current_x)
            dy = float(origin_map[1]) - float(current_y)
            residual = math.hypot(dx, dy)
            if residual <= tolerance_m:
                return True, "CENTER_RESTORED", residual
            if residual > max_return_m:
                return False, "CENTER_RETURN_OUT_OF_RANGE", residual
            command_speed = min(speed, max(0.015, residual * 1.5))
            chassis.drive_speed(
                x=dx / residual * command_speed,
                y=-dy / residual * command_speed,
                z=0.0,
                timeout=config.drive_timeout_sec,
            )
            if not _sleep_interruptible(0.04, stop_event):
                return False, "USER_STOP", residual
        return False, "CENTER_RETURN_TIMEOUT", residual
    finally:
        stop_chassis(chassis)


def _basic_motion_command(
    config: Classwork8Config,
    direction: int,
    target_yaw_deg: float,
    current_yaw_deg: Optional[float],
) -> Tuple[float, float, float, Optional[float]]:
    """Single longitudinal speed owner: no wall/ToF/cross-track speed inputs."""
    requested_speed = float(config.travel_speed_mps)
    ux, uy = DIR_VEC_DRIVE[int(direction) % 4]
    x_cmd = ux * requested_speed
    y_cmd = uy * requested_speed
    x_cmd, y_cmd, z_cmd, _mode, yaw_error = _fixed_heading_control_v02(
        config, target_yaw_deg, current_yaw_deg, x_cmd, y_cmd, "BASIC_MOVE"
    )
    if config.yaw_isolation_mode:
        z_cmd = 0.0  # Enforce at the sole translation-command producer.
    return x_cmd, y_cmd, z_cmd, yaw_error


def _drive_one_cell(
    chassis,
    gimbal,
    pose: PoseTracker,
    heading: HeadingManager,
    sensors: ToFOnlySensorManager,
    gimbal_tracker: GimbalTracker,
    vision: Optional[CorridorVision],
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    direction: int,
    current_cell: Tuple[int, int],
    target_cell: Tuple[int, int],
    scan_ranges: Optional[Dict[int, Optional[float]]],
    wall_sides: Set[int],
    moves: int,
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
) -> Tuple[bool, str, float]:
    """Stable V1 cell move with preflight, feedback hold and ToF braking."""
    direction %= 4
    guards_disabled = bool(config.unsafe_disable_motion_guards)
    _ = (heading, vision, scan_ranges, wall_sides)  # Legacy call compatibility.
    if guards_disabled:
        print(
            "[UNSAFE_MOTION] DIAGNOSTIC GUARDS OFF: no preflight, feedback "
            "hold, yaw abort or cross-track abort. Gradual ToF brake, "
            "three-sample hard-stop arrival ({:.1f}cm after 50% progress), "
            "odometry endpoint and USER_STOP remain active.".format(
                config.stop_front_cm
            ),
            flush=True,
        )
        # Keep the sensor facing the travel direction for useful logs/mapping,
        # but never veto translation if aiming or feedback fails.
        stop_chassis(chassis)
        if not _point_gimbal(
            gimbal,
            sensors,
            gimbal_tracker,
            direction,
            config,
            stop_event,
            _allow_endpoint_retry=False,
        ):
            print(
                "[UNSAFE_MOTION] Gimbal aim failed; continuing anyway.",
                flush=True,
            )
    elif not config.moving_gimbal_check_enabled:
        print(
            "[MOVE_GIMBAL_CHECK] OFF (diagnostic only): initial aim, fresh "
            "ToF, hard stop, heading guard and wheel-stop ACK remain enabled.",
            flush=True,
        )
    preflight_samples: List[float] = [float("inf")] if guards_disabled else []
    preflight_reason = "" if guards_disabled else "GIMBAL_UNAVAILABLE"
    sample_count = max(2, min(3, int(config.front_block_confirm_samples)))
    # One retry covers a transient aim or feedback gap. No chassis command is
    # sent until the second preflight also has fresh Gimbal and ToF feedback.
    for preflight_attempt in range(2):
        if guards_disabled:
            break
        stop_chassis(chassis)
        if not _point_gimbal(
            gimbal,
            sensors,
            gimbal_tracker,
            direction,
            config,
            stop_event,
            _allow_endpoint_retry=False,
        ):
            preflight_reason = "GIMBAL_UNAVAILABLE"
        else:
            aimed_age = gimbal_tracker.angle_age_sec()
            if (
                aimed_age is None
                or aimed_age
                > float(config.moving_gimbal_feedback_max_age_sec)
            ):
                preflight_reason = "PREFLIGHT_GIMBAL_STALE"
            else:
                preflight_samples = _collect_fresh_tof_samples(
                    sensors,
                    sample_count,
                    config.tof_recovery_wait_sec,
                    stop_event,
                )
                if len(preflight_samples) == sample_count:
                    preflight_reason = ""
                    break
                preflight_reason = "PREFLIGHT_TOF_STALE"

        if stop_event is not None and stop_event.is_set():
            return False, "USER_STOP", 0.0
        if preflight_attempt == 0:
            recorder.event(
                time.monotonic(),
                "MOVE_PREFLIGHT_RETRY",
                preflight_reason,
                logical_node=current_cell,
                intended_node=target_cell,
                direction=DIR_NAME[direction],
            )
            print(
                "[MOVE_PREFLIGHT_RETRY] {} direction={}; re-aiming once.".format(
                    preflight_reason, DIR_NAME[direction]
                ),
                flush=True,
            )

    if preflight_reason:
        stop_chassis(chassis)
        print(
            "[MOVE_PREFLIGHT] {} after one retry; no motor command".format(
                preflight_reason
            ),
            flush=True,
        )
        return False, preflight_reason, 0.0

    x0, y0 = pose.get_xy()
    if x0 is None or y0 is None:
        stop_chassis(chassis)
        return False, "ODOMETRY_UNAVAILABLE", 0.0
    start_map_x, start_map_y = _map_xy_from_raw(
        float(x0), float(y0), start_x, start_y, start_yaw_deg,
        config.odom_scale_x, config.odom_scale_y,
    )
    target_map_x = float(target_cell[0]) * config.cell_size_m
    target_map_y = float(target_cell[1]) * config.cell_size_m
    if direction == 0:
        initial_remaining = target_map_x - start_map_x
    elif direction == 1:
        initial_remaining = start_map_y - target_map_y
    elif direction == 2:
        initial_remaining = start_map_x - target_map_x
    else:
        initial_remaining = target_map_y - start_map_y

    # Use a median of distinct callbacks. One short transient cannot exclude
    # this edge, and one cached sample can never be counted three times.
    initial_front = float(statistics.median(preflight_samples))
    required_cm = preflight_required_cm(
        initial_remaining,
        config.step_tolerance_m,
        config.stop_front_cm,
        config.movement_preflight_margin_cm,
    )
    if not guards_disabled and not preflight_has_clearance(initial_front, required_cm):
        stop_chassis(chassis)
        print(
            "[MOVE_PREFLIGHT] BLOCKED_CONFIRMED direction={} median={:.1f}cm "
            "samples={} required={:.1f}cm; no motor command".format(
                DIR_NAME[direction], initial_front,
                [round(value, 1) for value in preflight_samples], required_cm
            ),
            flush=True,
        )
        recorder.event(
            time.monotonic(), "MOVE_PREFLIGHT_BLOCKED",
            "three fresh travel-direction ToF callbacks confirmed insufficient room",
            logical_node=current_cell, intended_node=target_cell,
            direction=DIR_NAME[direction], tof_cm=initial_front,
            tof_samples_cm=[round(value, 2) for value in preflight_samples],
            required_cm=round(required_cm, 2),
        )
        return False, "PREFLIGHT_BLOCKED", 0.0
    if not guards_disabled:
        print(
            "[MOVE_PREFLIGHT] PASS direction={} median={:.1f}cm samples={} "
            "required={:.1f}cm".format(
                DIR_NAME[direction], initial_front,
                [round(value, 1) for value in preflight_samples], required_cm
            ),
            flush=True,
        )
    max_abs_cross_track_m = 0.0
    max_abs_heading_error_deg = 0.0
    command_logged = False
    heading_probe = None
    last_heading_log = 0.0
    bad_gimbal_samples = 0
    moving_reaim_used = False
    tof_brake_active = False
    endpoint_brake_active = False
    last_tof_brake_log = 0.0
    last_endpoint_brake_log = 0.0
    hard_stop_confirm_count = 0
    hard_stop_last_stamp = None

    # No auto-reverse/backtrack: a hard stop mid-cell ends this move safely.
    while True:
        if stop_event is not None and stop_event.is_set():
            stop_chassis(chassis)
            return False, "USER_STOP", 0.0

        raw_x, raw_y = pose.get_xy()
        yaw = pose.get_yaw()
        front_cm = sensors.get_front_cm()
        tof_stamp = getattr(sensors, "tof_last_update", None)
        if (
            guards_disabled
            and front_cm is not None
            and float(front_cm) <= float(config.stop_front_cm)
        ):
            if tof_stamp is not None and tof_stamp != hard_stop_last_stamp:
                hard_stop_confirm_count += 1
                hard_stop_last_stamp = tof_stamp
        else:
            hard_stop_confirm_count = 0
            hard_stop_last_stamp = tof_stamp
        if raw_x is None or raw_y is None:
            stop_chassis(chassis)
            return False, "ODOMETRY_LOST", 0.0
        if not guards_disabled and config.heading_hold_enabled and yaw is None:
            stop_chassis(chassis)
            return False, "HEADING_FEEDBACK_LOST", 0.0
        # An old but non-None attitude value is not feedback. Never steer
        # against a frozen yaw sample or draw conclusions from a stale probe.
        yaw_age = pose.attitude_age_sec() if hasattr(pose, "attitude_age_sec") else None
        if not guards_disabled and (config.heading_hold_enabled or config.yaw_isolation_mode) and (
            yaw_age is None or yaw_age > 1.5
        ):
            stop_chassis(chassis)
            print("[HEADING_FAIL] STALE_ATTITUDE age_sec={}".format(yaw_age), flush=True)
            return False, "HEADING_FEEDBACK_STALE", 0.0

        rel_x, rel_y = _map_xy_from_raw(
            float(raw_x), float(raw_y), start_x, start_y, start_yaw_deg,
            config.odom_scale_x, config.odom_scale_y,
        )
        moved = math.hypot(rel_x - start_map_x, rel_y - start_map_y)
        if direction == 0:
            remaining = target_map_x - rel_x
            cross_track = rel_y - target_map_y
        elif direction == 1:
            remaining = rel_y - target_map_y
            cross_track = rel_x - target_map_x
        elif direction == 2:
            remaining = rel_x - target_map_x
            cross_track = rel_y - target_map_y
        else:
            remaining = target_map_y - rel_y
            cross_track = rel_x - target_map_x
        max_abs_cross_track_m = max(max_abs_cross_track_m, abs(cross_track))
        if yaw is not None:
            max_abs_heading_error_deg = max(
                max_abs_heading_error_deg,
                abs(normalize_angle_deg(float(start_yaw_deg) - float(yaw))),
            )
        # Abort a large yaw departure before producing another nonzero
        # chassis command. The original delayed divergence probe did not
        # fire until the field run had already rotated >40 degrees.
        if _moving_heading_over_limit(config, start_yaw_deg, yaw):
            if not guards_disabled:
                current_error = _heading_error(start_yaw_deg, yaw)
                stop_chassis(chassis)
                print(
                    "[HEADING_FAIL] MOVING_YAW_LIMIT reference={:+.2f} "
                    "actual={:+.2f} error={:+.2f} limit={:.1f}; "
                    "four-wheel zero stop acknowledged".format(
                        float(start_yaw_deg), float(yaw), float(current_error),
                        V05_MOVING_YAW_ABORT_DEG,
                    ), flush=True,
                )
                return False, "MOVING_YAW_LIMIT", moved
        # Validate observation geometry for mapping ONLY. Incorrect gimbal
        # pitch/yaw must not corrupt SLAM, but cannot alter chassis speed.
        sensor_pitch = gimbal_tracker.get_pitch()
        sensor_yaw = gimbal_tracker.get_yaw()
        if (
            sensor_pitch is not None
            and sensor_yaw is not None
            and abs(
                float(sensor_pitch) - float(config.gimbal_scan_pitch_deg)
            ) <= float(config.gimbal_pitch_tolerance_deg)
            and abs(normalize_angle_deg(
                float(sensor_yaw) - float(config.gimbal_yaw_for_direction(direction))
            )) <= float(config.gimbal_tolerance_deg)
        ):
            _update_tof_ray(grid, config, rel_x, rel_y, direction, front_cm)

        # Normal completion uses odometry in both axes. Aggressive mode also
        # accepts three fresh hard-stop readings after half-cell progress.
        unsafe_arrival = (
            guards_disabled and remaining <= float(config.step_tolerance_m)
        )
        if cell_pose_within_tolerance(
            remaining,
            cross_track,
            config.step_tolerance_m,
            config.cell_center_tolerance_m,
        ) or unsafe_arrival:
            stop_chassis(chassis)
            recorder.record_sample(
                time.monotonic(), rel_x, rel_y, yaw, direction, front_cm,
                None, None, None, None, "CELL_COMPLETE",
            )
            recorder.event(
                time.monotonic(), "CELL_MOTION_QUALITY",
                "odometry arrival verified in longitudinal and lateral axes",
                logical_node=target_cell,
                peak_cross_track_m=round(max_abs_cross_track_m, 4),
                peak_heading_error_deg=round(max_abs_heading_error_deg, 3),
                midcell_side_checked=False,
            )
            publish_state(
                status="Reached cell {}".format(target_cell),
                logical_cell=target_cell, gimbal_direction=direction,
                tof_cm=front_cm, moves=moves + 1, force=True,
            )
            print(
                "[MOVE] Reached {} progress={:.3f}m cross_track={:+.3f}m".format(
                    target_cell, config.cell_size_m - remaining, cross_track
                ), flush=True,
            )
            return True, "CELL_COMPLETE", moved

        # Assignment maze has no mid-cell obstacles. A close travel-direction
        # wall is therefore the far wall of the commanded destination cell.
        # Keep this cue active even in operator-supervised unsafe mode.
        unsafe_hard_stop_arrival = (
            guards_disabled
            and unsafe_hard_stop_is_arrival(
                front_cm,
                config.stop_front_cm,
                hard_stop_confirm_count,
                3,
                moved,
                config.cell_size_m,
                0.50,
            )
        )
        normal_wall_arrival = wall_arrival_reached(
            front_cm,
            config.movement_wall_arrival_cm,
            moved,
            config.cell_size_m,
            config.movement_wall_arrival_min_progress_ratio,
            cross_track,
            config.cell_center_tolerance_m,
        )
        if normal_wall_arrival or unsafe_hard_stop_arrival:
            arrival_reason = (
                "CELL_COMPLETE_UNSAFE_HARD_STOP"
                if unsafe_hard_stop_arrival
                else "CELL_COMPLETE_WALL_ARRIVAL"
            )
            arrival_ratio = (
                0.50 if unsafe_hard_stop_arrival
                else float(config.movement_wall_arrival_min_progress_ratio)
            )
            stop_chassis(chassis)
            recorder.record_sample(
                time.monotonic(), rel_x, rel_y, yaw, direction, front_cm,
                None, None, None, None, arrival_reason,
            )
            recorder.event(
                time.monotonic(),
                arrival_reason,
                (
                    "three fresh hard-stop samples after 50% cell progress"
                    if unsafe_hard_stop_arrival else
                    "travel-direction ToF reached the configured destination-wall range"
                ),
                logical_node=target_cell,
                direction=DIR_NAME[direction],
                tof_cm=front_cm,
                threshold_cm=float(
                    config.stop_front_cm if unsafe_hard_stop_arrival
                    else config.movement_wall_arrival_cm
                ),
                minimum_progress_ratio=arrival_ratio,
                confirmed_samples=hard_stop_confirm_count,
                maximum_cross_track_m=(
                    None if unsafe_hard_stop_arrival
                    else float(config.cell_center_tolerance_m)
                ),
                progress_m=round(moved, 4),
                remaining_m=round(remaining, 4),
                cross_track_m=round(cross_track, 4),
            )
            publish_state(
                status="Reached cell {} at {:.1f} cm wall".format(
                    target_cell, float(front_cm)
                ),
                logical_cell=target_cell,
                gimbal_direction=direction,
                tof_cm=front_cm,
                moves=moves + 1,
                force=True,
                reason=arrival_reason,
            )
            print(
                "[{}] Reached {} ToF={:.1f}cm threshold={:.1f}cm "
                "progress={:.3f}m samples={}; cell committed".format(
                    (
                        "UNSAFE_HARD_STOP_ARRIVAL"
                        if unsafe_hard_stop_arrival else "WALL_ARRIVAL_BRAKE"
                    ),
                    target_cell,
                    float(front_cm),
                    float(
                        config.stop_front_cm if unsafe_hard_stop_arrival
                        else config.movement_wall_arrival_cm
                    ),
                    moved,
                    hard_stop_confirm_count,
                ),
                flush=True,
            )
            return True, arrival_reason, moved

        if not guards_disabled and (remaining < -float(config.step_tolerance_m) or (
            remaining <= float(config.step_tolerance_m)
            and abs(cross_track) > float(config.cell_center_tolerance_m)
        )):
            stop_chassis(chassis)
            print(
                "[CELL_POSE_ERROR] target={} remaining={:+.3f}m "
                "cross_track={:+.3f}m; logical cell NOT committed".format(
                    target_cell, remaining, cross_track
                ),
                flush=True,
            )
            recorder.event(
                time.monotonic(), "CELL_POSE_OUT_OF_TOLERANCE",
                "odometry passed longitudinal target or missed lateral center",
                logical_node=current_cell, intended_node=target_cell,
                remaining_m=round(remaining, 4),
                cross_track_m=round(cross_track, 4),
            )
            return False, "CELL_POSE_OUT_OF_TOLERANCE", moved

        if guards_disabled:
            safety_reason, observed_cm = None, front_cm
        else:
            safety_reason, observed_cm = _moving_feedback_state(
                config, sensors, gimbal_tracker, direction
            )
        if safety_reason == "MOVING_GIMBAL_UNALIGNED":
            bad_gimbal_samples += 1
            if bad_gimbal_samples < int(config.moving_gimbal_bad_samples):
                safety_reason = None
        elif safety_reason is None:
            bad_gimbal_samples = 0

        if safety_reason in ("MOVING_GIMBAL_UNALIGNED", "MOVING_TOF_STALE"):
            stop_chassis(chassis)
            print(
                "[MOVE_FEEDBACK_HOLD] reason={} progress={:.3f}m; "
                "waiting for {} consecutive fresh samples".format(
                    safety_reason, moved,
                    config.moving_feedback_recovery_samples,
                ),
                flush=True,
            )
            recorder.event(
                time.monotonic(), "MOVE_FEEDBACK_HOLD", safety_reason,
                logical_node=current_cell, intended_node=target_cell,
                progress_m=round(moved, 4),
            )
            publish_state(
                status="Paused {}: waiting for fresh feedback".format(
                    DIR_NAME[direction]
                ),
                logical_cell=current_cell, gimbal_direction=direction,
                tof_cm=observed_cm, moves=moves, force=True,
            )
            safety_reason, observed_cm = _recover_moving_feedback(
                config, sensors, gimbal_tracker, direction, stop_event
            )
            if (
                safety_reason == "MOVING_FEEDBACK_TIMEOUT"
                and not moving_reaim_used
            ):
                moving_reaim_used = True
                recorder.event(
                    time.monotonic(),
                    "MOVE_FEEDBACK_REAIM",
                    "bounded re-aim and fresh preflight after feedback timeout",
                    logical_node=current_cell,
                    intended_node=target_cell,
                    progress_m=round(moved, 4),
                )
                print(
                    "[MOVE_FEEDBACK_REAIM] direction={} progress={:.3f}m; "
                    "one bounded re-aim + preflight.".format(
                        DIR_NAME[direction], moved
                    ),
                    flush=True,
                )
                if stop_event is not None and stop_event.is_set():
                    safety_reason = "USER_STOP"
                elif not _point_gimbal(
                    gimbal,
                    sensors,
                    gimbal_tracker,
                    direction,
                    config,
                    stop_event,
                    _allow_endpoint_retry=False,
                ):
                    safety_reason = "MOVING_REAIM_FAILED"
                else:
                    retry_samples = _collect_fresh_tof_samples(
                        sensors,
                        sample_count,
                        config.tof_recovery_wait_sec,
                        stop_event,
                    )
                    if len(retry_samples) != sample_count:
                        safety_reason = "MOVING_REPREFLIGHT_TOF_STALE"
                    else:
                        retry_front = float(statistics.median(retry_samples))
                        retry_required_cm = preflight_required_cm(
                            max(0.0, remaining),
                            config.step_tolerance_m,
                            config.stop_front_cm,
                            config.movement_preflight_margin_cm,
                        )
                        if preflight_has_clearance(
                            retry_front, retry_required_cm
                        ):
                            safety_reason = None
                            observed_cm = retry_front
                            recorder.event(
                                time.monotonic(),
                                "MOVE_REPREFLIGHT_PASS",
                                "fresh feedback and remaining clearance restored",
                                logical_node=current_cell,
                                intended_node=target_cell,
                                tof_cm=retry_front,
                                required_cm=round(retry_required_cm, 2),
                                tof_samples_cm=[
                                    round(value, 2)
                                    for value in retry_samples
                                ],
                            )
                        else:
                            safety_reason = "MOVING_REPREFLIGHT_BLOCKED"
                            observed_cm = retry_front
            if safety_reason is None:
                bad_gimbal_samples = 0
                print(
                    "[MOVE_FEEDBACK_RESUMED] direction={} ToF={:.1f}cm; "
                    "continuing the same cell from current pose".format(
                        DIR_NAME[direction], float(observed_cm)
                    ),
                    flush=True,
                )
                recorder.event(
                    time.monotonic(), "MOVE_FEEDBACK_RESUMED",
                    "fresh feedback restored while wheel-stopped",
                    logical_node=current_cell, intended_node=target_cell,
                    progress_m=round(moved, 4), tof_cm=observed_cm,
                )
                continue

        if safety_reason is not None:
            stop_chassis(chassis)
            if (
                safety_reason == "MOVING_HARD_STOP"
                and hard_stop_near_target_is_arrival(
                    moved,
                    config.cell_size_m,
                    cross_track,
                    config.blocked_near_target_accept_ratio,
                    config.cell_center_tolerance_m,
                )
            ):
                recorder.event(
                    time.monotonic(),
                    "CELL_COMPLETE_NEAR_WALL",
                    "hard stop after bounded near-target odometry progress",
                    logical_node=target_cell,
                    direction=DIR_NAME[direction],
                    tof_cm=observed_cm,
                    progress_m=round(moved, 4),
                    remaining_m=round(remaining, 4),
                    cross_track_m=round(cross_track, 4),
                )
                publish_state(
                    status="Reached cell {} at hard-stop boundary".format(
                        target_cell
                    ),
                    logical_cell=target_cell,
                    gimbal_direction=direction,
                    tof_cm=observed_cm,
                    moves=moves + 1,
                    force=True,
                    reason="CELL_COMPLETE_NEAR_WALL",
                )
                print(
                    "[MOVE] Reached {} at hard-stop boundary progress={:.3f}m "
                    "cross_track={:+.3f}m".format(
                        target_cell, moved, cross_track
                    ),
                    flush=True,
                )
                return True, "CELL_COMPLETE_NEAR_WALL", moved
            print(
                "[MOVE_SAFETY] {} direction={} ToF={}cm progress={:.3f}m "
                "remaining={:.3f}m; logical cell NOT committed".format(
                    safety_reason, DIR_NAME[direction],
                    "---" if observed_cm is None else "{:.1f}".format(observed_cm),
                    moved, remaining,
                ),
                flush=True,
            )
            recorder.record_sample(
                time.monotonic(), rel_x, rel_y, yaw, direction, observed_cm,
                None, None, None, None, safety_reason,
            )
            recorder.event(
                time.monotonic(), "MOVE_SAFETY_STOP", safety_reason,
                logical_node=current_cell, intended_node=target_cell,
                direction=DIR_NAME[direction], tof_cm=observed_cm,
                progress_m=round(moved, 4), remaining_m=round(remaining, 4),
            )
            publish_state(
                status="ERROR during {}: {}".format(
                    DIR_NAME[direction], safety_reason
                ),
                logical_cell=current_cell, gimbal_direction=direction,
                tof_cm=observed_cm, moves=moves, force=True,
                reason=safety_reason,
            )
            return False, safety_reason, moved

        front_cm = observed_cm
        tof_brake_speed = tof_braking_speed_mps(
            front_cm,
            config.travel_speed_mps,
            config.slow_front_cm,
            config.stop_front_cm,
            config.movement_brake_min_speed_mps,
        )
        endpoint_brake_speed = odometry_endpoint_speed_mps(
            remaining,
            config.step_tolerance_m,
            config.travel_speed_mps,
            config.movement_endpoint_brake_distance_m,
            config.movement_brake_min_speed_mps,
        )
        command_speed = min(tof_brake_speed, endpoint_brake_speed)

        x_cmd, y_cmd, z_cmd, _yaw_error = _basic_motion_command(
            config, direction, start_yaw_deg, yaw
        )
        speed_factor = command_speed / float(config.travel_speed_mps)
        x_cmd *= speed_factor
        y_cmd *= speed_factor
        now = time.monotonic()
        if tof_brake_speed < float(config.travel_speed_mps) - 1e-6:
            if not tof_brake_active or now - last_tof_brake_log >= 0.35:
                print(
                    "[TOF_BRAKE] direction={} live={}cm command={:.3f}m/s "
                    "cruise={:.3f}m/s remaining={:.3f}m".format(
                        DIR_NAME[direction],
                        "---" if front_cm is None else "{:.1f}".format(front_cm),
                        command_speed,
                        config.travel_speed_mps, remaining,
                    ),
                    flush=True,
                )
                last_tof_brake_log = now
            tof_brake_active = True
        else:
            tof_brake_active = False
        if endpoint_brake_speed < float(config.travel_speed_mps) - 1e-6:
            if (
                not endpoint_brake_active
                or now - last_endpoint_brake_log >= 0.35
            ):
                print(
                    "[ENDPOINT_BRAKE] direction={} remaining={:.3f}m "
                    "command={:.3f}m/s cruise={:.3f}m/s".format(
                        DIR_NAME[direction], remaining, command_speed,
                        config.travel_speed_mps,
                    ),
                    flush=True,
                )
                last_endpoint_brake_log = now
            endpoint_brake_active = True
        else:
            endpoint_brake_active = False
        if _yaw_error is not None:
            if now - last_heading_log >= 0.5:
                print("[HEADING_MOVE] yaw={:+.2f} reference={:+.2f} "
                      "diff={:+.2f} z={:+.2f} cross_track={:+.3f}".format(
                    float(yaw), float(start_yaw_deg), float(_yaw_error),
                    z_cmd, cross_track), flush=True)
                last_heading_log = now
            if abs(z_cmd) >= 2.0 and abs(_yaw_error) >= 1.5:
                if heading_probe is None:
                    heading_probe = (now, abs(_yaw_error))
                elif now - heading_probe[0] >= 0.65:
                    if not guards_disabled and abs(_yaw_error) >= heading_probe[1] + 2.0:
                        stop_chassis(chassis)
                        print("[HEADING_FAIL] DIVERGED {:.2f} -> {:.2f}; "
                              "verify heading_drive_sign.".format(
                            heading_probe[1], abs(_yaw_error)), flush=True)
                        return False, "HEADING_CORRECTION_DIVERGED", moved
                    heading_probe = (now, abs(_yaw_error))
            else:
                heading_probe = None
        if not command_logged:
            ux, uy = DIR_VEC_DRIVE[direction]
            print(
                "[MOTION] cruise={:.3f} current={:.3f} direction={} "
                "x={:+.3f} y={:+.3f} yaw_correction={:+.2f}".format(
                    float(config.travel_speed_mps), x_cmd * ux + y_cmd * uy,
                    DIR_NAME[direction], x_cmd, y_cmd, z_cmd,
                ), flush=True,
            )
            command_logged = True

        chassis.drive_speed(
            x=x_cmd, y=y_cmd, z=z_cmd, timeout=config.drive_timeout_sec,
        )
        recorder.record_sample(
            time.monotonic(), rel_x, rel_y, yaw, direction, front_cm,
            None, None, None, None, "STABLE_MOVE_{}".format(DIR_NAME[direction]),
        )
        publish_state(
            status="Moving {} to {}".format(DIR_NAME[direction], target_cell),
            logical_cell=current_cell, gimbal_direction=direction,
            tof_cm=front_cm, moves=moves, force=False,
        )
        time.sleep(config.loop_delay_sec)

def _canonical_edge(
    cell: Tuple[int, int],
    direction: int,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Return an undirected canonical key for one logical grid edge."""
    other = _neighbor(cell, direction)
    a = (int(cell[0]), int(cell[1]))
    b = (int(other[0]), int(other[1]))
    return (a, b) if a <= b else (b, a)


def _preference_order(last_direction: int) -> List[int]:
    last_direction %= 4
    return [
        last_direction,
        (last_direction - 1) % 4,
        (last_direction + 1) % 4,
        (last_direction + 2) % 4,
    ]


def _preference_rank(direction: int, last_direction: int) -> int:
    order = _preference_order(last_direction)
    try:
        return order.index(int(direction) % 4)
    except ValueError:
        return 99


def _visited_open_neighbors(
    cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> List[Tuple[int, Tuple[int, int]]]:
    result: List[Tuple[int, Tuple[int, int]]] = []
    for direction in range(4):
        if edge_states.get((cell[0], cell[1], direction)) != "OPEN":
            continue
        if (cell, direction) in blocked_edges:
            continue
        nxt = _neighbor(cell, direction)
        if nxt not in visited:
            continue
        if not _inside_working_canvas(nxt, config):
            continue
        result.append((direction, nxt))
    return result


def _frontier_options(
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> List[Tuple[Tuple[int, int], int, Tuple[int, int]]]:
    """All known OPEN edges from a visited cell into an unvisited cell."""
    options: List[Tuple[Tuple[int, int], int, Tuple[int, int]]] = []
    for cell in sorted(visited):
        for direction in range(4):
            if edge_states.get((cell[0], cell[1], direction)) != "OPEN":
                continue
            if (cell, direction) in blocked_edges:
                continue
            nxt = _neighbor(cell, direction)
            if nxt in visited:
                continue
            if not _inside_working_canvas(nxt, config):
                continue
            options.append((cell, direction, nxt))
    return options


def _shortest_open_path(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> Optional[List[Tuple[int, int]]]:
    """BFS shortest path through already-visited, confirmed-open cells."""
    if start == goal:
        return [start]

    queue_nodes: List[Tuple[int, int]] = [start]
    head = 0
    previous: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {
        start: None
    }

    while head < len(queue_nodes):
        cell = queue_nodes[head]
        head += 1

        for _direction, nxt in _visited_open_neighbors(
            cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        ):
            if nxt in previous:
                continue
            previous[nxt] = cell
            if nxt == goal:
                path = [goal]
                cursor = goal
                while previous[cursor] is not None:
                    cursor = previous[cursor]  # type: ignore[index]
                    path.append(cursor)
                path.reverse()
                return path
            queue_nodes.append(nxt)

    return None


def _frontier_information_gain(
    frontier_cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> int:
    gain = 0
    for direction in range(4):
        if edge_states.get((frontier_cell[0], frontier_cell[1], direction)) != "OPEN":
            continue
        if (frontier_cell, direction) in blocked_edges:
            continue
        nxt = _neighbor(frontier_cell, direction)
        if nxt not in visited and _inside_working_canvas(nxt, config):
            gain += 1
    return gain


def _plan_frontier_move(
    current_cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
    last_move_direction: int,
) -> Optional[dict]:
    """Plan one step using nearest-frontier BFS.

    Priority:
    1) If the current cell has a known-open unvisited neighbour, expand now.
    2) Otherwise BFS through confirmed-open visited cells to the nearest cell
       that still has an open edge into unknown territory.
    3) Ties prefer more information gain, then motion continuity, then the
       frontier farther from the start so equal-cost choices expand outward.
    """
    frontiers = _frontier_options(
        visited,
        edge_states,
        blocked_edges,
        config,
    )
    if not frontiers:
        return None

    preference = _preference_order(last_move_direction)

    local_options = [
        option for option in frontiers if option[0] == current_cell
    ]
    if local_options:
        local_options.sort(
            key=lambda option: (
                _preference_rank(option[1], last_move_direction),
                -(
                    abs(option[2][0])
                    + abs(option[2][1])
                ),
            )
        )
        frontier_cell, direction, target = local_options[0]
        return {
            "move_direction": direction,
            "next_cell": target,
            "is_new": True,
            "frontier_cell": frontier_cell,
            "frontier_target": target,
            "route": [current_cell, target],
            "frontier_count": len(frontiers),
            "mode": "EXPAND_LOCAL_FRONTIER",
        }

    candidates = []
    for frontier_cell, frontier_direction, frontier_target in frontiers:
        path = _shortest_open_path(
            current_cell,
            frontier_cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        )
        if not path or len(path) < 2:
            continue

        next_cell = path[1]
        move_direction = _direction_to(current_cell, next_cell)
        if move_direction is None:
            continue

        gain = _frontier_information_gain(
            frontier_cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        )
        path_steps = len(path) - 1
        continuity_rank = _preference_rank(
            move_direction,
            last_move_direction,
        )
        outward = abs(frontier_target[0]) + abs(frontier_target[1])

        score = (
            path_steps,
            -gain,
            continuity_rank,
            -outward,
            frontier_cell[0],
            frontier_cell[1],
            frontier_direction,
        )
        candidates.append(
            (
                score,
                {
                    "move_direction": move_direction,
                    "next_cell": next_cell,
                    "is_new": False,
                    "frontier_cell": frontier_cell,
                    "frontier_target": frontier_target,
                    "route": path + [frontier_target],
                    "frontier_count": len(frontiers),
                    "mode": "RELOCATE_TO_FRONTIER",
                },
            )
        )

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _plan_unknown_rescan_move(
    current_cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
    last_move_direction: int,
    exhausted_cells: Optional[Set[Tuple[int, int]]] = None,
    requested_cells: Optional[Set[Tuple[int, int]]] = None,
) -> Optional[dict]:
    """Route to the nearest requested or topology-incomplete visited cell."""
    exhausted_cells = exhausted_cells or set()
    choices = []
    for cell in sorted(visited):
        topology_complete = all(
            edge_states.get((cell[0], cell[1], direction))
            in ("OPEN", "WALL") for direction in range(4)
        )
        needs_rescan = (
            cell in requested_cells
            if requested_cells is not None else not topology_complete
        )
        if cell == current_cell or cell in exhausted_cells or not needs_rescan:
            continue
        route = _shortest_open_path(
            current_cell,
            cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        )
        if route is not None and len(route) >= 2:
            choices.append((len(route), cell, route))
    if not choices:
        return None
    _length, rescan_cell, route = min(choices)
    next_cell = route[1]
    move_direction = _direction_to(current_cell, next_cell)
    if move_direction is None:
        return None
    return {
        "mode": "RELOCATE_RESCAN",
        "move_direction": move_direction,
        "next_cell": next_cell,
        "is_new": False,
        "frontier_count": 0,
        "frontier_cell": rescan_cell,
        "frontier_target": rescan_cell,
        "route": route,
        "preference_rank": _preference_rank(
            move_direction, last_move_direction
        ),
    }


def _closed_maze_completion_v04(
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> dict:
    """Finish only after the configured assignment grid is fully visited.

    The assignment declares a 6x6 arena, so completion is based on 36 distinct
    logical cells forming that exact size.  Perimeter-wall ratios remain useful
    diagnostics, but a missed reflection on a low outer wall must not keep a
    fully explored run alive until the time limit.
    """
    result = {
        "enabled": bool(config.closed_maze_auto_stop),
        "complete": False,
        "filled": False,
        "rows": 0,
        "cols": 0,
        "bbox": None,
        "ratios": {},
        "threshold": float(config.closed_maze_perimeter_wall_ratio),
        "required_rows": int(config.assignment_maze_rows),
        "required_cols": int(config.assignment_maze_cols),
    }

    if not config.closed_maze_auto_stop or not visited:
        return result

    xs = [int(cell[0]) for cell in visited]
    ys = [int(cell[1]) for cell in visited]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    rows = max_x - min_x + 1
    cols = max_y - min_y + 1

    result["rows"] = rows
    result["cols"] = cols
    result["bbox"] = (min_x, max_x, min_y, max_y)

    if (
        rows != int(config.assignment_maze_rows)
        or cols != int(config.assignment_maze_cols)
        or len(visited) != int(config.assignment_maze_rows)
        * int(config.assignment_maze_cols)
    ):
        return result

    expected = {
        (x, y)
        for x in range(min_x, max_x + 1)
        for y in range(min_y, max_y + 1)
    }

    filled = expected.issubset(visited)
    result["filled"] = bool(filled)

    if not filled:
        return result

    sides = {
        "FRONT": [((max_x, y), 0) for y in range(min_y, max_y + 1)],
        "BACK": [((min_x, y), 2) for y in range(min_y, max_y + 1)],
        "RIGHT": [((x, min_y), 1) for x in range(min_x, max_x + 1)],
        "LEFT": [((x, max_y), 3) for x in range(min_x, max_x + 1)],
    }

    ratios: Dict[str, float] = {}

    for name, checks in sides.items():
        confirmed = 0

        for cell, direction in checks:
            state = edge_states.get((cell[0], cell[1], direction))
            # A preflight exclusion is a route decision, not wall evidence.
            if state == "WALL":
                confirmed += 1

        ratios[name] = confirmed / float(max(1, len(checks)))

    result["ratios"] = ratios

    result["complete"] = True

    return result

def run(
    config: Optional[Classwork8Config] = None,
    publish: Optional[Callable[[dict], None]] = None,
    stop_event: Optional[threading.Event] = None,
    ep_robot=None,
    survey_bridge: Optional[LiveSurveyBridge] = None,
) -> Path:
    config = config or Classwork8Config()
    survey_bridge = survey_bridge or LiveSurveyBridge(config)
    config.validate()
    target_mission = TargetMission(config)
    target_auto_aim = TargetAutoAim(config)
    stop_event = stop_event or threading.Event()

    grid = OccupancyGrid(
        config.map_width_m,
        config.map_height_m,
        config.resolution_m,
        free_delta=config.free_delta,
        occupied_delta=config.occupied_delta,
        min_score=config.min_score,
        max_score=config.max_score,
        free_threshold=config.free_threshold,
        occupied_threshold=config.occupied_threshold,
    )
    recorder = RunRecorder(config)
    recorder.start(time.monotonic())

    robot_was_preconnected = ep_robot is not None
    if ep_robot is None:
        ep_robot = robot.Robot()
    chassis = None
    gimbal = None
    blaster_module = None
    tof_sensor = None
    tof_subscribed = False
    pose_subscribed = False
    attitude_subscribed = False
    gimbal_subscribed = False

    start_pose = (0.0, 0.0, 0.0)
    finish_reason = "UNKNOWN"
    current_cell = (0, 0)
    moves = 0
    current_gimbal_direction = 0
    current_tof = None
    last_publish = [0.0]
    mission_started_at: Optional[float] = None
    mission_warning_sent = False
    mission_soft_deadline_sent = False

    planner_mode = "FRONTIER_BFS"
    planner_frontier_count = 0
    planner_frontier_cell: Optional[Tuple[int, int]] = None
    planner_frontier_target: Optional[Tuple[int, int]] = None
    planner_route: List[Tuple[int, int]] = []

    completion_status = {
        "enabled": bool(config.closed_maze_auto_stop),
        "complete": False,
        "filled": False,
        "rows": 0,
        "cols": 0,
        "bbox": None,
        "ratios": {},
        "threshold": float(config.closed_maze_perimeter_wall_ratio),
    }

    # Unknown-world topological map used by the realtime GUI.
    known_cells: Set[Tuple[int, int]] = {(0, 0)}
    edge_states: Dict[Tuple[int, int, int], str] = {}
    logical_path: List[Tuple[int, int]] = [(0, 0)]

    pose = V05PoseTracker()
    sensors = ToFOnlySensorManager()
    gimbal_tracker = GimbalTracker()

    # V05 uses one shared camera stream for target survey. Corridor steering is
    # intentionally disabled in this ToF+camera baseline; movement remains the
    # proven V04 ToF + odometry controller.
    vision: Optional[CorridorVision] = None
    camera_service: Optional[CameraService] = None
    target_detector: Optional[TargetDetector] = (
        TargetDetector(config) if config.target_detection_enabled else None
    )
    target_registry = TargetRegistry(config)
    target_debug_holder: List[object] = [None]
    start_scan_ranges: Optional[Dict[int, Optional[float]]] = None

    # Keep topology containers alive even if initialization fails so cleanup
    # can still export a useful partial run.
    visited: Set[Tuple[int, int]] = {current_cell}
    blocked_edges: Set[Tuple[Tuple[int, int], int]] = set()
    traversed_edges: Set[
        Tuple[Tuple[int, int], Tuple[int, int]]
    ] = set()
    last_move_direction = 0
    # A cached scan may be reused only for a FULLY scanned visited cell.
    # Never reuse readings as physical side-distance corrections after moving:
    # the cached wall topology is for planning; movement always samples fresh
    # travel-direction ToF before and throughout every cell.
    scanned_cells: Set[Tuple[int, int]] = set()
    unknown_scan_attempts: Dict[Tuple[int, int], int] = {}
    pending_wall_surveys: Set[Tuple[Tuple[int, int], int]] = set()
    wall_survey_failures: Dict[Tuple[Tuple[int, int], int], int] = {}

    raw_start_x = 0.0
    raw_start_y = 0.0
    raw_start_yaw = 0.0

    def publish_state(
        *,
        status: str,
        logical_cell: Tuple[int, int],
        gimbal_direction: int,
        tof_cm: Optional[float],
        moves: int,
        force: bool = False,
        reason: str = "",
        finished: bool = False,
        run_dir: Optional[str] = None,
    ) -> None:
        nonlocal current_gimbal_direction, current_tof

        current_gimbal_direction = int(gimbal_direction) % 4
        current_tof = tof_cm

        if publish is None:
            return

        now = time.monotonic()
        min_period = max(0.05, config.gui_refresh_ms / 1000.0)
        if not force and now - last_publish[0] < min_period:
            return
        last_publish[0] = now

        rel_x, rel_y = _relative_xy(
            pose, raw_start_x, raw_start_y, raw_start_yaw, config
        )

        camera_active = bool(
            camera_service is not None and camera_service.running
        )
        publish({
            "status": status,
            "reason": reason,
            "finished": bool(finished),
            # The runtime logical GUI does not read the 160x160 matrix.
            # Avoid allocating it at every pose refresh; export still writes
            # the full occupancy grid from the mapper itself.
            "resolution_m": grid.resolution_m,
            "cell_size_m": config.cell_size_m,
            "trajectory": recorder.trajectory_xy(),
            "robot_xy": None if rel_x is None or rel_y is None else (rel_x, rel_y),
            "logical_cell": logical_cell,
            "known_cells": sorted(known_cells),
            "wall_edges": sorted(
                key for key, state in edge_states.items() if state == "WALL"
            ),
            "open_edges": sorted(
                key for key, state in edge_states.items() if state == "OPEN"
            ),
            "logical_path": list(logical_path),
            "planner_mode": planner_mode,
            "planner_frontier_count": int(planner_frontier_count),
            "planner_frontier_cell": planner_frontier_cell,
            "planner_frontier_target": planner_frontier_target,
            "planner_route": list(planner_route),
            "completion_enabled": bool(completion_status.get("enabled")),
            "completion_ready": bool(completion_status.get("complete")),
            "completion_filled": bool(completion_status.get("filled")),
            "completion_rows": int(completion_status.get("rows", 0)),
            "completion_cols": int(completion_status.get("cols", 0)),
            "completion_bbox": completion_status.get("bbox"),
            "completion_perimeter_ratios": dict(
                completion_status.get("ratios", {})
            ),
            "completion_threshold": float(
                completion_status.get(
                    "threshold",
                    config.closed_maze_perimeter_wall_ratio,
                )
            ),
            "run_dir": run_dir,
            "gimbal_direction": current_gimbal_direction,
            "gimbal_direction_name": DIR_NAME[current_gimbal_direction],
            "gimbal_yaw_deg": gimbal_tracker.get_yaw(),
            "gimbal_pitch_deg": gimbal_tracker.get_pitch(),
            "gimbal_scan_pitch_target_deg": float(config.gimbal_scan_pitch_deg),
            "gimbal_pitch_tolerance_deg": float(config.gimbal_pitch_tolerance_deg),
            "tof_cm": tof_cm,
            "moves": int(moves),
            "mission_elapsed_sec": (
                0.0
                if mission_started_at is None
                else max(0.0, time.monotonic() - mission_started_at)
            ),
            "mission_warning_sec": float(config.mission_warning_sec),
            "mission_soft_deadline_sec": float(
                config.mission_soft_deadline_sec
            ),
            "coverage": grid.coverage_percent(),
            "vision_active": camera_active,
            "vision_steering_enabled": False,
            "vision_error": None,
            "vision_confidence": 0.0,
            # The annotated image is polled directly from LiveSurveyBridge by
            # Tk at preview cadence, never copied into every map snapshot.
            "vision_frame": None,
            "target_detection_active": bool(
                camera_active and target_detector is not None
            ),
            "target_count": len(target_registry.targets),
            "target_sighting_count": sum(
                1 for item in target_registry.targets
                if item.get("localization_status") == "SIGHTING_ONLY"
            ),
            "target_position_candidate_count": sum(
                1 for item in target_registry.targets
                if item.get("localization_status") == "NEAR_WALL_ESTIMATE"
            ),
            "targets": target_registry.public_targets(),
        })

    try:
        publish_state(
            status="Connecting to RoboMaster...",
            logical_cell=current_cell,
            gimbal_direction=0,
            tof_cm=None,
            moves=0,
            force=True,
        )

        if not robot_was_preconnected:
            print("Connecting to RoboMaster (mode={})...".format(config.connection), flush=True)
            ok = ep_robot.initialize(conn_type=config.connection)
            print("RoboMaster initialize returned: {!r}".format(ok), flush=True)
            if not ok:
                raise RuntimeError(
                    "RoboMaster connection failed. Check that the PC is connected "
                    "to the RoboMaster Wi-Fi/AP network."
                )
        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal
        # Stationary test exposes an explicit manual-fire button even when
        # automatic target firing is OFF. Normal missions keep the blaster
        # unavailable unless selected/all firing was armed in configuration.
        blaster_module = (
            ep_robot.blaster
            if config.target_fire_enabled or config.stationary_target_test
            else None
        )
        tof_sensor = ep_robot.sensor

        # FREE mode decouples chassis yaw from gimbal yaw. The chassis therefore
        # never performs 90-degree scan turns.
        print("[INIT] Setting robot mode to FREE...", flush=True)
        mode_ok = ep_robot.set_robot_mode(mode=robot.FREE)
        print("[INIT] FREE mode result: {!r}".format(mode_ok), flush=True)
        if not mode_ok:
            raise RuntimeError(
                "FREE_MODE_FAILED: refusing scan; chassis could be coupled to gimbal"
            )
        stop_chassis(chassis)  # Explicitly enter the measured-stable zero-wheel mode.

        if config.stationary_target_test and blaster_module is not None:
            def manual_fire_callback():
                stop_chassis(chassis)
                acknowledged = blaster_module.fire(
                    fire_type=config.target_fire_type,
                    times=config.target_fire_times,
                ) is True
                recorder.event(
                    time.monotonic(),
                    "MANUAL_FIRE",
                    "acknowledged" if acknowledged else "command failed",
                    fire_type=config.target_fire_type,
                    fire_times=config.target_fire_times,
                )
                print(
                    "[MANUAL_FIRE] type={} times={} ack={}".format(
                        config.target_fire_type,
                        config.target_fire_times,
                        acknowledged,
                    ),
                    flush=True,
                )
                return acknowledged

            survey_bridge.configure_manual_fire(manual_fire_callback)

        # Subscribe BEFORE recentering so we can verify the actual gimbal angle
        # even if the DJI action-completion packet is delayed/lost.
        print("[INIT] Subscribing ToF...", flush=True)
        tof_subscribed = bool(
            tof_sensor.sub_distance(
                freq=20,
                callback=sensors.tof_callback,
            )
        )
        print("[INIT] ToF subscription: {!r}".format(tof_subscribed), flush=True)

        print("[INIT] Subscribing odometry...", flush=True)
        pose_subscribed = bool(
            chassis.sub_position(
                cs=1,
                freq=20,
                callback=pose.position_callback,
            )
        )
        print("[INIT] Position subscription: {!r}".format(pose_subscribed), flush=True)

        print("[INIT] Subscribing attitude...", flush=True)
        attitude_subscribed = bool(
            chassis.sub_attitude(
                freq=20,
                callback=pose.attitude_callback,
            )
        )
        print("[INIT] Attitude subscription: {!r}".format(attitude_subscribed), flush=True)

        print("[INIT] Subscribing gimbal angle...", flush=True)
        gimbal_subscribed = bool(
            gimbal.sub_angle(
                freq=20,
                callback=gimbal_tracker.callback,
            )
        )
        print("[INIT] Gimbal subscription: {!r}".format(gimbal_subscribed), flush=True)

        print("[INIT] Waiting for initial position/yaw...", flush=True)
        raw_start_x, raw_start_y = 0.0, 0.0
        for attempt in range(3):
            raw_start_x, raw_start_y = wait_for_position(pose)
            if pose.has_position():
                break
            print(
                "[INIT_RETRY] odometry unavailable ({}/3).".format(attempt + 1),
                flush=True,
            )
        raw_start_yaw = None
        for attempt in range(3):
            raw_start_yaw = wait_for_yaw(pose)
            if raw_start_yaw is not None:
                break
            print(
                "[INIT_RETRY] attitude unavailable ({}/3).".format(attempt + 1),
                flush=True,
            )
        print(
            "[INIT] Pose ready: x={:+.3f} y={:+.3f} yaw={}".format(
                float(raw_start_x),
                float(raw_start_y),
                "---" if raw_start_yaw is None else "{:+.1f}".format(float(raw_start_yaw)),
            ),
            flush=True,
        )

        if not pose.has_position():
            raise RuntimeError("odometry subscription did not produce data")
        if raw_start_yaw is None:
            raise RuntimeError("attitude/yaw subscription did not produce data")

        print(
            "[INIT] Local map frame locked to mission-start yaw {:+.1f} deg.".format(
                float(raw_start_yaw)
            ),
            flush=True,
        )

        # Do not use gimbal.recenter().wait_for_completed() here. On this
        # RoboMaster the mechanical action can complete while its action-complete
        # packet is not received, which previously caused an indefinite wait.
        # Use the same closed-loop angle feedback as normal scanning instead.
        print("[INIT] Pointing gimbal to FRONT (0 deg) with angle feedback...", flush=True)
        if not _point_gimbal(
            gimbal,
            sensors,
            gimbal_tracker,
            0,
            config,
            stop_event,
        ):
            measured_yaw = gimbal_tracker.get_yaw()
            raise RuntimeError(
                "could not point gimbal to FRONT; measured yaw={}".format(
                    "---" if measured_yaw is None else "{:+.1f}".format(float(measured_yaw))
                )
            )
        print(
            "[INIT] Gimbal FRONT ready at yaw={:+.1f} deg.".format(
                float(gimbal_tracker.get_yaw())
            ),
            flush=True,
        )

        if config.yaw_isolation_mode:
            config.heading_hold_enabled = False
            print(
                "[YAW_ISOLATION] ACTIVE: every move uses chassis z=0; "
                "post-scan yaw correction is disabled.", flush=True,
            )
            stop_chassis(chassis)
            for count in range(6):
                _heading_snapshot(
                    "STATIONARY_{}".format(count), pose, gimbal_tracker,
                    float(raw_start_yaw),
                )
                if not _sleep_interruptible(0.5, stop_event):
                    break
        heading = HeadingManager()
        if not heading.initialize(raw_start_yaw):
            raise RuntimeError("yaw/attitude unavailable")

        # One shared camera stream. Round 1 uses it for target surveying only;
        # navigation remains ToF + odometry in this baseline.
        camera_ok = False
        if config.target_detection_enabled:
            camera_service = CameraService(
                ep_robot,
                resolution=config.target_camera_resolution,
                start_timeout_sec=config.target_camera_start_timeout_sec,
            )
            camera_ok = camera_service.start()
            if camera_ok:
                survey_bridge.attach_camera(camera_service)
            else:
                survey_bridge.set_status("Camera stream could not start")

        print(
            "[CAMERA] Round-1 target survey: {}".format(
                "ACTIVE" if camera_ok else "DISABLED / UNAVAILABLE"
            ),
            flush=True,
        )

        print("============================================================")
        print(" FINAL ROUND 1 V05 - ToF + CAMERA / MAP + TARGET SURVEY")
        print("============================================================")
        print("Cell size     : {:.2f} m".format(config.cell_size_m))
        print("Move per step : {:.2f} m (1 full cell)".format(config.exploration_step_m))
        print("Chassis yaw   : fixed; NO chassis scan turns")
        print("Scanning      : gimbal-only, 4 directions")
        print("Travel        : mecanum forward/right/back/left")
        print("Planner       : nearest-frontier BFS; no DFS parent-stack backtracking")
        print("Completion    : frontier exhaustion + closed-rectangle wall validation")
        print("Safety        : ToF points along travel direction continuously")
        print(
            "Camera        : {}".format(
                "target detection ACTIVE; steering remains ToF + odometry"
                if camera_service is not None and camera_service.running
                else "target detection unavailable"
            )
        )
        print("============================================================")

        mission_started_at = time.monotonic()
        recorder.event(
            mission_started_at,
            "START",
            "V05 Round 1 ToF + camera map and target survey",
            logical_node=current_cell,
        )

        publish_state(
            status="Ready - scanning start cell",
            logical_cell=current_cell,
            gimbal_direction=0,
            tof_cm=sensors.get_front_cm(),
            moves=moves,
            force=True,
        )

        if config.stationary_auto_lock_test:
            stop_chassis(chassis)
            selected_keys = {item.key for item in target_mission.selected}
            if not selected_keys:
                print(
                    "[AUTO_LOCK] Select at least one color/shape checkbox in "
                    "Mission Settings.",
                    flush=True,
                )
                finish_reason = "AUTO_LOCK_NO_TARGET_SELECTED"
            elif camera_service is None or not camera_service.running:
                finish_reason = "AUTO_LOCK_CAMERA_UNAVAILABLE"
            else:
                # ToF must be sampled while level; the camera/ToF share the
                # Gimbal and the later look-down pose is not a valid range ray.
                sensors.reset_filters()
                distance_cm = _sample_tof(sensors, config, stop_event)
                if distance_cm is None:
                    finish_reason = "AUTO_LOCK_TOF_UNAVAILABLE"
                elif not _set_camera_observation_pitch(
                    gimbal,
                    gimbal_tracker,
                    config,
                    config.target_camera_pitch_deg,
                    stop_event,
                ):
                    finish_reason = "AUTO_LOCK_CAMERA_PITCH_FAILED"
                else:
                    chosen = None
                    target_debug = None
                    for attempt in range(3):
                        verified, target_debug = _verify_targets_or_empty(
                            target_detector,
                            camera_service,
                            time.monotonic(),
                            recorder,
                            current_cell,
                            0,
                        )
                        matches = [
                            item for item in verified
                            if "{}:{}".format(
                                item.detection.color,
                                item.detection.shape,
                            ).lower() in selected_keys
                        ]
                        if matches:
                            chosen = max(
                                matches, key=lambda item: float(item.confidence)
                            )
                            break
                        print(
                            "[AUTO_LOCK] Selected target not verified "
                            "({}/3); keep it visible in FRONT camera.".format(
                                attempt + 1
                            ),
                            flush=True,
                        )

                    if chosen is None:
                        finish_reason = "AUTO_LOCK_TARGET_NOT_FOUND"
                    else:
                        frame_size = (
                            (640, 360)
                            if target_debug is None
                            else (
                                int(target_debug.shape[1]),
                                int(target_debug.shape[0]),
                            )
                        )
                        extra_y = float(config.target_aim_offset_y_ratio)
                        parallax_y = vertical_parallax_aim_offset_ratio(
                            config.target_camera_above_blaster_m,
                            max(0.10, float(distance_cm) / 100.0),
                            config.target_camera_horizontal_fov_deg,
                            frame_size,
                        )
                        config.target_aim_offset_y_ratio = max(
                            -0.25, min(0.25, extra_y + parallax_y)
                        )
                        print(
                            "[AUTO_LOCK] target={}:{} ToF={:.1f}cm "
                            "camera_above={:.1f}cm parallax_y={:+.3f} "
                            "extra_y={:+.3f} final_y={:+.3f}.".format(
                                chosen.detection.color,
                                chosen.detection.shape,
                                float(distance_cm),
                                float(config.target_camera_above_blaster_m) * 100.0,
                                parallax_y,
                                extra_y,
                                config.target_aim_offset_y_ratio,
                            ),
                            flush=True,
                        )
                        survey_bridge.set_status(
                            "Auto-Lock active; parallax-adjusted WATER reticle"
                        )
                        aim_result = target_auto_aim.aim(
                            gimbal=gimbal,
                            tracker=gimbal_tracker,
                            camera_service=camera_service,
                            detector=target_detector,
                            initial_detection=chosen.detection,
                            stop_event=stop_event,
                        )
                        if aim_result.debug_frame is not None:
                            target_debug_holder[0] = aim_result.debug_frame
                        print(
                            "[AUTO_LOCK] {} fresh={} pitch={} yaw={}.".format(
                                aim_result.reason,
                                aim_result.fresh_frames,
                                "---" if aim_result.final_pitch_deg is None
                                else "{:+.1f}".format(aim_result.final_pitch_deg),
                                "---" if aim_result.final_yaw_deg is None
                                else "{:+.1f}".format(aim_result.final_yaw_deg),
                            ),
                            flush=True,
                        )
                        if not aim_result.success:
                            finish_reason = "AUTO_LOCK_{}".format(
                                aim_result.reason
                            )
                        else:
                            survey_bridge.set_manual_fire_ready(
                                True,
                                "LOCKED: WATER manual fire ready; press STOP & SAVE to finish",
                            )
                            publish_state(
                                status="AUTO-LOCKED - WATER manual fire ready",
                                logical_cell=current_cell,
                                gimbal_direction=0,
                                tof_cm=distance_cm,
                                moves=0,
                                force=True,
                            )
                            print(
                                "[AUTO_LOCK] LOCKED. Use MANUAL FIRE; "
                                "press STOP & SAVE to finish.",
                                flush=True,
                            )
                            while not stop_event.wait(0.10):
                                pass
                            survey_bridge.set_manual_fire_ready(
                                False, "Auto-Lock session finished"
                            )
                            finish_reason = "STATIONARY_AUTO_LOCK_TEST_COMPLETE"

        while (
            not config.stationary_auto_lock_test
            and moves < config.max_moves
        ):
            if stop_event.is_set():
                finish_reason = "USER_STOP"
                break

            mission_elapsed_sec = time.monotonic() - mission_started_at
            mission_clock_state = _mission_clock_state(
                mission_elapsed_sec,
                config.mission_warning_sec,
                config.mission_soft_deadline_sec,
            )
            if (
                not mission_soft_deadline_sent
                and mission_clock_state == "SOFT_DEADLINE"
            ):
                mission_soft_deadline_sent = True
                recorder.event(
                    time.monotonic(),
                    "MISSION_SOFT_DEADLINE",
                    "urgency mode; clock does not stop exploration",
                    logical_node=current_cell,
                    elapsed_sec=round(mission_elapsed_sec, 3),
                    moves=moves,
                    visited_nodes=len(visited),
                )
                publish_state(
                    status="Urgency mode: continuing mission",
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                    reason="MISSION_SOFT_DEADLINE",
                )
                print(
                    "[MISSION_CLOCK] soft deadline {:.1f}s reached; "
                    "continuing until completion, operator stop, or a hard "
                    "safety fault.".format(float(config.mission_soft_deadline_sec)),
                    flush=True,
                )
            if (
                not mission_warning_sent
                and mission_clock_state == "WARNING"
            ):
                mission_warning_sent = True
                recorder.event(
                    time.monotonic(),
                    "MISSION_TIME_WARNING",
                    "seven-minute mission warning",
                    logical_node=current_cell,
                    elapsed_sec=round(mission_elapsed_sec, 3),
                    moves=moves,
                    visited_nodes=len(visited),
                )
                publish_state(
                    status="Mission time warning: {:.1f} minutes elapsed".format(
                        mission_elapsed_sec / 60.0
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                    reason="MISSION_TIME_WARNING",
                )
                print(
                    "[MISSION_CLOCK] warning at {:.1f}s; soft deadline "
                    "{:.1f}s.".format(
                        mission_elapsed_sec,
                        float(config.mission_soft_deadline_sec),
                    ),
                    flush=True,
                )

            _heading_snapshot("PRE_SCAN_{}".format(current_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            # Reuse only complete four-side topology. A direction left UNKNOWN
            # by a bounded scan failure is rechecked when this route returns.
            cache_valid = _should_reuse_scan(
                current_cell,
                scanned_cells,
                edge_states,
                config.skip_scanned_visited_cells,
                any(
                    cell == current_cell
                    for cell, _direction in pending_wall_surveys
                ),
            )

            if cache_valid:
                # Cache contains only confirmed topology, not a fresh ToF
                # distance. Never feed old side distances to centering.
                ranges = {}
                open_dirs = {
                    direction for direction in range(4)
                    if edge_states.get(
                        (current_cell[0], current_cell[1], direction)
                    ) == "OPEN"
                    and (current_cell, direction) not in blocked_edges
                }
                recorder.event(
                    time.monotonic(),
                    "SCAN_REUSED",
                    "Visited cell: confirmed topology reused; live ToF observation remains enabled",
                    logical_node=current_cell,
                    open_directions=sorted(open_dirs),
                )
                print(
                    "[SCAN_REUSED] {} already scanned; skip 4-way sweep. "
                    "Travel-direction ToF safety remains active.".format(current_cell),
                    flush=True,
                )
                publish_state(
                    status="Visited cell {}: reusing map; no repeat 4-way scan".format(
                        current_cell
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )
            else:
                # Enter the experimentally stable zero-wheel mode before each scan.
                stop_chassis(chassis)
                _heading_snapshot(
                    "SCAN_STOP_SENT_{}".format(current_cell),
                    pose, gimbal_tracker, float(raw_start_yaw),
                )
                scan = _scan_four_directions(
                    chassis,
                    gimbal,
                    pose,
                    sensors,
                    gimbal_tracker,
                    grid,
                    recorder,
                    config,
                    float(raw_start_x),
                    float(raw_start_y),
                    float(raw_start_yaw),
                    stop_event,
                    publish_state,
                    current_cell,
                    moves,
                    known_cells,
                    edge_states,
                    traversed_edges,
                    camera_service,
                    target_detector,
                    target_registry,
                    target_mission,
                    target_auto_aim,
                    blaster_module,
                    target_debug_holder,
                    survey_bridge,
                    pending_wall_surveys,
                    wall_survey_failures,
                    verified_retreat_direction=(
                        (int(last_move_direction) + 2) % 4
                        if moves > 0 else None
                    ),
                )
                if scan is None:
                    finish_reason = (
                        "USER_STOP"
                        if stop_event.is_set()
                        else "GIMBAL_SCAN_FAILED"
                    )
                    break

                ranges, open_dirs = scan

                _heading_snapshot("POST_SCAN_{}".format(current_cell),
                                  pose, gimbal_tracker, float(raw_start_yaw))
                scanned_cells.add(current_cell)
                unknown_directions = [
                    direction for direction in range(4)
                    if edge_states.get(
                        (current_cell[0], current_cell[1], direction)
                    ) not in ("OPEN", "WALL")
                ]
                if unknown_directions:
                    unknown_scan_attempts[current_cell] = (
                        unknown_scan_attempts.get(current_cell, 0) + 1
                    )
                    recorder.event(
                        time.monotonic(),
                        "SCAN_INCOMPLETE",
                        "topology remains UNKNOWN after bounded direction retry",
                        logical_node=current_cell,
                        unknown_directions=[
                            DIR_NAME[direction]
                            for direction in unknown_directions
                        ],
                        attempts=unknown_scan_attempts[current_cell],
                    )
                else:
                    unknown_scan_attempts.pop(current_cell, None)

                if start_scan_ranges is None and current_cell == (0, 0) and moves == 0:
                    start_scan_ranges = dict(ranges)

                recorder.event(
                    time.monotonic(),
                    "SCAN",
                    "gimbal-only four-direction ToF scan",
                    logical_node=current_cell,
                    open_directions=sorted(open_dirs),
                    ranges_cm={str(k): v for k, v in sorted(ranges.items())},
                )

            if config.stationary_target_test:
                stop_chassis(chassis)
                manual_fire_session = bool(
                    blaster_module is not None and publish is not None
                )
                if manual_fire_session:
                    survey_bridge.set_manual_fire_ready(
                        True,
                        "READY: reticle is the calibrated impact point; "
                        "manual shots bypass target gates",
                    )
                    print(
                        "[MANUAL_FIRE] READY. Chassis remains wheel-stopped; "
                        "press STOP & SAVE to finish.",
                        flush=True,
                    )
                    publish_state(
                        status="Stationary manual fire ready - press STOP & SAVE to finish",
                        logical_cell=current_cell,
                        gimbal_direction=current_gimbal_direction,
                        tof_cm=sensors.get_front_cm(),
                        moves=moves,
                        force=True,
                    )
                    while not stop_event.wait(0.10):
                        pass
                    survey_bridge.set_manual_fire_ready(
                        False, "Manual fire session finished"
                    )
                finish_reason = "STATIONARY_TARGET_TEST_COMPLETE"
                recorder.event(
                    time.monotonic(),
                    "FINISH",
                    "stationary target/auto-aim test completed without translation",
                    logical_node=current_cell,
                    moves=moves,
                )
                publish_state(
                    status="Stationary target/aim test complete",
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                    reason=finish_reason,
                )
                break

            # Time budget: never sweep the same logical cell twice.
            # Camera pitch requests are consumed but do NOT restart 4-way
            # scanning; the new pitch is applied at the next unvisited cell.
            if survey_bridge.consume_rescan():
                print(
                    "[SCAN_BUDGET] Rescan request deferred: current cell "
                    "already used its four directions.", flush=True,
                )
                recorder.event(
                    time.monotonic(), "SCAN_BUDGET",
                    "four scan directions already used; pitch change on next new cell",
                    logical_node=current_cell,
                )

            completion_status = _closed_maze_completion_v04(
                visited,
                edge_states,
                blocked_edges,
                config,
            )
            pending_wall_cells = {
                cell for cell, _direction in pending_wall_surveys
            }

            if completion_status["complete"] and not pending_wall_surveys:
                stop_chassis(chassis)

                ratios_text = ", ".join(
                    "{}={:.0f}%".format(name, value * 100.0)
                    for name, value in sorted(
                        completion_status["ratios"].items()
                    )
                )

                finish_reason = "CLOSED_MAZE_COMPLETE"

                recorder.event(
                    time.monotonic(),
                    "FINISH",
                    (
                        "closed rectangular maze fully visited; "
                        "perimeter wall ratios: {}"
                    ).format(ratios_text),
                    logical_node=current_cell,
                    visited_nodes=len(visited),
                    moves=moves,
                )

                publish_state(
                    status=(
                        "Closed maze complete: {}x{} cells, {}".format(
                            completion_status["rows"],
                            completion_status["cols"],
                            ratios_text,
                        )
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )

                print(
                    "[COMPLETE] Closed maze fully explored: "
                    "{}x{} cells | {}".format(
                        completion_status["rows"],
                        completion_status["cols"],
                        ratios_text,
                    ),
                    flush=True,
                )
                break

            plan = _plan_frontier_move(
                current_cell,
                visited,
                edge_states,
                blocked_edges,
                config,
                last_move_direction,
            )

            if plan is None:
                plan = _plan_unknown_rescan_move(
                    current_cell,
                    visited,
                    edge_states,
                    blocked_edges,
                    config,
                    last_move_direction,
                    {
                        cell for cell, attempts
                        in unknown_scan_attempts.items()
                        if attempts >= 2
                    },
                )
                if plan is not None:
                    recorder.event(
                        time.monotonic(),
                        "UNKNOWN_RESCAN_PLAN",
                        "routing to revisit incomplete four-direction scan",
                        logical_node=current_cell,
                        rescan_cell=plan["frontier_cell"],
                        route=plan["route"],
                    )

            if plan is None and current_cell in pending_wall_cells:
                recorder.event(
                    time.monotonic(),
                    "WALL_SURVEY_RETRY_LOCAL",
                    "retrying pending wall camera survey before departure",
                    logical_node=current_cell,
                )
                continue

            if plan is None and pending_wall_cells:
                plan = _plan_unknown_rescan_move(
                    current_cell,
                    visited,
                    edge_states,
                    blocked_edges,
                    config,
                    last_move_direction,
                    requested_cells=pending_wall_cells,
                )
                if plan is not None:
                    recorder.event(
                        time.monotonic(),
                        "WALL_SURVEY_RETRY_PLAN",
                        "routing to retry incomplete wall camera survey",
                        logical_node=current_cell,
                        rescan_cell=plan["frontier_cell"],
                        route=plan["route"],
                    )

            if (
                plan is None
                and completion_status["complete"]
                and pending_wall_surveys
            ):
                recorder.event(
                    time.monotonic(),
                    "WALL_SURVEY_UNREACHABLE",
                    "map complete; no confirmed-open route to pending wall survey",
                    logical_node=current_cell,
                    pending=[
                        {
                            "cell": list(cell),
                            "direction": DIR_NAME[direction],
                        }
                        for cell, direction in sorted(pending_wall_surveys)
                    ],
                )
                pending_wall_surveys.clear()
                continue

            if plan is None:
                planner_frontier_count = 0
                planner_frontier_cell = None
                planner_frontier_target = None
                planner_route = []

                excluded_frontiers = _frontier_options(
                    visited,
                    edge_states,
                    set(),
                    config,
                )
                if excluded_frontiers:
                    # Preflight exclusions are temporary sensor evidence, not
                    # permanent walls. Clear them and give the topology another
                    # chance instead of ending an otherwise recoverable run.
                    cleared_count = len(blocked_edges)
                    blocked_edges.clear()
                    recorder.event(
                        time.monotonic(), "PREFLIGHT_EXCLUSIONS_CLEARED",
                        "temporary edge vetoes cleared; retrying frontier plan",
                        logical_node=current_cell,
                        excluded_count=len(excluded_frontiers),
                        cleared_edges=cleared_count,
                    )
                    publish_state(
                        status="Retrying previously blocked frontier routes",
                        logical_cell=current_cell,
                        gimbal_direction=current_gimbal_direction,
                        tof_cm=sensors.get_front_cm(),
                        moves=moves,
                        force=True,
                        reason="PREFLIGHT_EXCLUSIONS_CLEARED",
                    )
                    continue

                finish_reason = "INCOMPLETE_NO_REACHABLE_FRONTIER"
                recorder.event(
                    time.monotonic(),
                    "INCOMPLETE",
                    "no reachable frontier before exact 6x6 completion",
                    visited_nodes=len(visited),
                    required_nodes=(
                        int(config.assignment_maze_rows)
                        * int(config.assignment_maze_cols)
                    ),
                    moves=moves,
                )
                publish_state(
                    status="Incomplete: no route before all 36 cells were visited",
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                    reason=finish_reason,
                )
                break

            _heading_snapshot("PRE_DEPARTURE_{}".format(current_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            align_ok, align_reason = _align_chassis_after_scan(
                chassis, pose, config, float(raw_start_yaw), stop_event)
            if not align_ok and align_reason in (
                "HEADING_FEEDBACK_LOST",
                "HEADING_ALIGN_TIMEOUT",
            ):
                recorder.event(
                    time.monotonic(), "HEADING_ALIGNMENT_RETRY", align_reason,
                    logical_node=current_cell,
                )
                _sleep_interruptible(0.20, stop_event)
                align_ok, align_reason = _align_chassis_after_scan(
                    chassis, pose, config, float(raw_start_yaw), stop_event)
            if not align_ok and align_reason == "HEADING_ALIGN_TIMEOUT":
                relaxed_yaw = pose.get_yaw()
                if (
                    relaxed_yaw is not None
                    and abs(_heading_error(float(raw_start_yaw), relaxed_yaw))
                    <= V05_MOVING_YAW_ABORT_DEG
                ):
                    align_ok = True
                    align_reason = "HEADING_ALIGN_RELAXED"
                    recorder.event(
                        time.monotonic(),
                        "HEADING_ALIGNMENT_RELAXED",
                        "bounded residual yaw accepted after two alignment attempts",
                        logical_node=current_cell,
                        yaw=relaxed_yaw,
                        reference=float(raw_start_yaw),
                    )
            recorder.event(time.monotonic(), "HEADING_ALIGNMENT", align_reason,
                           logical_node=current_cell, yaw=pose.get_yaw(),
                           reference=float(raw_start_yaw))
            if not align_ok:
                finish_reason = align_reason
                break

            move_direction = int(plan["move_direction"])
            next_cell = tuple(plan["next_cell"])
            is_new = bool(plan["is_new"])
            planner_frontier_count = int(plan["frontier_count"])
            planner_frontier_cell = tuple(plan["frontier_cell"])
            planner_frontier_target = tuple(plan["frontier_target"])
            planner_route = [
                tuple(cell) for cell in plan["route"]
            ]

            recorder.event(
                time.monotonic(),
                "FRONTIER_PLAN",
                str(plan["mode"]),
                logical_node=current_cell,
                from_node=current_cell,
                to_node=next_cell,
                direction=DIR_NAME[move_direction],
            )

            publish_state(
                status=(
                    "Exploring new cell {} via {}".format(
                        next_cell,
                        DIR_NAME[move_direction],
                    )
                    if is_new
                    else "Routing to frontier {} via {}".format(
                        planner_frontier_cell,
                        DIR_NAME[move_direction],
                    )
                ),
                logical_cell=current_cell,
                gimbal_direction=move_direction,
                tof_cm=sensors.get_front_cm(),
                moves=moves,
                force=True,
            )

            recorder.event(
                time.monotonic(),
                "EXPLORE" if is_new else "RELOCATE_FRONTIER",
                (
                    "enter unvisited frontier cell"
                    if is_new
                    else "shortest-path relocation to nearest frontier"
                ),
                from_node=current_cell,
                to_node=next_cell,
                direction=DIR_NAME[move_direction],
            )

            ok, reason, moved = _drive_one_cell(
                chassis,
                gimbal,
                pose,
                heading,
                sensors,
                gimbal_tracker,
                vision,
                grid,
                recorder,
                config,
                float(raw_start_x),
                float(raw_start_y),
                float(raw_start_yaw),
                move_direction,
                current_cell,
                next_cell,
                ranges,
                adjacent_wall_sides(
                    move_direction, current_cell, next_cell, edge_states
                ),
                moves,
                stop_event,
                publish_state,
            )

            _heading_snapshot("POST_MOVE_{}".format(next_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            if ok:
                previous_cell = current_cell
                current_cell = next_cell

                traversed_edges.add(
                    _canonical_edge(previous_cell, move_direction)
                )
                _set_edge_state(
                    edge_states,
                    previous_cell,
                    move_direction,
                    "OPEN",
                )

                if is_new:
                    visited.add(current_cell)
                known_cells.add(current_cell)
                logical_path.append(current_cell)

                last_move_direction = move_direction
                moves += 1
                # A veto is local, temporary evidence. Any subsequent progress
                # proves the planner can safely reconsider those routes later.
                blocked_edges.clear()
                continue

            # A preflight veto happens before any translation, so the robot is
            # still at the confirmed current cell and the planner may safely
            # choose another edge. Mid-cell failures never auto-backtrack.
            if reason == "PREFLIGHT_BLOCKED" and moved <= 1e-6:
                blocked_edges.add((current_cell, move_direction))
                blocked_edges.add((next_cell, (move_direction + 2) % 4))
                recorder.event(
                    time.monotonic(), "PREFLIGHT_EDGE_EXCLUDED",
                    "three fresh ToF callbacks vetoed edge before motor command",
                    logical_node=current_cell, intended_node=next_cell,
                    direction=DIR_NAME[move_direction],
                )
                publish_state(
                    status="Preflight blocked {}; choosing another route".format(
                        DIR_NAME[move_direction]
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=move_direction,
                    tof_cm=sensors.get_front_cm(), moves=moves, force=True,
                )
                continue

            if moved <= 1e-6 and reason in (
                "GIMBAL_UNAVAILABLE",
                "PREFLIGHT_GIMBAL_STALE",
                "PREFLIGHT_TOF_STALE",
                "ODOMETRY_UNAVAILABLE",
            ):
                recorder.event(
                    time.monotonic(), "MOVE_TRANSIENT_RETRY", reason,
                    logical_node=current_cell,
                    intended_node=next_cell,
                    direction=DIR_NAME[move_direction],
                )
                publish_state(
                    status="Transient {} recovered by retrying from same cell".format(
                        reason
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=move_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                    reason=reason,
                )
                continue

            finish_reason = (
                "FRONTIER_RELOCATE_{}".format(reason)
                if not is_new
                else reason
            )
            break

        else:
            if finish_reason == "UNKNOWN":
                finish_reason = "MAX_MOVES_REACHED"

        if finish_reason == "UNKNOWN":
            # Never label an inferred/frontier-only condition as completion.
            finish_reason = "INCOMPLETE_NO_REACHABLE_FRONTIER"

    except KeyboardInterrupt:
        stop_event.set()
        finish_reason = "USER_STOP"

    except Exception as exc:
        finish_reason = "ERROR: {}".format(exc)
        recorder.event(time.monotonic(), "ERROR", str(exc))
        raise

    finally:
        # The preview worker must stop before CameraService releases the
        # shared OpenCV stream; it never issues robot commands.
        survey_bridge.stop()
        if chassis is not None:
            try:
                stop_chassis(chassis)
            except Exception:
                pass

        try:
            if camera_service is not None:
                camera_service.stop()
        except Exception:
            pass

        try:
            if vision is not None:
                vision.stop()
        except Exception:
            pass

        # Never wait on a gimbal action while shutting down. Just stop angular
        # motion; this keeps Ctrl+C / errors from hanging during cleanup.
        try:
            if gimbal is not None:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
        except Exception:
            pass

        end_pose = None
        try:
            rel_x, rel_y = _relative_xy(
                pose,
                float(raw_start_x),
                float(raw_start_y),
                float(raw_start_yaw),
                config,
            )
            yaw = pose.get_yaw()
            if rel_x is not None and rel_y is not None and yaw is not None:
                end_pose = (
                    rel_x,
                    rel_y,
                    normalize_angle_deg(float(yaw) - float(raw_start_yaw)),
                )
        except Exception:
            pass

        try:
            if tof_sensor is not None and tof_subscribed:
                tof_sensor.unsub_distance()
        except Exception:
            pass
        try:
            if chassis is not None and pose_subscribed:
                chassis.unsub_position()
        except Exception:
            pass
        try:
            if chassis is not None and attitude_subscribed:
                chassis.unsub_attitude()
        except Exception:
            pass
        try:
            if gimbal is not None and gimbal_subscribed:
                gimbal.unsub_angle()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print(
            "[MISSION] Finish reason: {}".format(finish_reason),
            flush=True,
        )
        run_dir = recorder.export(
            grid,
            reason=finish_reason,
            start_pose=start_pose,
            end_pose=end_pose,
        )

        try:
            target_registry.save(run_dir)
            save_topology(
                run_dir,
                cell_size_m=config.cell_size_m,
                start_cell=(0, 0),
                final_cell=current_cell,
                visited=visited,
                edge_states=edge_states,
                traversed_edges=traversed_edges,
                start_scan_ranges=start_scan_ranges,
                finish_reason=finish_reason,
            )
            print(
                "[EXPORT] Saved topology.json and targets.json ({} targets).".format(
                    len(target_registry.targets)
                ),
                flush=True,
            )
        except Exception as exc:
            print("[EXPORT] Final metadata export failed: {}".format(exc), flush=True)

        publish_state(
            status="Finished: {}".format(finish_reason),
            logical_cell=current_cell,
            gimbal_direction=current_gimbal_direction,
            tof_cm=current_tof,
            moves=moves,
            force=True,
            reason="{} | saved: {}".format(finish_reason, run_dir),
            finished=True,
            run_dir=str(run_dir),
        )

        print("Classwork 8 results saved to: {}".format(run_dir))

    return run_dir
