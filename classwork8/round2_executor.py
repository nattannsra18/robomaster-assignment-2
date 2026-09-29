"""Fail-closed physical executor for a verified Round-2 target plan."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from robomaster import robot

from .camera_service import CameraService
from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .robot_support import HeadingManager, wait_for_position, wait_for_yaw
from .round2_mission import DIR_NAME, DIR_VEC, load_round2_plan
from .target_aim import TargetAutoAim
from .target_detection import TargetDetector
from .target_mission import TargetMission, TargetMissionState
from .tof_camera_round1_v05 import (
    DIR_VEC_MAP,
    GimbalTracker,
    ToFOnlySensorManager,
    V05PoseTracker,
    _drive_one_cell,
    _point_gimbal,
    _set_camera_observation_pitch,
    _wait_for_fresh_tof,
    stop_chassis,
)


Cell = Tuple[int, int]
MoveCallback = Callable[[dict, int], Tuple[bool, str]]
TargetCallback = Callable[[dict], Tuple[bool, str]]
EventCallback = Callable[[str, dict], None]


@dataclass(frozen=True)
class Round2ExecutionResult:
    completed: bool
    reason: str
    final_cell: Cell
    steps_completed: int
    targets_completed: Tuple[str, ...]


def _cell(value: Sequence[int]) -> Cell:
    if value is None or len(value) != 2:
        raise ValueError("Round-2 cell must contain exactly two coordinates")
    return int(value[0]), int(value[1])


def _step_direction(source: Cell, destination: Cell) -> int:
    delta = destination[0] - source[0], destination[1] - source[1]
    for direction, vector in DIR_VEC.items():
        if delta == vector:
            return int(direction)
    raise ValueError("Round-2 route contains non-adjacent cells")


def validate_execution_plan(plan: dict) -> None:
    """Reject stale, discontinuous, or directionally inconsistent plans."""
    if int(plan.get("version", 0)) < 3:
        raise ValueError("Round-2 plan is stale; regenerate it with the current planner")
    facing = int(plan.get("start_facing_direction", -1))
    if facing not in DIR_NAME:
        raise ValueError("Round-2 plan has an invalid start facing direction")
    required = tuple(sorted(str(item) for item in plan.get("required_targets", [])))
    if not required:
        raise ValueError("Round-2 plan contains no required targets")
    actions = plan.get("actions")
    if not isinstance(actions, list) or len(actions) != len(required):
        raise ValueError("Round-2 action count does not match required targets")

    current = _cell(plan.get("start_cell"))
    full_route = [current]
    seen_ids = set()
    seen_specs = set()
    for action in actions:
        target_id = str(action.get("target_id", ""))
        spec = "{}:{}".format(
            str(action.get("color", "")).lower(),
            str(action.get("shape", "")).lower(),
        )
        if not target_id or target_id in seen_ids:
            raise ValueError("Round-2 plan has a missing or duplicate target id")
        if spec not in required or spec in seen_specs:
            raise ValueError("Round-2 plan target selection is inconsistent")
        seen_ids.add(target_id)
        seen_specs.add(spec)

        route = [_cell(value) for value in action.get("route", [])]
        steps = action.get("route_steps", [])
        approach = _cell(action.get("approach_cell"))
        if not route or route[0] != current or route[-1] != approach:
            raise ValueError("Round-2 target route is not continuous")
        if len(steps) != max(0, len(route) - 1):
            raise ValueError("Round-2 route step count is inconsistent")
        for index, (source, destination) in enumerate(zip(route, route[1:])):
            step = steps[index]
            if _cell(step.get("from_cell")) != source:
                raise ValueError("Round-2 route step has the wrong source cell")
            if _cell(step.get("to_cell")) != destination:
                raise ValueError("Round-2 route step has the wrong destination cell")
            map_direction = _step_direction(source, destination)
            if int(step.get("map_direction", -1)) != map_direction:
                raise ValueError("Round-2 route step has the wrong map direction")
            if int(step.get("body_direction", -1)) != (map_direction - facing) % 4:
                raise ValueError("Round-2 route step has the wrong body direction")
        map_view = int(action.get("map_view_direction", -1))
        body_view = int(action.get("body_view_direction", -1))
        if map_view not in DIR_NAME or body_view != (map_view - facing) % 4:
            raise ValueError("Round-2 target view direction is inconsistent")
        expected_centroid = action.get("expected_centroid_px")
        if expected_centroid is None or len(expected_centroid) != 2:
            raise ValueError(
                "Round-2 execution requires a Round-1 reference centroid"
            )
        try:
            if not all(math.isfinite(float(value)) for value in expected_centroid):
                raise ValueError
        except (TypeError, ValueError):
            raise ValueError("Round-2 expected target centroid is invalid")
        full_route.extend(route[1:])
        current = approach

    if tuple(sorted(seen_specs)) != required:
        raise ValueError("Round-2 plan does not cover every required target")
    if [_cell(value) for value in plan.get("full_route", [])] != full_route:
        raise ValueError("Round-2 full route does not match its target routes")


def load_verified_execution_plan(run_dir: Path, plan_path: Path) -> dict:
    """Rebuild a saved plan from current artifacts and reject any drift."""
    run_dir = Path(run_dir)
    plan_path = Path(plan_path)
    saved = json.loads(plan_path.read_text(encoding="utf-8"))
    required = ",".join(str(item) for item in saved.get("required_targets", []))
    canonical = load_round2_plan(
        run_dir,
        required,
        start_cell=_cell(saved.get("start_cell")),
        start_facing_direction=int(saved.get("start_facing_direction", -1)),
    )
    checked_keys = (
        "version",
        "source_finish_reason",
        "start_cell",
        "start_facing_direction",
        "required_targets",
        "actions",
        "full_route",
        "full_route_steps",
    )
    if any(saved.get(key) != canonical.get(key) for key in checked_keys):
        raise ValueError(
            "Round-2 plan no longer matches Round-1 artifacts; regenerate it"
        )
    validate_execution_plan(canonical)
    return canonical


def load_round1_config(run_dir: Path) -> Classwork8Config:
    """Load the exact movement/sensor calibration saved by Round 1."""
    summary_path = Path(run_dir) / "summary.json"
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    saved = payload.get("config")
    if not isinstance(saved, dict):
        raise ValueError("Round-1 summary.json does not contain saved configuration")
    allowed = {item.name for item in fields(Classwork8Config)}
    config = Classwork8Config(**{
        key: value for key, value in saved.items() if key in allowed
    })
    return config


def execute_round2_plan(
    plan: dict,
    move_step: MoveCallback,
    engage_target: TargetCallback,
    *,
    stop_event: Optional[threading.Event] = None,
    max_route_steps: Optional[int] = None,
    event: Optional[EventCallback] = None,
) -> Round2ExecutionResult:
    """Execute the deterministic task sequence through injected operations."""
    validate_execution_plan(plan)
    stop_event = stop_event or threading.Event()
    emit = event or (lambda _name, _data: None)
    current = _cell(plan["start_cell"])
    steps_completed = 0
    targets_completed: List[str] = []
    target_failures: List[str] = []

    for action in plan["actions"]:
        for step in action["route_steps"]:
            if stop_event.is_set():
                return Round2ExecutionResult(
                    False, "USER_STOP", current, steps_completed,
                    tuple(targets_completed),
                )
            if (
                max_route_steps is not None
                and steps_completed >= int(max_route_steps)
            ):
                return Round2ExecutionResult(
                    False, "ROUTE_STEP_LIMIT_REACHED", current,
                    steps_completed, tuple(targets_completed),
                )
            if _cell(step["from_cell"]) != current:
                return Round2ExecutionResult(
                    False, "ROUTE_STATE_MISMATCH", current,
                    steps_completed, tuple(targets_completed),
                )
            emit("MOVE_START", dict(step))
            try:
                ok, reason = move_step(step, steps_completed)
            except Exception as exc:
                reason = "MOVE_ERROR: {}".format(exc)
                emit("MOVE_FAILED", {"reason": reason, "step": dict(step)})
                return Round2ExecutionResult(
                    False, reason, current, steps_completed,
                    tuple(targets_completed),
                )
            if not ok:
                emit("MOVE_FAILED", {"reason": reason, "step": dict(step)})
                return Round2ExecutionResult(
                    False, str(reason), current, steps_completed,
                    tuple(targets_completed),
                )
            current = _cell(step["to_cell"])
            steps_completed += 1
            emit("MOVE_COMPLETE", dict(step))

        if current != _cell(action["approach_cell"]):
            return Round2ExecutionResult(
                False, "APPROACH_CELL_MISMATCH", current,
                steps_completed, tuple(targets_completed),
            )
        emit("TARGET_START", dict(action))
        ok = False
        reason = "TARGET_NOT_ATTEMPTED"
        for target_attempt in range(2):
            try:
                ok, reason = engage_target(action)
            except Exception as exc:
                reason = "TARGET_ERROR: {}".format(exc)
                ok = False
            if ok:
                break
            if target_attempt == 0:
                emit("TARGET_RETRY", {
                    "reason": str(reason),
                    "action": dict(action),
                })
        if not ok:
            emit("TARGET_FAILED", {"reason": reason, "action": dict(action)})
            target_failures.append(
                "{}:{}".format(action["target_id"], str(reason))
            )
            # The chassis is still at the verified approach cell. Preserve the
            # chance to score later targets instead of abandoning the round for
            # one camera/aim/fire failure.
            continue
        targets_completed.append(str(action["target_id"]))
        emit("TARGET_COMPLETE", dict(action))

    completed = not target_failures
    return Round2ExecutionResult(
        completed,
        (
            "ROUND2_COMPLETE"
            if completed
            else "ROUND2_PARTIAL_TARGET_FAILURES:{}".format(
                ",".join(target_failures)
            )
        ),
        current,
        steps_completed,
        tuple(targets_completed),
    )


class Round2Recorder:
    """Small JSON recorder implementing the movement recorder contract."""

    def __init__(self, run_dir: Path, plan: dict) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_dir = Path(run_dir) / "round2_execution_{}".format(stamp)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.plan = plan
        self.events: List[dict] = []
        self.samples: List[dict] = []

    def _elapsed(self, now: float) -> float:
        return round(float(now) - self.started, 4)

    def event(self, now: float, event_type: str, detail: str, **extra) -> None:
        row = {
            "t_sec": self._elapsed(now),
            "event": str(event_type),
            "detail": str(detail),
        }
        row.update(extra)
        self.events.append(row)

    def record_sample(
        self,
        now: float,
        x_m,
        y_m,
        yaw_deg,
        heading_index,
        front_cm,
        left_cm,
        right_cm,
        ir_left,
        ir_right,
        mode,
        *unused,
        **unused_named
    ) -> None:
        self.samples.append({
            "t_sec": self._elapsed(now),
            "x_m": x_m,
            "y_m": y_m,
            "yaw_deg": yaw_deg,
            "heading_index": int(heading_index) % 4,
            "front_cm": front_cm,
            "mode": str(mode),
        })

    def save(self, result: Round2ExecutionResult) -> Path:
        payload = {
            "version": 1,
            "result": asdict(result),
            "plan": self.plan,
            "events": self.events,
            "samples": self.samples,
        }
        path = self.output_dir / "round2_execution.json"
        path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return path


def _select_expected_target(
    verified_targets,
    action: dict,
    max_jump_px: float,
):
    matches = [
        item for item in verified_targets
        if str(item.detection.color).lower() == str(action["color"]).lower()
        and str(item.detection.shape).lower() == str(action["shape"]).lower()
    ]
    if not matches:
        return None
    expected = action.get("expected_centroid_px")
    if expected is None:
        return None
    ex, ey = float(expected[0]), float(expected[1])
    selected = min(
        matches,
        key=lambda item: math.hypot(
            float(item.detection.centroid[0]) - ex,
            float(item.detection.centroid[1]) - ey,
        ),
    )
    jump = math.hypot(
        float(selected.detection.centroid[0]) - ex,
        float(selected.detection.centroid[1]) - ey,
    )
    return selected if jump <= float(max_jump_px) else None


def _engage_physical_target(
    action: dict,
    *,
    chassis,
    gimbal,
    blaster,
    sensors: ToFOnlySensorManager,
    gimbal_tracker: GimbalTracker,
    camera_service: CameraService,
    detector: TargetDetector,
    mission: TargetMission,
    auto_aim: TargetAutoAim,
    config: Classwork8Config,
    recorder: Round2Recorder,
    stop_event: threading.Event,
) -> Tuple[bool, str]:
    """Revalidate one planned target with fresh range/image before firing."""
    stop_chassis(chassis)
    view_direction = int(action["body_view_direction"]) % 4
    target_id = str(action["target_id"])
    if not _point_gimbal(
        gimbal, sensors, gimbal_tracker, view_direction, config, stop_event
    ):
        return False, "TARGET_GIMBAL_AIM_FAILED"

    tof_cm = _wait_for_fresh_tof(
        sensors, config.tof_recovery_wait_sec, stop_event
    )
    if tof_cm is None:
        return False, "TARGET_TOF_STALE"
    if float(tof_cm) >= float(config.tof_open_cm):
        return False, "TARGET_RANGE_NOT_WALL_CONFIRMED"

    camera_position_ok = _set_camera_observation_pitch(
        gimbal,
        gimbal_tracker,
        config,
        float(config.target_camera_pitch_deg),
        stop_event,
    )
    result: Tuple[bool, str] = (False, "TARGET_CAMERA_PITCH_FAILED")

    def evaluate_target() -> Tuple[bool, str]:
        if not camera_position_ok:
            return False, "TARGET_CAMERA_PITCH_FAILED"
        frame_epoch = time.monotonic()
        verified_targets, debug_frame = detector.verify_latest(
            camera_service,
            not_before=frame_epoch,
        )
        selected = _select_expected_target(
            verified_targets,
            action,
            float(config.target_auto_aim_max_jump_px),
        )
        if selected is None:
            return False, "TARGET_NOT_REVALIDATED"
        if debug_frame is None:
            return False, "TARGET_DEBUG_FRAME_MISSING"
        frame_size = int(debug_frame.shape[1]), int(debug_frame.shape[0])
        target = {
            "target_id": target_id,
            "color": str(action["color"]).lower(),
            "shape": str(action["shape"]).lower(),
        }
        decision = mission.assess(
            target,
            centroid_px=selected.detection.centroid,
            frame_size_px=frame_size,
            tof_cm=float(tof_cm),
            range_confirmed=True,
            aim_confirmed=False,
        )
        if decision.state != TargetMissionState.NEEDS_AIM:
            return False, decision.state.value

        mission.mark_aiming(target_id)
        aim_result = auto_aim.aim(
            gimbal=gimbal,
            tracker=gimbal_tracker,
            camera_service=camera_service,
            detector=detector,
            initial_detection=selected.detection,
            stop_event=stop_event,
        )
        recorder.event(
            time.monotonic(),
            "TARGET_AIM",
            aim_result.reason,
            target_id=target_id,
            success=aim_result.success,
            fresh_frames=aim_result.fresh_frames,
            tof_cm=float(tof_cm),
        )
        mission.mark_aim_result(target_id, aim_result.success)
        if not aim_result.success or aim_result.detection is None:
            return False, aim_result.reason

        decision = mission.assess(
            target,
            centroid_px=aim_result.detection.centroid,
            frame_size_px=aim_result.frame_size_px,
            tof_cm=float(tof_cm),
            range_confirmed=True,
            aim_confirmed=True,
        )
        if decision.state == TargetMissionState.READY_DRY_RUN:
            return True, "TARGET_READY_DRY_RUN"
        if decision.should_fire:
            if mission.fire(decision, blaster):
                return True, "TARGET_FIRE_ACKNOWLEDGED"
            return False, "TARGET_FIRE_FAILED"
        return False, decision.state.value

    try:
        result = evaluate_target()
        recorder.event(
            time.monotonic(),
            "TARGET_RESULT",
            result[1],
            target_id=target_id,
            color=action["color"],
            shape=action["shape"],
            tof_cm=float(tof_cm),
            fire_enabled=bool(config.target_fire_enabled),
        )
    finally:
        restore_ok = False
        for restore_attempt in range(2):
            restore_ok = _set_camera_observation_pitch(
                gimbal,
                gimbal_tracker,
                config,
                float(config.gimbal_scan_pitch_deg),
                stop_event,
                tolerance_deg=float(config.gimbal_pitch_tolerance_deg),
                clamp_camera_limits=False,
            )
            if restore_ok:
                break
            if restore_attempt == 0:
                recorder.event(
                    time.monotonic(),
                    "TARGET_GIMBAL_RESTORE_RETRY",
                    "retrying horizontal ToF pose once",
                    target_id=target_id,
                )
        if not restore_ok:
            result = False, "TARGET_GIMBAL_RESTORE_FAILED"
    return result


def run_round2_physical(
    plan: dict,
    config: Classwork8Config,
    run_dir: Path,
    *,
    ep_robot=None,
    stop_event: Optional[threading.Event] = None,
    publish: Optional[Callable[[dict], None]] = None,
    max_route_steps: Optional[int] = None,
) -> Tuple[Round2ExecutionResult, Path]:
    """Connect to RoboMaster and execute a verified plan with live guards."""
    validate_execution_plan(plan)
    config.target_detection_enabled = True
    config.stationary_target_test = False
    config.target_fire_mode = "selected"
    config.target_required_specs = ",".join(plan["required_targets"])
    config.validate()
    stop_event = stop_event or threading.Event()
    recorder = Round2Recorder(run_dir, plan)
    result = Round2ExecutionResult(
        False,
        "INITIALIZATION_NOT_COMPLETED",
        _cell(plan["start_cell"]),
        0,
        (),
    )

    owns_robot = ep_robot is None
    if ep_robot is None:
        ep_robot = robot.Robot()
    chassis = None
    gimbal = None
    tof_sensor = None
    camera_service = None
    tof_subscribed = False
    pose_subscribed = False
    attitude_subscribed = False
    gimbal_subscribed = False
    pose = V05PoseTracker()
    sensors = ToFOnlySensorManager()
    gimbal_tracker = GimbalTracker()

    def record_event(name: str, data: dict) -> None:
        recorder.event(time.monotonic(), name, name, **data)

    try:
        if owns_robot:
            ok = ep_robot.initialize(conn_type=config.connection)
            if not ok:
                raise RuntimeError("ROBOMASTER_CONNECTION_FAILED")
        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal
        tof_sensor = ep_robot.sensor
        blaster = ep_robot.blaster if config.target_fire_enabled else None

        if not ep_robot.set_robot_mode(mode=robot.FREE):
            raise RuntimeError("FREE_MODE_FAILED")
        stop_chassis(chassis)

        tof_subscribed = bool(tof_sensor.sub_distance(
            freq=20, callback=sensors.tof_callback
        ))
        pose_subscribed = bool(chassis.sub_position(
            cs=1, freq=20, callback=pose.position_callback
        ))
        attitude_subscribed = bool(chassis.sub_attitude(
            freq=20, callback=pose.attitude_callback
        ))
        gimbal_subscribed = bool(gimbal.sub_angle(
            freq=20, callback=gimbal_tracker.callback
        ))
        if not all((
            tof_subscribed,
            pose_subscribed,
            attitude_subscribed,
            gimbal_subscribed,
        )):
            raise RuntimeError("ROUND2_SENSOR_SUBSCRIPTION_FAILED")

        raw_start_x, raw_start_y = wait_for_position(pose)
        raw_start_yaw = wait_for_yaw(pose)
        if not pose.has_position() or raw_start_yaw is None:
            raise RuntimeError("ROUND2_INITIAL_POSE_UNAVAILABLE")

        heading = HeadingManager()
        if not heading.initialize(raw_start_yaw):
            raise RuntimeError("ROUND2_INITIAL_HEADING_UNAVAILABLE")
        if not _point_gimbal(
            gimbal, sensors, gimbal_tracker, 0, config, stop_event
        ):
            raise RuntimeError("ROUND2_INITIAL_GIMBAL_FAILED")

        camera_service = CameraService(
            ep_robot,
            resolution=config.target_camera_resolution,
            start_timeout_sec=config.target_camera_start_timeout_sec,
        )
        if not camera_service.start():
            raise RuntimeError("ROUND2_CAMERA_UNAVAILABLE")

        detector = TargetDetector(config)
        mission = TargetMission(config)
        auto_aim = TargetAutoAim(config)
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
        local_cell = [0, 0]
        active_map_cell = [
            int(plan["start_cell"][0]), int(plan["start_cell"][1])
        ]

        def publish_state(**state) -> None:
            if publish is None:
                return
            state["logical_cell"] = tuple(active_map_cell)
            state["round2"] = True
            publish(state)

        def move_step(step: dict, step_index: int) -> Tuple[bool, str]:
            body_direction = int(step["body_direction"]) % 4
            dx, dy = DIR_VEC_MAP[body_direction]
            target_local = local_cell[0] + dx, local_cell[1] + dy
            active_map_cell[:] = list(_cell(step["from_cell"]))
            recorder.event(
                time.monotonic(),
                "ROUND2_MOVE",
                "{} -> {} via body {}".format(
                    tuple(step["from_cell"]),
                    tuple(step["to_cell"]),
                    DIR_NAME[body_direction],
                ),
                map_from=step["from_cell"],
                map_to=step["to_cell"],
                body_direction=body_direction,
            )
            moved_ok = False
            reason = "MOVE_NOT_ATTEMPTED"
            for move_attempt in range(2):
                moved_ok, reason, moved = _drive_one_cell(
                    chassis,
                    gimbal,
                    pose,
                    heading,
                    sensors,
                    gimbal_tracker,
                    None,
                    grid,
                    recorder,
                    config,
                    float(raw_start_x),
                    float(raw_start_y),
                    float(raw_start_yaw),
                    body_direction,
                    tuple(local_cell),
                    target_local,
                    None,
                    set(),
                    step_index,
                    stop_event,
                    publish_state,
                )
                if moved_ok:
                    break
                if (
                    move_attempt == 0
                    and moved <= 1e-6
                    and reason in (
                        "PREFLIGHT_BLOCKED",
                        "GIMBAL_UNAVAILABLE",
                        "PREFLIGHT_GIMBAL_STALE",
                        "PREFLIGHT_TOF_STALE",
                        "ODOMETRY_UNAVAILABLE",
                    )
                ):
                    recorder.event(
                        time.monotonic(),
                        "ROUND2_MOVE_RETRY",
                        reason,
                        map_from=step["from_cell"],
                        map_to=step["to_cell"],
                    )
                    continue
                break
            if moved_ok:
                local_cell[:] = list(target_local)
                active_map_cell[:] = list(_cell(step["to_cell"]))
            return moved_ok, reason

        def engage_target(action: dict) -> Tuple[bool, str]:
            active_map_cell[:] = list(_cell(action["approach_cell"]))
            return _engage_physical_target(
                action,
                chassis=chassis,
                gimbal=gimbal,
                blaster=blaster,
                sensors=sensors,
                gimbal_tracker=gimbal_tracker,
                camera_service=camera_service,
                detector=detector,
                mission=mission,
                auto_aim=auto_aim,
                config=config,
                recorder=recorder,
                stop_event=stop_event,
            )

        result = execute_round2_plan(
            plan,
            move_step,
            engage_target,
            stop_event=stop_event,
            max_route_steps=max_route_steps,
            event=record_event,
        )
    except KeyboardInterrupt:
        stop_event.set()
        result = Round2ExecutionResult(
            False, "USER_STOP", result.final_cell,
            result.steps_completed, result.targets_completed,
        )
    except Exception as exc:
        recorder.event(time.monotonic(), "ERROR", str(exc))
        result = Round2ExecutionResult(
            False,
            "ERROR: {}".format(exc),
            result.final_cell,
            result.steps_completed,
            result.targets_completed,
        )
    finally:
        if chassis is not None:
            try:
                stop_chassis(chassis)
            except Exception as exc:
                recorder.event(time.monotonic(), "STOP_ERROR", str(exc))
        if camera_service is not None:
            try:
                camera_service.stop()
            except Exception:
                pass
        if gimbal is not None:
            try:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            except Exception:
                pass
        if tof_sensor is not None and tof_subscribed:
            try:
                tof_sensor.unsub_distance()
            except Exception:
                pass
        if chassis is not None and pose_subscribed:
            try:
                chassis.unsub_position()
            except Exception:
                pass
        if chassis is not None and attitude_subscribed:
            try:
                chassis.unsub_attitude()
            except Exception:
                pass
        if gimbal is not None and gimbal_subscribed:
            try:
                gimbal.unsub_angle()
            except Exception:
                pass
        if owns_robot:
            try:
                ep_robot.close()
            except Exception:
                pass

    output_path = recorder.save(result)
    return result, output_path
