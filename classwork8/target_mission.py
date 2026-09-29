"""Fail-safe target selection and RoboMaster blaster command state."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Iterable, Optional, Set, Tuple


VALID_COLORS = {"blue", "green", "red", "yellow"}
VALID_SHAPES = {"circle", "rectangle", "square"}
VALID_FIRE_TYPES = {"ir", "water"}
MAX_TARGET_FIRE_TIMES = 30
SDK_MAX_FIRE_TIMES = 5


def fire_blaster_burst(blaster, fire_type: str, times: int) -> bool:
    """Fire up to 30 shots using SDK-safe commands of at most five shots."""
    remaining = int(times)
    while remaining > 0:
        batch = min(remaining, SDK_MAX_FIRE_TIMES)
        if blaster.fire(fire_type=fire_type, times=batch) is not True:
            return False
        remaining -= batch
    return True


@dataclass(frozen=True, order=True)
class TargetSpec:
    color: str
    shape: str

    @property
    def key(self) -> str:
        return "{}:{}".format(self.color, self.shape)


class TargetMissionState(str, Enum):
    DETECTED = "DETECTED"
    VERIFIED = "VERIFIED"
    NOT_SELECTED = "NOT_SELECTED"
    RANGE_UNCONFIRMED = "RANGE_UNCONFIRMED"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    NEEDS_AIM = "NEEDS_AIM"
    AIMING = "AIMING"
    AIM_SETTLED = "AIM_SETTLED"
    AIM_FAILED = "AIM_FAILED"
    READY_DRY_RUN = "READY_DRY_RUN"
    READY = "READY"
    FIRING = "FIRING"
    COMMAND_ACKNOWLEDGED = "COMMAND_ACKNOWLEDGED"
    FIRE_FAILED = "FIRE_FAILED"
    ALREADY_FIRED = "ALREADY_FIRED"


@dataclass(frozen=True)
class FireDecision:
    target_id: str
    spec: TargetSpec
    state: TargetMissionState
    should_fire: bool
    distance_m: Optional[float]
    detail: str


def parse_target_specs(value: str) -> Set[TargetSpec]:
    """Parse ``color:shape`` pairs without an unsafe implicit ALL option."""
    result: Set[TargetSpec] = set()
    for raw in str(value or "").split(","):
        item = raw.strip().lower()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(
                "target selection must use color:shape pairs, for example "
                "blue:circle,red:rectangle"
            )
        color, shape = (part.strip() for part in item.split(":", 1))
        if color not in VALID_COLORS:
            raise ValueError("unsupported target color: {}".format(color))
        if shape not in VALID_SHAPES:
            raise ValueError("unsupported target shape: {}".format(shape))
        result.add(TargetSpec(color, shape))
    return result


def target_distance_m(tof_cm: Optional[float], tof_forward_offset_m: float) -> Optional[float]:
    if tof_cm is None or not math.isfinite(float(tof_cm)) or float(tof_cm) < 0.0:
        return None
    return float(tof_forward_offset_m) + float(tof_cm) / 100.0


def target_is_centered(
    centroid_px: Tuple[int, int],
    frame_size_px: Tuple[int, int],
    tolerance_ratio: float,
    offset_x_ratio: float = 0.0,
    offset_y_ratio: float = 0.0,
) -> bool:
    width, height = frame_size_px
    if width <= 0 or height <= 0:
        return False
    cx, cy = centroid_px
    desired_x = width * (0.5 + float(offset_x_ratio))
    desired_y = height * (0.5 + float(offset_y_ratio))
    return bool(
        abs(float(cx) - desired_x) / float(width) <= float(tolerance_ratio)
        and abs(float(cy) - desired_y) / float(height) <= float(tolerance_ratio)
    )


class TargetMission:
    """Small state owner: selection/range/aim gates precede every fire command."""

    def __init__(self, config) -> None:
        self.selected = parse_target_specs(config.target_required_specs)
        self.fire_enabled = bool(config.target_fire_enabled)
        configured_mode = str(getattr(config, "target_fire_mode", "off")).lower()
        # Old saved configs and --arm-fire remain selected-target mode.
        self.fire_mode = (
            "selected" if self.fire_enabled and configured_mode == "off"
            else configured_mode
        )
        self.fire_type = str(config.target_fire_type).lower()
        self.fire_times = int(config.target_fire_times)
        self.max_distance_m = (
            float(config.target_max_fire_distance_cells)
            * float(config.cell_size_m)
        )
        self.tof_forward_offset_m = float(config.tof_forward_offset_m)
        self.aim_tolerance_ratio = float(config.target_aim_tolerance_ratio)
        self.aim_offset_x_ratio = float(config.target_aim_offset_x_ratio)
        self.aim_offset_y_ratio = float(config.target_aim_offset_y_ratio)
        self.fired_specs: Set[TargetSpec] = set()
        self.fired_target_ids: Set[str] = set()
        self.states: Dict[str, TargetMissionState] = {}

    def assess(
        self,
        target: dict,
        *,
        centroid_px: Tuple[int, int],
        frame_size_px: Tuple[int, int],
        tof_cm: Optional[float],
        range_confirmed: bool,
        aim_confirmed: bool = False,
        aim_offset_x_ratio: Optional[float] = None,
        aim_offset_y_ratio: Optional[float] = None,
    ) -> FireDecision:
        target_id = str(target["target_id"])
        spec = TargetSpec(str(target["color"]).lower(), str(target["shape"]).lower())
        self.states[target_id] = TargetMissionState.DETECTED
        self.states[target_id] = TargetMissionState.VERIFIED
        distance_m = target_distance_m(tof_cm, self.tof_forward_offset_m)

        if self.fire_mode != "all" and spec not in self.selected:
            return self._decision(target_id, spec, TargetMissionState.NOT_SELECTED,
                                  False, distance_m, "color/shape not requested")
        if (target_id in self.fired_target_ids
                or (self.fire_mode != "all" and spec in self.fired_specs)):
            return self._decision(target_id, spec, TargetMissionState.ALREADY_FIRED,
                                  False, distance_m, "selected target type already fired")
        if not range_confirmed or distance_m is None:
            return self._decision(target_id, spec, TargetMissionState.RANGE_UNCONFIRMED,
                                  False, distance_m, "fresh ToF wall range is required")
        if distance_m > self.max_distance_m:
            return self._decision(target_id, spec, TargetMissionState.OUT_OF_RANGE,
                                  False, distance_m, "target exceeds two-cell firing range")
        if not aim_confirmed or not target_is_centered(
            centroid_px,
            frame_size_px,
            self.aim_tolerance_ratio,
            (
                self.aim_offset_x_ratio
                if aim_offset_x_ratio is None else float(aim_offset_x_ratio)
            ),
            (
                self.aim_offset_y_ratio
                if aim_offset_y_ratio is None else float(aim_offset_y_ratio)
            ),
        ):
            return self._decision(target_id, spec, TargetMissionState.NEEDS_AIM,
                                  False, distance_m, "fresh auto-aim settle is required")
        if not self.fire_enabled:
            return self._decision(target_id, spec, TargetMissionState.READY_DRY_RUN,
                                  False, distance_m, "all gates passed; firing is not armed")
        return self._decision(target_id, spec, TargetMissionState.READY,
                              True, distance_m, "selected, verified, in range and centered")

    def mark_aiming(self, target_id: str) -> None:
        self.states[str(target_id)] = TargetMissionState.AIMING

    def mark_aim_result(self, target_id: str, success: bool) -> None:
        self.states[str(target_id)] = (
            TargetMissionState.AIM_SETTLED
            if success else TargetMissionState.AIM_FAILED
        )

    def fire(self, decision: FireDecision, blaster) -> bool:
        if not decision.should_fire or decision.state != TargetMissionState.READY:
            return False
        if blaster is None:
            self.states[decision.target_id] = TargetMissionState.FIRE_FAILED
            return False
        self.states[decision.target_id] = TargetMissionState.FIRING
        try:
            acknowledged = fire_blaster_burst(
                blaster, self.fire_type, self.fire_times
            )
        except Exception:
            acknowledged = False
        if acknowledged:
            self.fired_specs.add(decision.spec)
            self.fired_target_ids.add(decision.target_id)
            self.states[decision.target_id] = TargetMissionState.COMMAND_ACKNOWLEDGED
        else:
            self.states[decision.target_id] = TargetMissionState.FIRE_FAILED
        return acknowledged

    def annotate_target(
        self,
        target: dict,
        decision: FireDecision,
        detail_override: Optional[str] = None,
    ) -> None:
        state = self.states.get(decision.target_id, decision.state)
        target["mission_state"] = state.value
        target["selected_for_fire"] = (
            self.fire_mode == "all" or decision.spec in self.selected
        )
        target["fire_distance_m"] = decision.distance_m
        target["fire_command_acknowledged"] = (
            state == TargetMissionState.COMMAND_ACKNOWLEDGED
        )
        target["fire_detail"] = (
            decision.detail if detail_override is None else str(detail_override)
        )

    def _decision(
        self,
        target_id: str,
        spec: TargetSpec,
        state: TargetMissionState,
        should_fire: bool,
        distance_m: Optional[float],
        detail: str,
    ) -> FireDecision:
        self.states[target_id] = state
        return FireDecision(
            target_id=target_id,
            spec=spec,
            state=state,
            should_fire=should_fire,
            distance_m=distance_m,
            detail=detail,
        )


def specs_as_text(specs: Iterable[TargetSpec]) -> str:
    return ",".join(item.key for item in sorted(set(specs)))
