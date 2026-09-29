"""Pure Stable V1 decisions for one-cell movement safety."""

from __future__ import annotations

import math
from typing import Optional


def preflight_required_cm(
    remaining_m: float,
    tolerance_m: float,
    hard_stop_cm: float,
    margin_cm: float,
) -> float:
    travel_cm = max(0.0, float(remaining_m) - float(tolerance_m)) * 100.0
    return travel_cm + float(hard_stop_cm) + float(margin_cm)


def preflight_has_clearance(observed_cm: Optional[float], required_cm: float) -> bool:
    return bool(
        observed_cm is not None
        and math.isfinite(float(observed_cm))
        and float(observed_cm) >= float(required_cm)
    )


def tof_braking_speed_mps(
    observed_cm: Optional[float],
    cruise_speed_mps: float,
    brake_start_cm: float,
    hard_stop_cm: float,
    minimum_speed_mps: float,
) -> float:
    """Linear ToF approach speed; zero at or below the hard-stop range."""
    if observed_cm is None or not math.isfinite(float(observed_cm)):
        return 0.0
    observed = float(observed_cm)
    cruise = float(cruise_speed_mps)
    if observed <= float(hard_stop_cm):
        return 0.0
    if observed >= float(brake_start_cm):
        return cruise
    minimum = min(cruise, float(minimum_speed_mps))
    span = float(brake_start_cm) - float(hard_stop_cm)
    fraction = (observed - float(hard_stop_cm)) / span
    return minimum + (cruise - minimum) * max(0.0, min(1.0, fraction))


def cell_pose_within_tolerance(
    remaining_m: float,
    cross_track_m: float,
    step_tolerance_m: float,
    center_tolerance_m: float,
) -> bool:
    """A logical arrival needs both longitudinal and lateral odometry."""
    return bool(
        abs(float(remaining_m)) <= float(step_tolerance_m)
        and abs(float(cross_track_m)) <= float(center_tolerance_m)
    )
