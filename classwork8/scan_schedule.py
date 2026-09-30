"""Geometry-only scheduling for stationary mapping scans (no SDK required)."""
import math


def shortest_scan_order(directions, current_yaw, yaw_for_direction):
    """Visit all requested angles with minimum absolute mechanical yaw travel.

    The gimbal has end stops: +180 to -90 costs 270 degrees, not 90.
    On this line, an optimal sweep starts at one end of the requested interval.
    Keep the caller's order when feedback is missing or invalid.
    """
    directions = list(directions)
    if not directions or current_yaw is None or not math.isfinite(current_yaw):
        return directions
    ascending = sorted(directions, key=yaw_for_direction)
    descending = ascending[::-1]

    def cost(order):
        angles = [current_yaw] + [yaw_for_direction(d) for d in order]
        return sum(abs(b - a) for a, b in zip(angles, angles[1:]))

    return min((directions, ascending, descending), key=cost)


def field_scan_order(directions, current_yaw, yaw_for_direction):
    """Apply the field-test three-face order before normal yaw minimization.

    After a BACK move the incoming FRONT edge is already confirmed OPEN, so a
    new cell normally needs exactly BACK, LEFT and RIGHT.  Keep that explicit
    order for the current field test; all other subsets retain the mechanical
    minimum-travel ordering.
    """
    directions = list(directions)
    if len(directions) == 3 and set(directions) == {1, 2, 3}:
        return [2, 3, 1]
    return shortest_scan_order(directions, current_yaw, yaw_for_direction)


def omit_completed_wall_surveys(directions, current_cell, edge_states,
                                completed_surveys, pending_surveys):
    """Reuse only a successfully observed wall face at this exact cell.

    Unknown edges, pending retries and the opposite face at another cell must
    still be scanned. This optimization is not used for wall maintenance.
    """
    scan, reused = [], []
    for direction in directions:
        key = (current_cell, direction)
        complete = (key in completed_surveys and key not in pending_surveys
                    and edge_states.get((*current_cell, direction)) == "WALL")
        (reused if complete else scan).append(direction)
    return scan, reused
