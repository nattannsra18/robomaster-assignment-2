"""Load Round-1 artifacts and build deterministic Round-2 target routes."""

from __future__ import annotations

from collections import deque
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .target_mission import TargetSpec, parse_target_specs


Cell = Tuple[int, int]
DIR_VEC = {0: (1, 0), 1: (0, -1), 2: (-1, 0), 3: (0, 1)}
DIR_NAME = {0: "FRONT", 1: "RIGHT", 2: "BACK", 3: "LEFT"}


def _cell(value: Sequence[int]) -> Cell:
    if len(value) != 2:
        raise ValueError("cell must contain exactly two coordinates")
    return int(value[0]), int(value[1])


def load_round1_artifacts(run_dir: Path) -> Tuple[dict, dict]:
    run_dir = Path(run_dir)
    topology_path = run_dir / "topology.json"
    targets_path = run_dir / "targets.json"
    if not topology_path.is_file() or not targets_path.is_file():
        raise ValueError("run directory must contain topology.json and targets.json")
    topology = json.loads(topology_path.read_text(encoding="utf-8"))
    targets = json.loads(targets_path.read_text(encoding="utf-8"))
    return topology, targets


def _validate_complete_6x6(topology: dict) -> Set[Cell]:
    if topology.get("finish_reason") != "CLOSED_MAZE_COMPLETE":
        raise ValueError(
            "Round 2 requires a Round-1 run with finish_reason "
            "CLOSED_MAZE_COMPLETE"
        )
    visited = {_cell(value) for value in topology.get("visited_cells", [])}
    if len(visited) != 36:
        raise ValueError("Round 2 requires a complete 6x6 Round-1 map (36 cells)")
    xs = [cell[0] for cell in visited]
    ys = [cell[1] for cell in visited]
    if max(xs) - min(xs) + 1 != 6 or max(ys) - min(ys) + 1 != 6:
        raise ValueError("Round-1 visited cells do not form a filled 6x6 rectangle")
    expected = {
        (x, y)
        for x in range(min(xs), max(xs) + 1)
        for y in range(min(ys), max(ys) + 1)
    }
    if visited != expected:
        raise ValueError("Round-1 6x6 rectangle contains unvisited cells")
    return visited


def build_adjacency(topology: dict, visited: Set[Cell]) -> Dict[Cell, Set[Cell]]:
    adjacency = {cell: set() for cell in visited}
    for raw_x, raw_y, raw_direction in topology.get("open_edges", []):
        cell = int(raw_x), int(raw_y)
        direction = int(raw_direction) % 4
        dx, dy = DIR_VEC[direction]
        neighbour = cell[0] + dx, cell[1] + dy
        if cell in visited and neighbour in visited:
            adjacency[cell].add(neighbour)
            adjacency[neighbour].add(cell)
    return adjacency


def shortest_path(adjacency: Dict[Cell, Set[Cell]], start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if start == goal:
        return [start]
    queue = deque([start])
    previous: Dict[Cell, Optional[Cell]] = {start: None}
    while queue:
        current = queue.popleft()
        for neighbour in sorted(adjacency.get(current, ())):
            if neighbour in previous:
                continue
            previous[neighbour] = current
            if neighbour == goal:
                path = [goal]
                while previous[path[-1]] is not None:
                    path.append(previous[path[-1]])
                return list(reversed(path))
            queue.append(neighbour)
    return None


def route_steps(route: Sequence[Cell], start_facing_direction: int) -> List[dict]:
    """Translate saved-map directions into commands relative to the chassis."""
    steps = []
    facing = int(start_facing_direction) % 4
    vector_to_direction = {vector: direction for direction, vector in DIR_VEC.items()}
    for source, destination in zip(route, route[1:]):
        delta = destination[0] - source[0], destination[1] - source[1]
        if delta not in vector_to_direction:
            raise ValueError("Round-2 route contains non-adjacent cells")
        map_direction = vector_to_direction[delta]
        body_direction = (map_direction - facing) % 4
        steps.append({
            "from_cell": list(source),
            "to_cell": list(destination),
            "map_direction": map_direction,
            "map_direction_name": DIR_NAME[map_direction],
            "body_direction": body_direction,
            "body_direction_name": DIR_NAME[body_direction],
        })
    return steps


def _ready_targets(target_payload: dict, required: Set[TargetSpec]) -> List[dict]:
    matches: List[dict] = []
    by_spec: Dict[TargetSpec, List[dict]] = {spec: [] for spec in required}
    for target in target_payload.get("targets", []):
        spec = TargetSpec(
            str(target.get("color", "")).lower(),
            str(target.get("shape", "")).lower(),
        )
        if spec in by_spec and target.get("round2_position_ready"):
            by_spec[spec].append(target)
    for spec in sorted(required):
        candidates = by_spec[spec]
        if not candidates:
            raise ValueError("no Round-2-ready pose for {}".format(spec.key))
        if len(candidates) > 1:
            raise ValueError(
                "multiple Round-2 targets match {}; select unique color/shape targets".format(
                    spec.key
                )
            )
        matches.append(candidates[0])
    return matches


def build_round2_plan(
    topology: dict,
    target_payload: dict,
    required_specs_text: str,
    start_cell: Optional[Cell] = None,
    start_facing_direction: int = 0,
) -> dict:
    required = parse_target_specs(required_specs_text)
    if not required:
        raise ValueError("Round 2 requires at least one explicit color:shape target")
    visited = _validate_complete_6x6(topology)
    adjacency = build_adjacency(topology, visited)
    start = (
        _cell(topology.get("start_cell", [0, 0]))
        if start_cell is None else _cell(start_cell)
    )
    if start not in visited:
        raise ValueError("Round-2 start cell is outside the saved 6x6 map")
    facing = int(start_facing_direction)
    if facing not in DIR_NAME:
        raise ValueError("Round-2 facing direction must be 0, 1, 2, or 3")
    pending = _ready_targets(target_payload, required)
    current = start
    actions = []
    full_route = [start]

    while pending:
        choices = []
        for target in pending:
            pose = target.get("round2_approach_pose") or {}
            approach = _cell(pose.get("cell", []))
            route = shortest_path(adjacency, current, approach)
            if route is not None:
                choices.append((len(route), str(target["target_id"]), target, route, pose))
        if not choices:
            raise ValueError("a selected Round-2 target is unreachable on the saved topology")
        _length, _target_id, target, route, pose = min(choices, key=lambda item: item[:2])
        map_view_direction = int(pose["view_direction"]) % 4
        actions.append({
            "target_id": str(target["target_id"]),
            "color": str(target["color"]),
            "shape": str(target["shape"]),
            "approach_cell": list(_cell(pose["cell"])),
            "view_direction": map_view_direction,
            "map_view_direction": map_view_direction,
            "map_view_direction_name": DIR_NAME[map_view_direction],
            "body_view_direction": (map_view_direction - facing) % 4,
            "body_view_direction_name": DIR_NAME[
                (map_view_direction - facing) % 4
            ],
            "route": [list(cell) for cell in route],
            "route_steps": route_steps(route, facing),
        })
        full_route.extend(route[1:])
        current = _cell(pose["cell"])
        pending.remove(target)

    return {
        "version": 2,
        "source_finish_reason": topology.get("finish_reason"),
        "start_cell": list(start),
        "start_facing_direction": facing,
        "start_facing_direction_name": DIR_NAME[facing],
        "required_targets": sorted(spec.key for spec in required),
        "actions": actions,
        "full_route": [list(cell) for cell in full_route],
        "full_route_steps": route_steps(full_route, facing),
    }


def load_round2_plan(
    run_dir: Path,
    required_specs_text: str,
    start_cell: Optional[Cell] = None,
    start_facing_direction: int = 0,
) -> dict:
    topology, targets = load_round1_artifacts(run_dir)
    return build_round2_plan(
        topology,
        targets,
        required_specs_text,
        start_cell=start_cell,
        start_facing_direction=start_facing_direction,
    )


def save_round2_plan(plan: dict, output_path: Path) -> Path:
    output_path = Path(output_path)
    output_path.write_text(
        json.dumps(plan, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return output_path
