"""Round 2 route planning from the exact Round 1 JSON export format.

This module has no SDK dependency. It visits recorded camera observation cells,
never estimated target coordinates or unconfirmed sighting-cell hints.
"""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path

VECTORS = ((1, 0), (0, -1), (-1, 0), (0, 1))


def cell(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2 or any(
        type(v) is not int for v in value
    ):
        raise ValueError(f"Invalid map cell: {value!r}")
    return tuple(value)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


class SavedMap:
    def __init__(self, topology):
        if topology.get("version") != 1:
            raise ValueError("Expected Round 1 topology version 1")
        self.cell_size = float(topology["cell_size_m"])
        if not math.isfinite(self.cell_size) or self.cell_size <= 0:
            raise ValueError("Invalid cell_size_m")
        self.start = cell(topology["start_cell"])
        self.visited = {cell(c) for c in topology["visited_cells"]}
        if self.start not in self.visited:
            raise ValueError("Start cell is not in visited_cells")
        self.graph = {c: set() for c in self.visited}

        def edges(rows):
            result = set()
            for row in rows:
                if len(row) != 3 or type(row[2]) is not int or row[2] not in range(4):
                    raise ValueError(f"Invalid edge: {row!r}")
                a = cell(row[:2])
                dx, dy = VECTORS[row[2]]
                b = (a[0] + dx, a[1] + dy)
                result.add(frozenset((a, b)))
            return result

        opened = edges(topology["open_edges"])
        walls = edges(topology["wall_edges"])
        if opened & walls:
            raise ValueError("Map contains contradictory OPEN/WALL edges")
        for edge in opened:
            a, b = tuple(edge)
            if a in self.visited and b in self.visited:
                self.graph[a].add(b)
                self.graph[b].add(a)

    def path(self, start, goal):
        if start not in self.graph or goal not in self.graph:
            return None
        parents = {start: None}
        queue = deque([start])
        while queue:
            node = queue.popleft()
            if node == goal:
                route = []
                while node is not None:
                    route.append(node)
                    node = parents[node]
                return route[::-1]
            for neighbor in sorted(self.graph[node]):
                if neighbor not in parents:
                    parents[neighbor] = node
                    queue.append(neighbor)
        return None


def plan_round2(run_dir, target_ids=None, return_home=False):
    run_dir = Path(run_dir)
    maze = SavedMap(read_json(run_dir / "topology.json"))
    payload = read_json(run_dir / "targets.json")
    # Version 3 retains the same reference_views schema and adds approach/aim
    # metadata. Navigation uses only recorded camera views in either version.
    if payload.get("version") not in (2, 3):
        raise ValueError("Expected Round 1 targets version 2 or 3")
    targets = payload["targets"]
    ids = [t["target_id"] for t in targets]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate target IDs")
    if target_ids:
        unknown = set(target_ids) - set(ids)
        if unknown:
            raise ValueError(f"Unknown target IDs: {sorted(unknown)}")
        targets = [t for t in targets if t["target_id"] in target_ids]
    current = maze.start
    pending = list(targets)
    stops, skipped = [], []
    while pending:
        candidates = []
        for index, target in enumerate(pending):
            for view in target.get("reference_views", []):
                destination = cell(view["approach_cell"])
                direction = view["view_direction"]
                if type(direction) is not int or direction not in range(4):
                    raise ValueError("Invalid target view direction")
                route = maze.path(current, destination)
                if route is not None:
                    candidates.append((len(route), index, direction, route, view))
        if not candidates:
            skipped.extend({"target_id": t["target_id"],
                            "status": "NO_REACHABLE_REFERENCE_VIEW"} for t in pending)
            break
        _, index, direction, route, view = min(candidates, key=lambda c: c[:3])
        target = pending.pop(index)
        stops.append({"target_id": target["target_id"], "color": target["color"],
                      "shape": target["shape"], "route": route,
                      "view_direction": direction,
                      "centroid_px": view.get("centroid_px"),
                      "localization_status": target.get("localization_status")})
        current = route[-1]
    return {"source_run": str(run_dir.resolve()), "cell_size_m": maze.cell_size,
            "start_cell": maze.start, "stops": stops, "skipped": skipped,
            "return_route": maze.path(current, maze.start) if return_home else [],
            "action": "CAMERA_REVALIDATION_ONLY"}
