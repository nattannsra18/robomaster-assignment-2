import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from classwork8.round2_mission import build_round2_plan, route_steps
from final_round2_target_plan_01 import main


def complete_topology():
    cells = [(x, y) for x in range(6) for y in range(6)]
    open_edges = []
    for x, y in cells:
        if x < 5:
            open_edges.extend(([x, y, 0], [x + 1, y, 2]))
        if y < 5:
            open_edges.extend(([x, y, 3], [x, y + 1, 1]))
    return {
        "start_cell": [0, 0],
        "finish_reason": "CLOSED_MAZE_COMPLETE",
        "visited_cells": [list(cell) for cell in cells],
        "open_edges": open_edges,
    }


def ready_target(target_id, color, shape, cell, direction):
    return {
        "target_id": target_id,
        "color": color,
        "shape": shape,
        "round2_position_ready": True,
        "round2_approach_pose": {
            "cell": list(cell),
            "view_direction": direction,
        },
    }


class Round2MissionTests(unittest.TestCase):
    def test_loads_selected_targets_and_uses_nearest_next_route(self):
        targets = {"targets": [
            ready_target("T01", "blue", "circle", (5, 5), 2),
            ready_target("T02", "green", "square", (1, 0), 3),
        ]}
        plan = build_round2_plan(
            complete_topology(), targets, "blue:circle,green:square"
        )
        self.assertEqual([item["target_id"] for item in plan["actions"]], [
            "T02", "T01",
        ])
        self.assertEqual(plan["full_route"][0], [0, 0])
        self.assertEqual(plan["full_route"][-1], [5, 5])

    def test_explicit_round2_start_cell_overrides_saved_start(self):
        targets = {"targets": [
            ready_target("T01", "blue", "circle", (5, 5), 2),
        ]}
        plan = build_round2_plan(
            complete_topology(), targets, "blue:circle", start_cell=(4, 5)
        )
        self.assertEqual(plan["start_cell"], [4, 5])
        self.assertEqual(plan["full_route"], [[4, 5], [5, 5]])

    def test_facing_converts_map_route_to_body_relative_commands(self):
        # Physical front points toward map RIGHT. Moving toward map FRONT is
        # therefore a body-relative LEFT strafe, not a forward command.
        steps = route_steps([(0, 0), (1, 0)], start_facing_direction=1)
        self.assertEqual(steps[0]["map_direction_name"], "FRONT")
        self.assertEqual(steps[0]["body_direction_name"], "LEFT")
        targets = {"targets": [
            ready_target("T01", "blue", "circle", (1, 0), 0),
        ]}
        plan = build_round2_plan(
            complete_topology(), targets, "blue:circle",
            start_facing_direction=1,
        )
        self.assertEqual(plan["start_facing_direction_name"], "RIGHT")
        self.assertEqual(
            plan["actions"][0]["body_view_direction_name"], "LEFT"
        )

    def test_rejects_incomplete_map(self):
        topology = complete_topology()
        topology["visited_cells"].pop()
        with self.assertRaisesRegex(ValueError, "36 cells"):
            build_round2_plan(topology, {"targets": []}, "blue:circle")

    def test_rejects_missing_or_ambiguous_selected_target(self):
        with self.assertRaisesRegex(ValueError, "no Round-2-ready"):
            build_round2_plan(complete_topology(), {"targets": []}, "blue:circle")
        duplicate = {"targets": [
            ready_target("T01", "blue", "circle", (1, 1), 0),
            ready_target("T02", "blue", "circle", (2, 2), 0),
        ]}
        with self.assertRaisesRegex(ValueError, "multiple Round-2"):
            build_round2_plan(complete_topology(), duplicate, "blue:circle")

    def test_rejects_run_not_confirmed_complete(self):
        topology = complete_topology()
        topology["finish_reason"] = "MAX_MOVES_REACHED"
        with self.assertRaisesRegex(ValueError, "CLOSED_MAZE_COMPLETE"):
            build_round2_plan(topology, {"targets": []}, "blue:circle")

    def test_plan_cli_loads_artifacts_and_writes_json(self):
        with TemporaryDirectory() as folder:
            run_dir = Path(folder)
            (run_dir / "topology.json").write_text(
                json.dumps(complete_topology()), encoding="utf-8"
            )
            (run_dir / "targets.json").write_text(json.dumps({"targets": [
                ready_target("T01", "blue", "circle", (2, 0), 0),
            ]}), encoding="utf-8")
            self.assertEqual(main([
                "--run-dir", str(run_dir),
                "--targets", "blue:circle",
            ]), 0)
            saved = json.loads(
                (run_dir / "round2_plan.json").read_text(encoding="utf-8")
            )
            self.assertEqual(saved["actions"][0]["target_id"], "T01")

    def test_plan_cli_can_start_from_saved_final_cell_with_facing(self):
        with TemporaryDirectory() as folder:
            run_dir = Path(folder)
            topology = complete_topology()
            topology["final_cell"] = [4, 0]
            (run_dir / "topology.json").write_text(
                json.dumps(topology), encoding="utf-8"
            )
            (run_dir / "targets.json").write_text(json.dumps({"targets": [
                ready_target("T01", "blue", "circle", (5, 0), 0),
            ]}), encoding="utf-8")
            self.assertEqual(main([
                "--run-dir", str(run_dir),
                "--targets", "blue:circle",
                "--start-from", "final",
                "--facing", "right",
            ]), 0)
            saved = json.loads(
                (run_dir / "round2_plan.json").read_text(encoding="utf-8")
            )
            self.assertEqual(saved["start_cell"], [4, 0])
            self.assertEqual(saved["start_facing_direction_name"], "RIGHT")
            self.assertEqual(
                saved["full_route_steps"][0]["body_direction_name"], "LEFT"
            )


if __name__ == "__main__":
    unittest.main()
