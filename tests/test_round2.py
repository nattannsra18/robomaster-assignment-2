import json
from pathlib import Path
import tempfile
import unittest

from classwork8.round2 import SavedMap, plan_round2


def topology():
    return {"version": 1, "cell_size_m": 0.6, "start_cell": [0, 0],
            "visited_cells": [[0, 0], [1, 0], [1, -1], [4, 0]],
            "open_edges": [[0, 0, 0], [1, 0, 1], [1, -1, 1]],
            "wall_edges": [[0, 0, 1]]}


def target(identifier, position, direction=0):
    return {"target_id": identifier, "color": "red", "shape": "circle",
            "localization_status": "SIGHTING_ONLY",
            "estimated_target_xy_m": [100, 100],
            "sighting_cell_hint": [99, 99],
            "reference_views": [{"approach_cell": position,
                                 "view_direction": direction,
                                 "centroid_px": [100, 100]}]}


class Round2PlannerTests(unittest.TestCase):
    def plan(self, targets, version=2, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "topology.json").write_text(json.dumps(topology()), encoding="utf-8")
            (root / "targets.json").write_text(json.dumps({"version": version, "targets": targets}), encoding="utf-8")
            return plan_round2(root, **kwargs)

    def test_bfs_respects_map_axes_and_visited_cells(self):
        maze = SavedMap(topology())
        self.assertEqual(maze.path((0, 0), (1, -1)), [(0, 0), (1, 0), (1, -1)])
        self.assertEqual(maze.path((1, -1), (0, 0)), [(1, -1), (1, 0), (0, 0)])
        self.assertIsNone(maze.path((0, 0), (1, -2)))
        self.assertIsNone(maze.path((0, 0), (4, 0)))

    def test_wall_conflict_in_reverse_direction_rejected(self):
        data = topology()
        data["wall_edges"].append([1, 0, 2])
        with self.assertRaisesRegex(ValueError, "contradictory"):
            SavedMap(data)

    def test_nearest_visit_return_home_and_unreachable(self):
        plan = self.plan([target("far", [1, -1], 3), target("near", [1, 0]),
                          target("unreachable", [4, 0])], return_home=True)
        self.assertEqual([s["target_id"] for s in plan["stops"]], ["near", "far"])
        self.assertEqual(plan["return_route"], [(1, -1), (1, 0), (0, 0)])
        self.assertEqual(plan["skipped"][0]["target_id"], "unreachable")
        self.assertEqual(plan["stops"][1]["view_direction"], 3)

    def test_unknown_coordinates_never_used_as_destinations(self):
        plan = self.plan([target("one", [0, 0])])
        self.assertEqual(plan["stops"][0]["route"], [(0, 0)])

    def test_filter_and_unknown_id(self):
        targets = [target("one", [0, 0]), target("two", [1, 0])]
        self.assertEqual(len(self.plan(targets, target_ids=["two"])["stops"]), 1)
        with self.assertRaisesRegex(ValueError, "Unknown target"):
            self.plan(targets, target_ids=["typo"])

    def test_duplicate_ids_rejected(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.plan([target("one", [0, 0]), target("one", [1, 0])])

    def test_no_reference_view_is_skipped(self):
        item = target("one", [0, 0])
        item["reference_views"] = []
        plan = self.plan([item])
        self.assertFalse(plan["stops"])
        self.assertEqual(len(plan["skipped"]), 1)

    def test_empty_targets(self):
        plan = self.plan([], return_home=True)
        self.assertEqual(plan["stops"], [])
        self.assertEqual(plan["return_route"], [(0, 0)])

    def test_version_three_uses_reference_views(self):
        plan = self.plan([target("one", [1, 0])], version=3)
        self.assertEqual(plan["stops"][0]["route"], [(0, 0), (1, 0)])

    def test_unknown_schema_rejected(self):
        with self.assertRaisesRegex(ValueError, "version"):
            self.plan([], version=99)


if __name__ == "__main__":
    unittest.main()
