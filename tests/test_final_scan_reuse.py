"""Offline scan-cache regression for Final Round 1 V05.

No robot, chassis command, or camera connection is required.
"""

import inspect
import sys
import types
import unittest


if "libmedia_codec" not in sys.modules:
    media = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media.H264Decoder = H264Decoder
    media.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media


from classwork8.config import Classwork8Config
from classwork8.tof_camera_round1_v05 import (
    _directions_requiring_scan,
    _plan_unknown_rescan_move,
    _should_reuse_scan,
)


class ScanReuseTests(unittest.TestCase):
    def setUp(self):
        self.cell = (2, -1)
        self.edges = {
            (2, -1, 0): "WALL",
            (2, -1, 1): "OPEN",
            (2, -1, 2): "OPEN",
            (2, -1, 3): "WALL",
        }

    def test_first_visit_never_skips_scan(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, set(), self.edges, True, False
        ))

    def test_completed_visited_cell_can_reuse_scan(self):
        self.assertTrue(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, True, False
        ))

    def test_operator_can_disable_reuse(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, False, False
        ))

    def test_rescan_request_is_always_honored(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, True, True
        ))

    def test_missing_or_unknown_edge_requires_fresh_scan(self):
        edges = dict(self.edges)
        edges.pop((2, -1, 1))
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, edges, True, False
        ))
        edges[(2, -1, 1)] = "UNKNOWN"
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, edges, True, False
        ))

    def test_unknown_edges_and_known_wall_faces_get_a_physical_scan(self):
        scan, reused = _directions_requiring_scan(
            self.cell,
            [3, 0, 1, 2],
            {
                (2, -1, 0): "WALL",
                (2, -1, 1): "OPEN",
                (2, -1, 2): "UNKNOWN",
            },
            set(),
        )
        self.assertEqual(scan, [3, 0, 2])
        self.assertEqual(reused, [1])

    def test_just_traversed_edge_is_reused_even_without_cached_state(self):
        traversed = {((1, -1), (2, -1))}
        scan, reused = _directions_requiring_scan(
            self.cell,
            [3, 0, 1, 2],
            {},
            traversed,
        )
        self.assertEqual(scan, [3, 0, 1])
        self.assertEqual(reused, [2])

    def test_wall_faces_bypass_budget_but_still_use_quick_gate(self):
        from classwork8 import tof_camera_round1_v05 as mission
        source = inspect.getsource(mission._scan_four_directions)
        self.assertIn("wall_face = near_wall or known_wall_face", source)
        self.assertIn("_quick_target_candidate_or_false(", source)
        self.assertNotIn("[TARGET_WALL_VERIFY]", source)

    def test_planner_routes_to_other_visited_unknown_cell(self):
        current = (0, 0)
        rescan = (1, 0)
        visited = {current, rescan}
        edges = {
            (0, 0, 0): "OPEN",
            (1, 0, 2): "OPEN",
            (0, 0, 1): "WALL",
            (0, 0, 2): "WALL",
            (0, 0, 3): "WALL",
            (1, 0, 0): "UNKNOWN",
            (1, 0, 1): "WALL",
            (1, 0, 3): "WALL",
        }
        plan = _plan_unknown_rescan_move(
            current,
            visited,
            edges,
            set(),
            Classwork8Config(),
            0,
        )
        self.assertIsNotNone(plan)
        self.assertEqual(plan["mode"], "RELOCATE_RESCAN")
        self.assertEqual(plan["next_cell"], rescan)

    def test_planner_does_not_immediately_loop_on_current_failed_scan(self):
        plan = _plan_unknown_rescan_move(
            (0, 0),
            {(0, 0)},
            {(0, 0, 0): "UNKNOWN"},
            set(),
            Classwork8Config(),
            0,
        )
        self.assertIsNone(plan)

    def test_planner_does_not_revisit_exhausted_unknown_cell(self):
        current = (0, 0)
        rescan = (1, 0)
        plan = _plan_unknown_rescan_move(
            current,
            {current, rescan},
            {
                (0, 0, 0): "OPEN",
                (1, 0, 2): "OPEN",
                (1, 0, 0): "UNKNOWN",
            },
            set(),
            Classwork8Config(),
            0,
            {rescan},
        )
        self.assertIsNone(plan)


if __name__ == "__main__":
    unittest.main()
