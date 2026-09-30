import unittest

from classwork8.config import Classwork8Config
from round1_assignment import _prepare_optional_media_codec

_prepare_optional_media_codec()

from classwork8.tof_camera_round1_v05 import (
    _closed_maze_completion_v04,
    _inside_working_canvas,
)


def cells(width, height):
    return {(x, y) for x in range(width) for y in range(height)}


class ExactCompletionTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.assignment_maze_rows = 6
        self.config.assignment_maze_cols = 6

    def status(self, visited, edge_states=None):
        return _closed_maze_completion_v04(
            visited,
            edge_states or {},
            set(),
            self.config,
        )

    def test_inferred_smaller_rectangle_never_completes(self):
        result = self.status(cells(5, 5))
        self.assertFalse(result["complete"])
        self.assertFalse(result["filled"])

    def test_exact_6x6_completes_without_perimeter_reflection_gate(self):
        result = self.status(cells(6, 6))
        self.assertTrue(result["filled"])
        self.assertTrue(result["complete"])
        self.assertEqual((result["rows"], result["cols"]), (6, 6))

    def test_36_cells_in_wrong_shape_never_complete(self):
        result = self.status(cells(9, 4))
        self.assertFalse(result["complete"])

    def test_planner_bound_is_translated_6x6_not_eight_meter_canvas(self):
        visited = {(x, y) for x in range(-2, 4) for y in range(1, 7)}
        self.assertTrue(_inside_working_canvas((-2, 1), self.config, visited))
        self.assertFalse(_inside_working_canvas((4, 3), self.config, visited))
        self.assertFalse(_inside_working_canvas((0, 7), self.config, visited))


if __name__ == "__main__":
    unittest.main()
