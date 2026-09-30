import itertools
import unittest

from classwork8.scan_schedule import (
    field_scan_order,
    omit_completed_wall_surveys,
    shortest_scan_order,
)


class ScanScheduleTests(unittest.TestCase):
    angles = {0: 0.0, 1: 90.0, 2: 180.0, 3: -90.0}

    def cost(self, yaw, order):
        points = [yaw] + [self.angles[d] for d in order]
        return sum(abs(b - a) for a, b in zip(points, points[1:]))

    def test_every_subset_matches_exhaustive_minimum(self):
        for count in range(1, 5):
            for subset in itertools.combinations(range(4), count):
                for yaw in (-180, -90, -5, 0, 46, 90, 135, 180, 240):
                    order = shortest_scan_order(subset, yaw, self.angles.__getitem__)
                    expected = min(self.cost(yaw, p) for p in itertools.permutations(subset))
                    self.assertCountEqual(order, subset)
                    self.assertEqual(self.cost(yaw, order), expected)

    def test_subset_avoids_old_endpoint_detour(self):
        old = [3, 0, 1]
        new = shortest_scan_order(old, 40, self.angles.__getitem__)
        self.assertEqual(new, [1, 0, 3])
        self.assertEqual(self.cost(40, old) - self.cost(40, new), 80)

    def test_missing_feedback_preserves_order(self):
        for yaw in (None, float('nan')):
            self.assertEqual(shortest_scan_order([2, 0], yaw, self.angles.__getitem__), [2, 0])
        self.assertEqual(shortest_scan_order([], 0, self.angles.__getitem__), [])

    def test_field_three_face_order_is_back_left_right(self):
        self.assertEqual(
            field_scan_order([2, 1, 3], 178.2, self.angles.__getitem__),
            [2, 3, 1],
        )

    def test_other_field_subsets_still_minimize_yaw_travel(self):
        self.assertEqual(
            field_scan_order([3, 0, 1], 40, self.angles.__getitem__),
            [1, 0, 3],
        )

    def test_completed_face_only_and_retry_preserved(self):
        node = (1, 2)
        edges = {(1, 2, d): 'WALL' for d in range(4)}
        edges[(1, 2, 2)] = 'UNKNOWN'
        completed = {(node, 0), (node, 1), (node, 2), ((2, 2), 3)}
        scan, reused = omit_completed_wall_surveys(
            [0, 1, 2, 3], node, edges, completed, {(node, 1)})
        self.assertEqual(scan, [1, 2, 3])
        self.assertEqual(reused, [0])

    def test_first_visit_keeps_all_faces(self):
        scan, reused = omit_completed_wall_surveys(
            [0, 1, 2, 3], (0, 0), {(0, 0, 0): 'WALL'}, set(), set())
        self.assertEqual(scan, [0, 1, 2, 3])
        self.assertEqual(reused, [])
