import unittest

from classwork8.config import Classwork8Config
from classwork8.target_mission import (
    TargetMission,
    TargetMissionState,
    parse_target_specs,
)


class FakeBlaster:
    def __init__(self, acknowledged=True):
        self.acknowledged = acknowledged
        self.calls = []

    def fire(self, fire_type, times):
        self.calls.append((fire_type, times))
        return self.acknowledged


class TargetMissionP0Tests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.target_required_specs = "blue:circle"

    def assess(self, mission, **overrides):
        values = {
            "target": {"target_id": "T01", "color": "blue", "shape": "circle"},
            "centroid_px": (320, 180),
            "frame_size_px": (640, 360),
            "tof_cm": 45.0,
            "range_confirmed": True,
            "aim_confirmed": True,
        }
        values.update(overrides)
        target = values.pop("target")
        return mission.assess(target, **values)

    def test_selection_parser_is_explicit_and_strict(self):
        specs = parse_target_specs("blue:circle, red:rectangle")
        self.assertEqual({item.key for item in specs}, {
            "blue:circle", "red:rectangle",
        })
        with self.assertRaises(ValueError):
            parse_target_specs("all")

    def test_unselected_or_unconfirmed_target_never_fires(self):
        mission = TargetMission(self.config)
        unselected = self.assess(
            mission,
            target={"target_id": "T02", "color": "green", "shape": "square"},
        )
        self.assertEqual(unselected.state, TargetMissionState.NOT_SELECTED)
        unconfirmed = self.assess(mission, range_confirmed=False)
        self.assertEqual(unconfirmed.state, TargetMissionState.RANGE_UNCONFIRMED)

    def test_two_cell_limit_and_center_gate(self):
        mission = TargetMission(self.config)
        out_of_range = self.assess(mission, tof_cm=120.0)
        self.assertEqual(out_of_range.state, TargetMissionState.OUT_OF_RANGE)
        off_center = self.assess(mission, centroid_px=(500, 180))
        self.assertEqual(off_center.state, TargetMissionState.NEEDS_AIM)

    def test_default_is_dry_run(self):
        mission = TargetMission(self.config)
        decision = self.assess(mission)
        self.assertEqual(decision.state, TargetMissionState.READY_DRY_RUN)
        self.assertFalse(decision.should_fire)

    def test_fresh_auto_aim_confirmation_is_required(self):
        mission = TargetMission(self.config)
        decision = self.assess(mission, aim_confirmed=False)
        self.assertEqual(decision.state, TargetMissionState.NEEDS_AIM)
        self.assertFalse(decision.should_fire)

    def test_calibrated_centroid_offset_is_used(self):
        self.config.target_aim_offset_x_ratio = 0.10
        mission = TargetMission(self.config)
        at_image_center = self.assess(
            mission,
            centroid_px=(320, 180),
            aim_confirmed=True,
        )
        self.assertEqual(at_image_center.state, TargetMissionState.NEEDS_AIM)
        calibrated_point = self.assess(
            mission,
            centroid_px=(384, 180),
            aim_confirmed=True,
        )
        self.assertEqual(
            calibrated_point.state, TargetMissionState.READY_DRY_RUN
        )

    def test_per_engagement_impact_offset_is_used_by_final_fire_gate(self):
        mission = TargetMission(self.config)
        decision = self.assess(
            mission,
            centroid_px=(320, 225),
            aim_offset_x_ratio=0.0,
            aim_offset_y_ratio=0.125,
        )
        self.assertEqual(decision.state, TargetMissionState.READY_DRY_RUN)

    def test_armed_selected_target_fires_once_per_unique_spec(self):
        self.config.target_fire_enabled = True
        self.config.target_fire_mode = "selected"
        mission = TargetMission(self.config)
        blaster = FakeBlaster()
        decision = self.assess(mission)
        self.assertTrue(decision.should_fire)
        self.assertTrue(mission.fire(decision, blaster))
        self.assertEqual(blaster.calls, [("ir", 3)])
        duplicate = self.assess(
            mission,
            target={"target_id": "T99", "color": "blue", "shape": "circle"},
        )
        self.assertEqual(duplicate.state, TargetMissionState.ALREADY_FIRED)

    def test_all_mode_fires_each_verified_target_id_with_ir(self):
        self.config.target_fire_enabled = True
        self.config.target_fire_mode = "all"
        self.config.target_required_specs = ""
        self.config.validate()
        mission = TargetMission(self.config)
        blaster = FakeBlaster()
        first = self.assess(mission)
        self.assertTrue(mission.fire(first, blaster))
        second = self.assess(
            mission,
            target={"target_id": "T02", "color": "blue", "shape": "circle"},
        )
        self.assertTrue(mission.fire(second, blaster))
        self.assertEqual(blaster.calls, [("ir", 3), ("ir", 3)])
        duplicate_id = self.assess(mission)
        self.assertEqual(duplicate_id.state, TargetMissionState.ALREADY_FIRED)

    def test_config_rejects_armed_empty_selection_and_range_above_two_cells(self):
        self.config.target_fire_enabled = True
        self.config.target_fire_mode = "selected"
        self.config.target_required_specs = ""
        with self.assertRaises(ValueError):
            self.config.validate()
        self.config.target_required_specs = "blue:circle"
        self.config.target_max_fire_distance_cells = 2.1
        with self.assertRaises(ValueError):
            self.config.validate()

    def test_stationary_target_test_requires_detection(self):
        self.config.target_fire_enabled = False
        self.config.stationary_target_test = True
        self.config.target_detection_enabled = False
        with self.assertRaisesRegex(ValueError, "requires target detection"):
            self.config.validate()


if __name__ == "__main__":
    unittest.main()
