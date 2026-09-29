import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from final_round1_tof_camera_01 import _prepare_optional_media_codec

_prepare_optional_media_codec()

from classwork8.config import Classwork8Config
from classwork8.round2_executor import (
    _engage_physical_target,
    execute_round2_plan,
    load_round1_config,
    load_verified_execution_plan,
    validate_execution_plan,
)
from classwork8.round2_mission import build_round2_plan, save_round2_plan
from classwork8.target_aim import AimResult
from classwork8.target_mission import TargetMission
from final_round2_target_execute_01 import main


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
        "final_cell": [5, 5],
        "finish_reason": "CLOSED_MAZE_COMPLETE",
        "visited_cells": [list(cell) for cell in cells],
        "open_edges": open_edges,
    }


def ready_target(target_id, color, shape, cell, direction, centroid=(320, 180)):
    return {
        "target_id": target_id,
        "color": color,
        "shape": shape,
        "round2_position_ready": True,
        "round2_approach_pose": {
            "cell": list(cell),
            "view_direction": direction,
        },
        "reference_views": [{
            "approach_cell": list(cell),
            "view_direction": direction,
            "centroid_px": list(centroid),
        }],
    }


def two_target_plan():
    return build_round2_plan(
        complete_topology(),
        {"targets": [
            ready_target("T01", "blue", "circle", (1, 0), 0),
            ready_target("T02", "green", "square", (1, 1), 1),
        ]},
        "blue:circle,green:square",
        start_facing_direction=1,
    )


class Round2ExecutorSequenceTests(unittest.TestCase):
    def test_executes_each_route_before_revalidating_its_target(self):
        plan = two_target_plan()
        calls = []

        def move(step, index):
            calls.append(("move", index, tuple(step["to_cell"])))
            return True, "CELL_COMPLETE"

        def engage(action):
            calls.append(("target", action["target_id"]))
            return True, "TARGET_READY_DRY_RUN"

        result = execute_round2_plan(plan, move, engage)
        self.assertTrue(result.completed)
        self.assertEqual(result.reason, "ROUND2_COMPLETE")
        self.assertEqual(result.targets_completed, ("T01", "T02"))
        self.assertEqual(calls, [
            ("move", 0, (1, 0)),
            ("target", "T01"),
            ("move", 1, (1, 1)),
            ("target", "T02"),
        ])

    def test_motion_failure_aborts_before_target_or_later_commands(self):
        calls = []

        def move(step, index):
            calls.append(("move", index))
            return False, "PREFLIGHT_BLOCKED"

        result = execute_round2_plan(
            two_target_plan(), move, lambda action: calls.append(("target", action))
        )
        self.assertFalse(result.completed)
        self.assertEqual(result.reason, "PREFLIGHT_BLOCKED")
        self.assertEqual(calls, [("move", 0)])

    def test_target_failure_retries_then_continues_to_later_target(self):
        calls = []

        def move(step, index):
            calls.append(("move", index))
            return True, "CELL_COMPLETE"

        def engage(action):
            calls.append(("target", action["target_id"]))
            if action["target_id"] == "T01":
                return False, "TARGET_NOT_REVALIDATED"
            return True, "TARGET_READY_DRY_RUN"

        result = execute_round2_plan(two_target_plan(), move, engage)
        self.assertFalse(result.completed)
        self.assertIn("ROUND2_PARTIAL_TARGET_FAILURES:T01", result.reason)
        self.assertEqual(result.targets_completed, ("T02",))
        self.assertEqual(calls, [
            ("move", 0),
            ("target", "T01"),
            ("target", "T01"),
            ("move", 1),
            ("target", "T02"),
        ])

    def test_step_limit_is_a_wheel_command_boundary(self):
        calls = []
        result = execute_round2_plan(
            two_target_plan(),
            lambda step, index: (calls.append(index) or (True, "CELL_COMPLETE")),
            lambda action: (True, "TARGET_READY_DRY_RUN"),
            max_route_steps=1,
        )
        self.assertFalse(result.completed)
        self.assertEqual(result.reason, "ROUTE_STEP_LIMIT_REACHED")
        self.assertEqual(calls, [0])

    def test_tampered_body_direction_is_rejected_before_callbacks(self):
        plan = two_target_plan()
        plan["actions"][0]["route_steps"][0]["body_direction"] = 0
        with self.assertRaisesRegex(ValueError, "body direction"):
            validate_execution_plan(plan)


class Round2ArtifactTests(unittest.TestCase):
    def make_run(self, folder):
        run_dir = Path(folder)
        topology = complete_topology()
        targets = {"targets": [
            ready_target("T01", "blue", "circle", (1, 0), 0),
        ]}
        (run_dir / "topology.json").write_text(
            json.dumps(topology), encoding="utf-8"
        )
        (run_dir / "targets.json").write_text(
            json.dumps(targets), encoding="utf-8"
        )
        config = Classwork8Config()
        config.odom_scale_x = 1.07
        (run_dir / "summary.json").write_text(
            json.dumps({"config": config.to_dict()}), encoding="utf-8"
        )
        plan = build_round2_plan(topology, targets, "blue:circle")
        plan_path = save_round2_plan(plan, run_dir / "round2_plan.json")
        return run_dir, plan_path

    def test_saved_plan_is_rebuilt_and_compared_with_round1_artifacts(self):
        with TemporaryDirectory() as folder:
            run_dir, plan_path = self.make_run(folder)
            plan = load_verified_execution_plan(run_dir, plan_path)
            self.assertEqual(plan["version"], 3)
            saved = json.loads(plan_path.read_text(encoding="utf-8"))
            saved["actions"][0]["body_view_direction"] = 3
            plan_path.write_text(json.dumps(saved), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no longer matches"):
                load_verified_execution_plan(run_dir, plan_path)

    def test_round2_reuses_round1_calibration(self):
        with TemporaryDirectory() as folder:
            run_dir, _plan_path = self.make_run(folder)
            config = load_round1_config(run_dir)
            self.assertAlmostEqual(config.odom_scale_x, 1.07)

    def test_cli_defaults_to_validation_only_without_robot_connection(self):
        with TemporaryDirectory() as folder:
            run_dir, plan_path = self.make_run(folder)
            with patch(
                "final_round2_target_execute_01.run_round2_physical"
            ) as physical:
                code = main([
                    "--run-dir", str(run_dir),
                    "--plan", str(plan_path),
                ])
            self.assertEqual(code, 0)
            physical.assert_not_called()


class FakeDetector:
    def __init__(self, verified):
        self.verified = verified

    def verify_latest(self, camera, not_before=None):
        return list(self.verified), np.zeros((360, 640, 3), dtype=np.uint8)


class FakeAim:
    def __init__(self, detection):
        self.detection = detection

    def aim(self, **kwargs):
        return AimResult(
            success=True,
            reason="AIM_SETTLED",
            detection=self.detection,
            frame_size_px=(640, 360),
            debug_frame=None,
            fresh_frames=3,
            final_pitch_deg=-20.0,
            final_yaw_deg=0.0,
        )


class FakeBlaster:
    def __init__(self):
        self.calls = []

    def fire(self, fire_type, times):
        self.calls.append((fire_type, times))
        return True


class FakeRecorder:
    def __init__(self):
        self.events = []

    def event(self, *args, **kwargs):
        self.events.append((args, kwargs))


class Round2LiveTargetGateTests(unittest.TestCase):
    def detection(self, color="blue", shape="circle"):
        detection = SimpleNamespace(
            color=color,
            shape=shape,
            centroid=(320, 180),
        )
        return SimpleNamespace(detection=detection, confidence=0.95)

    def test_live_exact_match_range_and_aim_precede_real_fire(self):
        config = Classwork8Config()
        config.target_required_specs = "blue:circle"
        config.target_fire_enabled = True
        mission = TargetMission(config)
        verified = self.detection()
        blaster = FakeBlaster()
        action = {
            "target_id": "T01",
            "color": "blue",
            "shape": "circle",
            "body_view_direction": 0,
            "expected_centroid_px": [320, 180],
        }
        with patch.multiple(
            "classwork8.round2_executor",
            stop_chassis=lambda chassis: None,
            _point_gimbal=lambda *args, **kwargs: True,
            _wait_for_fresh_tof=lambda *args, **kwargs: 30.0,
            _set_camera_observation_pitch=lambda *args, **kwargs: True,
        ):
            ok, reason = _engage_physical_target(
                action,
                chassis=object(),
                gimbal=object(),
                blaster=blaster,
                sensors=object(),
                gimbal_tracker=object(),
                camera_service=object(),
                detector=FakeDetector([verified]),
                mission=mission,
                auto_aim=FakeAim(verified.detection),
                config=config,
                recorder=FakeRecorder(),
                stop_event=SimpleNamespace(is_set=lambda: False),
            )
        self.assertTrue(ok, reason)
        self.assertEqual(reason, "TARGET_FIRE_ACKNOWLEDGED")
        self.assertEqual(blaster.calls, [("ir", 1)])

    def test_wrong_live_target_never_reaches_auto_aim_or_fire(self):
        config = Classwork8Config()
        config.target_required_specs = "blue:circle"
        config.target_fire_enabled = True
        mission = TargetMission(config)
        blaster = FakeBlaster()
        wrong = self.detection(color="red")
        action = {
            "target_id": "T01",
            "color": "blue",
            "shape": "circle",
            "body_view_direction": 0,
            "expected_centroid_px": [320, 180],
        }
        with patch.multiple(
            "classwork8.round2_executor",
            stop_chassis=lambda chassis: None,
            _point_gimbal=lambda *args, **kwargs: True,
            _wait_for_fresh_tof=lambda *args, **kwargs: 30.0,
            _set_camera_observation_pitch=lambda *args, **kwargs: True,
        ):
            ok, reason = _engage_physical_target(
                action,
                chassis=object(),
                gimbal=object(),
                blaster=blaster,
                sensors=object(),
                gimbal_tracker=object(),
                camera_service=object(),
                detector=FakeDetector([wrong]),
                mission=mission,
                auto_aim=FakeAim(wrong.detection),
                config=config,
                recorder=FakeRecorder(),
                stop_event=SimpleNamespace(is_set=lambda: False),
            )
        self.assertFalse(ok)
        self.assertEqual(reason, "TARGET_NOT_REVALIDATED")
        self.assertEqual(blaster.calls, [])

    def test_same_spec_far_from_round1_reference_never_fires(self):
        config = Classwork8Config()
        config.target_required_specs = "blue:circle"
        config.target_fire_enabled = True
        config.target_auto_aim_max_jump_px = 50.0
        mission = TargetMission(config)
        blaster = FakeBlaster()
        far = self.detection()
        far.detection.centroid = (500, 300)
        action = {
            "target_id": "T01",
            "color": "blue",
            "shape": "circle",
            "body_view_direction": 0,
            "expected_centroid_px": [320, 180],
        }
        with patch.multiple(
            "classwork8.round2_executor",
            stop_chassis=lambda chassis: None,
            _point_gimbal=lambda *args, **kwargs: True,
            _wait_for_fresh_tof=lambda *args, **kwargs: 30.0,
            _set_camera_observation_pitch=lambda *args, **kwargs: True,
        ):
            ok, reason = _engage_physical_target(
                action,
                chassis=object(),
                gimbal=object(),
                blaster=blaster,
                sensors=object(),
                gimbal_tracker=object(),
                camera_service=object(),
                detector=FakeDetector([far]),
                mission=mission,
                auto_aim=FakeAim(far.detection),
                config=config,
                recorder=FakeRecorder(),
                stop_event=SimpleNamespace(is_set=lambda: False),
            )
        self.assertFalse(ok)
        self.assertEqual(reason, "TARGET_NOT_REVALIDATED")
        self.assertEqual(blaster.calls, [])


if __name__ == "__main__":
    unittest.main()
