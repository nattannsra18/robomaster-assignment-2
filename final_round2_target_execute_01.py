"""Execute a verified Round-2 plan on RoboMaster EP.

Validation-only is the default. Physical movement requires both ``--execute``
and ``--confirm-start``; real firing additionally requires ``--arm-fire``.
"""

import argparse
import json
from pathlib import Path
import sys

from final_round1_tof_camera_01 import _prepare_optional_media_codec


_prepare_optional_media_codec()

from classwork8.round2_executor import (
    load_round1_config,
    load_verified_execution_plan,
    run_round2_physical,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate or physically execute a saved Round-2 target plan"
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        type=Path,
        help="completed Round-1 output directory",
    )
    parser.add_argument(
        "--plan",
        type=Path,
        default=None,
        help="plan JSON; default is RUN_DIR/round2_plan.json",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="choose start cell, facing and targets before validation/execution",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="connect to the robot and execute movement; default is validation only",
    )
    parser.add_argument(
        "--confirm-start",
        action="store_true",
        help="confirm the physical robot is on the plan start cell and facing",
    )
    parser.add_argument(
        "--arm-fire",
        action="store_true",
        help="allow real blaster commands after live range/detection/aim gates",
    )
    parser.add_argument(
        "--fire-type",
        choices=("ir", "water"),
        default="ir",
        help="RoboMaster blaster mode (default: ir)",
    )
    parser.add_argument(
        "--fire-times",
        type=int,
        default=1,
        metavar="N",
        help="shots per selected target (1-5)",
    )
    parser.add_argument(
        "--connection",
        choices=("ap", "sta", "rndis"),
        default=None,
        help="override the Round-1 saved connection mode",
    )
    parser.add_argument(
        "--travel-speed",
        type=float,
        default=None,
        metavar="MPS",
        help="override saved speed, from 0.03 to 0.30 m/s",
    )
    parser.add_argument(
        "--max-route-steps",
        type=int,
        default=None,
        metavar="N",
        help="bounded hardware test: stop after N completed cell moves",
    )
    parser.add_argument(
        "--aim-offset-x",
        type=float,
        default=None,
        metavar="RATIO",
        help="override saved camera-to-blaster X offset (-0.25 to 0.25)",
    )
    parser.add_argument(
        "--aim-offset-y",
        type=float,
        default=None,
        metavar="RATIO",
        help="override saved camera-to-blaster Y offset (-0.25 to 0.25)",
    )
    args = parser.parse_args(argv)

    if args.execute and not args.confirm_start:
        parser.error("--execute requires --confirm-start")
    if args.arm_fire and not args.execute:
        parser.error("--arm-fire requires --execute")
    if not 1 <= args.fire_times <= 5:
        parser.error("--fire-times must be between 1 and 5")
    if args.travel_speed is not None and not 0.03 <= args.travel_speed <= 0.30:
        parser.error("--travel-speed must be between 0.03 and 0.30 m/s")
    if args.max_route_steps is not None and args.max_route_steps < 1:
        parser.error("--max-route-steps must be at least 1")
    for name, value in (
        ("--aim-offset-x", args.aim_offset_x),
        ("--aim-offset-y", args.aim_offset_y),
    ):
        if value is not None and not -0.25 <= value <= 0.25:
            parser.error("{} must be between -0.25 and 0.25".format(name))

    plan_path = args.plan or args.run_dir / "round2_plan.json"
    try:
        if args.gui:
            from classwork8.round2_gui import run_round2_plan_gui
            selected = run_round2_plan_gui(args.run_dir, plan_path)
            if selected is None:
                print("Round-2 execution cancelled before robot connection.")
                return 1
            plan_path = selected
        plan = load_verified_execution_plan(args.run_dir, plan_path)
        config = load_round1_config(args.run_dir)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        parser.exit(2, "Round-2 input rejected: {}\n".format(exc))

    config.target_fire_enabled = bool(args.arm_fire)
    config.target_fire_mode = "selected" if args.arm_fire else "off"
    config.target_fire_type = str(args.fire_type)
    config.target_fire_times = int(args.fire_times)
    if args.connection is not None:
        config.connection = args.connection
    if args.travel_speed is not None:
        config.travel_speed_mps = float(args.travel_speed)
    if args.aim_offset_x is not None:
        config.target_aim_offset_x_ratio = float(args.aim_offset_x)
    if args.aim_offset_y is not None:
        config.target_aim_offset_y_ratio = float(args.aim_offset_y)

    print("Round-2 plan verified: {}".format(plan_path))
    print("Start cell : {}".format(tuple(plan["start_cell"])))
    print("Robot front: {}".format(plan["start_facing_direction_name"]))
    print("Targets    : {}".format(", ".join(plan["required_targets"])))
    print("Route cells: {}".format(len(plan["full_route"])))
    if not args.execute:
        print("Validation only: no robot connection or command was made.")
        return 0

    print(
        "Physical execution: firing {}.".format(
            "ARMED" if config.target_fire_enabled else "OFF (aim dry-run)"
        )
    )
    result, output_path = run_round2_physical(
        plan,
        config,
        args.run_dir,
        max_route_steps=args.max_route_steps,
    )
    print("Round-2 result: {} ({})".format(
        "COMPLETE" if result.completed else "STOPPED",
        result.reason,
    ))
    print("Execution log: {}".format(output_path))
    return 0 if result.completed else 1


if __name__ == "__main__":
    sys.exit(main())
