"""Validate Round-1 artifacts and generate a deterministic Round-2 route.

This command is intentionally plan-only: it never connects to the robot and
never sends wheel, gimbal, or blaster commands.  The generated approach poses
must be revalidated with fresh camera/ToF feedback by the Round-2 runner before
any shot is allowed.
"""

import argparse
from pathlib import Path
import sys

from classwork8.round2_mission import (
    DIR_NAME,
    load_round1_artifacts,
    load_round2_plan,
    save_round2_plan,
)


def _parse_cell(value: str):
    try:
        x_text, y_text = value.split(",", 1)
        return int(x_text.strip()), int(y_text.strip())
    except (AttributeError, TypeError, ValueError):
        raise argparse.ArgumentTypeError("cell must be written as X,Y")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a safe Round-2 target route from Round-1 output"
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        type=Path,
        help="Round-1 output directory containing topology.json and targets.json",
    )
    parser.add_argument(
        "--targets",
        default=None,
        metavar="COLOR:SHAPE,...",
        help="exact required targets, for example blue:circle,red:rectangle",
    )
    parser.add_argument(
        "--start-cell",
        type=_parse_cell,
        default=None,
        metavar="X,Y",
        help="Round-2 start cell; default uses the saved Round-1 start cell",
    )
    parser.add_argument(
        "--start-from",
        choices=("start", "final"),
        default="start",
        help="use saved start_cell or final_cell when --start-cell is omitted",
    )
    parser.add_argument(
        "--facing",
        choices=("front", "right", "back", "left"),
        default="front",
        help="where the physical robot front points on the saved map",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="choose start/final/custom cell, facing, and targets on the map",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="output JSON path; default is RUN_DIR/round2_plan.json",
    )
    args = parser.parse_args(argv)

    try:
        if args.gui:
            from classwork8.round2_gui import run_round2_plan_gui
            saved = run_round2_plan_gui(args.run_dir, args.output)
            if saved is not None:
                print("Round-2 plan saved: {}".format(saved))
            return 0
        if not args.targets:
            parser.error("--targets is required unless --gui is used")
        topology, _targets = load_round1_artifacts(args.run_dir)
        start_cell = args.start_cell
        if start_cell is None and args.start_from == "final":
            start_cell = tuple(topology.get("final_cell", []))
        facing = {name.lower(): direction for direction, name in DIR_NAME.items()}[
            args.facing
        ]
        plan = load_round2_plan(
            args.run_dir,
            args.targets,
            start_cell=start_cell,
            start_facing_direction=facing,
        )
        output = args.output or args.run_dir / "round2_plan.json"
        save_round2_plan(plan, output)
    except (OSError, ValueError) as exc:
        parser.exit(2, "Round-2 plan rejected: {}\n".format(exc))

    print("Round-2 plan saved: {}".format(output))
    print("Targets: {} | route cells: {}".format(
        len(plan["actions"]), len(plan["full_route"])
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
