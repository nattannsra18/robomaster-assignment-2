"""Final Assignment - Round 1 baseline (ToF + camera only).

Round 1 responsibilities:
- explore unknown maze with nearest-frontier BFS
- build occupancy + logical topology map
- detect colored shape targets during the same gimbal scan
- record navigation-ready target observations
- fire only explicitly selected, verified and range-qualified targets
- stop only after exact 6x6 exploration completes
- export map, topology.json, targets.json and GUI map

Real firing is fail-safe OFF unless the operator supplies an exact target
allow-list and explicitly arms it in the GUI or with ``--arm-fire``.
"""

import argparse
import sys
import types


def _prepare_optional_media_codec():
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

    media_codec = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media_codec.H264Decoder = H264Decoder
    media_codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media_codec


_prepare_optional_media_codec()

from robomaster import robot

from classwork8.config import Classwork8Config
from classwork8.tof_camera_round1_v05 import run


def _defaults(config: Classwork8Config) -> None:
    # Keep geometry and odometry calibration; use Classwork8Config travel speed.
    config.cell_size_m = 0.60
    config.exploration_step_m = 0.60
    config.step_tolerance_m = 0.02
    config.odom_scale_x = 1.00
    config.odom_scale_y = 1.00
    # Field-test branch: operator-supervised motion without sensor guards.
    config.moving_gimbal_check_enabled = False
    config.wall_clearance_enabled = False
    config.unsafe_disable_motion_guards = True
    # No speed override here: the configured value is the direct chassis request.

    config.tof_recovery_wait_sec = 1.20
    config.tof_recovery_retries = 2
    config.front_block_confirm_samples = 3
    config.movement_preflight_margin_cm = 0.0

    config.closed_maze_auto_stop = True
    config.closed_maze_perimeter_wall_ratio = 0.70
    config.gui_auto_save_map = True

    # Final Round 1 camera target survey.
    config.target_detection_enabled = True
    config.stationary_target_test = False
    config.target_camera_resolution = "360p"
    config.target_min_confidence = 0.50
    config.target_save_confidence = 0.60
    config.target_quick_gate_frames = 2
    config.target_sample_frames = 4
    config.target_verify_frames = 3
    config.target_survey_open_directions = False
    config.target_fire_enabled = False
    config.target_fire_mode = "selected"
    config.target_required_specs = ""

    # Do not run the older corridor-steering camera pipeline in this baseline.
    config.vision_enabled = False
    config.vision_steering_enabled = False


def _apply_cli_overrides(config, args) -> None:
    """Reapply diagnostic limits AFTER GUI so the 1-cell trial is bounded."""
    if args.travel_speed is not None:
        config.travel_speed_mps = args.travel_speed
    if args.no_camera:
        config.target_detection_enabled = False
    if args.yaw_isolation:
        config.yaw_isolation_mode = True
        config.heading_hold_enabled = False
    if args.max_moves is not None:
        config.max_moves = args.max_moves
    if args.max_yaw_correction is not None:
        config.heading_max_z_dps = args.max_yaw_correction
        config.heading_align_max_z_dps = min(
            float(config.heading_align_max_z_dps),
            float(args.max_yaw_correction),
        )
    targets = getattr(args, "targets", None)
    arm_fire = bool(getattr(args, "arm_fire", False))
    fire_type = getattr(args, "fire_type", None)
    fire_times = getattr(args, "fire_times", None)
    if targets is not None:
        config.target_required_specs = targets
    if arm_fire:
        config.target_fire_enabled = True
        config.target_fire_mode = "selected"
    if fire_type is not None:
        config.target_fire_type = fire_type
    if fire_times is not None:
        config.target_fire_times = fire_times
    if bool(getattr(args, "stationary_target_test", False)):
        config.stationary_target_test = True
    if bool(getattr(args, "stationary_auto_lock_test", False)):
        config.stationary_target_test = True
        config.stationary_auto_lock_test = True
        config.target_fire_enabled = False
        config.target_fire_mode = "off"
        config.target_fire_type = "water"
    aim_offset_x = getattr(args, "aim_offset_x", None)
    aim_offset_y = getattr(args, "aim_offset_y", None)
    if aim_offset_x is not None:
        config.target_aim_offset_x_ratio = aim_offset_x
    if aim_offset_y is not None:
        config.target_aim_offset_y_ratio = aim_offset_y


def main():
    parser = argparse.ArgumentParser(
        description="Final Assignment Round 1 - ToF + camera map and target survey"
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="run terminal-only using current defaults",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="disable target camera and run ToF mapping only",
    )
    parser.add_argument(
        "--travel-speed",
        type=float,
        default=None,
        metavar="MPS",
        help="direct longitudinal chassis speed in m/s (overrides config default)",
    )
    parser.add_argument(
        "--yaw-isolation",
        action="store_true",
        help="diagnostic: force chassis z=0 during every move and disable all post-scan yaw alignment; log chassis/gimbal yaw separately",
    )
    parser.add_argument(
        "--max-moves",
        type=int,
        default=None,
        metavar="N",
        help="hard mission cap (e.g. 1 for a one-cell test), also after GUI",
    )
    parser.add_argument(
        "--max-yaw-correction",
        type=float,
        default=None,
        metavar="DPS",
        help="cap moving and post-scan chassis yaw commands, also after GUI",
    )
    parser.add_argument(
        "--targets",
        default=None,
        metavar="COLOR:SHAPE,...",
        help="exact target allow-list, for example blue:circle,red:rectangle",
    )
    parser.add_argument(
        "--arm-fire",
        action="store_true",
        help="explicitly enable real blaster commands after all target gates pass",
    )
    parser.add_argument(
        "--fire-type",
        choices=("ir", "water"),
        default=None,
        help="RoboMaster blaster mode (default: ir)",
    )
    parser.add_argument(
        "--fire-times",
        type=int,
        default=None,
        metavar="N",
        help="shots per selected target (1-5, default: 1)",
    )
    parser.add_argument(
        "--stationary-target-test",
        action="store_true",
        help="scan/auto-aim/export while wheel-stopped; never enter movement",
    )
    parser.add_argument(
        "--stationary-auto-lock-test",
        action="store_true",
        help="FRONT-only selected-target Auto-Lock, then manual WATER fire",
    )
    parser.add_argument(
        "--aim-offset-x",
        type=float,
        default=None,
        metavar="RATIO",
        help="camera-to-blaster desired centroid X offset (-0.25 to 0.25)",
    )
    parser.add_argument(
        "--aim-offset-y",
        type=float,
        default=None,
        metavar="RATIO",
        help="camera-to-blaster desired centroid Y offset (-0.25 to 0.25)",
    )
    args = parser.parse_args()
    if args.max_moves is not None and args.max_moves < 1:
        parser.error("--max-moves must be at least 1")
    if args.max_yaw_correction is not None and not (
        0.0 < args.max_yaw_correction <= 30.0
    ):
        parser.error("--max-yaw-correction must be >0 and <=30 deg/s")
    if args.fire_times is not None and not 1 <= args.fire_times <= 5:
        parser.error("--fire-times must be between 1 and 5")
    if args.arm_fire and not args.targets:
        parser.error("--arm-fire requires --targets COLOR:SHAPE,...")
    for name, value in (
        ("--aim-offset-x", args.aim_offset_x),
        ("--aim-offset-y", args.aim_offset_y),
    ):
        if value is not None and not -0.25 <= value <= 0.25:
            parser.error("{} must be between -0.25 and 0.25".format(name))

    config = Classwork8Config()
    _defaults(config)
    _apply_cli_overrides(config, args)

    if args.no_gui:
        config.validate()
        print(
            "[DIAG_LIMITS] max_moves={} heading_max_z={} "
            "align_max_z={} speed={} yaw_isolation={}".format(
                config.max_moves, config.heading_max_z_dps,
                config.heading_align_max_z_dps,
                config.travel_speed_mps, config.yaw_isolation_mode
            ), flush=True,
        )
        run(config=config)
        return

    from classwork8.config_gui_v05 import configure_before_run

    if not configure_before_run(config):
        print("Mission cancelled before connection.")
        return

    _apply_cli_overrides(config, args)
    config.validate()
    print(
        "[DIAG_LIMITS] max_moves={} heading_max_z={} "
        "align_max_z={} speed={} yaw_isolation={}".format(
            config.max_moves, config.heading_max_z_dps,
            config.heading_align_max_z_dps,
            config.travel_speed_mps, config.yaw_isolation_mode
        ), flush=True,
    )

    print("Connecting to RoboMaster after configuration...")
    ep_robot = robot.Robot()

    try:
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok))
        if not ok:
            raise RuntimeError(
                "RoboMaster connection failed. Check the robot Wi-Fi/AP connection."
            )
    except Exception:
        try:
            ep_robot.close()
        except Exception:
            pass
        raise

    from classwork8.gui_v05 import run_with_gui

    run_with_gui(
        run,
        config,
        ep_robot,
    )


if __name__ == "__main__":
    main()
