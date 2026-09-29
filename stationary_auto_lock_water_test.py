"""FRONT-only Auto-Lock followed by operator-triggered water fire."""

import sys

from final_round1_tof_camera_01 import main as mission_main


def auto_lock_args(args):
    args = list(args)
    if "--stationary-auto-lock-test" not in args:
        args.append("--stationary-auto-lock-test")
    return args


if __name__ == "__main__":
    sys.argv[1:] = auto_lock_args(sys.argv[1:])
    mission_main()
