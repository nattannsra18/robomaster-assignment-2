"""Stationary four-direction target selection, auto-aim, and firing test.

The configuration GUI remains authoritative for firing mode, exact targets,
blaster type, shot count, and aim calibration.  This launcher only forces the
existing mission into stationary-test mode so chassis translation cannot start.
"""

import sys

from final_round1_tof_camera_01 import main as mission_main


def stationary_args(args):
    args = list(args)
    if "--stationary-target-test" not in args:
        args.append("--stationary-target-test")
    return args


if __name__ == "__main__":
    sys.argv[1:] = stationary_args(sys.argv[1:])
    mission_main()
